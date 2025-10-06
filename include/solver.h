#pragma once
#include <rjps.h>
#include <scanner.h>
#include <Ray.h>
#include <Log.h>
#include <warthog/heuristic/octile_heuristic.h>
#include <warthog/util/timer.h>
#include <queue>
#include <unordered_map>
// #include <boost/heap/pairing_heap.hpp>
#include <boost/heap/pairing_heap.hpp>

// typedef typename boost::heap::fibonacci_heap<search_node>::handle_type handle_t;
//0x1.6a09e6p0 - root2

struct scan_dir
{
    direction cw_init   {}; //the direction of initial scan in CW orientation
    direction cw_subseq {}; //subsequent scan directions in CW orientation
    direction cw_jps    {}; //direction of scan after all jps scans in CW orientation
    direction ccw_init  {}; //the direction of initial scan in CCW orientation
    direction ccw_subseq{}; //subsequent scan directions in CCW orientation
    direction ccw_jps   {}; //direction of scan after all jps scans in CCW orientation
};

struct DirectionInfo
{
    direction init      {};
    direction subseq    {};
    direction jps       {};
    direction terminate {};

    DirectionInfo(direction _i, direction _s, direction _j,direction _t): init(_i), subseq(_s), jps(_j), terminate(_t){};
    DirectionInfo(){};
};

struct experiment_result
{
    std::chrono::nanoseconds  nanos{};
    double  ray_scan_time{};
    double  plenth{};
    int     heap_pops{};
    int     generated{};
    int     expanded{};
    int     reopend{};
    int     updated{};
};

template<SolverTraits ST>
class Solver
{
public:
    Solver(jump::jump_point_online<>* _jps) : 
    m_jps(_jps), m_tracer(new Tracer), m_map(m_jps->get_map()), m_rmap(m_jps->get_rmap()), 
    m_ray(m_tracer, m_jps), m_scanner(m_tracer, m_jps), m_octile_h(m_map.width(), m_map.height()),
    m_timer()
    {
        static_assert(ST == SolverTraits::Default || ST == SolverTraits::OutputToPosthoc);
        m_tracer->set_dim(m_map.dim());
        m_all_node_list.reserve(2048);
    }
    ~Solver() = default;
    void get_path(pad_id start, pad_id target);
    inline experiment_result get_result(){return m_stats;};

private:

    jump::jump_point_online<>*          m_jps;
    std::shared_ptr<Tracer>             m_tracer;
    warthog::domain::gridmap::bittable  m_map;
    warthog::domain::gridmap::bittable  m_rmap;
    heuristic::octile_heuristic         m_octile_h;
    Ray                                 m_ray;
    Scanner                             m_scanner;
    pad_id                              m_target;
    std::pair<uint32_t, uint32_t>       m_tcoord;
    experiment_result                   m_stats;
    warthog::util::timer                m_timer;
    std::vector<rjps_state>             m_succ;
    std::unordered_map<string, search_node> m_all_node_list;
    boost::heap::pairing_heap<search_node> m_pq;

    void expand_node(search_node n);
    template <direction D>
    void expand(search_node cur);
    void generate(search_node* parent);
    void insert(rjps_state succ, search_node *pred);
    void insert_start_state(const rjps_state& succ);
    bool target_in_scan_quad(pad_id start, direction quad);
    template <ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
    uint32_t scan_in_bound(pad_id start, search_node parent, uint32_t boundary, direction start_d);
    //initilizes a scan_dir struct based on a cell on a grid and an rjps scan direction
    //returns true if starts on a convex point
    bool init_scan_dir(pad_id start, direction p_dir, scan_dir &dir);
    pad_id grid_ray_incident(pad_id from, pad_id to, direction d);
    double interval_h(const rjps_state& v);
};

template <SolverTraits ST>
inline void Solver<ST>::expand_node(search_node n)
{
    switch (n.state.dir)
    {
    case NORTHWEST:
        expand<NORTHWEST>(n);
        break;
    case NORTHEAST:
        expand<NORTHEAST>(n);        
        break;
    case SOUTHWEST:
        expand<SOUTHWEST>(n);        
        break;
    case SOUTHEAST:
        expand<SOUTHEAST>(n);        
        break;
    default:
        assert(false && "invalide node expansion");
        break;
    }
}

template <SolverTraits ST>
template <direction D>
void Solver<ST>::expand(search_node cur)
{
    using namespace ScanAttribute;
    static_assert(
	    D == NORTHEAST || D == NORTHWEST || D == SOUTHEAST || D == SOUTHWEST,
	    "D must be inter-cardinal.");
    auto cur_coord = m_map.id_to_xy(cur.state.id);
    if constexpr(ST == SolverTraits::OutputToPosthoc)
    {
        m_tracer->expand(cur_coord, "orange", 
            "expanding, h: " + to_string(cur.hval) +
            " ,g: " + to_string(cur.gval) + 
            " ,f: "+ to_string(cur.gval + cur.hval));
        m_tracer->draw_bounds(cur_coord, cur.state.dir);
    }
    m_succ.clear();
    m_stats.expanded++;
    //coord postion of ray intersec from shooting to target, potentially unused
    auto temp = pad_id{}, target_scan_start = temp;
    auto s_dir = scan_dir{};
    bool target_blocked = false;
    //left octant and right octant
    constexpr Octants loct = get_left_octant<D>(), roct = get_right_octant<D>();
    //if target is in same quadrant, shoot to it
    if(target_in_scan_quad(cur.state.id, cur.state.dir))
    {
        std::pair<bool, pad_id> vis_res;
        if(on_left_octant<D>(cur_coord, m_tcoord))
        {
            vis_res = m_ray.check_target_visible<ST, loct>(cur.state.id, m_target, cur.state.dir);
        }
        else
        {
            vis_res = m_ray.check_target_visible<ST, roct>(cur.state.id, m_target, cur.state.dir);
        }
        //if target is visible, return function
        if(vis_res.second == m_target)
        {
            m_succ.emplace_back(m_target);
            return;
        }
        //save the coord of the blocking cell, then scan later from cell after getting the bounds for scanning
        else    
        {
            //optimization: if the collision point of the ray shot to target was on the diaganal, we can skip scanning 
            //from that point since we will always shoot a diag ray then scan from there when expanding a node
            target_blocked = vis_res.first;
            target_scan_start = vis_res.second;
        }
    }
    temp = m_ray.shoot_diag_ray_id<ST>(cur.state.id, cur.state.dir);
    auto diag_bounds = m_map.id_to_xy(temp);
    auto on_convex = init_scan_dir(temp, cur.state.dir, s_dir);
    auto cw_start = temp, ccw_start = temp;
    if(on_convex)
    {
        cw_start = shift_in_dir(cw_start, 1, s_dir.cw_init, m_map);
        ccw_start = shift_in_dir(ccw_start, 1, s_dir.ccw_init, m_map);
    }
    auto dir_info = DirectionInfo{};
    uint32_t cwbound = 0, ccwbound = 0;
    cwbound = horizontally_bound(roct) ? cur_coord.first: cur_coord.second;
    ccwbound = horizontally_bound(loct) ? cur_coord.first: cur_coord.second;
    //scan both ways from the point the ray intercepted
    //CW scan
    dir_info.init = s_dir.cw_init;
    cwbound = scan_in_bound<CW, roct>(cw_start, cur, cwbound, s_dir.cw_init);
    // dir_info.jps = s_dir.cw_jps;
    // dir_info.subseq = s_dir.cw_subseq;
    // dir_info.terminate = rotate_eighth<Orientation::CCW>(cur.state.dir);
    // uint32_t cwbound = scan_in_bound<Orientation::CW>(cw_start, cur, heap, cur_coord.first, cur_coord.second, dir_info);

    //CCW scan
    dir_info.init = s_dir.ccw_init;
    ccwbound = scan_in_bound<CCW, loct>(ccw_start, cur, ccwbound, s_dir.ccw_init);
    // dir_info.jps = s_dir.ccw_jps;
    // dir_info.subseq = s_dir.ccw_subseq;
    // dir_info.terminate = rotate_eighth<Orientation::CW>(cur.state.dir);
    // uint32_t ccwbound = scan_in_bound<Orientation::CCW>(ccw_start, cur, heap, cur_coord.first, cur_coord.second, dir_info);

    if(target_blocked)[[unlikely]]
    {
        uint32_t cw_bound = 0, ccw_bound = 0;
        if(on_left_octant<D>(cur_coord, m_tcoord))
        {
            //left octant always uses diag bounds as scanning bound for CW, and horizontal bounds for CCW
            if constexpr (horizontally_bound(loct)) {cw_bound = diag_bounds.first, ccw_bound = cur_coord.first;}
            else                                    {cw_bound = diag_bounds.second, ccw_bound = cur_coord.second;}
            
            scan_in_bound<CW, loct> (target_scan_start, cur, cw_bound, get_subseq_dir<CW, loct>());
            scan_in_bound<CCW, loct>(target_scan_start, cur, ccw_bound, get_subseq_dir<CCW, loct>());
        }
        else    
        {
            if constexpr (horizontally_bound(roct)) {cw_bound = cur_coord.first, ccw_bound = diag_bounds.first;}
            else                                    {cw_bound = cur_coord.second, ccw_bound = diag_bounds.second;}
            scan_in_bound<CW, roct> (target_scan_start, cur, cw_bound, get_subseq_dir<CW, roct>());
            scan_in_bound<CCW, roct>(target_scan_start, cur, ccw_bound, get_subseq_dir<CCW, roct>());
        }
    }

    //CW scan from left extremety(left of scan center)
    dir_info.init = s_dir.cw_jps;
    temp = m_ray.shoot_rjps_ray<ST>(cur.state.id, s_dir.ccw_jps, m_succ);
    scan_in_bound<CW, loct>(temp, cur, ccwbound, s_dir.cw_jps);
    
    //CCW scan from right extremety(right of scan center)
    dir_info.init = s_dir.ccw_jps;
    temp = m_ray.shoot_rjps_ray<ST>(cur.state.id, s_dir.cw_jps, m_succ);
    scan_in_bound<CCW, roct>(temp, cur, cwbound, s_dir.ccw_jps);
}

template <SolverTraits ST>
void Solver<ST>::get_path(pad_id start, pad_id target)
{
    m_stats = experiment_result{};
    m_pq.clear();
    m_all_node_list.clear();
    m_target = target;
    m_tcoord = m_map.id_to_xy(target);
    auto start_coord = m_map.id_to_xy(start);
    if constexpr(ST == SolverTraits::OutputToPosthoc)
    {
        m_tracer->init(start_coord, m_tcoord);
    }
    auto cmp = [](search_node a, search_node b){return (a.gval + a.hval) > (b.gval + b.hval);};
    m_succ.reserve(2048);
    m_timer.start();
    auto start_state = rjps_state{};
    start_state.id  = start;
    start_state.dir = NORTHEAST;  
    insert_start_state(start_state);
    start_state.dir = NORTHWEST;
    insert_start_state(start_state);
    start_state.dir = SOUTHEAST;
    insert_start_state(start_state);
    start_state.dir = SOUTHWEST;
    insert_start_state(start_state);
    while(!m_pq.empty())
    {
        //pop the node with lowest fval off the heap
        auto cur = m_pq.top();
        // std::cout << (cur.gval + cur.hval)<<'\n';
        if(cur.state.id == m_target)
        {
            m_stats.nanos = m_timer.elapsed_time_nano();
            m_stats.plenth = cur.gval;
            break;
        }
        m_pq.pop();
        // std::cout << (cur.gval + cur.hval)<< " id: " << to_string((uint64_t)cur.state.id) << " size: " +to_string(m_pq.size())+'\n';
        auto cur_coord = m_map.id_to_xy(cur.state.id);
        auto exp_start = m_timer.elapsed_time_nano();
        expand_node(cur);
        auto exp_end = m_timer.elapsed_time_nano();
        m_stats.ray_scan_time += exp_end.count()-exp_start.count();
        auto cur_ptr = &m_all_node_list[cur.get_key()];   //pass the cur node pointer for successors
        cur_ptr->closed = true;
        generate(cur_ptr);
        if constexpr(ST == SolverTraits::OutputToPosthoc)
        {
            m_tracer->close_node(cur_coord);
        }
    }
    auto c = m_pq.top();
    auto stk = std::stack<search_node>{};
    stk.push(c);
    auto p = c.parent;
    while (p != nullptr)
    {
        c = *p;
        p = c.parent;
        stk.push(c);
    }
    stk.pop();
    while (!stk.empty())
    {
        const auto &cur = stk.top();
        // m_ray.check_target_visible(cur.parent->state.id, cur.state.id, cur.parent->state.dir);
        stk.pop();
    }
    if constexpr(ST == SolverTraits::OutputToPosthoc)
    {
        m_tracer->close();
    }
    return;
}

template <SolverTraits ST>
inline bool Solver<ST>::target_in_scan_quad(pad_id start, direction quad)
{    
    auto s = m_map.id_to_xy(start);
    switch (quad)
    {
    case NORTHEAST:
        return(m_tcoord.first >= s.first && m_tcoord.second <= s.second);
        break;
    case NORTHWEST:
        return(m_tcoord.first <= s.first && m_tcoord.second <= s.second);
        break;
    case SOUTHEAST:
        return(m_tcoord.first >= s.first && m_tcoord.second >= s.second);
        break;
    case SOUTHWEST:
        return(m_tcoord.first <= s.first && m_tcoord.second >= s.second);
        break;
    default:
        assert(false);
        break;
    }
}

template <SolverTraits ST>
bool Solver<ST>::init_scan_dir(pad_id start, direction p_dir, scan_dir &dir)
{
    assert(
    p_dir == NORTHEAST || p_dir == NORTHWEST || p_dir == SOUTHEAST || p_dir == SOUTHWEST 
    && "Init scan direction: in dir must be intercardinal direction");
    auto dir_ind =std::countr_zero<uint8_t>(p_dir) - 4;
    auto adjx = adj[dir_ind][0], adjy = adj[dir_ind][1];
    bool vert, hori, ret = false;
    vert = m_map.get(pad_id{start.id + adjy * (int32_t)m_map.width()});
    hori = m_map.get(pad_id{start.id + adjx});
    if(vert) //vert not blocked
    {
        if(hori) //convex point
        {
            dir.cw_init = d_init_scan_CW[dir_ind][dir_ind == 0 || dir_ind == 3];
            dir.ccw_init = d_init_scan_CCW[dir_ind][dir_ind == 1 || dir_ind == 2];
            ret = true;
        }
        else//vert
        {
            dir.cw_init = d_init_scan_CW[dir_ind][0];
            dir.ccw_init = d_init_scan_CCW[dir_ind][0];
        }
    }
    else //vert blocked
    {
        if(hori)//hori
        {
            dir.cw_init = d_init_scan_CW[dir_ind][1];
            dir.ccw_init = d_init_scan_CCW[dir_ind][1];
        }
        else//concave point
        {
            dir.cw_init = d_init_scan_CW[dir_ind][dir_ind == 1 || dir_ind == 2];
            dir.ccw_init = d_init_scan_CCW[dir_ind][dir_ind == 0 || dir_ind == 3];
        }
    }
    dir.cw_subseq = d_scan[dir_ind][0];
    dir.cw_jps = d_jps[dir_ind][0];
    dir.ccw_subseq = d_scan[dir_ind][1];
    dir.ccw_jps = d_jps[dir_ind][1];
    return ret;
}

template <SolverTraits ST>
pad_id Solver<ST>::grid_ray_incident(pad_id from, pad_id to, direction d)
{
    auto f_coord = m_map.id_to_xy(from), t_coord = m_map.id_to_xy(to);
    auto m = min(abs((int)f_coord.first - (int)t_coord.first), abs((int)f_coord.second - (int)t_coord.second));
    return shift_in_dir(from, m, d, m_map);
}

template <SolverTraits ST>
inline void Solver<ST>::generate(search_node* parent)
{
    auto top_adj = direction{}, bottom_adj = top_adj;
    for(rjps_state& succ : m_succ)
    {        
        switch (succ.dir)
        {
        case NORTH:
            top_adj = SOUTHWEST; bottom_adj = SOUTHEAST;
            break;
        case SOUTH:
            top_adj = NORTHWEST; bottom_adj = NORTHEAST;
            break;
        case EAST:
            top_adj = NORTHWEST; bottom_adj = SOUTHWEST;
            break;
        case WEST:
            top_adj = NORTHEAST; bottom_adj = SOUTHEAST;
            break;
        default:
            if(succ.id == m_target) 
            {
                auto n = search_node{succ};
                n.parent = parent;
                n.hval = 0;
                n.gval = m_octile_h.h(n.state.id.id, parent->state.id.id) + parent->gval;
                m_pq.push(n);
                return;
            }
            else assert(false && "successor dir is NONE");
        }
        bool top = !m_map.get(shift_in_dir(succ.id, 1, top_adj, m_map));
        bool bottom = !m_map.get(shift_in_dir(succ.id, 1, bottom_adj, m_map));
        if(top && bottom) [[unlikely]]
        {
            auto aux_succ = succ;
            succ.dir = quad.at(get_succ_sector(parent->state.dir, succ.dir, true));
            insert(succ, parent);
            aux_succ.dir = quad.at(get_succ_sector(parent->state.dir, aux_succ.dir, false));
            insert(aux_succ, parent);
        }
        else
        {
            succ.dir = quad.at(get_succ_sector(parent->state.dir, succ.dir, top));
            insert(succ, parent);        
        }
    }
}

template <SolverTraits ST>
void Solver<ST>::insert(rjps_state succ, search_node *pred)
{
    auto n = search_node{succ};
    const auto exist = m_all_node_list.find(n.get_key());
    n.parent = pred;
    n.gval = m_octile_h.h(n.state.id.id, n.parent->state.id.id) + n.parent->gval;
    if(exist == m_all_node_list.end())
    {
        m_stats.generated++;
        n.hval = interval_h(n.state);
        if constexpr(ST == SolverTraits::OutputToPosthoc)
        {
        m_tracer->expand(m_map.id_to_xy(n.state.id), "fuchsia", 
            "generating, h: " + to_string(n.hval) +
            " ,g: " + to_string(n.gval) + 
            " ,f: "+ to_string(n.gval + n.hval) + 
            " ,dir:" + to_string(n.state.dir));
        }
        boost::heap::pairing_heap<search_node>::handle_type h = m_pq.push(n);
        (*h).handle = h;
        auto err = m_all_node_list.emplace(std::make_pair(n.get_key(), *h));        
        assert(err.second);
    }
    else
    {
        auto& e = exist->second;
        if(n.gval < e.gval)
        {
            if(e.closed)
            {
                m_stats.reopend++;
                n.hval = e.hval;
                boost::heap::pairing_heap<search_node>::handle_type h = m_pq.push(n);
                (*h).handle = h;
                exist->second = *h;
                if constexpr(ST == SolverTraits::OutputToPosthoc)
                {
                m_tracer->expand(m_map.id_to_xy(n.state.id), "red", 
                    "re-opening, h: " + to_string(n.hval) +
                    " ,g: " + to_string(n.gval) + 
                    " ,f: "+ to_string(n.gval + n.hval) + 
                    " ,dir:" + to_string(n.state.dir));
                }
                // assert(false && "reopeing not handled");
            }
            else
            {
                m_stats.updated++;
                n.hval = e.hval;
                e.gval = n.gval;
                e.parent = n.parent;
                if constexpr(ST == SolverTraits::OutputToPosthoc)
                {
                m_tracer->expand(m_map.id_to_xy(n.state.id), "yellow", 
                    "updating, h: " + to_string(n.hval) +
                    " ,g: " + to_string(n.gval) + 
                    " ,f: "+ to_string(n.gval + n.hval) + 
                    " ,dir:" + to_string(n.state.dir));
                }
                m_pq.decrease(e.handle, n);
            }
        }
    }
}

template <SolverTraits ST>
inline void Solver<ST>::insert_start_state(const rjps_state& succ)
{
    auto n = search_node{succ};
    n.gval = 0;
    n.hval = interval_h(succ);
    auto key = std::string(to_string((uint64_t)n.state.id) + to_string(n.state.dir));

    boost::heap::pairing_heap<search_node>::handle_type h = m_pq.push(n);
    (*h).handle = h;
    auto err = m_all_node_list.emplace(std::make_pair(n.get_key(), *h));        
    assert(err.second);
}

//Scans all visible obstacles in a set orientation, appends succesors to the vector passed in
//@param start: position to start the scan
//@param parent: rjps search node the scan is based on
//@param &vec: reference to the return vector of rjps nodes
//@param xbound: x boundary
//@param ybound: y boundary
//@param dir_info: collection of direction info which includes initial scan direction, subseq scan direction and terminating direction
template <SolverTraits ST>
template <ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
uint32_t Solver<ST>::scan_in_bound(pad_id start, search_node parent, uint32_t boundary, direction start_d)
{
    using ScanAttribute::Octants;
    using namespace ScanAttribute;

    auto scan_frontier = uint32_t{};
    auto start_coord = m_map.id_to_xy(start);
    //returning scan_frontier only matters when scanning from diag boundary
    if constexpr(horizontally_bound(Octant)) {scan_frontier = start_coord.first;}
    else                                     {scan_frontier = start_coord.second;}

    DirectionInfo dir_info{};
    dir_info.init = start_d;
    dir_info.subseq = get_subseq_dir<O, Octant>();
    dir_info.jps    = get_jps_dir<Octant>();
    dir_info.terminate = get_terminate_dir<O, Octant>();
    auto scan_res = scanResult{};
    auto poi = pad_id{}, succ = poi;
    scan_res.d = start_d;
    uint32_t dir_ind = std::countr_zero<uint8_t>(parent.state.dir)-4;
    scan_res.top = get_init_scan_top<Octant>(scan_res.d);
    if constexpr(horizontally_bound(Octant))    //x is the primary boundary
    {
        poi = m_scanner.find_turning_point<ST>(start, scan_res, dir_info.terminate, boundary, UINT32_MAX);
    }
    else//y is the primary boundary
    {
        poi = m_scanner.find_turning_point<ST>(start, scan_res, dir_info.terminate, UINT32_MAX, boundary);        
    }
    // poi = m_scanner.find_turning_point(start, scan_res, dir_info.terminate, xbound, ybound);

    while (!poi.is_none())  // poi will be none if scan leaves bound, if not recurse scan
    { 
        auto ince = grid_ray_incident(parent.state.id, poi, parent.state.dir);
        succ = m_ray.shoot_rjps_ray_to_target<ST>(ince, poi, dir_info.jps, m_succ);
        if constexpr(ST == SolverTraits::OutputToPosthoc)
        {
            m_tracer->trace_ray(m_map.id_to_xy(parent.state.id), m_map.id_to_xy(ince), "aqua", "shoot ray to point");
            m_tracer->trace_ray(m_map.id_to_xy(ince), m_map.id_to_xy(succ), "aqua", "shoot ray to point");
        }
        //if poi is blocked by another obstacle, recurse scan in both orientation on collision point
        //depending on the orientation and the quadrant, x or y bound is changed
        if(succ != poi)
        {
            auto start_coord = m_map.id_to_xy(start);
            uint32_t cw_frontier = 0, ccw_frontier = 0, cw_bound = boundary, ccw_bound = boundary;
            if constexpr (O == ScanAttribute::CW)   //scanning bound for scanning in opposite orientation will be the current frontier
            {
                ccw_bound = scan_frontier;
            }
            else
            {
                cw_bound = scan_frontier;
            }
            cw_frontier =  scan_in_bound<CW, Octant>(succ, parent, cw_bound, get_subseq_dir<CW, Octant>());
            ccw_frontier = scan_in_bound<CCW, Octant>(succ, parent, ccw_bound, get_subseq_dir<CCW, Octant>());
            return O == Orientation::CW ? cw_frontier : ccw_frontier;
        }
        //poi is visible, shoot a jps ray towards it then continue scanning in orientation O
        else
        {
            auto s_coord = m_map.id_to_xy(poi);
            if constexpr(horizontally_bound(Octant)) scan_frontier = s_coord.first;
            else                                     scan_frontier = s_coord.second;
            //the scan result indicates the first poi was a concave point, which returns the next convex point
            if(scan_res.on_concave) [[unlikely]]
            {
                //shift 1 step more to the actual corner point
                poi = shift_in_dir(poi, 1, scan_res.d, m_map);                
            }
            //poi was a convex point, shoot a jps ray to and pass it
            else
            {
                //since poin is guranteed to be a jump point(turning point), shoot a jps ray towards it to push it onto the heap
                //along with all jump points along the way. TODO: extra jump points might be unnecessary?
                poi = m_ray.shoot_rjps_ray<ST>(poi, dir_info.jps, m_succ);             
                scan_res.d = dir_info.subseq;
                scan_res.top = (dir_info.jps == NORTH || dir_info.jps == WEST);
            }
            if constexpr(horizontally_bound(Octant))    //x is the primary boundary
            {
                poi = m_scanner.find_turning_point<ST>(poi, scan_res, dir_info.terminate, boundary, UINT32_MAX);
            }
            else//y is the primary boundary
            {
                poi = m_scanner.find_turning_point<ST>(poi, scan_res, dir_info.terminate, UINT32_MAX, boundary);        
            }
        }
    }
    return scan_frontier;
}

template <SolverTraits ST>
double Solver<ST>::interval_h(const rjps_state& v)
{
    //if the target resides inside scanning sector, return regular octile heuristic
    return m_octile_h.h(v.id.id, m_target.id);
    // if (target_in_scan_quad(v.id, v.dir))
    // {
    //     return m_octile_h.h(v.id.id, m_target.id);
    // }
    
    // double hx = 0, hy = 0;
    // auto hori_dir = direction{}, vert_dir = direction{};
    // auto x_intv = pad_id{}, y_intv = x_intv;
    // vert_dir = (v.dir == NORTHEAST || v.dir == NORTHWEST) ? NORTH : SOUTH;
    // hori_dir = (v.dir == NORTHEAST || v.dir == SOUTHEAST) ? EAST : WEST;

    // auto jump = m_jps->jump_cardinal(vert_dir, jps_id{v.id}, m_jps->id_to_rid(jps_id{v.id}));
    // hy += abs(jump.first);
    // //if jump finds a turning point, jump.first will be positive, deadends will be negative
    // //interval point will be either a turning point if one is found, otherwise the first point that leaves the obstacle in x direction
    // if(jump.first > 0) y_intv = jump.second;
    // else
    // {
    //     //need to shift by 1 to start inside obstacle
    //     auto r = m_ray.shoot_hori_ray<Obstacle>( shift_in_dir(jump.second, 1, vert_dir, m_map) , vert_dir);
    //     y_intv = r.second;
    //     hy += r.first + DBL_ROOT_TWO;
    // }

    // jump = m_jps->jump_cardinal(hori_dir, jps_id{v.id}, m_jps->id_to_rid(jps_id{v.id}));
    // hx += abs(jump.first);
    // if(jump.first > 0) x_intv = jump.second; 
    // else
    // {
    //     auto r = m_ray.shoot_hori_ray<Obstacle>( shift_in_dir(jump.second, 1, hori_dir, m_map) , hori_dir);
    //     x_intv = r.second;
    //     hx += r.first + DBL_ROOT_TWO;
    // }
    // hx += m_octile_h.h(x_intv.id, m_target.id);
    // hy += m_octile_h.h(y_intv.id, m_target.id);
    // if constexpr(ST == SolverTraits::OutputToPosthoc)
    // {
    //     m_tracer->draw_cell(m_map.id_to_xy(x_intv), "orange", "intx, h: " + to_string(hx));
    //     m_tracer->draw_cell(m_map.id_to_xy(y_intv), "orange", "inty, h: " + to_string(hy));
    // }
    // return std::min(hx, hy);
}

