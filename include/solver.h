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
    direction_id cw_init   {}; //the direction of initial scan in CW orientation
    direction_id cw_subseq {}; //subsequent scan directions in CW orientation
    direction_id cw_jps    {}; //direction of scan after all jps scans in CW orientation
    direction_id ccw_init  {}; //the direction of initial scan in CCW orientation
    direction_id ccw_subseq{}; //subsequent scan directions in CCW orientation
    direction_id ccw_jps   {}; //direction of scan after all jps scans in CCW orientation
};

struct DirectionInfo
{
    direction_id init      {};
    direction_id subseq    {};
    direction_id jps       {};
    direction_id terminate {};

    DirectionInfo(direction_id _i, direction_id _s, direction_id _j,direction_id _t): init(_i), subseq(_s), jps(_j), terminate(_t){};
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
    Solver(jump::jump_point_online* _jps, gridmap_rotate_table_convs _map) : 
    m_jps(_jps), m_map(_map), m_tracer(new Tracer),
    m_ray(m_tracer, m_jps, m_map), m_scanner(m_tracer, m_jps, m_map), m_octile_h(m_map.width(), m_map.height()),
    m_timer()
    {
        static_assert(ST == SolverTraits::Default || ST == SolverTraits::OutputToPosthoc);
        m_tracer->set_dim(m_map.table().dim());
        m_all_node_list.reserve(2048);
    }
    ~Solver() = default;
    void get_path(grid_id start, grid_id target);
    inline experiment_result get_result(){return m_stats;};

private:

    jump::jump_point_online*            m_jps;
    gridmap_rotate_table_convs          m_map;
    std::shared_ptr<Tracer>             m_tracer;
    heuristic::octile_heuristic         m_octile_h;
    Ray                                 m_ray;
    Scanner                             m_scanner;
    grid_id                             m_target;
    point                               m_tcoord;
    experiment_result                   m_stats;
    warthog::util::timer                m_timer;
    std::vector<rjps_state>             m_succ;
    std::unordered_map<uint64_t, search_node> m_all_node_list;
    boost::heap::pairing_heap<search_node> m_pq;

    void expand_node(search_node n);
    template <direction_id D>
    void expand(search_node cur);
    void generate(search_node* parent);
    void insert(rjps_state succ, search_node *pred);
    void insert_start_state(const rjps_state& succ);
    bool target_in_scan_quad(grid_id start, direction_id quad);
    template <ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
    uint32_t scan_in_bound(grid_id start, search_node parent, uint32_t boundary, direction_id start_d);
    //initilizes a scan_dir struct based on a cell on a grid and an rjps scan direction
    //returns true if starts on a convex point
    bool init_scan_dir(grid_id start, direction_id p_dir, scan_dir &dir);
    grid_id grid_ray_incident(grid_id from, grid_id to, direction_id d);
    double interval_h(const rjps_state& v);
};

template <SolverTraits ST>
inline void Solver<ST>::expand_node(search_node n)
{
    switch (n.state.dir)
    {
    case NORTHWEST_ID:
        expand<NORTHWEST_ID>(n);
        break;
    case NORTHEAST_ID:
        expand<NORTHEAST_ID>(n);        
        break;
    case SOUTHWEST_ID:
        expand<SOUTHWEST_ID>(n);        
        break;
    case SOUTHEAST_ID:
        expand<SOUTHEAST_ID>(n);        
        break;
    default:
        assert(false && "invalide node expansion");
        break;
    }
}

template <SolverTraits ST>
template <direction_id D>
void Solver<ST>::expand(search_node cur)
{
    using namespace ScanAttribute;
    static_assert(
	    D == NORTHEAST_ID || D == NORTHWEST_ID || D == SOUTHEAST_ID || D == SOUTHWEST_ID,
	    "D must be inter-cardinal.");
    auto cur_coord = m_map.id_to_point(cur.state.id);
    if constexpr(ST == SolverTraits::OutputToPosthoc)
    {
        m_tracer->expand(cur_coord, "orange", 
            "expanding, h: " + std::to_string(cur.hval) +
            " ,g: " + std::to_string(cur.gval) + 
            " ,f: "+ std::to_string(cur.gval + cur.hval));
        m_tracer->draw_bounds(cur_coord, cur.state.dir);
    }
    m_succ.clear();
    m_stats.expanded++;
    //coord postion of ray intersec from shooting to target, potentially unused
    auto temp = grid_id{}, target_scan_start = temp;
    auto s_dir = scan_dir{};
    bool target_blocked = false;
    //left octant and right octant
    constexpr Octants loct = get_left_octant<D>(), roct = get_right_octant<D>();
    //if target is in same quadrant, shoot to it
    if(target_in_scan_quad(cur.state.id, cur.state.dir))
    {
        std::pair<bool, grid_id> vis_res;
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
    auto diag_bounds = m_map.id_to_point(temp);
    auto on_convex = init_scan_dir(temp, cur.state.dir, s_dir);
    auto cw_start = temp, ccw_start = temp;
    if(on_convex)
    {
        cw_start = shift_in_dir(cw_start, 1, s_dir.cw_init, m_map.table());
        ccw_start = shift_in_dir(ccw_start, 1, s_dir.ccw_init, m_map.table());
    }
    auto dir_info = DirectionInfo{};
    uint16_t cwbound = 0, ccwbound = 0;
    cwbound = horizontally_bound(roct) ? cur_coord.x: cur_coord.y;
    ccwbound = horizontally_bound(loct) ? cur_coord.x: cur_coord.y;
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
            if constexpr (horizontally_bound(loct)) {cw_bound = diag_bounds.x, ccw_bound = cur_coord.x;}
            else                                    {cw_bound = diag_bounds.y, ccw_bound = cur_coord.y;}
            
            scan_in_bound<CW, loct> (target_scan_start, cur, cw_bound, get_subseq_dir<CW, loct>());
            scan_in_bound<CCW, loct>(target_scan_start, cur, ccw_bound, get_subseq_dir<CCW, loct>());
        }
        else    
        {
            if constexpr (horizontally_bound(roct)) {cw_bound = cur_coord.x, ccw_bound = diag_bounds.x;}
            else                                    {cw_bound = cur_coord.y, ccw_bound = diag_bounds.y;}
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
void Solver<ST>::get_path(grid_id start, grid_id target)
{
    m_stats = experiment_result{};
    m_pq.clear();
    m_all_node_list.clear();
    m_target = target;
    m_tcoord = m_map.id_to_point(target);
    auto start_coord = m_map.id_to_point(start);
    if constexpr(ST == SolverTraits::OutputToPosthoc)
    {
        m_tracer->init(start_coord, m_tcoord);
    }
    auto cmp = [](search_node a, search_node b){return (a.gval + a.hval) > (b.gval + b.hval);};
    m_succ.reserve(2048);
    m_timer.start();
    auto start_state = rjps_state{};
    start_state.id  = start;
    start_state.dir = NORTHEAST_ID;  
    insert_start_state(start_state);
    start_state.dir = NORTHWEST_ID;
    insert_start_state(start_state);
    start_state.dir = SOUTHEAST_ID;
    insert_start_state(start_state);
    start_state.dir = SOUTHWEST_ID;
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
        auto cur_coord = m_map.id_to_point(cur.state.id);
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
inline bool Solver<ST>::target_in_scan_quad(grid_id start, direction_id quad)
{    
    auto s = m_map.id_to_point(start);
    switch (quad)
    {
    case NORTHEAST_ID:
        return(m_tcoord.x >= s.x && m_tcoord.y <= s.y);
        break;
    case NORTHWEST_ID:
        return(m_tcoord.x <= s.x && m_tcoord.y <= s.y);
        break;
    case SOUTHEAST_ID:
        return(m_tcoord.x >= s.x && m_tcoord.y >= s.y);
        break;
    case SOUTHWEST_ID:
        return(m_tcoord.x <= s.x && m_tcoord.y >= s.y);
        break;
    default:
        assert(false);
        break;
    }
}

template <SolverTraits ST>
bool Solver<ST>::init_scan_dir(grid_id start, direction_id p_dir, scan_dir &dir)
{
    assert(
    p_dir == NORTHEAST_ID || p_dir == NORTHWEST_ID || p_dir == SOUTHEAST_ID || p_dir == SOUTHWEST_ID 
    && "Init scan direction: in dir must be intercardinal direction");
    // auto adjx = adj[p_dir].x, adjy = adj[p_dir].y;
    const int dir_id = (int)p_dir - 4;
    bool vert, hori, ret = false;
    vert = m_map.map().get(grid_id(start.id + dir_id_adj_vert(p_dir, m_map.width())));
    hori = m_map.map().get(grid_id(start.id + dir_id_adj_hori(p_dir)));
    if(vert) //vert not blocked
    {
        if(hori) //convex point
        {
            dir.cw_init = d_init_scan_CW[dir_id][dir_id == NORTH_ID || dir_id == WEST_ID];
            dir.ccw_init = d_init_scan_CCW[dir_id][dir_id == SOUTH_ID || dir_id == EAST_ID];
            ret = true;
        }
        else//vert
        {
            dir.cw_init = d_init_scan_CW[dir_id][0];
            dir.ccw_init = d_init_scan_CCW[dir_id][0];
        }
    }
    else //vert blocked
    {
        if(hori)//hori
        {
            dir.cw_init = d_init_scan_CW[dir_id][1];
            dir.ccw_init = d_init_scan_CCW[dir_id][1];
        }
        else//concave point
        {
            dir.cw_init = d_init_scan_CW[dir_id][dir_id == SOUTH_ID || dir_id == EAST_ID];
            dir.ccw_init = d_init_scan_CCW[dir_id][dir_id == NORTH_ID || dir_id == WEST_ID];
        }
    }
    dir.cw_subseq = d_scan[dir_id][0];
    dir.cw_jps = d_jps[dir_id][0];
    dir.ccw_subseq = d_scan[dir_id][1];
    dir.ccw_jps = d_jps[dir_id][1];
    return ret;
}

template <SolverTraits ST>
grid_id Solver<ST>::grid_ray_incident(grid_id from, grid_id to, direction_id d)
{
    auto f_coord = m_map.id_to_point(from), t_coord = m_map.id_to_point(to);
    auto diff_coord = point_signed_diff(f_coord, t_coord);
    auto m = std::min(std::abs(diff_coord.first), std::abs(diff_coord.second));
    return shift_in_dir(from, m, d, m_map.table());
}

template <SolverTraits ST>
inline void Solver<ST>::generate(search_node* parent)
{
    auto top_adj = direction_id{}, bottom_adj = top_adj;
    for(rjps_state& succ : m_succ)
    {        
        switch (succ.dir)
        {
        case NORTH_ID:
            top_adj = SOUTHWEST_ID; bottom_adj = SOUTHEAST_ID;
            break;
        case SOUTH_ID:
            top_adj = NORTHWEST_ID; bottom_adj = NORTHEAST_ID;
            break;
        case EAST_ID:
            top_adj = NORTHWEST_ID; bottom_adj = SOUTHWEST_ID;
            break;
        case WEST_ID:
            top_adj = NORTHEAST_ID; bottom_adj = SOUTHEAST_ID;
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
        bool top = !m_map.map().get(shift_in_dir(succ.id, 1, top_adj, m_map.table()));
        bool bottom = !m_map.map().get(shift_in_dir(succ.id, 1, bottom_adj, m_map.table()));
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
        m_tracer->expand(m_map.id_to_point(n.state.id), "fuchsia", 
            "generating, h: " + std::to_string(n.hval) +
            " ,g: " + std::to_string(n.gval) + 
            " ,f: "+ std::to_string(n.gval + n.hval) + 
            " ,dir:" + std::to_string(n.state.dir));
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
                m_tracer->expand(m_map.id_to_point(n.state.id), "red", 
                    "re-opening, h: " + std::to_string(n.hval) +
                    " ,g: " + std::to_string(n.gval) + 
                    " ,f: "+ std::to_string(n.gval + n.hval) + 
                    " ,dir:" + std::to_string(n.state.dir));
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
                m_tracer->expand(m_map.id_to_point(n.state.id), "yellow", 
                    "updating, h: " + std::to_string(n.hval) +
                    " ,g: " + std::to_string(n.gval) + 
                    " ,f: "+ std::to_string(n.gval + n.hval) + 
                    " ,dir:" + std::to_string(n.state.dir));
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
    // auto key = std::string(std::to_string((uint64_t)n.state.id) + std::to_string(n.state.dir));

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
//@param dir_info: collection of direction info which includes initial scan direction, subseq scan direction_id and terminating direction
template <SolverTraits ST>
template <ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
uint32_t Solver<ST>::scan_in_bound(grid_id start, search_node parent, uint32_t boundary, direction_id start_d)
{
    using ScanAttribute::Octants;
    using namespace ScanAttribute;

    auto scan_frontier = uint32_t{};
    auto start_coord = m_map.id_to_point(start);
    //returning scan_frontier only matters when scanning from diag boundary
    if constexpr(horizontally_bound(Octant)) {scan_frontier = start_coord.x;}
    else                                     {scan_frontier = start_coord.y;}

    DirectionInfo dir_info{};
    dir_info.init = start_d;
    dir_info.subseq = get_subseq_dir<O, Octant>();
    dir_info.jps    = get_jps_dir<Octant>();
    dir_info.terminate = get_terminate_dir<O, Octant>();
    auto scan_res = scanResult{};
    auto poi = grid_id{}, succ = poi;
    scan_res.d = start_d;
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
            m_tracer->trace_ray(m_map.id_to_point(parent.state.id), m_map.id_to_point(ince), "aqua", "shoot ray to point");
            m_tracer->trace_ray(m_map.id_to_point(ince), m_map.id_to_point(succ), "aqua", "shoot ray to point");
        }
        //if poi is blocked by another obstacle, recurse scan in both orientation on collision point
        //depending on the orientation and the quadrant, x or y bound is changed
        if(succ != poi)
        {
            auto start_coord = m_map.id_to_point(start);
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
            auto s_coord = m_map.id_to_point(poi);
            if constexpr(horizontally_bound(Octant)) scan_frontier = s_coord.x;
            else                                     scan_frontier = s_coord.y;
            //the scan result indicates the first poi was a concave point, which returns the next convex point
            if(scan_res.on_concave) [[unlikely]]
            {
                //shift 1 step more to the actual corner point
                poi = shift_in_dir(poi, 1, scan_res.d, m_map.table());                
            }
            //poi was a convex point, shoot a jps ray to and pass it
            else
            {
                //since poin is guranteed to be a jump point(turning point), shoot a jps ray towards it to push it onto the heap
                //along with all jump points along the way. TODO: extra jump points might be unnecessary?
                poi = m_ray.shoot_rjps_ray<ST>(poi, dir_info.jps, m_succ);             
                scan_res.d = dir_info.subseq;
                scan_res.top = (dir_info.jps == NORTH_ID || dir_info.jps == WEST_ID);
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
    // auto hori_dir = direction_id{}, vert_dir = direction_id{};
    // auto x_intv = grid_id{}, y_intv = x_intv;
    // vert_dir = (v.dir == NORTHEAST_ID || v.dir == NORTHWEST_ID) ? NORTH_ID : SOUTH_ID;
    // hori_dir = (v.dir == NORTHEAST_ID || v.dir == SOUTHEAST_ID) ? EAST_ID : WEST_ID;

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
    //     m_tracer->draw_cell(m_map.id_to_point(x_intv), "orange", "intx, h: " + to_string(hx));
    //     m_tracer->draw_cell(m_map.id_to_point(y_intv), "orange", "inty, h: " + to_string(hy));
    // }
    // return std::min(hx, hy);
}

