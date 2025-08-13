#pragma once
#include <rjps.h>
#include <scanner.h>
#include <Ray.h>
#include <Log.h>
#include <warthog/heuristic/octile_heuristic.h>
#include <warthog/util/timer.h>
#include <queue>
#include <unordered_map>

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
    double  plenth{};
    int     heap_pops{};
    int     generated{};
    int     expanded{};
};

template<SolverTraits ST>
class Solver
{
public:
    Solver(jump::jump_point_online<>* _jps) : 
    m_jps(_jps), m_tracer(new Tracer), m_map(m_jps->get_map()), m_rmap(m_jps->get_rmap()), 
    m_ray(m_tracer, m_jps), m_scanner(m_tracer, m_jps), m_heuristic(m_map.width(), m_map.height()),
    m_timer()
    {
        static_assert(ST == SolverTraits::Default || ST == SolverTraits::OutputToPosthoc);
        m_tracer->set_dim(m_map.dim());
        m_node_map.reserve(2048);
    }
    ~Solver() = default;
    
    void expand_node(rjps_node n, std::vector<rjps_node> &heap);

    template <direction D>
    void expand(rjps_node cur, std::vector<rjps_node> &heap);

    // void expand(rjps_node cur, std::vector<rjps_node> &heap);

    void query(pad_id start, pad_id target);
    experiment_result get_result(){return m_stats;};

    template <ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
    uint32_t scan_in_bound(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t boundary, direction start_d);

    template <ScanAttribute::Orientation O>
    uint32_t scan_in_bound(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t xbound, uint32_t ybound, DirectionInfo dir_info);
    template <ScanAttribute::Orientation O>
    uint32_t scan_in_bound_alt(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t xbound, uint32_t ybound, DirectionInfo dir_info);
    
    //initilizes a scan_dir struct based on a cell on a grid and an rjps scan direction
    //returns true if starts on a convex point
    bool init_scan_dir(pad_id start, direction p_dir, scan_dir &dir);
    pad_id grid_ray_incident(pad_id from, pad_id to, direction d);
private:

    jump::jump_point_online<>*          m_jps;
    std::shared_ptr<Tracer>             m_tracer;
    warthog::domain::gridmap::bittable  m_map;
    warthog::domain::gridmap::bittable  m_rmap;
    Ray                                 m_ray;
    Scanner                             m_scanner;
    pad_id                              m_target;
    heuristic::octile_heuristic         m_heuristic;
    std::unordered_map<uint64_t, rjps_node> m_node_map;
    experiment_result                   m_stats;
    warthog::util::timer                m_timer;
    template <direction D>
    bool target_in_scan_quad(pad_id start, pad_id target);
    // void scan_target_blocker(rjps_node cur, std::vector<rjps_node> &vec, Octants O);
    void init_rjps_nodes(vector<rjps_node> &heap, rjps_node parent, size_t prev_end);

    double interval_h(rjps_node cur);

    pad_id shoot_interval_ray(pad_id start, direction dir);
};

template <SolverTraits ST>
inline void Solver<ST>::expand_node(rjps_node n, std::vector<rjps_node> &heap)
{
    switch (n.dir)
    {
    case NORTHWEST:
        expand<NORTHWEST>(n, heap);
        break;
    case NORTHEAST:
        expand<NORTHEAST>(n, heap);        
        break;
    case SOUTHWEST:
        expand<SOUTHWEST>(n, heap);        
        break;
    case SOUTHEAST:
        expand<SOUTHEAST>(n, heap);        
        break;
    default:
        assert(false && "invalide node expansion");
        break;
    }
}

template <SolverTraits ST>
template <direction D>
void Solver<ST>::expand(rjps_node cur, std::vector<rjps_node> &heap)
{
    using namespace ScanAttribute;
    static_assert(
	    D == NORTHEAST || D == NORTHWEST || D == SOUTHEAST || D == SOUTHWEST,
	    "D must be inter-cardinal.");
    auto cur_coord = m_map.id_to_xy(cur.id);
    auto target_coord = m_map.id_to_xy(m_target);
    //coord postion of ray intersec from shooting to target, potentially unused
    auto temp = pad_id{}, target_scan_start = temp;
    auto s_dir = scan_dir{};
    bool target_blocked = false;
    //left octant and right octant
    constexpr Octants loct = left_octant<D>(), roct = right_octant<D>();
    //if target is in same quadrant, shoot to it
    if(target_in_scan_quad<D>(cur.id, m_target))
    {
        std::pair<bool, pad_id> vis_res;
        if(on_left_octant<D>(cur_coord, target_coord))
        {
            vis_res = m_ray.check_target_visible<ST, loct>(cur.id, m_target, cur.dir);
        }
        else
        {
            vis_res = m_ray.check_target_visible<ST, roct>(cur.id, m_target, cur.dir);
        }
        //if target is visible, return function
        if(vis_res.second == m_target)
        {
            auto t = rjps_node{m_target, &m_node_map.find(uint64_t(cur.id))->second, m_map.id_to_xy(m_target), NONE};
            t.hval = 0;
            t.gval = m_heuristic.h(target_coord.first, target_coord.second, cur_coord.first, cur_coord.second) + cur.gval;
            heap.emplace_back(t);
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
    temp = m_ray.shoot_diag_ray_id<ST>(cur.id, cur.dir);
    auto diag_bounds = m_map.id_to_xy(temp);
    auto on_convex = init_scan_dir(temp, cur.dir, s_dir);
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
    cwbound = scan_in_bound<CW, roct>(cw_start, cur, heap, cwbound, s_dir.cw_init);
    // dir_info.jps = s_dir.cw_jps;
    // dir_info.subseq = s_dir.cw_subseq;
    // dir_info.terminate = rotate_eighth<Orientation::CCW>(cur.dir);
    // uint32_t cwbound = scan_in_bound<Orientation::CW>(cw_start, cur, heap, cur_coord.first, cur_coord.second, dir_info);

    //CCW scan
    dir_info.init = s_dir.ccw_init;
    ccwbound = scan_in_bound<CCW, loct>(ccw_start, cur, heap, ccwbound, s_dir.ccw_init);
    // dir_info.jps = s_dir.ccw_jps;
    // dir_info.subseq = s_dir.ccw_subseq;
    // dir_info.terminate = rotate_eighth<Orientation::CW>(cur.dir);
    // uint32_t ccwbound = scan_in_bound<Orientation::CCW>(ccw_start, cur, heap, cur_coord.first, cur_coord.second, dir_info);

    if(target_blocked)[[unlikely]]
    {
        uint32_t cw_bound = 0, ccw_bound = 0;
        if(on_left_octant<D>(cur_coord, target_coord))
        {
            //left octant always uses diag bounds as scanning bound for CW, and horizontal bounds for CCW
            if constexpr (horizontally_bound(loct)) {cw_bound = diag_bounds.first, ccw_bound = cur_coord.first;}
            else                                    {cw_bound = diag_bounds.second, ccw_bound = cur_coord.second;}
            
            scan_in_bound<CW, loct> (target_scan_start, cur, heap, cw_bound, get_subseq_dir<CW, loct>());
            scan_in_bound<CCW, loct>(target_scan_start, cur, heap, ccw_bound, get_subseq_dir<CCW, loct>());
        }
        else    
        {
            if constexpr (horizontally_bound(roct)) {cw_bound = cur_coord.first, ccw_bound = diag_bounds.first;}
            else                                    {cw_bound = cur_coord.second, ccw_bound = diag_bounds.second;}
            scan_in_bound<CW, roct> (target_scan_start, cur, heap, cw_bound, get_subseq_dir<CW, roct>());
            scan_in_bound<CCW, roct>(target_scan_start, cur, heap, ccw_bound, get_subseq_dir<CCW, roct>());
        }
    }

    //CW scan from left extremety(left of scan center)
    dir_info.init = s_dir.cw_jps;
    temp = m_ray.shoot_rjps_ray<ST>(cur.id, s_dir.ccw_jps, heap, cur);
    scan_in_bound<CW, loct>(temp, cur, heap, ccwbound, s_dir.cw_jps);
    
    //CCW scan from right extremety(right of scan center)
    dir_info.init = s_dir.ccw_jps;
    temp = m_ray.shoot_rjps_ray<ST>(cur.id, s_dir.cw_jps, heap, cur);
    scan_in_bound<CCW, roct>(temp, cur, heap, cwbound, s_dir.ccw_jps);
}

template <SolverTraits ST>
void Solver<ST>::query(pad_id start, pad_id target)
{
    m_stats = experiment_result{};
    m_node_map.clear();
    m_target = target;
    auto start_coord = m_map.id_to_xy(start), target_coord = m_map.id_to_xy(target);
    if constexpr(ST == SolverTraits::OutputToPosthoc)
    {
        m_tracer->init(start_coord, target_coord);
    }
    auto cmp = [](rjps_node a, rjps_node b){return (a.gval + a.hval) > (b.gval + b.hval);};
    std::vector<rjps_node> heap{};
    heap.reserve(2048);
    
    m_timer.start();

    auto start_node = rjps_node{start, nullptr, m_map.id_to_xy(start), NONE};
    start_node.gval = 0;
    start_node.hval = m_heuristic.h(start_coord.first, start_coord.second, target_coord.first, target_coord.second);
    {
    start_node.dir = NORTHEAST;
    heap.push_back(start_node);

    start_node.dir = NORTHWEST;
    heap.push_back(start_node);

    start_node.dir = SOUTHEAST;
    heap.push_back(start_node);

    start_node.dir = SOUTHWEST;
    heap.push_back(start_node);
    // start_node.quad_mask = (direction)UINT8_MAX;    // == 11111111
    m_node_map.try_emplace((uint64_t)start_node.id, start_node);
    }
    int iter = 0;
    while(!heap.empty())
    {
        // if(iter++; iter > 2500)
        // {
        //     std::cout<<"limit exceed\n";
        //     m_stats.plenth = DBL_MAX; return;
        // }
        //pop the node with lowest fval off the heap
        auto cur = heap.front();
        auto cur_coord = m_map.id_to_xy(cur.id);
        if(cur.id == m_target)
        {
            m_stats.nanos = m_timer.elapsed_time_nano();
            break;
        }
        std::pop_heap(heap.begin(), heap.end(), cmp); m_stats.heap_pops++;
        heap.pop_back();
        auto &node_in_closed = m_node_map.find((uint64_t)cur.id)->second;
        if(rjps_node::quad_closed(node_in_closed, cur.dir))
        {
            continue;
        }
        //record heap size, if heap size changed during expansion, heap invalidated
        auto tmp_size = heap.size();
        if constexpr(ST == SolverTraits::OutputToPosthoc)
        {
            m_tracer->expand(cur_coord, "orange", "expanding, g: " + to_string(cur.gval) + " ,f: "+ to_string(cur.gval + cur.hval));
            m_tracer->draw_bounds(cur_coord, cur.dir);
        }
        expand_node(cur, heap); m_stats.expanded++;
        if(heap.size() > tmp_size)
        {
            //newly generated nodes are appended to the heap, they're not yet initialized
            init_rjps_nodes(heap, cur, tmp_size);
            m_stats.generated += heap.size() - tmp_size;
            if constexpr(ST == SolverTraits::OutputToPosthoc)
            {
                for(auto i = tmp_size; i < heap.size(); i++)
                {
                    m_tracer->expand(m_map.id_to_xy(heap[i].id), "fuchsia", "generating, g: " + to_string(heap[i].gval) + " ,f: "+ to_string(heap[i].gval + heap[i].hval));
                }
            }
            //reconsolidate heap
            std::make_heap(heap.begin(), heap.end(), cmp);
        }
        rjps_node::close_quad(node_in_closed, cur.dir);
        if constexpr(ST == SolverTraits::OutputToPosthoc)
        {
            m_tracer->close_node(cur_coord);
        }
    }
    auto c = heap.front();
    m_stats.plenth = c.gval;
    auto stk = std::stack<rjps_node>{};
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
        // m_ray.shoot_to_target(cur.parent->id, cur.id, cur.parent->dir);
        stk.pop();
    }
    if constexpr(ST == SolverTraits::OutputToPosthoc)
    {
        m_tracer->close();
    }
    return;
}

template <SolverTraits ST>
template <direction D>
inline bool Solver<ST>::target_in_scan_quad(pad_id start, pad_id target)
{
    auto s = m_map.id_to_xy(start), t = m_map.id_to_xy(target);
    if      constexpr(D == NORTHEAST)
    {
        return(t.first >= s.first && t.second <= s.second);
    }
    else if constexpr(D == NORTHWEST)
    {
        return(t.first <= s.first && t.second <= s.second);
    }    
    else if constexpr(D == SOUTHEAST)
    {
        return(t.first >= s.first && t.second >= s.second);
    }
    else if constexpr(D == SOUTHWEST)
    {
        return(t.first <= s.first && t.second >= s.second);
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
void Solver<ST>::init_rjps_nodes(vector<rjps_node> &heap, rjps_node parent, size_t prev_end)
{
    const auto &parent_node = m_node_map.find((uint64_t)parent.id);
    
    assert(parent_node != m_node_map.end());
    auto top_adj = direction{}, bottom_adj = top_adj;
    auto t_coord = m_map.id_to_xy(m_target);
    for(auto i = prev_end, j = heap.size(); i<j; i++)
    {
        auto &node = heap[i];
        auto cur_coord = m_map.id_to_xy(node.id);
        auto p_coord = m_map.id_to_xy(parent.id);
        //gval should be the shortest path from parent, since path is taut
        node.gval = m_heuristic.h(cur_coord.first, cur_coord.second, p_coord.first, p_coord.second) + parent_node->second.gval;
        node.hval = m_heuristic.h(cur_coord.first, cur_coord.second, t_coord.first, t_coord.second);
        switch (node.dir)
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
            // assert(false);
            if(node.id == m_target) return;
        }
        bool top = m_map.get(shift_in_dir(node.id, 1, top_adj, m_map));
        bool bottom = m_map.get(shift_in_dir(node.id, 1, bottom_adj, m_map));
        top =!top; 
        bottom=!bottom;
        if(top && bottom) [[unlikely]]
        {
            auto tmp_dir = node.dir;
            const auto &q = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(true));
            assert(q != quad.end());
            node.dir = q->second;
            heap.push_back(node);

            const auto &q2 = quad.find(to_string(parent.dir) + to_string(tmp_dir) + to_string(false));
            assert(q2 != quad.end());
            node.dir = q2->second;
            heap.push_back(node);
        }
        else if(top)
        {
            const auto &q = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(true));
            assert(q != quad.end());
            node.dir = q->second;
        }
        else
        {
            const auto &q = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(false));
            assert(q != quad.end());
            node.dir = q->second;
        }
    }
    for(auto iter = prev_end; iter < heap.size();)
    {
        auto& cur = heap[iter];
        //if node is not visited, push a copy into map(closed list) then continue
        if(m_node_map.find(uint64_t(cur.id)) == m_node_map.end())
        {
            cur.parent = &parent_node->second;
            // cur.close_quad(cur.dir);
            m_node_map.emplace((uint64_t)cur.id, cur);
            ++iter;
        }
        else
        {            
            auto &node_in_map = m_node_map.find(uint64_t(cur.id))->second;
            //
            if(cur.gval <= node_in_map.gval)
            {
                node_in_map.gval = cur.gval; 
                node_in_map.parent = &parent_node->second;
                if(rjps_node::quad_closed(node_in_map, cur.dir))
                {
                    std::swap(cur, heap.back());
                    heap.pop_back();
                    continue;
                }
                else
                {
                    // node_in_map.close_quad(cur.dir);
                    ++iter;
                }
            }
            else
            {
                std::swap(cur, heap.back());
                heap.pop_back();
                continue;
            }
        }
    }
}

//Scans all visible obstacles in a set orientation, appends succesors to the vector passed in
//@param start: position to start the scan
//@param parent: rjps search node the scan is based on
//@param &vec: reference to the return vector of rjps nodes
//@param xbound: x boundary
//@param ybound: y boundary
//@param dir_info: collection of direction info which includes initial scan direction, subseq scan direction and terminating direction
template <SolverTraits ST>
template <ScanAttribute::Orientation O>
uint32_t Solver<ST>::scan_in_bound(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t xbound, uint32_t ybound, DirectionInfo dir_info)
{
    auto ret = uint32_t{};
    auto start_coord = m_map.id_to_xy(start);
    using namespace ScanAttribute;
    if constexpr (O == ScanAttribute::CW)
    {
        if(parent.dir == SOUTHEAST || parent.dir == NORTHWEST)
        {
            ret = start_coord.first;    //x
        }
        else    
        {
            ret = start_coord.second;   //y
        }
    }
    else
    {
        if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)
        {
            ret = start_coord.first;    //x
        }
        else    
        {
            ret = start_coord.second;   //y
        }
    }
    auto scan_res = scanResult{};
    auto poi = pad_id{}, succ = poi;
    scan_res.d = dir_info.init;
    uint32_t dir_ind = std::countr_zero<uint8_t>(parent.dir)-4;
    scan_res.top = init_scan_top[dir_ind][(scan_res.d == EAST || scan_res.d == WEST)];
    poi = m_scanner.find_turning_point<ST>(start, scan_res, dir_info.terminate, xbound, ybound);
    while (!poi.is_none())  // poi will be none if scan leaves bound, if not recurse scan
    { 
        auto ince = grid_ray_incident(parent.id, poi, parent.dir);
        succ = m_ray.shoot_rjps_ray_to_target<ST>(ince, poi, dir_info.jps, vec, parent);
        if constexpr(ST == SolverTraits::OutputToPosthoc)
        {
            m_tracer->trace_ray(m_map.id_to_xy(parent.id), m_map.id_to_xy(ince), "aqua", "shoot ray to point");
            m_tracer->trace_ray(m_map.id_to_xy(ince), m_map.id_to_xy(succ), "aqua", "shoot ray to point");
        }
        //if poi is blocked by another obstacle, recurse scan in both orientation on collision point
        //depending on the orientation and the quadrant, x or y bound is changed
        if(succ != poi)
        {
            auto start_coord = m_map.id_to_xy(start);
            if constexpr (O == ScanAttribute::CW)
            {
                auto cw_dir_info = DirectionInfo{dir_info.subseq, dir_info.subseq, dir_info.jps, dir_ccw(dir_info.jps)};
                auto ccw_dir_info = DirectionInfo{dir_flip(dir_info.subseq), dir_flip(dir_info.subseq), dir_info.jps, dir_cw(dir_info.jps)};
                if(parent.dir == SOUTHEAST || parent.dir == NORTHWEST)  //x is the primary boundary
                {
                    ret = scan_in_bound<CW>(succ, parent, vec, xbound, ybound, cw_dir_info);
                    scan_in_bound<CCW>(succ, parent, vec, start_coord.first, ybound, ccw_dir_info);
                }
                else                                                    //y is the primary boundary
                {
                    ret = scan_in_bound<CW>(succ, parent, vec, xbound, ybound, cw_dir_info);
                    scan_in_bound<CCW>(succ, parent, vec, xbound, start_coord.second, ccw_dir_info);
                }
            }
            else
            {                
                auto cw_dir_info = DirectionInfo{dir_flip(dir_info.subseq), dir_flip(dir_info.subseq), dir_info.jps, dir_ccw(dir_info.jps)};
                auto ccw_dir_info = DirectionInfo{dir_info.subseq, dir_info.subseq, dir_info.jps, dir_cw(dir_info.jps)};
                if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)  //x is the primary boundary
                {
                    scan_in_bound<CW>(succ, parent, vec, start_coord.first, ybound, cw_dir_info);
                    ret = scan_in_bound<CCW>(succ, parent, vec, xbound, ybound, ccw_dir_info);
                }
                else                                                    //y is the primary boundary    
                {
                    scan_in_bound<CW>(succ, parent, vec, xbound, start_coord.second, cw_dir_info);
                    ret = scan_in_bound<CCW>(succ, parent, vec, xbound, ybound, ccw_dir_info);
                }
            }
            return ret;
        }
        else    //poi is visible, shoot a jps ray towards it then continue scanning in orientation O
        {
            auto s_coord = m_map.id_to_xy(succ);
            if constexpr (O == ScanAttribute::CW)
            {
                if(parent.dir == SOUTHEAST || parent.dir == NORTHWEST)
                {
                    ret = s_coord.first;
                }
                else    
                {
                    ret = s_coord.second;
                }
            }
            else
            {
                if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)
                {
                    ret = s_coord.first;
                }
                else    
                {
                    ret = s_coord.second;
                }
            }
            //if the turning point was a concave point, continue scan from the next first convex point returned
            if(scan_res.on_concave) 
            {
                succ = shift_in_dir(succ, 1, scan_res.d, m_map);                
            }
            //else continue the jps ray then scan in same direction
            else
            {
                succ = m_ray.shoot_rjps_ray<ST>(succ, dir_info.jps, vec, parent);             
                scan_res.d = dir_info.subseq;
                scan_res.top = (dir_info.jps == NORTH || dir_info.jps == WEST);
            }
            poi = m_scanner.find_turning_point<ST>(succ, scan_res, dir_info.terminate, xbound, ybound);
        }
    }
    return ret;
}

template <SolverTraits ST>
template <ScanAttribute::Orientation O>
uint32_t Solver<ST>::scan_in_bound_alt(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t xbound, uint32_t ybound, DirectionInfo dir_info)
{
    auto ret = uint32_t{};
    using namespace ScanAttribute;
    if constexpr (O == ScanAttribute::CW)
    {
        if(parent.dir == SOUTHEAST || parent.dir == NORTHWEST)
        {
            ret = xbound;
        }
        else    
        {
            ret = ybound;
        }
    }
    else
    {
        if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)
        {
            ret = xbound;
        }
        else    
        {
            ret = ybound;
        }
    }
    auto scan_res = scanResult{};
    auto poi = pad_id{}, succ = poi;
    scan_res.d = dir_info.init;
    uint32_t dir_ind = std::countr_zero<uint8_t>(parent.dir)-4;
    scan_res.top = init_scan_top[dir_ind][(scan_res.d == EAST || scan_res.d == WEST)];
    poi = m_scanner.find_turning_point<ST>(start, scan_res, dir_info.terminate, xbound, ybound);
    while (!poi.is_none())  // poi will be none if scan leaves bound, if not recurse scan
    { 
        auto ince = grid_ray_incident(parent.id, poi, parent.dir);
        succ = m_ray.shoot_rjps_ray_to_target<ST>(ince, poi, dir_info.jps, vec, parent);
        if constexpr(ST == SolverTraits::OutputToPosthoc)
        {
            m_tracer->trace_ray(m_map.id_to_xy(parent.id), m_map.id_to_xy(ince), "aqua", "shoot ray to point");
            m_tracer->trace_ray(m_map.id_to_xy(ince), m_map.id_to_xy(succ), "aqua", "shoot ray to point");
        }
        //if poi is blocked by another obstacle, recurse scan in both orientation on collision point
        //depending on the orientation and the quadrant, x or y bound is changed
        if(succ != poi)
        {
            auto start_coord = m_map.id_to_xy(start);
            if constexpr (O == ScanAttribute::CW)
            {
                auto cw_dir_info = DirectionInfo{dir_info.subseq, dir_info.subseq, dir_info.jps, dir_ccw(dir_info.jps)};
                auto ccw_dir_info = DirectionInfo{dir_flip(dir_info.subseq), dir_flip(dir_info.subseq), dir_info.jps, dir_cw(dir_info.jps)};
                if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)  //x is the primary boundary
                {
                    ret = scan_in_bound<CW>(succ, parent, vec, xbound, ybound, cw_dir_info);
                    scan_in_bound<CCW>(succ, parent, vec, start_coord.first, ybound, ccw_dir_info);
                }
                else                                                    //y is the primary boundary
                {
                    ret = scan_in_bound<CW>(succ, parent, vec, xbound, ybound, cw_dir_info);
                    scan_in_bound<CCW>(succ, parent, vec, xbound, start_coord.second, ccw_dir_info);
                }
            }
            else
            {                
                auto cw_dir_info = DirectionInfo{dir_flip(dir_info.subseq), dir_flip(dir_info.subseq), dir_info.jps, dir_ccw(dir_info.jps)};
                auto ccw_dir_info = DirectionInfo{dir_info.subseq, dir_info.subseq, dir_info.jps, dir_cw(dir_info.jps)};
                if(parent.dir == SOUTHEAST || parent.dir == NORTHWEST)  //x is the primary boundary
                {
                    scan_in_bound<CW>(succ, parent, vec, start_coord.first, ybound, cw_dir_info);
                    ret = scan_in_bound<CCW>(succ, parent, vec, xbound, ybound, ccw_dir_info);
                }
                else                                                    //y is the primary boundary    
                {
                    scan_in_bound<CW>(succ, parent, vec, xbound, start_coord.second, cw_dir_info);
                    ret = scan_in_bound<CCW>(succ, parent, vec, xbound, ybound, ccw_dir_info);
                }
            }
            return ret;
        }
        else    //poi is visible, shoot a jps ray towards it then continue scanning in orientation O
        {
            auto s_coord = m_map.id_to_xy(succ);
            if constexpr (O == ScanAttribute::CW)
            {
                if(parent.dir == SOUTHEAST || parent.dir == NORTHWEST)
                {
                    ret = s_coord.first;
                }
                else    
                {
                    ret = s_coord.second;
                }
            }
            else
            {
                if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)
                {
                    ret = s_coord.first;
                }
                else    
                {
                    ret = s_coord.second;
                }
            }
            //if the turning point was a concave point, continue scan from the next first convex point returned
            if(scan_res.on_concave) 
            {
                succ = shift_in_dir(succ, 1, scan_res.d, m_map);                
            }
            //else continue the jps ray then scan in same direction
            else
            {
                succ = m_ray.shoot_rjps_ray<ST>(succ, dir_info.jps, vec, parent);             
                scan_res.d = dir_info.subseq;
                scan_res.top = (dir_info.jps == NORTH || dir_info.jps == WEST);
            }
            poi = m_scanner.find_turning_point<ST>(succ, scan_res, dir_info.terminate, xbound, ybound);
        }
    }
    return ret;
}


template <SolverTraits ST>
template <ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
uint32_t Solver<ST>::scan_in_bound(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t boundary, direction start_d)
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
    uint32_t dir_ind = std::countr_zero<uint8_t>(parent.dir)-4;
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
        auto ince = grid_ray_incident(parent.id, poi, parent.dir);
        succ = m_ray.shoot_rjps_ray_to_target<ST>(ince, poi, dir_info.jps, vec, parent);
        if constexpr(ST == SolverTraits::OutputToPosthoc)
        {
            m_tracer->trace_ray(m_map.id_to_xy(parent.id), m_map.id_to_xy(ince), "aqua", "shoot ray to point");
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
            cw_frontier =  scan_in_bound<CW, Octant>(succ, parent, vec, cw_bound, get_subseq_dir<CW, Octant>());
            ccw_frontier = scan_in_bound<CCW, Octant>(succ, parent, vec, ccw_bound, get_subseq_dir<CCW, Octant>());
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
                poi = m_ray.shoot_rjps_ray<ST>(poi, dir_info.jps, vec, parent);             
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
inline double Solver<ST>::interval_h(rjps_node cur)
{
    double hx = 0, hy = 0;
    auto hori_dir = direction{}, vert_dir = direction{};
    auto x_intv = pad_id{}, y_intv = x_intv;
    vert_dir = (cur.dir == NORTHEAST || cur.dir == NORTHWEST) ? NORTH : SOUTH;
    hori_dir = (cur.dir == NORTHEAST || cur.dir == SOUTHEAST) ? EAST : WEST;
    if (vert_dir == NORTH)
    {
        auto dy = m_ray.shoot_ray_north<Travasable>(cur.id);
        y_intv = shift_in_dir(cur.id, dy+1, NORTH, m_map);
        dy = m_ray.shoot_ray_north<Obstacle>(cur.id);
        y_intv = shift_in_dir(cur.id, dy+1, NORTH, m_map);
    }
    else    
    {
        auto dy = m_ray.shoot_ray_south<Travasable>(cur.id);
        y_intv = shift_in_dir(cur.id, dy+1, SOUTH, m_map);
        dy = m_ray.shoot_ray_south<Obstacle>(cur.id);
        y_intv = shift_in_dir(cur.id, dy+1, SOUTH, m_map);
    }
    if (hori_dir == EAST)
    {
        auto dx = m_ray.shoot_ray_east<Travasable>(cur.id);
        x_intv = shift_in_dir(cur.id, dx+1, EAST, m_map);
        dx = m_ray.shoot_ray_east<Obstacle>(cur.id);
        x_intv = shift_in_dir(cur.id, dx+1, EAST, m_map);
    }
    else    
    {
        auto dx = m_ray.shoot_ray_west<Travasable>(cur.id);
        x_intv = shift_in_dir(cur.id, dx+1, WEST, m_map);
        dx = m_ray.shoot_ray_west<Obstacle>(cur.id);
        x_intv = shift_in_dir(cur.id, dx+1, WEST, m_map);
    }
    hx = m_heuristic.h((sn_id_t)x_intv, (sn_id_t)m_target);
    hy = m_heuristic.h((sn_id_t)y_intv, (sn_id_t)m_target);
    return std::min(hx, hy);
}

