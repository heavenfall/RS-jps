#include <solver.h>

Solver::Solver(jump::jump_point_online<>* _jps) : 
    m_jps(_jps), m_tracer(new Tracer), m_map(m_jps->get_map()), m_rmap(m_jps->get_rmap()), 
    m_ray(m_tracer, m_jps), m_scanner(m_tracer, m_jps), m_heuristic(m_map.width(), m_map.height())
{
    m_node_map.reserve(2048);
}

inline direction Solver::target_dir(pad_id start, pad_id target)
{
    auto c1 = m_map.id_to_xy(start), c2 = m_map.id_to_xy(target);
    uint32_t x = c1.first, y = c1.second, tx = c2.first, ty = c2.second;
    if(tx<x)
    {
        if(ty<y) return NORTHWEST;
        else     return SOUTHWEST;
    }
    else
    {
        if(ty<y) return NORTHEAST;
        else     return SOUTHEAST;
    }
}

bool Solver::init_scan_dir(pad_id start, direction p_dir, scan_dir &dir)
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

void Solver::expand(rjps_node cur, std::vector<rjps_node> &heap)
{
    auto cur_coord = m_map.id_to_xy(cur.id);
    auto temp = pad_id{};
    // auto vec = std::vector<rjps_node>{};
    auto s_dir = scan_dir{};
    auto temp_end = heap.end();
    //if target is in same quadrant, shoot to it
    if(target_dir(cur.id, m_target) == cur.dir)
    {
        temp = m_ray.shoot_to_target(cur.id, m_target);
        if(temp == m_target)
        {
            auto t = rjps_node{m_target, &m_node_map.find(uint64_t(cur.id))->second, m_map.id_to_xy(m_target), NONE};
            auto target_coord = m_map.id_to_xy(m_target);
            t.hval = 0;
            t.gval = m_heuristic.h(target_coord.first, target_coord.second, cur_coord.first, cur_coord.second);
            heap.emplace_back(t);
            return;
        }
    }
    temp = m_ray.shoot_diag_ray_id(cur.id, m_map, cur.dir);
    auto on_convex = init_scan_dir(temp, cur.dir, s_dir);
    auto cw_start = temp, ccw_start = temp;
    if(on_convex)
    {
        cw_start = shift_in_dir(cw_start, 1, s_dir.cw_init, m_map);
        ccw_start = shift_in_dir(ccw_start, 1, s_dir.ccw_init, m_map);
    }
    auto dir_info = DirectionInfo{};
    //scan both ways from the point the ray intercepted
    //CW scan
    dir_info.init = s_dir.cw_init;
    dir_info.jps = s_dir.cw_jps;
    dir_info.subseq = s_dir.cw_subseq;
    dir_info.terminate = rotate_eighth<ScanAttribute::CCW>(cur.dir);
    uint32_t cwbound = scan_in_bound<ScanAttribute::CW>(cw_start, cur, heap, cur_coord.first, cur_coord.second, dir_info);

    //CCW scan
    dir_info.init = s_dir.ccw_init;
    dir_info.jps = s_dir.ccw_jps;
    dir_info.subseq = s_dir.ccw_subseq;
    dir_info.terminate = rotate_eighth<ScanAttribute::CW>(cur.dir);
    uint32_t ccwbound = scan_in_bound<ScanAttribute::CCW>(ccw_start, cur, heap, cur_coord.first, cur_coord.second, dir_info);

    //CW scan from left extremety(left of scan center)
    auto xbound = uint32_t{}, ybound = xbound;
    if(cur.dir == NORTHEAST || cur.dir == SOUTHWEST)
    {
        xbound = ccwbound;
        ybound = cur_coord.second;
    }
    else    
    {
        xbound = cur_coord.first;
        ybound = ccwbound;
    }
    dir_info.init = s_dir.cw_jps;
    dir_info.jps =  rotate_eighth<ScanAttribute::CCW>(cur.dir);
    dir_info.subseq = s_dir.cw_jps;
    dir_info.terminate = dir_ccw(dir_info.jps);
    temp = m_ray.shoot_rjps_ray(cur.id, s_dir.ccw_jps, heap, cur);
    //IF FIRST CCW SCAN FAILS, CCWBOUND WILL STAY THE SAME AS ORIGINAL BOUND THEREFORE SHORTCIRCUIT
    scan_in_bound<ScanAttribute::CW>(temp, cur, heap, xbound, ybound, dir_info);
    
    //CCW scan from right extremety(right of scan center)
    if(cur.dir == NORTHWEST || cur.dir == SOUTHEAST)
    {
        xbound = cwbound;
        ybound = cur_coord.second;
    }
    else    
    {
        xbound = cur_coord.first;
        ybound = cwbound;
    }
    //CCW scan from right extremety
    dir_info.init = s_dir.ccw_jps;
    dir_info.jps =  rotate_eighth<ScanAttribute::CW>(cur.dir);
    dir_info.subseq = s_dir.ccw_jps;
    dir_info.terminate = dir_cw(dir_info.jps);;
    temp = m_ray.shoot_rjps_ray(cur.id, s_dir.cw_jps, heap, cur);
    scan_in_bound<ScanAttribute::CCW>(temp, cur, heap, xbound, ybound, dir_info);
}

void Solver::query(pad_id start, pad_id target)
{
    m_target = target;
    auto start_coord = m_map.id_to_xy(start), target_coord = m_map.id_to_xy(target);
    m_tracer->init(start_coord, target_coord);
    auto cmp = [](rjps_node a, rjps_node b){return (a.gval + a.hval) > (b.gval + b.hval);};
    // std::priority_queue<rjps_node, std::vector<rjps_node>, decltype(cmp)> pqueue;
    std::vector<rjps_node> heap{};
    heap.reserve(2048);
    
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
    start_node.quad_mask = (direction)UINT8_MAX;    // == 11111111
    m_node_map.try_emplace((uint64_t)start_node.id, start_node);
    }
    int iter = 0;
    while(!heap.empty())
    {
        if(iter++; iter > 5000)
        {
            std::cout<<"limit exceed\n";
            break;
        }
        auto cur = heap.front();
        if(cur.id == m_target)
        {
            break;
        }
        std::pop_heap(heap.begin(), heap.end(), cmp);
        heap.pop_back();
        auto tmp_size = heap.size();
        m_tracer->expand(m_map.id_to_xy(cur.id), "orange", "expanding, g: " + to_string(cur.gval) + " ,f: "+ to_string(cur.gval + cur.hval));
        expand(cur, heap);
        if(heap.size() > tmp_size)
        {
            for(auto i = tmp_size; i < heap.size(); i++)
            {
                const auto &n = heap[i];
                m_tracer->expand(m_map.id_to_xy(n.id), "fuchsia", "turning points");
            }
            init_rjps_nodes(heap, cur, tmp_size);
            std::make_heap(heap.begin(), heap.end(), cmp);
        }
    }
}

pad_id Solver::grid_ray_incident(pad_id from, pad_id to, direction d)
{
    auto f_coord = m_map.id_to_xy(from), t_coord = m_map.id_to_xy(to);
    auto m = min(abs((int)f_coord.first - (int)t_coord.first), abs((int)f_coord.second - (int)t_coord.second));
    return shift_in_dir(from, m, d, m_map);
}

void Solver::init_rjps_nodes(vector<rjps_node> &heap, rjps_node parent, size_t prev_end)
{
    auto parent_node = m_node_map.find((uint64_t)parent.id);
    assert(parent_node != m_node_map.end());
    auto top_adj = direction{}, bottom_adj = top_adj;
    auto t_coord = m_map.id_to_xy(m_target);
    for(auto i = prev_end, j = heap.size(); i<j; i++)
    {
        auto &node = heap[i];
        auto cur_coord = m_map.id_to_xy(node.id);
        auto p_coord = m_map.id_to_xy(parent.id);
        //gval should be the shortest path from parent, since path is taut
        node.gval = m_heuristic.h(cur_coord.first, cur_coord.second, p_coord.first, p_coord.second) + parent.gval;
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
            if(node.id == m_target) return;
            assert(false);
        }
        bool top = m_map.get(shift_in_dir(node.id, 1, top_adj, m_map));
        bool bottom = m_map.get(shift_in_dir(node.id, 1, bottom_adj, m_map));
        top =!top; 
        bottom=!bottom;
        if(top && bottom) [[unlikely]]
        {
            auto tmp_dir = node.dir;
            const auto &i = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(true));
            assert(i != quad.end());
            node.dir = i->second;
            heap.push_back(node);

            const auto &j = quad.find(to_string(parent.dir) + to_string(tmp_dir) + to_string(false));
            assert(j != quad.end());
            node.dir = j->second;
            heap.push_back(node);
        }
        else if(top)
        {
            const auto &i = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(true));
            assert(i != quad.end());
            node.dir = i->second;
        }
        else
        {
            const auto &i = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(false));
            assert(i != quad.end());
            node.dir = i->second;
        }
    }
    for(auto iter = prev_end; iter < heap.size();)
    {
        auto& cur = heap[iter];
        //if node is not visited, push a copy into map then continue
        if(m_node_map.find(uint64_t(cur.id)) == m_node_map.end())
        {
            cur.parent = &parent_node->second;
            cur.close_quad(cur.dir);
            m_node_map.emplace((uint64_t)cur.id, cur);
            ++iter;
        }
        else
        {
            
            // auto &node_in_map = m_node_map.find(uint64_t(cur.id))->second;
            //if a node has been expanded in the same position and quadrant, remove it from heap
            if(m_node_map[uint64_t(cur.id)].quad_expanded(cur.dir))
            {
                std::swap(cur, heap.back());
                heap.pop_back();
                continue;
            }
            //else update the node in map, and leave search node in heap for future expansion
            else
            {
                m_node_map[uint64_t(cur.id)].close_quad(cur.dir);
                //if a shorter path has been discovered, update path and parent
                if(m_node_map[uint64_t(cur.id)].gval > cur.gval)
                {
                    m_node_map[uint64_t(cur.id)].parent = &parent_node->second;
                    m_node_map[uint64_t(cur.id)].gval = cur.gval;
                }
                ++iter;
            }
        }
    }
}