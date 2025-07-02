#include <solver.h>

Solver::Solver(jump::jump_point_online<>* _jps) : 
    m_jps(_jps), m_tracer(new Tracer), m_map(m_jps->get_map()), m_rmap(m_jps->get_rmap()), 
    m_ray(m_tracer, m_jps), m_scanner(m_tracer, m_jps)
{
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

// std::vector<rjps_node> Solver::scan_in_boundary(rjps_node parent, pad_id start)
// {
//     using namespace ScanAttribute;
//     auto ret = std::vector<rjps_node>{};
//     auto poi = pad_id{}, succ = poi, cw_start = start, ccw_start = cw_start;  
//     auto dir = scan_dir{};  
//     //init scan directions, adjust init scan node if start resides on a convex point of an obstacle
//     auto on_convex = init_scan_dir(start, parent.dir, dir);
//     if(on_convex)
//     {
//         cw_start = shift_in_dir(cw_start, 1, dir.cw_init, m_map);
//         ccw_start = shift_in_dir(ccw_start, 1, dir.ccw_init, m_map);
//     }
//     auto inner_x_bound = int{0}, inner_y_bound = inner_x_bound; 
//     //clock-wise scan
//     poi = m_scanner.find_turning_point<ScanAttribute::CW>(cw_start, dir.cw_init, parent.dir, parent.xbound, parent.ybound);
//     auto poi_coord = m_map.id_to_xy(poi);
//     m_tracer->expand(poi_coord);
//     while (!poi.is_none())  // poi will be none if scan leaves bound, if not recurse scan
//     { 
//         auto ince = grid_ray_incident(parent.id, poi, parent.dir);
//         succ = m_ray.shoot_rjps_ray_to_target(ince, poi, dir.cw_jps, ret, parent);
//         //poi not visible from current node, recurse scan
//         //TODO: stop when crossing diag bound?
//         if(succ != poi)
//         {
//             //full scan again
//             auto r = scan_in_boundary(parent, succ);
//             ret.insert(ret.end(), r.begin(), r.end());
//         }
//         else 
//         {
//             auto s_coord = m_map.id_to_xy(succ);
//             if(parent.dir == SOUTHEAST || parent.dir == NORTHWEST)
//             {
//                 inner_x_bound = s_coord.first;
//             }
//             else    
//             {
//                 inner_y_bound = s_coord.second;
//             }
//             // ret.emplace_back(succ, parent.id, m_map.id_to_xy(succ), parent.dir);
//             // poi = grid_ray_incident(parent.id, poi, parent.dir);
//             succ = m_ray.shoot_rjps_ray(succ, dir.cw_jps, ret, parent);
//             //jps ray
//             poi = m_scanner.find_turning_point<CW>(succ, dir.cw_subseq, parent.dir, parent.xbound, parent.ybound);
//             if (!poi.is_none()) m_tracer->expand(m_map.id_to_xy(poi));
//         }
//     }
//     //counter-clock-wise scan
//     poi = m_scanner.find_turning_point<CCW>(ccw_start, dir.ccw_init, parent.dir, parent.xbound, parent.ybound);
//     m_tracer->expand(m_map.id_to_xy(poi));
//     while (!poi.is_none())  
//     {
//         auto ince = grid_ray_incident(parent.id, poi, parent.dir);
//         succ = m_ray.shoot_rjps_ray_to_target(ince, poi, dir.ccw_jps, ret, parent);
//         //poi not visible from current node, recurse scan
//         if(succ != poi)
//         {
//             auto r = scan_in_boundary(parent, succ);
//             ret.insert(ret.end(), r.begin(), r.end());
//         }
//         else 
//         {
//             auto s_coord = m_map.id_to_xy(succ);
//             if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)
//             {
//                 inner_x_bound = s_coord.first;
//             }
//             else    
//             {
//                 inner_y_bound = s_coord.second;
//             }
//             // ret.emplace_back(succ, parent.id, m_map.id_to_xy(succ), parent.dir);
//             // poi = grid_ray_incident(parent.id, poi, parent.dir);
//             succ = m_ray.shoot_rjps_ray(succ, dir.ccw_jps, ret, parent);
//             //jps ray
//             poi = m_scanner.find_turning_point<CCW>(succ, dir.ccw_subseq, parent.dir, parent.xbound, parent.ybound);
//             if (!poi.is_none()) m_tracer->expand(m_map.id_to_xy(poi));
//         }
//     }
//     auto x_bound = parent.xbound, y_bound = parent.ybound; 
//     if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)
//     {
//         x_bound = inner_x_bound;
//     }
//     else    
//     {
//         y_bound = inner_y_bound;
//     }
//     //CW
//     succ = m_ray.shoot_rjps_ray(parent.id, dir.ccw_jps, ret, parent);
//     poi = m_scanner.find_turning_point<CW>(succ, dir.cw_jps, parent.dir, x_bound, y_bound);
//     m_tracer->expand(m_map.id_to_xy(poi));
//     while (!poi.is_none())  // poi will be none if scan leaves bound, if not recurse scan
//     { 
//         auto ince = grid_ray_incident(parent.id, poi, parent.dir);
//         succ = m_ray.shoot_rjps_ray_to_target(ince, poi, dir.ccw_jps, ret, parent);
//         //poi not visible from current node, recurse scan
//         //TODO: stop when crossing diag bound?
//         if(succ != poi)
//         {
//             //full scan again
//             auto r = scan_in_boundary(parent, succ);
//             ret.insert(ret.end(), r.begin(), r.end());
//         }
//         else 
//         {
//             auto s_coord = m_map.id_to_xy(succ);
//             // ret.emplace_back(succ, parent.id, m_map.id_to_xy(succ), parent.dir);
//             // poi = grid_ray_incident(parent.id, poi, parent.dir);
//             succ = m_ray.shoot_rjps_ray(succ, dir.ccw_jps, ret, parent);
//             //jps ray
//             poi = m_scanner.find_turning_point<CW>(succ, dir.cw_jps, parent.dir, x_bound, y_bound);
//             if (!poi.is_none()) m_tracer->expand(m_map.id_to_xy(poi));
//         }
//     }
//     //CCW
//     if(parent.dir == NORTHEAST || parent.dir == SOUTHWEST)
//     {
//         x_bound = parent.xbound;
//         y_bound = inner_y_bound;
//     }
//     else    
//     {
//         x_bound = inner_x_bound;
//         y_bound = parent.ybound;
//     }
//     succ = m_ray.shoot_rjps_ray(parent.id, dir.cw_jps, ret, parent);
//     poi = m_scanner.find_turning_point<CCW>(succ, dir.ccw_jps, parent.dir, x_bound, y_bound);

//     return ret;
// }

void Solver::test_func(pad_id id1, pad_id id2)
{
    auto start_coord = m_map.id_to_xy(id1);
    m_tracer->init(start_coord, m_map.id_to_xy(id2));
    direction myd = SOUTHEAST;
    auto temp = pad_id{};
    auto vec = std::vector<rjps_node>{};
    auto testnode = rjps_node{id1, id1, m_map.id_to_xy(id1), myd};
    auto s_dir = scan_dir{};
    //if target is in same quadrant, shoot to it
    if(target_dir(id1, id2) == testnode.dir)
    {
        temp = m_ray.shoot_to_target(id1, id2);
        if(temp == id2)
        {
            std::cout<<"target visible\n";
            return;
        }
    }
    temp = m_ray.shoot_diag_ray_id(testnode.id, m_map, testnode.dir);
    auto on_convex = init_scan_dir(temp, myd, s_dir);
    auto cw_start = temp, ccw_start = temp;
    if(on_convex)
    {
        cw_start = shift_in_dir(cw_start, 1, s_dir.cw_init, m_map);
        ccw_start = shift_in_dir(ccw_start, 1, s_dir.ccw_init, m_map);
        m_tracer->expand(m_map.id_to_xy(cw_start), "purple", "shift");
        m_tracer->expand(m_map.id_to_xy(ccw_start), "purple", "shift");
    }
    auto dir_info = DirectionInfo{};
    //scan both ways from the point the ray intercepted
    //CW scan
    dir_info.init = s_dir.cw_init;
    dir_info.jps = s_dir.cw_jps;
    dir_info.subseq = s_dir.cw_subseq;
    dir_info.terminate = rotate_eighth<ScanAttribute::CCW>(myd);
    uint32_t cwbound = scan_in_bound<ScanAttribute::CW>(cw_start, testnode, vec, start_coord.first, start_coord.second, dir_info);

    //CCW scan
    dir_info.init = s_dir.ccw_init;
    dir_info.jps = s_dir.ccw_jps;
    dir_info.subseq = s_dir.ccw_subseq;
    dir_info.terminate = rotate_eighth<ScanAttribute::CW>(myd);
    uint32_t ccwbound = scan_in_bound<ScanAttribute::CCW>(ccw_start, testnode, vec, start_coord.first, start_coord.second, dir_info);

    //CW scan from left extremety(left of scan center)
    auto xbound = uint32_t{}, ybound = xbound;
    if(myd == NORTHEAST || myd == SOUTHWEST)
    {
        xbound = ccwbound;
        ybound = start_coord.second;
    }
    else    
    {
        xbound = start_coord.first;
        ybound = ccwbound;
    }
    dir_info.init = s_dir.cw_jps;
    dir_info.jps =  rotate_eighth<ScanAttribute::CCW>(myd);
    dir_info.subseq = s_dir.cw_jps;
    dir_info.terminate = dir_ccw(dir_info.jps);
    temp = m_ray.shoot_rjps_ray(id1, s_dir.ccw_jps, vec, testnode);
    //IF FIRST CCW SCAN FAILS, CCWBOUND WILL STAY THE SAME AS ORIGINAL BOUND THEREFORE SHORTCIRCUIT
    scan_in_bound<ScanAttribute::CW>(temp, testnode, vec, xbound, ybound, dir_info);
    
    //CCW scan from right extremety(right of scan center)
    if(myd == NORTHWEST || myd == SOUTHEAST)
    {
        xbound = cwbound;
        ybound = start_coord.second;
    }
    else    
    {
        xbound = start_coord.first;
        ybound = cwbound;
    }
    //CCW scan from right extremety
    dir_info.init = s_dir.ccw_jps;
    dir_info.jps =  rotate_eighth<ScanAttribute::CW>(myd);
    dir_info.subseq = s_dir.ccw_jps;
    dir_info.terminate = dir_cw(dir_info.jps);;
    temp = m_ray.shoot_rjps_ray(id1, s_dir.cw_jps, vec, testnode);
    scan_in_bound<ScanAttribute::CCW>(temp, testnode, vec, xbound, ybound, dir_info);
}

void Solver::test_func2(pad_id start, pad_id target)
{
    m_tracer->init(m_map.id_to_xy(start), m_map.id_to_xy(target));

    auto temp = pad_id::none(), poi = temp;
    auto vec = std::vector<rjps_node>{};
    auto testnode = rjps_node{start, start, m_map.id_to_xy(start), SOUTHEAST};

    //if target is in same quadrant, shoot to it
    if(target_dir(start, target) == testnode.dir)
    {
        temp = m_ray.shoot_to_target(start, target);
        if(temp == target)
        {
            std::cout<<"target visible\n";
            return;
        }
    }
    temp = m_ray.shoot_diag_ray_id(testnode.id, m_map, testnode.dir);
    // auto v = scan_in_boundary(testnode, temp);
}

pad_id Solver::grid_ray_incident(pad_id from, pad_id to, direction d)
{
    auto f_coord = m_map.id_to_xy(from), t_coord = m_map.id_to_xy(to);
    auto m = min(abs((int)f_coord.first - (int)t_coord.first), abs((int)f_coord.second - (int)t_coord.second));
    return shift_in_dir(from, m, d, m_map);
}

