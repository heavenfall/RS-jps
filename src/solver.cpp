#include <solver.h>

// template <SolverTraits ST>
// template <ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
// uint32_t Solver<ST>::scan_in_bound(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t boundary, DirectionInfo dir_info)
// {
//     using ScanAttribute::Octants;
//     using ScanAttribute::ScanType;
//     using namespace ScanAttribute;
//     auto ret = uint32_t{};
//     auto start_coord = m_map.id_to_xy(start);
//     //ret only matters when scanning from diag boundary
//     if constexpr(horizontally_bound(Octant))
//     {
//         ret = start_coord.first;    //x
//     }
//     else
//     {
//         ret = start_coord.second;   //y
//     }
//     auto scan_res = scanResult{};
//     auto poi = pad_id{}, succ = poi;
//     scan_res.d = dir_info.init;
//     uint32_t dir_ind = std::countr_zero<uint8_t>(parent.dir)-4;
//     scan_res.top = init_scan_top[dir_ind][(scan_res.d == EAST || scan_res.d == WEST)];
//     if constexpr(horizontally_bound(Octant))    //x is the primary boundary
//     {
//         poi = m_scanner.find_turning_point(start, scan_res, dir_info.terminate, boundary, UINT32_MAX);
//     }
//     else//y is the primary boundary
//     {
//         poi = m_scanner.find_turning_point(start, scan_res, dir_info.terminate, UINT32_MAX, boundary);        
//     }
//     // poi = m_scanner.find_turning_point(start, scan_res, dir_info.terminate, xbound, ybound);

//     while (!poi.is_none())  // poi will be none if scan leaves bound, if not recurse scan
//     { 
//         auto ince = grid_ray_incident(parent.id, poi, parent.dir);
//         succ = m_ray.shoot_rjps_ray_to_target(ince, poi, dir_info.jps, vec, parent);
//         m_tracer->trace_ray(m_map.id_to_xy(parent.id), m_map.id_to_xy(ince), "aqua", "shoot ray to point");
//         m_tracer->trace_ray(m_map.id_to_xy(ince), m_map.id_to_xy(succ), "aqua", "shoot ray to point");
        
//         //if poi is blocked by another obstacle, recurse scan in both orientation on collision point
//         //depending on the orientation and the quadrant, x or y bound is changed
//         if(succ != poi)
//         {
//             auto start_coord = m_map.id_to_xy(start);
//             uint32_t cw_frontier = 0, ccw_frontier = 0, cw_bound = boundary, ccw_bound = boundary;
//             auto cw_dir_info = DirectionInfo{dir_info.subseq, dir_info.subseq, dir_info.jps, dir_ccw(dir_info.jps)};
//             auto ccw_dir_info = DirectionInfo{dir_info.subseq, dir_info.subseq, dir_info.jps, dir_cw(dir_info.jps)};
//             if constexpr (O == ScanAttribute::CW)
//             {
//                 ccw_dir_info.init = dir_flip(dir_info.subseq); ccw_dir_info.subseq = dir_flip(dir_info.subseq); 
//                 //the boundary in same orientation stays the same, the scan in the opposite orientation updates
//                 if constexpr(horizontally_bound(Octant))
//                 {
//                     ccw_bound = start_coord.first;
//                 }
//                 else
//                 {
//                     ccw_bound = start_coord.second;
//                 }
//             }
//             else
//             {
//                 cw_dir_info.init = dir_flip(dir_info.subseq); cw_dir_info.subseq = dir_flip(dir_info.subseq); 
//                 if constexpr(horizontally_bound(Octant))
//                 {
//                     cw_bound = start_coord.first;
//                 }
//                 else
//                 {
//                     cw_bound = start_coord.second;
//                 }
//             }
//             cw_frontier =  scan_in_bound<CW, Octant>(succ, parent, vec, cw_bound, cw_dir_info);
//             ccw_frontier = scan_in_bound<CCW, Octant>(succ, parent, vec, ccw_bound, ccw_dir_info);
//             return O == Orientation::CW ? cw_frontier : ccw_frontier;
//         }
//         //poi is visible, shoot a jps ray towards it then continue scanning in orientation O
//         else
//         {
//             auto s_coord = m_map.id_to_xy(poi);
//             if constexpr(horizontally_bound(Octant))
//             {
//                 ret = s_coord.first;
//             }
//             else
//             {
//                 ret = s_coord.second;
//             }
//             //the scan result indicates the first poi was a concave point, which returns the next convex point
//             if(scan_res.on_concave) [[unlikely]]
//             {
//                 //shift 1 step more to the actual corner point
//                 poi = shift_in_dir(poi, 1, scan_res.d, m_map);                
//             }
//             //poi was a convex point, shoot a jps ray to and pass it
//             else
//             {
//                 //since poin is guranteed to be a jump point(turning point), shoot a jps ray towards it to push it onto the heap
//                 //along with all jump points along the way. TODO: extra jump points might be unnecessary?
//                 poi = m_ray.shoot_rjps_ray(poi, dir_info.jps, vec, parent);             
//                 scan_res.d = dir_info.subseq;
//                 scan_res.top = (dir_info.jps == NORTH || dir_info.jps == WEST);
//             }
//             if constexpr(horizontally_bound(Octant))    //x is the primary boundary
//             {
//                 poi = m_scanner.find_turning_point(poi, scan_res, dir_info.terminate, boundary, UINT32_MAX);
//             }
//             else//y is the primary boundary
//             {
//                 poi = m_scanner.find_turning_point(poi, scan_res, dir_info.terminate, UINT32_MAX, boundary);        
//             }
//         }
//     }
//     return ret;
// }

// template <SolverTraits ST>
// inline direction Solver<ST>::target_dir(pad_id start, pad_id target)
// {
//     auto c1 = m_map.id_to_xy(start), c2 = m_map.id_to_xy(target);
//     uint32_t x = c1.first, y = c1.second, tx = c2.first, ty = c2.second;
//     if(tx<x)
//     {
//         if(ty<y) return NORTHWEST;
//         else     return SOUTHWEST;
//     }
//     else
//     {
//         if(ty<y) return NORTHEAST;
//         else     return SOUTHEAST;
//     }
// }

// template <SolverTraits ST>
// bool Solver<ST>::init_scan_dir(pad_id start, direction p_dir, scan_dir &dir)
// {
//     assert(
//     p_dir == NORTHEAST || p_dir == NORTHWEST || p_dir == SOUTHEAST || p_dir == SOUTHWEST 
//     && "Init scan direction: in dir must be intercardinal direction");
//     auto dir_ind =std::countr_zero<uint8_t>(p_dir) - 4;
//     auto adjx = adj[dir_ind][0], adjy = adj[dir_ind][1];
//     bool vert, hori, ret = false;
//     vert = m_map.get(pad_id{start.id + adjy * (int32_t)m_map.width()});
//     hori = m_map.get(pad_id{start.id + adjx});
//     if(vert) //vert not blocked
//     {
//         if(hori) //convex point
//         {
//             dir.cw_init = d_init_scan_CW[dir_ind][dir_ind == 0 || dir_ind == 3];
//             dir.ccw_init = d_init_scan_CCW[dir_ind][dir_ind == 1 || dir_ind == 2];
//             ret = true;
//         }
//         else//vert
//         {
//             dir.cw_init = d_init_scan_CW[dir_ind][0];
//             dir.ccw_init = d_init_scan_CCW[dir_ind][0];
//         }
//     }
//     else //vert blocked
//     {
//         if(hori)//hori
//         {
//             dir.cw_init = d_init_scan_CW[dir_ind][1];
//             dir.ccw_init = d_init_scan_CCW[dir_ind][1];
//         }
//         else//concave point
//         {
//             dir.cw_init = d_init_scan_CW[dir_ind][dir_ind == 1 || dir_ind == 2];
//             dir.ccw_init = d_init_scan_CCW[dir_ind][dir_ind == 0 || dir_ind == 3];
//         }
//     }
//     dir.cw_subseq = d_scan[dir_ind][0];
//     dir.cw_jps = d_jps[dir_ind][0];
//     dir.ccw_subseq = d_scan[dir_ind][1];
//     dir.ccw_jps = d_jps[dir_ind][1];
//     return ret;
// }

// template <SolverTraits ST>
// void Solver<ST>::expand(rjps_node cur, std::vector<rjps_node> &heap)
// {
//     auto cur_coord = m_map.id_to_xy(cur.id);
//     auto temp = pad_id{};
//     auto s_dir = scan_dir{};
//     //if target is in same quadrant, shoot to it
//     if(target_dir(cur.id, m_target) == cur.dir)
//     {
//         temp = m_ray.shoot_to_target(cur.id, m_target, cur.dir);
//         if(temp == m_target)
//         {
//             auto t = rjps_node{m_target, &m_node_map.find(uint64_t(cur.id))->second, m_map.id_to_xy(m_target), NONE};
//             auto target_coord = m_map.id_to_xy(m_target);
//             t.hval = 0;
//             t.gval = m_heuristic.h(target_coord.first, target_coord.second, cur_coord.first, cur_coord.second);
//             heap.emplace_back(t);
//             return;
//         }
//     }
//     temp = m_ray.shoot_diag_ray_id(cur.id, cur.dir);
//     auto on_convex = init_scan_dir(temp, cur.dir, s_dir);
//     auto cw_start = temp, ccw_start = temp;
//     if(on_convex)
//     {
//         cw_start = shift_in_dir(cw_start, 1, s_dir.cw_init, m_map);
//         ccw_start = shift_in_dir(ccw_start, 1, s_dir.ccw_init, m_map);
//     }
//     auto dir_info = DirectionInfo{};
//     //scan both ways from the point the ray intercepted
//     //CW scan
//     dir_info.init = s_dir.cw_init;
//     dir_info.jps = s_dir.cw_jps;
//     dir_info.subseq = s_dir.cw_subseq;
//     dir_info.terminate = rotate_eighth<ScanAttribute::CCW>(cur.dir);
//     uint32_t cwbound = scan_in_bound<ScanAttribute::CW>(cw_start, cur, heap, cur_coord.first, cur_coord.second, dir_info);

//     //CCW scan
//     dir_info.init = s_dir.ccw_init;
//     dir_info.jps = s_dir.ccw_jps;
//     dir_info.subseq = s_dir.ccw_subseq;
//     dir_info.terminate = rotate_eighth<ScanAttribute::CW>(cur.dir);
//     uint32_t ccwbound = scan_in_bound<ScanAttribute::CCW>(ccw_start, cur, heap, cur_coord.first, cur_coord.second, dir_info);

//     //CW scan from left extremety(left of scan center)
//     auto xbound = uint32_t{}, ybound = xbound;
//     if(cur.dir == NORTHEAST || cur.dir == SOUTHWEST)
//     {
//         xbound = ccwbound;
//         ybound = cur_coord.second;
//     }
//     else    
//     {
//         xbound = cur_coord.first;
//         ybound = ccwbound;
//     }
//     dir_info.init = s_dir.cw_jps;
//     dir_info.jps =  rotate_eighth<ScanAttribute::CCW>(cur.dir);
//     dir_info.subseq = s_dir.cw_jps;
//     dir_info.terminate = dir_ccw(dir_info.jps);
//     temp = m_ray.shoot_rjps_ray(cur.id, s_dir.ccw_jps, heap, cur);
//     //IF FIRST CCW SCAN FAILS, CCWBOUND WILL STAY THE SAME AS ORIGINAL BOUND THEREFORE SHORTCIRCUIT
//     scan_in_bound_alt<ScanAttribute::CW>(temp, cur, heap, xbound, ybound, dir_info);
    
//     //CCW scan from right extremety(right of scan center)
//     if(cur.dir == NORTHWEST || cur.dir == SOUTHEAST)
//     {
//         xbound = cwbound;
//         ybound = cur_coord.second;
//     }
//     else    
//     {
//         xbound = cur_coord.first;
//         ybound = cwbound;
//     }
//     //CCW scan from right extremety
//     dir_info.init = s_dir.ccw_jps;
//     dir_info.jps =  rotate_eighth<ScanAttribute::CW>(cur.dir);
//     dir_info.subseq = s_dir.ccw_jps;
//     dir_info.terminate = dir_cw(dir_info.jps);;
//     temp = m_ray.shoot_rjps_ray(cur.id, s_dir.cw_jps, heap, cur);
//     scan_in_bound_alt<ScanAttribute::CCW>(temp, cur, heap, xbound, ybound, dir_info);
// }


// template <SolverTraits ST>
// pad_id Solver<ST>::grid_ray_incident(pad_id from, pad_id to, direction d)
// {
//     auto f_coord = m_map.id_to_xy(from), t_coord = m_map.id_to_xy(to);
//     auto m = min(abs((int)f_coord.first - (int)t_coord.first), abs((int)f_coord.second - (int)t_coord.second));
//     return shift_in_dir(from, m, d, m_map);
// }

// template <SolverTraits ST>
// void Solver<ST>::init_rjps_nodes(vector<rjps_node> &heap, rjps_node parent, size_t prev_end)
// {
//     auto parent_node = m_node_map.find((uint64_t)parent.id);
//     assert(parent_node != m_node_map.end());
//     auto top_adj = direction{}, bottom_adj = top_adj;
//     auto t_coord = m_map.id_to_xy(m_target);
//     for(auto i = prev_end, j = heap.size(); i<j; i++)
//     {
//         auto &node = heap[i];
//         auto cur_coord = m_map.id_to_xy(node.id);
//         auto p_coord = m_map.id_to_xy(parent.id);
//         //gval should be the shortest path from parent, since path is taut
//         node.gval = m_heuristic.h(cur_coord.first, cur_coord.second, p_coord.first, p_coord.second) + parent.gval;
//         node.hval = m_heuristic.h(cur_coord.first, cur_coord.second, t_coord.first, t_coord.second);
//         switch (node.dir)
//         {
//         case NORTH:
//             top_adj = SOUTHWEST; bottom_adj = SOUTHEAST;
//             break;
//         case SOUTH:
//             top_adj = NORTHWEST; bottom_adj = NORTHEAST;
//             break;
//         case EAST:
//             top_adj = NORTHWEST; bottom_adj = SOUTHWEST;
//             break;
//         case WEST:
//             top_adj = NORTHEAST; bottom_adj = SOUTHEAST;
//             break;
//         default:
//             if(node.id == m_target) return;
//             assert(false);
//         }
//         bool top = m_map.get(shift_in_dir(node.id, 1, top_adj, m_map));
//         bool bottom = m_map.get(shift_in_dir(node.id, 1, bottom_adj, m_map));
//         top =!top; 
//         bottom=!bottom;
//         if(top && bottom) [[unlikely]]
//         {
//             auto tmp_dir = node.dir;
//             const auto &i = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(true));
//             assert(i != quad.end());
//             node.dir = i->second;
//             heap.push_back(node);

//             const auto &j = quad.find(to_string(parent.dir) + to_string(tmp_dir) + to_string(false));
//             assert(j != quad.end());
//             node.dir = j->second;
//             heap.push_back(node);
//         }
//         else if(top)
//         {
//             const auto &i = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(true));
//             assert(i != quad.end());
//             node.dir = i->second;
//         }
//         else
//         {
//             const auto &i = quad.find(to_string(parent.dir) + to_string(node.dir) + to_string(false));
//             assert(i != quad.end());
//             node.dir = i->second;
//         }
//     }
//     for(auto iter = prev_end; iter < heap.size();)
//     {
//         auto& cur = heap[iter];
//         //if node is not visited, push a copy into map then continue
//         if(m_node_map.find(uint64_t(cur.id)) == m_node_map.end())
//         {
//             cur.parent = &parent_node->second;
//             cur.close_quad(cur.dir);
//             m_node_map.emplace((uint64_t)cur.id, cur);
//             ++iter;
//         }
//         else
//         {
            
//             // auto &node_in_map = m_node_map.find(uint64_t(cur.id))->second;
//             //if a node has been expanded in the same position and quadrant, remove it from heap
//             if(m_node_map[uint64_t(cur.id)].quad_closed(cur.dir))
//             {
//                 std::swap(cur, heap.back());
//                 heap.pop_back();
//                 continue;
//             }
//             //else update the node in map, and leave search node in heap for future expansion
//             else
//             {
//                 m_node_map[uint64_t(cur.id)].close_quad(cur.dir);
//                 //if a shorter path has been discovered, update path and parent
//                 if(m_node_map[uint64_t(cur.id)].gval > cur.gval)
//                 {
//                     m_node_map[uint64_t(cur.id)].parent = &parent_node->second;
//                     m_node_map[uint64_t(cur.id)].gval = cur.gval;
//                 }
//                 ++iter;
//             }
//         }
//     }
// }