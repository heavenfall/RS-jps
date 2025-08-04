// #include <Ray.h>

// template<SolverTraits ST>
// pad_id Ray::<ST>shoot_diag_ray_id(pad_id start, direction dir)
// {
//     assert( dir == NORTHEAST || dir == NORTHWEST || dir == SOUTHEAST || dir == SOUTHWEST &&
// 	    "Must be intercardinal direction");
//     auto dir_ind =std::countr_zero<uint8_t>(dir) - 4;
//     int adjx = adj[dir_ind][0], adjy = adj[dir_ind][1];
//     uint32_t curx, cury;
//     auto c = m_map.id_to_xy(start);
//     curx = c.first; cury = c.second;
//     bool next = true;
//     while(true)
//     {
//         //check 3 neibouring cells in direction to avoiding "corner cutting"
//         next &= m_map.get(m_map.xy_to_id(curx+adjx, cury+adjy));
//         next &= m_map.get(m_map.xy_to_id(curx, cury+adjy));
//         next &= m_map.get(m_map.xy_to_id(curx+adjx, cury));
//         //stops if next cell is blocked
//         if(next == 0)
//         {
//             m_tracer->trace_ray(c, std::make_pair(curx, cury), "aqua", "diag ray?");
//             return m_map.xy_to_id(curx, cury);
//         }
//         else 
//         {
//             curx+=adjx; cury+=adjy;
//         }
//     }
// }

// //shoots diaganal ray in direction dir, stops either on obstacle hit or reached a cell thats parallel to the target (same x or y)
// template<SolverTraits ST>
// pad_id Ray::<ST>shoot_diag_ray_id(pad_id start, pad_id target, direction dir)
// {
//     assert( dir == NORTHEAST || dir == NORTHWEST || dir == SOUTHEAST || dir == SOUTHWEST &&
// 	    "Must be intercardinal direction");
//     auto dir_ind =std::countr_zero<uint8_t>(dir) - 4;
//     int adjx = adj[dir_ind][0], adjy = adj[dir_ind][1];
//     auto tcoord = m_map.id_to_xy(target);
//     uint32_t curx, cury, tx = tcoord.first, ty = tcoord.second;
//     auto c = m_map.id_to_xy(start);
//     curx = c.first; cury = c.second;
//     bool next = true;
//     while(true)
//     {
//         //check 3 neibouring cells in direction to avoiding "corner cutting"
//         next &= m_map.get(m_map.xy_to_id(curx+adjx, cury+adjy));
//         next &= m_map.get(m_map.xy_to_id(curx, cury+adjy));
//         next &= m_map.get(m_map.xy_to_id(curx+adjx, cury));
//         //stops if next cell is blocked or parallel to target
//         if(next == 0 || curx == tx || cury == ty)
//         {
//             m_tracer->trace_ray(c, std::make_pair(curx, cury), "aqua", "diag ray");
//             return m_map.xy_to_id(curx, cury);
//         }
//         else 
//         {
//             curx+=adjx; cury+=adjy;
//         }
//     }
// }

// template<SolverTraits ST>
// pad_id Ray::<ST>shoot_rjps_ray(pad_id start, direction d, std::vector<rjps_node> &vec, rjps_node parent)
// {
//     auto steps = uint32_t{0};
//     auto ret = m_jps->jump_cardinal(d, jps_id{start.id}, m_jps->id_to_rid(jps_id{start}));
//     m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
//     while (ret.first > 0)
//     {
//         //at this point, d is the direction of jps when pushed onto the vector, the scan quadrant will be updated at the end of scanning the parent's quadrant
//         vec.emplace_back(ret.second, nullptr, m_map.id_to_xy(ret.second), d);
//         auto r = m_map.id_to_xy(pad_id{ret.second});
//         assert(r.first < 10000 && r.second < 10000);
//         ret = m_jps->jump_cardinal(d, ret.second, m_jps->id_to_rid(ret.second));
//         m_tracer->trace_ray(r, m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
//     }
//     return pad_id{ret.second.id};
// }

// template<SolverTraits ST>
// pad_id Ray::<ST>shoot_rjps_ray_to_target(pad_id start, pad_id target, direction d, std::vector<rjps_node> &vec, rjps_node parent)
// {
//     auto steps = uint32_t{0};
//     auto ret = m_jps->jump_cardinal(d, jps_id{start.id}, m_jps->id_to_rid(jps_id{start}));
//     m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
//     while (ret.first > 0)
//     {
//         //at this point, d is the direction of jps when pushed onto the vector, the scan quadrant will be updated at the end of scanning the parent's quadrant
//         vec.emplace_back(ret.second, nullptr, m_map.id_to_xy(ret.second), d);
//         auto r = m_map.id_to_xy(pad_id{ret.second});
//         assert(r.first < 10000 && r.second < 10000);
//         if(ret.second == target)
//         {
//             return target;
//         }
//         else
//         {
//             ret = m_jps->jump_cardinal(d, ret.second, m_jps->id_to_rid(ret.second));
//             m_tracer->trace_ray(r, m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
//         }
//     }
//     return pad_id{ret.second.id};
// }
