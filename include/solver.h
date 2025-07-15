#pragma once
#include <rjps.h>
#include <scanner.h>
#include <Ray.h>
#include <Log.h>
#include <warthog/heuristic/octile_heuristic.h>
#include <queue>
#include <unordered_map>
// using namespace warthog::domain;
// using namespace jps;


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

class Solver
{
public:
    Solver(jump::jump_point_online<>* _jps);
    ~Solver() = default;
    
    void expand(rjps_node cur, std::vector<rjps_node> &heap);
    void query(pad_id start, pad_id target);

    // std::vector<rjps_node> scan_in_boundary(rjps_node parent, pad_id start);
    template <ScanAttribute::Orientation O>
    uint32_t scan_in_bound(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t xbound, uint32_t ybound, DirectionInfo dir_info);
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

    inline direction target_dir(pad_id start, pad_id target);
    void init_rjps_nodes(vector<rjps_node> &heap, rjps_node parent, size_t prev_end);
};


//Scans all visible obstacles in a set orientation, appends succesors to the vector passed in
//@param start: position to start the scan
//@param parent: rjps search node the scan is based on
//@param &vec: reference to the return vector of rjps nodes
//@param xbound: x boundary
//@param ybound: y boundary
//@param dir_info: collection of direction info which includes initial scan direction, subseq scan direction and terminating direction
template <ScanAttribute::Orientation O>
uint32_t Solver::scan_in_bound(pad_id start, rjps_node parent, std::vector<rjps_node> &vec, uint32_t xbound, uint32_t ybound, DirectionInfo dir_info)
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
    poi = m_scanner.find_turning_point(start, scan_res, dir_info.terminate, xbound, ybound);
    while (!poi.is_none())  // poi will be none if scan leaves bound, if not recurse scan
    { 
        auto ince = grid_ray_incident(parent.id, poi, parent.dir);
        succ = m_ray.shoot_rjps_ray_to_target(ince, poi, dir_info.jps, vec, parent);
        m_tracer->trace_ray(m_map.id_to_xy(parent.id), m_map.id_to_xy(ince), "aqua", "shoot ray to point");
        m_tracer->trace_ray(m_map.id_to_xy(ince), m_map.id_to_xy(succ), "aqua", "shoot ray to point");
        
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
                succ = m_ray.shoot_rjps_ray(succ, dir_info.jps, vec, parent);             
                scan_res.d = dir_info.subseq;
                scan_res.top = (dir_info.jps == NORTH || dir_info.jps == WEST);
            }
            poi = m_scanner.find_turning_point(succ, scan_res, dir_info.terminate, xbound, ybound);
        }
    }
    return ret;
}
