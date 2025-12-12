#include <rjps.h>
#include <Log.h>
#include <scanner.h>

using namespace warthog;
using namespace warthog::domain;
using namespace ScanAttribute;

class Ray
{
private:
    jump::jump_point_online* m_jps;
    std::shared_ptr<Tracer> m_tracer;
    gridmap_rotate_table_convs m_map; 

    template<bool East, Domain D>
    uint32_t shoot_hori_ray(grid_id start, gridmap::bittable map);

public:
    Ray(std::shared_ptr<Tracer> tracer, jump::jump_point_online* _jps, gridmap_rotate_table_convs _map)
        : m_tracer(tracer), m_jps(_jps), m_map(_map) {};
    ~Ray() = default;

    template <Domain D>
    std::pair<uint32_t, grid_id> shoot_hori_ray(grid_id start, direction_id dir);

    template <Domain D>
    inline uint32_t shoot_ray_north(grid_id start){return shoot_hori_ray<true, D>(grid_id(m_map.id_to_rid(start)), m_map.rtable());};
    template <Domain D>
    inline uint32_t shoot_ray_south(grid_id start){return shoot_hori_ray<false, D>(grid_id(m_map.id_to_rid(start)), m_map.rtable());};
    template <Domain D>
    inline uint32_t shoot_ray_east(grid_id start){return shoot_hori_ray<true, D>(start, m_map.table());};
    template <Domain D>
    inline uint32_t shoot_ray_west(grid_id start){return shoot_hori_ray<false, D>(start, m_map.table());};

    jump::jump_distance jump_cardinal(direction_id, grid_id id);
    

    //shoot a diag-first grid ray towards the target, returns the target grid_id if visible, otherwise the first intersection
    template<SolverTraits ST, Octants octant>
    std::pair<bool, grid_id> check_target_visible(grid_id start, grid_id target, direction_id dir);

    template<SolverTraits ST>
    grid_id shoot_diag_ray_id(grid_id start, direction_id dir);
    template<SolverTraits ST>
    grid_id shoot_diag_ray_id(grid_id start, grid_id target, direction_id dir);



    //shoots jps ray in direction d, appends all jump points to vec ref, returns dead end point
    template<SolverTraits ST>
    grid_id shoot_rjps_ray(grid_id start, direction_id d, std::vector<rjps_state> &succ);

    //shoots a jps ray in direction d, terminates and returns target if its reached
    //otherwised funcitons identical to shoot_rjps_ray 
    template<SolverTraits ST>
    grid_id shoot_rjps_ray_to_target(grid_id start, grid_id target, direction_id d, std::vector<rjps_state> &succ);
};

//simple ray that steps continuouisly until an obstacle is hit

template<bool East, Domain D>
uint32_t Ray::shoot_hori_ray(grid_id start, gridmap::bittable map)
{
    uint32_t steps = 0;
    auto slider = gridmap_slider::from_bittable(map, start);
    if constexpr(!East) 
    {
        slider.adj_bytes(-7);
        slider.width8_bits = (7-slider.width8_bits);
    }
    //because we're counting 0's, and map stores travasable as 1, 
    //flip the bits if the ray's domain is in Travasable space
    uint64_t mid = (D==Travasable)? ~slider.get_neighbours_64bit_le()[0]
                                    :slider.get_neighbours_64bit_le()[0];
    maskzero<East>(mid, slider.width8_bits);

    if(mid)
    {
        steps = East? std::countr_zero(mid) : std::countl_zero(mid);
        //for ray inside obstacles, we need to account for ray shooting outside the map boundary and wrapping to the next row
        if constexpr(D == Obstacle)
        {
            bool wrapped = East? steps > (map.width() - (start.id % map.width())) :
                                 steps > (start.id % map.width());
            if(wrapped) return UINT32_MAX;
        }
        return steps - slider.width8_bits - 1;
    }
    steps += 63 - slider.width8_bits -1;
    slider.adj_bytes(East? 7 : -7);
    slider.width8_bits = 7;
    while (true)
    {
        mid = (D==Travasable)?  ~slider.get_neighbours_64bit_le()[0]
                                :slider.get_neighbours_64bit_le()[0];
        maskzero<East>(mid, slider.width8_bits);
        if(mid)
        {
            steps += East? std::countr_zero(mid) : std::countl_zero(mid);
            if constexpr(D == Obstacle)
            {
                bool wrapped = East? steps > (map.width() - (start.id % map.width())) :
                                    steps > (start.id % map.width());
                if(wrapped) return UINT32_MAX;
            }
            return steps - slider.width8_bits;
        }
        slider.adj_bytes(East? 7 : -7);
        steps += 63  - slider.width8_bits;
    }    
}

template <Domain D>
std::pair<uint32_t, grid_id> Ray::shoot_hori_ray(grid_id start, direction_id dir)
{
    auto ret = std::pair<uint32_t, grid_id>{};
    switch (dir)
    {
    case NORTH_ID:
        ret.first = this->shoot_ray_north<D>(start);
        ret.second = shift_in_dir(start, ret.first, NORTH_ID, m_map.table());
        break;
    case SOUTH_ID:
        ret.first = this->shoot_ray_south<D>(start);
        ret.second = shift_in_dir(start, ret.first, SOUTH_ID, m_map.table());
        break;
    case EAST_ID:
        ret.first = this->shoot_ray_east<D>(start);
        ret.second = shift_in_dir(start, ret.first, EAST_ID, m_map.table());
        break;
    case WEST_ID:
        ret.first = this->shoot_ray_west<D>(start);
        ret.second = shift_in_dir(start, ret.first, WEST_ID, m_map.table());
        break;
    default:
        break;
    }
    if(ret.first == UINT32_MAX) ret.second = grid_id::max();
    return ret;
}

/*Shoots a generic ray towards the target, if target is vislble returns target, 
else returns the first colliiosn point*/
//@return pair.first -> if ray ends on diaganal ray. pair.second -> intersection
template<SolverTraits ST, Octants octant>
std::pair<bool, grid_id> Ray::check_target_visible(grid_id start, grid_id target, direction_id dir)
{
    using ScanAttribute::Octants;
    auto tp = shoot_diag_ray_id<ST>(start, target, dir);
    if(tp == target)
    {
        return std::make_pair(false, target);
    }
    auto tp_coord = m_map.id_to_point(tp), target_coord = m_map.id_to_point(target);
    uint32_t curx = tp_coord.x;
    uint32_t cury = tp_coord.y;
    uint32_t tx = target_coord.x;
    uint32_t ty = target_coord.y;
    if(curx != tx && cury != ty) return std::make_pair(false, tp);
    uint32_t steps = 0;
    auto ret = grid_id{};
    if constexpr(octant == NNW || octant == NNE)
    {
        steps = shoot_ray_north<Travasable>(tp);
        if(cury - steps <= ty) ret = target;
        else ret = shift_in_dir(tp, steps, NORTH_ID, m_map.table());
    }
    else if constexpr(octant == ENE || octant == ESE)
    {
        steps = shoot_ray_east<Travasable>(tp);
        if(curx + steps >= tx) ret = target;
        else ret = shift_in_dir(tp, steps, EAST_ID, m_map.table());
    }
    else if constexpr(octant == SSE || octant == SSW)
    {
        steps = shoot_ray_south<Travasable>(tp);
        if(cury + steps >= ty) ret = target;
        else ret = shift_in_dir(tp, steps, SOUTH_ID, m_map.table());
    }
    else if constexpr(octant == WSW || octant == WNW)
    {
        steps = shoot_ray_west<Travasable>(tp);
        if(curx - steps <= tx) ret = target;
        else ret = shift_in_dir(tp, steps, WEST_ID, m_map.table());
    }
    if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(tp_coord, m_map.id_to_point(ret), "blue", "shoot to target");
    return std::make_pair(true, ret);;
}

template<SolverTraits ST>
grid_id Ray::shoot_diag_ray_id(grid_id start, direction_id dir)
{
    assert( dir == NORTHEAST_ID || dir == NORTHWEST_ID || dir == SOUTHEAST_ID || dir == SOUTHWEST_ID &&
	    "Must be intercardinal direction");
    uint16_t adjx = adj[(int)dir-4].x, adjy = adj[(int)dir-4].y;
    auto cur = m_map.id_to_point(start);
    auto cstart[[maybe_unused]] = cur;
    bool next = true;
    while(true)
    {
        //check 3 neibouring cells in direction to avoiding "corner cutting"
        next &= m_map.map().get(m_map.point_to_id(point(cur.x+adjx, cur.y+adjy)));
        next &= m_map.map().get(m_map.point_to_id(point(cur.x, cur.y+adjy)));
        next &= m_map.map().get(m_map.point_to_id(point(cur.x+adjx, cur.y)));
        //stops if next cell is blocked
        if(next == 0)
        {
            if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray({cstart.x, cstart.y}, {cur.x, cur.y}, "aqua", "diag ray?");
            return m_map.point_to_id(cur);
        }
        else 
        {
            cur.x+=adjx; cur.y+=adjy;
        }
    }
}

// TODO: look into using BasicIntercardinalWalker or IntercardinalWalker<D>s
//shoots diaganal ray in direction dir, stops either on obstacle hit or reached a cell thats parallel to the target (same x or y)
template<SolverTraits ST>
grid_id Ray::shoot_diag_ray_id(grid_id start, grid_id target, direction_id dir)
{
    assert( dir == NORTHEAST_ID || dir == NORTHWEST_ID || dir == SOUTHEAST_ID || dir == SOUTHWEST_ID &&
	    "Must be intercardinal direction");
    uint16_t adjx = adj[(int)dir-4].x, adjy = adj[(int)dir-4].y;
    auto cur = m_map.id_to_point(start);
    auto cstart[[maybe_unused]] = cur;
    auto tcoord = m_map.id_to_point(target);
    bool next = true;
    while(true)
    {
        //check 3 neibouring cells in direction to avoiding "corner cutting"
        next &= m_map.map().get(m_map.point_to_id(point(cur.x+adjx, cur.y+adjy)));
        next &= m_map.map().get(m_map.point_to_id(point(cur.x, cur.y+adjy)));
        next &= m_map.map().get(m_map.point_to_id(point(cur.x+adjx, cur.y)));
        //stops if next cell is blocked or parallel to target
        if(next == 0 || cur.x == tcoord.x || cur.y == tcoord.y)
        {
            if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray({cstart.x, cstart.y}, {cur.x, cur.y}, "aqua", "diag ray");
            return m_map.point_to_id(cur);
        }
        else 
        {
            cur.x+=adjx; cur.y+=adjy;
        }
    }
}

inline jump::jump_distance Ray::jump_cardinal(direction_id d, grid_id id)
{
    jump::jump_distance ret{};
    grid_pair_id pid{id, m_map.id_to_rid(id)};
    switch (d) {
    case NORTH_ID:
        ret = m_jps->jump_cardinal_next<NORTH_ID>(pid);
        break;
    case EAST_ID:
        ret = m_jps->jump_cardinal_next<EAST_ID>(pid);
        break;
    case SOUTH_ID:
        ret = m_jps->jump_cardinal_next<SOUTH_ID>(pid);
        break;
    case WEST_ID:
        ret = m_jps->jump_cardinal_next<WEST_ID>(pid);
        break;
    }
    return ret;
}

// TODO: look at converting over from grid_id to domain::grid_pair_id
// TODO: look at making direction_id a template parameter for this function
template<SolverTraits ST>
grid_id Ray::shoot_rjps_ray(grid_id start, direction_id d, std::vector<rjps_state> &succ)
{
    const uint32_t idadj = grid::dir_id_adj(d, m_map.width());
    auto dist = jump_cardinal(d, start);
    grid_id ret = start;
    ret.id += idadj * static_cast<uint32_t>(std::abs(dist));
    if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(m_map.id_to_point(start), m_map.id_to_point(ret), "green", "jps ray");
    while (dist > 0)
    {
        //at this point, d is the direction of jps when pushed onto the vector, the scan quadrant will be updated at the end of scanning the parent's quadrant
        // succ.emplace_back(ret.second, nullptr, m_map.id_to_point(ret.second), d);
        auto n = rjps_state(ret);
        //cache the jps direction the succ is found, used for generating the succ search node
        succ.emplace_back(ret, d);
        dist = jump_cardinal(d, ret);
        ret.id += idadj * static_cast<uint32_t>(std::abs(dist));
        if constexpr(ST == SolverTraits::OutputToPosthoc) 
        {
            auto r = m_map.id_to_point(ret);
            m_tracer->trace_ray(r, m_map.id_to_point(ret), "green", "jps ray");
        }
    }
    return ret;
}

template<SolverTraits ST>
grid_id Ray::shoot_rjps_ray_to_target(grid_id start, grid_id target, direction_id d, std::vector<rjps_state> &succ)
{
    const uint32_t idadj = grid::dir_id_adj(d, m_map.width());
    auto dist = jump_cardinal(d, start);
    grid_id ret = start;
    ret.id += idadj * static_cast<uint32_t>(std::abs(dist));
    if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(m_map.id_to_point(start), m_map.id_to_point(ret), "green", "jps ray");
    while (dist > 0)
    {
        //at this point, d is the direction of jps when pushed onto the vector, the scan quadrant will be updated at the end of scanning the parent's quadrant
        // succ.emplace_back(ret.second, nullptr, m_map.id_to_point(ret.second), d);
        auto n = rjps_state{ret};
        succ.emplace_back(ret, d);
        if(ret == target)
        {
            return target;
        }
        else
        {
            dist = jump_cardinal(d, ret);
            ret.id += idadj * static_cast<uint32_t>(std::abs(dist));
            if constexpr(ST == SolverTraits::OutputToPosthoc) 
            {
                auto r = m_map.id_to_point(ret);
                m_tracer->trace_ray(r, m_map.id_to_point(ret), "green", "jps ray");
            }
        }
    }
    return ret;
}
