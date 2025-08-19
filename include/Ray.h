#include <rjps.h>
#include <Log.h>
#include <scanner.h>

using namespace warthog;
using namespace warthog::domain;
using namespace ScanAttribute;

class Ray
{
private:
    jump::jump_point_online<>* m_jps;
    std::shared_ptr<Tracer> m_tracer;    
    gridmap::bittable m_map;
	gridmap::bittable m_rmap;    

    template<bool East, Domain D>
    uint32_t shoot_hori_ray(pad_id start, domain::gridmap::bittable map);

public:
    Ray(std::shared_ptr<Tracer> tracer, jump::jump_point_online<>* _jps)
        : m_tracer(tracer), m_jps(_jps), m_map(m_jps->get_map()), m_rmap(m_jps->get_rmap()) {};
    ~Ray() = default;

    template <Domain D>
    std::pair<uint32_t, pad_id> shoot_hori_ray(pad_id start, direction dir);

    template <Domain D>
    inline uint32_t shoot_ray_north(pad_id start){return shoot_hori_ray<true, D>(pad_id{m_jps->id_to_rid(jps_id{start}).id}, m_rmap);};
    template <Domain D>
    inline uint32_t shoot_ray_south(pad_id start){return shoot_hori_ray<false, D>(pad_id{m_jps->id_to_rid(jps_id{start}).id}, m_rmap);};
    template <Domain D>
    inline uint32_t shoot_ray_east(pad_id start){return shoot_hori_ray<true, D>(start, m_map);};
    template <Domain D>
    inline uint32_t shoot_ray_west(pad_id start){return shoot_hori_ray<false, D>(start, m_map);};

    //shoot a diag-first grid ray towards the target, returns the target pad_id if visible, otherwise the first intersection
    template<SolverTraits ST, Octants octant>
    std::pair<bool, pad_id> check_target_visible(pad_id start, pad_id target, direction dir);

    template<SolverTraits ST>
    pad_id shoot_diag_ray_id(pad_id start, direction dir);
    template<SolverTraits ST>
    pad_id shoot_diag_ray_id(pad_id start, pad_id target, direction dir);



    //shoots jps ray in direction d, appends all jump points to vec ref, returns dead end point
    template<SolverTraits ST>
    pad_id shoot_rjps_ray(pad_id start, direction d, std::vector<rjps_node> &vec, rjps_node parent);

    //shoots a jps ray in direction d, terminates and returns target if its reached
    //otherwised funcitons identical to shoot_rjps_ray 
    template<SolverTraits ST>
    pad_id shoot_rjps_ray_to_target(pad_id start, pad_id target, direction d, std::vector<rjps_node> &vec, rjps_node parent);
};

//simple ray that steps continuouisly until an obstacle is hit

template<bool East, Domain D>
uint32_t Ray::shoot_hori_ray(pad_id start, domain::gridmap::bittable map)
{
    uint32_t steps = 0;
    auto slider = domain::gridmap_slider::from_bittable(map, start);
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
std::pair<uint32_t, pad_id> Ray::shoot_hori_ray(pad_id start, direction dir)
{
    auto ret = std::pair<uint32_t, pad_id>{};
    switch (dir)
    {
    case NORTH:
        ret.first = this->shoot_ray_north<D>(start);
        ret.second = shift_in_dir(start, ret.first, NORTH, m_map);
        break;
    case SOUTH:
        ret.first = this->shoot_ray_south<D>(start);
        ret.second = shift_in_dir(start, ret.first, SOUTH, m_map);
        break;
    case EAST:
        ret.first = this->shoot_ray_east<D>(start);
        ret.second = shift_in_dir(start, ret.first, EAST, m_map);
        break;
    case WEST:
        ret.first = this->shoot_ray_west<D>(start);
        ret.second = shift_in_dir(start, ret.first, WEST, m_map);
        break;
    default:
        break;
    }
    if(ret.first == UINT32_MAX) ret.second = pad_id::max();
    return ret;
}

/*Shoots a generic ray towards the target, if target is vislble returns target, 
else returns the first colliiosn point*/
//@return pair.first -> if ray ends on diaganal ray. pair.second -> intersection
template<SolverTraits ST, Octants octant>
std::pair<bool, pad_id> Ray::check_target_visible(pad_id start, pad_id target, direction dir)
{
    using ScanAttribute::Octants;
    auto tp = shoot_diag_ray_id<ST>(start, target, dir);
    if(tp == target)
    {
        return std::make_pair(false, target);
    }
    auto tp_coord = m_map.id_to_xy(tp), target_coord = m_map.id_to_xy(target);
    uint32_t curx = tp_coord.first;
    uint32_t cury = tp_coord.second;
    uint32_t tx = target_coord.first;
    uint32_t ty = target_coord.second;
    if(curx != tx && cury != ty) return std::make_pair(false, tp);
    uint32_t steps = 0;
    auto ret = pad_id{};
    if constexpr(octant == NNW || octant == NNE)
    {
        steps = shoot_ray_north<Travasable>(tp);
        if(cury - steps <= ty) ret = target;
        else ret = shift_in_dir(tp, steps, NORTH, m_map);
    }
    else if constexpr(octant == ENE || octant == ESE)
    {
        steps = shoot_ray_east<Travasable>(tp);
        if(curx + steps >= tx) ret = target;
        else ret = shift_in_dir(tp, steps, EAST, m_map);
    }
    else if constexpr(octant == SSE || octant == SSW)
    {
        steps = shoot_ray_south<Travasable>(tp);
        if(cury + steps >= ty) ret = target;
        else ret = shift_in_dir(tp, steps, SOUTH, m_map);
    }
    else if constexpr(octant == WSW || octant == WNW)
    {
        steps = shoot_ray_west<Travasable>(tp);
        if(curx - steps <= tx) ret = target;
        else ret = shift_in_dir(tp, steps, WEST, m_map);
    }
    if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(tp_coord, m_map.id_to_xy(ret), "blue", "shoot to target");
    return std::make_pair(true, ret);;
}

template<SolverTraits ST>
pad_id Ray::shoot_diag_ray_id(pad_id start, direction dir)
{
    assert( dir == NORTHEAST || dir == NORTHWEST || dir == SOUTHEAST || dir == SOUTHWEST &&
	    "Must be intercardinal direction");
    auto dir_ind =std::countr_zero<uint8_t>(dir) - 4;
    int adjx = adj[dir_ind][0], adjy = adj[dir_ind][1];
    uint32_t curx, cury;
    auto c = m_map.id_to_xy(start);
    curx = c.first; cury = c.second;
    bool next = true;
    while(true)
    {
        //check 3 neibouring cells in direction to avoiding "corner cutting"
        next &= m_map.get(m_map.xy_to_id(curx+adjx, cury+adjy));
        next &= m_map.get(m_map.xy_to_id(curx, cury+adjy));
        next &= m_map.get(m_map.xy_to_id(curx+adjx, cury));
        //stops if next cell is blocked
        if(next == 0)
        {
            if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(c, std::make_pair(curx, cury), "aqua", "diag ray?");
            return m_map.xy_to_id(curx, cury);
        }
        else 
        {
            curx+=adjx; cury+=adjy;
        }
    }
}

//shoots diaganal ray in direction dir, stops either on obstacle hit or reached a cell thats parallel to the target (same x or y)
template<SolverTraits ST>
pad_id Ray::shoot_diag_ray_id(pad_id start, pad_id target, direction dir)
{
    assert( dir == NORTHEAST || dir == NORTHWEST || dir == SOUTHEAST || dir == SOUTHWEST &&
	    "Must be intercardinal direction");
    auto dir_ind =std::countr_zero<uint8_t>(dir) - 4;
    int adjx = adj[dir_ind][0], adjy = adj[dir_ind][1];
    auto tcoord = m_map.id_to_xy(target);
    uint32_t curx, cury, tx = tcoord.first, ty = tcoord.second;
    auto c = m_map.id_to_xy(start);
    curx = c.first; cury = c.second;
    bool next = true;
    while(true)
    {
        //check 3 neibouring cells in direction to avoiding "corner cutting"
        next &= m_map.get(m_map.xy_to_id(curx+adjx, cury+adjy));
        next &= m_map.get(m_map.xy_to_id(curx, cury+adjy));
        next &= m_map.get(m_map.xy_to_id(curx+adjx, cury));
        //stops if next cell is blocked or parallel to target
        if(next == 0 || curx == tx || cury == ty)
        {
            if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(c, std::make_pair(curx, cury), "aqua", "diag ray");
            return m_map.xy_to_id(curx, cury);
        }
        else 
        {
            curx+=adjx; cury+=adjy;
        }
    }
}

template<SolverTraits ST>
pad_id Ray::shoot_rjps_ray(pad_id start, direction d, std::vector<rjps_node> &vec, rjps_node parent)
{
    auto ret = m_jps->jump_cardinal(d, jps_id{start.id}, m_jps->id_to_rid(jps_id{start.id}));
    if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    while (ret.first > 0)
    {
        //at this point, d is the direction of jps when pushed onto the vector, the scan quadrant will be updated at the end of scanning the parent's quadrant
        vec.emplace_back(ret.second, nullptr, m_map.id_to_xy(ret.second), d);
        auto r = m_map.id_to_xy(pad_id{ret.second});
        ret = m_jps->jump_cardinal(d, ret.second, m_jps->id_to_rid(ret.second));
        if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(r, m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    }
    return pad_id{ret.second.id};
}

template<SolverTraits ST>
pad_id Ray::shoot_rjps_ray_to_target(pad_id start, pad_id target, direction d, std::vector<rjps_node> &vec, rjps_node parent)
{
    auto ret = m_jps->jump_cardinal(d, jps_id{start.id}, m_jps->id_to_rid(jps_id{start.id}));
    if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    while (ret.first > 0)
    {
        //at this point, d is the direction of jps when pushed onto the vector, the scan quadrant will be updated at the end of scanning the parent's quadrant
        vec.emplace_back(ret.second, nullptr, m_map.id_to_xy(ret.second), d);
        auto r = m_map.id_to_xy(pad_id{ret.second});
        if(ret.second == target)
        {
            return target;
        }
        else
        {
            ret = m_jps->jump_cardinal(d, ret.second, m_jps->id_to_rid(ret.second));
            if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(r, m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
        }
    }
    return pad_id{ret.second.id};
}
