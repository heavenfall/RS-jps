#include <rjps.h>
#include <Log.h>
#include <scanner.h>

using namespace warthog;
using namespace warthog::domain;
// using namespace warthog::grid;

class Ray
{
private:
    jump::jump_point_online<>* m_jps;
    std::shared_ptr<Tracer> m_tracer;    
    gridmap::bittable m_map;
	gridmap::bittable m_rmap;
    
    inline uint32_t shoot_ray_north(pad_id start){return shoot_hori_ray<true>(pad_id{m_jps->id_to_rid(jps_id{start}).id}, m_rmap);};
    inline uint32_t shoot_ray_south(pad_id start){return shoot_hori_ray<false>(pad_id{m_jps->id_to_rid(jps_id{start}).id}, m_rmap);};
    inline uint32_t shoot_ray_east(pad_id start){return shoot_hori_ray<true>(start, m_map);};
    inline uint32_t shoot_ray_west(pad_id start){return shoot_hori_ray<false>(start, m_map);};

public:
    Ray(std::shared_ptr<Tracer> tracer, jump::jump_point_online<>* _jps)
        : m_tracer(tracer), m_jps(_jps), m_map(m_jps->get_map()), m_rmap(m_jps->get_rmap()) {};
    ~Ray() = default;

    template<bool East>
    uint32_t shoot_hori_ray(pad_id start, domain::gridmap::bittable map);

    uint32_t shoot_cardinal_ray(pad_id start, direction dir);
    pad_id shoot_diag_ray_id(pad_id start, direction dir);

    //shoot a diag-first grid ray towards the target, returns the target pad_id if visible, otherwise the first intersection
    pad_id shoot_to_target(pad_id start, pad_id target, direction dir);

    //shoots jps ray in direction d, appends all jump points to vec ref, returns dead end point
    pad_id shoot_rjps_ray(pad_id start, direction d, std::vector<rjps_node> &vec, rjps_node parent);

    //shoots a jps ray in direction d, terminates and returns target if its reached
    //otherwised funcitons identical to shoot_rjps_ray 
    pad_id shoot_rjps_ray_to_target(pad_id start, pad_id target, direction d, std::vector<rjps_node> &vec, rjps_node parent);
};


//simple ray that steps continuouisly until an obstacle is hit
template<bool East>
uint32_t Ray::shoot_hori_ray(pad_id start, domain::gridmap::bittable map)
{
    uint32_t steps = 0;
    auto slider = domain::gridmap_slider::from_bittable(map, start);
    if constexpr(!East) 
    {
        slider.adj_bytes(-7);
        slider.width8_bits = (7-slider.width8_bits);
    }
    uint64_t mid = ~slider.get_neighbours_64bit_le()[0];
    maskzero<East>(mid, slider.width8_bits);
    printEastScan(mid);
    if(mid)
    {
        steps = East? std::countr_zero(mid) : std::countl_zero(mid);
        return steps - slider.width8_bits - 1;
    }
    slider.adj_bytes(East? 7 : -7);
    steps += 63 - slider.width8_bits -1;
    while (true)
    {
        // assert(false && "not implemented");
        uint64_t mid = ~slider.get_neighbours_64bit_le()[0];
        maskzero<East>(mid, slider.width8_bits);
        if(mid)
        {
            steps = East? std::countr_zero(mid) : std::countl_zero(mid);
            return steps;
        }
        slider.adj_bytes(East? 7 : -7);
        steps += 63;
    }    
}
