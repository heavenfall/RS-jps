#include <Ray.h>

/*Shoots a generic ray towards the target, if target is vislble returns target, 
else returns the first colliiosn point*/
pad_id Ray::shoot_to_target(pad_id start, pad_id target, direction dir)
{
    int32_t curx = m_map.id_to_xy(start).first;
    int32_t cury = m_map.id_to_xy(start).second;
    int32_t tx =m_map.id_to_xy(target).first;
    int32_t ty =m_map.id_to_xy(target).second;
    auto dir_ind =std::countr_zero<uint8_t>(dir) - 4;
    auto adjx = adj[dir_ind][0], adjy = adj[dir_ind][1];

    while(curx != tx && cury != ty)
    {
        auto next = m_map.get(m_map.xy_to_id(curx+adjx, cury+adjy));
        //stops if next cell is blocked
        if(next == 0)
        {
            m_tracer->trace_ray(m_map.id_to_xy(start), std::make_pair(curx, cury), "blue", "shoot to target");
            return m_map.xy_to_id(curx, cury);
        }
        else 
        {
            curx+=adjx; cury+=adjy;
        }
    }
    m_tracer->trace_ray(m_map.id_to_xy(start), std::make_pair(curx, cury), "blue", "shoot to target");
    start = m_map.xy_to_id(curx, cury);
    if(curx == tx)
    {
        if(cury < ty)
        {
            int32_t steps = shoot_ray_south(start);
            if(cury + steps >= ty)
            {
                m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(target), "blue", "shoot to target");
                return target;
            }
            else 
            {
                auto ret = shift_in_dir(start, steps, SOUTH, m_map);
                m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(ret), "blue", "shoot to target");
                return ret;
            }
        }
        else    
        {
            uint32_t steps = shoot_ray_north(start);
            if(cury - steps <= ty)
            {
                m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(target), "blue", "shoot to target");
                return target;
            }
            else 
            {
                auto ret = shift_in_dir(start, steps, NORTH, m_map);
                m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(ret), "blue", "shoot to target");
                return ret;
            }
        }
    }
    else //(cury == ty)
    {
        if(curx < tx)
        {
            uint32_t steps = shoot_ray_east(start);
            if(curx + steps >= tx)
            {
                m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(target), "blue", "shoot to target");
                return target;
            }
            else 
            {
                auto ret = shift_in_dir(start, steps, EAST, m_map);
                m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(ret), "blue", "shoot to target");
                return ret;
            }
        }
        else    
        {
            uint32_t steps = shoot_ray_west(start);
            if(curx - steps <= tx)
            {
                m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(target), "blue", "shoot to target");
                return target;
            }
            else 
            {
                auto ret = shift_in_dir(start, steps, WEST, m_map);
                m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(ret), "blue", "shoot to target");
                return ret;
            }
        }
    }
}

uint32_t Ray::shoot_cardinal_ray(pad_id start, direction dir)
{
    assert( dir == NORTH || dir == SOUTH || dir == EAST || dir == WEST &&
	    "Must be cardinal direction");
    switch (dir)
    {
    case NORTH:
        return shoot_ray_north(start);
        break;
    case SOUTH:
        return shoot_ray_south(start);
        break;
    case EAST:
        return shoot_ray_east(start);
        break;
    case WEST:
        return shoot_ray_west(start);
        break;
    }
}

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
        next &= m_map.get(m_map.xy_to_id(curx+adjx, cury+adjy));
        next &= m_map.get(m_map.xy_to_id(curx, cury+adjy));
        next &= m_map.get(m_map.xy_to_id(curx+adjx, cury));
        //stops if next cell is blocked
        if(next == 0)
        {
            m_tracer->trace_ray(c, std::make_pair(curx, cury), "aqua", "diag ray?");
            return m_map.xy_to_id(curx, cury);
        }
        else 
        {
            curx+=adjx; cury+=adjy;
        }
    }
}

pad_id Ray::shoot_rjps_ray(pad_id start, direction d, std::vector<rjps_node> &vec, rjps_node parent)
{
    auto steps = uint32_t{0};
    auto ret = m_jps->jump_cardinal(d, jps_id{start.id}, m_jps->id_to_rid(jps_id{start}));
    m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    while (ret.first > 0)
    {
        //at this point, d is the direction of jps when pushed onto the vector, the scan quadrant will be updated at the end of scanning the parent's quadrant
        vec.emplace_back(ret.second, nullptr, m_map.id_to_xy(ret.second), d);
        auto r = m_map.id_to_xy(pad_id{ret.second});
        assert(r.first < 10000 && r.second < 10000);
        ret = m_jps->jump_cardinal(d, ret.second, m_jps->id_to_rid(ret.second));
        m_tracer->trace_ray(r, m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    }
    return pad_id{ret.second.id};
}

pad_id Ray::shoot_rjps_ray_to_target(pad_id start, pad_id target, direction d, std::vector<rjps_node> &vec, rjps_node parent)
{
    auto steps = uint32_t{0};
    auto ret = m_jps->jump_cardinal(d, jps_id{start.id}, m_jps->id_to_rid(jps_id{start}));
    m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    while (ret.first > 0)
    {
        //at this point, d is the direction of jps when pushed onto the vector, the scan quadrant will be updated at the end of scanning the parent's quadrant
        vec.emplace_back(ret.second, nullptr, m_map.id_to_xy(ret.second), d);
        auto r = m_map.id_to_xy(pad_id{ret.second});
        assert(r.first < 10000 && r.second < 10000);
        if(ret.second == target)
        {
            return target;
        }
        else
        {
            ret = m_jps->jump_cardinal(d, ret.second, m_jps->id_to_rid(ret.second));
            m_tracer->trace_ray(r, m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
        }
    }
    return pad_id{ret.second.id};
}
