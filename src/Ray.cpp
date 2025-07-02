#include <Ray.h>

pad_id Ray::shoot_to_target(pad_id start, pad_id target)
{
    int curx = m_map.id_to_xy(start).first;
    int cury = m_map.id_to_xy(start).second;
    int tx = m_map.id_to_xy(target).first;
    int ty = m_map.id_to_xy(target).second;
    int adjx, adjy;
    if(curx < tx)   //TODO: optimize this
    {
        if (cury < ty)
        {
            adjx = 1; adjy = 1;
        } 
        else 
        {
            adjx = 1; adjy = -1;
        }
    }
    else    
    {
        if (cury < ty) 
        {
            adjx = -1; adjy = 1;
        }
        else 
        {
            adjx = -1; adjy = -1;
        }
    }

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
            // m_tracer->expand(curx, cury);
        }
    }
    m_tracer->trace_ray(m_map.id_to_xy(start), std::make_pair(curx, cury), "blue", "shoot to target");
    start = m_map.xy_to_id(curx, cury);
    if(curx == tx)
    {
        if(cury < ty)
        {
            uint32_t steps = shoot_ray_south(start);
            // m_tracer->expand(curx, cury + steps);
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
            // m_tracer->expand(curx, cury - steps);
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
            // m_tracer->expand(curx + steps, cury);
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
            // m_tracer->expand(curx - steps, cury);
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

pad_id Ray::shoot_rjps_ray(pad_id start, direction d, std::vector<rjps_node> &vec, rjps_node parent)
{
    auto steps = uint32_t{0};
    auto ret = m_jps->jump_cardinal(d, jps_id{start.id}, m_jps->id_to_rid(jps_id{start}));
    m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    while (ret.first > 0)
    {
        auto r = m_map.id_to_xy(pad_id{ret.second});
        // m_tracer->expand(r);
        ret = m_jps->jump_cardinal(d, ret.second, m_jps->id_to_rid(ret.second));
        m_tracer->trace_ray(r, m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    }
    return pad_id{ret.second.id};
}

pad_id Ray::shoot_rjps_ray_to_target(pad_id start, pad_id target,direction d, std::vector<rjps_node> &vec, rjps_node parent)
{
    auto steps = uint32_t{0};
    auto ret = m_jps->jump_cardinal(d, jps_id{start.id}, m_jps->id_to_rid(jps_id{start}));
    m_tracer->trace_ray(m_map.id_to_xy(start), m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
    while (ret.first > 0)
    {
        auto r = m_map.id_to_xy(pad_id{ret.second});
        if(ret.second == target)
        {
            return target;
        }
        else
        {
            m_tracer->expand(r);
            ret = m_jps->jump_cardinal(d, ret.second, m_jps->id_to_rid(ret.second));
            m_tracer->trace_ray(r, m_map.id_to_xy(pad_id{ret.second}), "green", "jps ray");
        }

    }
    return pad_id{ret.second.id};
}

pad_id Ray::shoot_diag_ray_id(pad_id start, domain::gridmap::bittable map, direction dir)
{
    assert( dir == NORTHEAST || dir == NORTHWEST || dir == SOUTHEAST || dir == SOUTHWEST &&
	    "Must be intercardinal direction");
    auto dir_ind =std::countr_zero<uint8_t>(dir) - 4;
    int adjx = adj[dir_ind][0], adjy = adj[dir_ind][1];
    uint32_t curx, cury;
    auto c = map.id_to_xy(start);
    curx = c.first; cury = c.second;
    bool next = true;
    while(true)
    {
        next &= map.get(map.xy_to_id(curx+adjx, cury+adjy));
        next &= map.get(map.xy_to_id(curx, cury+adjy));
        next &= map.get(map.xy_to_id(curx+adjx, cury));
        //stops if next cell is blocked
        if(next == 0)
        {
            m_tracer->trace_ray(c, std::make_pair(curx, cury));
            return map.xy_to_id(curx, cury);
        }
        else 
        {
            curx+=adjx; cury+=adjy;
            // m_tracer->expand(curx, cury);             
        }
    }
}