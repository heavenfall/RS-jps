#include <scanner.h>
using namespace warthog::domain;


Scanner::Scanner(std::shared_ptr<Tracer> _tracer, jump::jump_point_online* _jps, gridmap_rotate_table_convs _map) : 
m_tracer(_tracer), m_jps(_jps), m_map(_map)
{};

std::vector<grid_id> Scanner::test_scan_full(grid_id start, uint32_t bx, uint32_t by)
{
    m_bx = bx; m_by = by;
    std::vector<grid_id> ret;
    scan_obstacle(start, ret, EAST_ID, true);
    return ret;
}

void Scanner::scan_obstacle(grid_id start, std::vector<grid_id> &ret, direction_id dir, bool _top)
{
    scanResult res;
    res.top = _top;
    direction_id d =dir, tempd;
    uint32_t steps;
    grid_id curpos = start, nextpos;
    bool EeastorNorth, NorthorSouth, looped = false;
    while (!looped)
    {        
        assert(d==EAST_ID||d==WEST_ID||d==NORTH_ID||d==SOUTH_ID);
        EeastorNorth = (d==EAST_ID||d==NORTH_ID);
        tempd = d;
        switch (d)
        {            
        case EAST_ID:
            steps = scan_east(curpos, res);            
            break;
        case WEST_ID:
            steps = scan_west(curpos, res);
            break;
        case NORTH_ID:
            steps = scan_north(curpos, res);
            break;
        case SOUTH_ID:
            steps = scan_south(curpos, res);
            break;
        }
        nextpos = shift_in_dir(curpos, steps, d, m_map.table());
        //if the scan oversteps the boundry, stop the scan and return
        if (d==EAST_ID||d==WEST_ID)
        {
            int curx, nx;
            curx = m_map.id_to_point(curpos).x;
            nx = m_map.id_to_point(nextpos).x;
            if(between2(m_bx, curx, nx)) 
            {
                std::cout<<"overstepped x bound\n";
                return;
            }
        }
        else 
        {
            int cury, ny;
            cury = m_map.id_to_point(curpos).y;
            ny = m_map.id_to_point(nextpos).y;
            if(between2(m_by, cury, ny)) 
            {
                std::cout<<"overstepped y bound\n";
                return;
            }
        }
        
        //refer to my beautiful drawing
        if(res.c < res.m)//convex corners
        {
            //the scan hits a convex point, push the corner to the vector
            //shift 1 because the scan stops at a step before the poi
            curpos = shift_in_dir(nextpos, 1, d, m_map.table());
            if(ret.size() != 0)
            {
                if (curpos == ret[0])
                {
                    std::cout<<"looped\n";
                    looped = true;
                    break;
                }
                
            }
            ret.push_back(curpos);
            if(res.top)         //4
            {
                // if(!EeastorNorth) res.top = true;                
                d = EeastorNorth? dir_id_ccw90(d): dir_id_cw90(d);
            }
            else if(!res.top)   //1
            {
                // if(EeastorNorth) res.top = false;
                d = EeastorNorth? dir_id_cw90(d): dir_id_ccw90(d);                
            }            
            //curpos is currently on the turning point, shift 1 step before next scan
            curpos = shift_in_dir(curpos, 1, d, m_map.table());
        }
        else if (res.c > res.m)//concave corners
        {
            if(res.top)         //3
            {
                // if(EeastorNorth) res.top = false;
                d = EeastorNorth? dir_id_cw90(d): dir_id_ccw90(d);
            }
            else if(!res.top)   //2
            {
                // if(!EeastorNorth) res.top = true;
                d = EeastorNorth? dir_id_ccw90(d): dir_id_cw90(d);
            }
            curpos = nextpos;
        }
        if(EN_diff_WS(d, tempd)) res.top = !res.top; //magic
    }
}

grid_id Scanner::scan_obstacle(grid_id start, scanResult &res)
{
    direction_id tempd = res.d;
    uint32_t steps;
    grid_id ret;
    // grid_id curpos = start, nextpos;
    bool EeastorNorth, NorthorSouth, looped = false;
    assert(res.d==EAST_ID||res.d==WEST_ID||res.d==NORTH_ID||res.d==SOUTH_ID);
    EeastorNorth = (res.d==EAST_ID||res.d==NORTH_ID);
    switch (res.d)
    {            
    case EAST_ID:
        steps = scan_east(start, res);            
        break;
    case WEST_ID:
        steps = scan_west(start, res);
        break;
    case NORTH_ID:
        steps = scan_north(start, res);
        break;
    case SOUTH_ID:
        steps = scan_south(start, res);
        break;
    }
    ret = shift_in_dir(start, steps, res.d, m_map.table());
    
    //refer to my beautiful drawing
    if(res.c < res.m)//convex corners
    {
        ret = shift_in_dir(ret, 1, res.d, m_map.table());
        if(res.top)         //4
        {            
            res.d = EeastorNorth? dir_id_ccw90(res.d): dir_id_cw90(res.d);
        }
        else if(!res.top)   //1
        {
            res.d = EeastorNorth? dir_id_cw90(res.d): dir_id_ccw90(res.d);                
        }
        ret = shift_in_dir(ret, 1, res.d, m_map.table());
    }
    else if (res.c > res.m)//concave corners
    {
        if(res.top)         //3
        {
            res.d = EeastorNorth? dir_id_cw90(res.d): dir_id_ccw90(res.d);
        }
        else if(!res.top)   //2
        {
            res.d = EeastorNorth? dir_id_ccw90(res.d): dir_id_cw90(res.d);
        }
    }
    if(EN_diff_WS(res.d, tempd)) res.top = !res.top; //magic
    return ret;
}

uint32_t Scanner::scan_east(grid_id start, scanResult &res)
{
    return scan_hori<true>(m_map.table(), start, res);
}

uint32_t Scanner::scan_west(grid_id start, scanResult &res)
{
    return scan_hori<false>(m_map.table(), start, res);
}

uint32_t Scanner::scan_north(grid_id start, scanResult &res)
{
    start = grid_id(m_map.id_to_rid(start).id);
    //same as scan east on the r map
    return scan_hori<true>(m_map.rtable(), start, res);
}

uint32_t Scanner::scan_south(grid_id start, scanResult &res)
{
    start = grid_id(m_map.id_to_rid(start).id);
    //same as scan west on the r map
    return scan_hori<false>(m_map.rtable(), start, res);
}

uint32_t Scanner::test_scan_single(grid_id start, bool top, char c)
{
    scanResult temp; 
    temp.top = top;
    switch (c)
    {
    case 'e':
        return scan_east(start, temp);
        break;
    case 'w':
        return scan_west(start, temp);
        break;
    case 'n':
        return scan_north(start, temp);
        break;
    case 's':
        return scan_south(start, temp);
        break;            
    
    default:
        std::cout<<"enter a valid dir\n";
        return INT32_MAX;
        break;
    }
}

// TODO: look at changing this into vector angles
double Scanner::vecangle(grid_id _o, grid_id _a, grid_id _b)
{
    auto o = m_map.id_to_point(_o);
    auto a = m_map.id_to_point(_a);
    auto b = m_map.id_to_point(_b);
    auto oa = point_signed_diff(a, o);
    auto ob = point_signed_diff(b, o);
    double cross = (oa.first * ob.second - oa.second * ob.first);
    double dot = (oa.first * ob.first + oa.second * ob.second); 
    return std::atan2(cross, dot) * (180.0/3.141592653589793238463);
}

bool Scanner::init_scan_eastwest(grid_id& start, direction_id in_dir)
{
    assert(
        in_dir == NORTHEAST_ID || in_dir == NORTHWEST_ID || in_dir == SOUTHEAST_ID || in_dir == SOUTHWEST_ID 
        && "Must be intercardinal direction");

    bool e, w, n, s;
    e = m_map.map().get(shift_in_dir(start, 1, EAST_ID, m_map.table()));
    w = m_map.map().get(shift_in_dir(start, 1, WEST_ID, m_map.table()));
    n = m_map.map().get(shift_in_dir(start, 1, NORTH_ID, m_map.table()));
    s = m_map.map().get(shift_in_dir(start, 1, SOUTH_ID, m_map.table()));
    //if start is on the corner of a obstacle
    if(e && w && n && s)
    {
        direction_id dd = (in_dir == SOUTHEAST_ID || in_dir == SOUTHWEST_ID) ? SOUTH_ID : NORTH_ID;
        start = shift_in_dir(start, 1, dd, m_map.table());//lol
        // std::cout<<"shift ";
        // if(dd == NORTH_ID) std::cout<<"north\n";
        // else if(dd == SOUTH_ID) std::cout<<"south\n";
        return false;
    }
    return(e && w);
}

