#include <scanner.h>
using namespace warthog::domain;


Scanner::Scanner(std::shared_ptr<Tracer> _tracer, jump::jump_point_online<>* _jps) : 
m_tracer(_tracer), m_jps(_jps), m_map(m_jps->get_map()), m_rmap(m_jps->get_rmap())
{};

std::vector<pad_id> Scanner::test_scan_full(pad_id start, uint32_t bx, uint32_t by)
{
    m_bx = bx; m_by = by;
    std::vector<pad_id> ret;
    scan_obstacle(start, ret, EAST, true);
    return ret;
}

void Scanner::scan_obstacle(pad_id start, std::vector<pad_id> &ret, direction dir, bool _top)
{
    scanResult res;
    res.top = _top;
    direction d =dir, tempd;
    uint32_t steps;
    pad_id curpos = start, nextpos;
    bool EeastorNorth, NorthorSouth, looped = false;
    while (!looped)
    {        
        assert(d==EAST||d==WEST||d==NORTH||d==SOUTH);
        EeastorNorth = (d==EAST||d==NORTH);
        tempd = d;
        switch (d)
        {            
        case EAST:
            steps = scan_east(curpos, res);            
            break;
        case WEST:
            steps = scan_west(curpos, res);
            break;
        case NORTH:
            steps = scan_north(curpos, res);
            break;
        case SOUTH:
            steps = scan_south(curpos, res);
            break;
        }
        nextpos = shift_in_dir(curpos, steps, d, m_map);
        //if the scan oversteps the boundry, stop the scan and return
        if (d==EAST||d==WEST)
        {
            int curx, nx;
            curx = m_jps->id_to_point(jps_id{curpos}).x;
            nx = m_jps->id_to_point(jps_id{nextpos}).x;
            if(between2(m_bx, curx, nx)) 
            {
                std::cout<<"overstepped x bound\n";
                return;
            }
        }
        else 
        {
            int cury, ny;
            cury = m_jps->id_to_point(jps_id{curpos}).y;
            ny = m_jps->id_to_point(jps_id{nextpos}).y;
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
            curpos = shift_in_dir(nextpos, 1, d, m_map);
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
                d = EeastorNorth? dir_ccw(d): dir_cw(d);
            }
            else if(!res.top)   //1
            {
                // if(EeastorNorth) res.top = false;
                d = EeastorNorth? dir_cw(d): dir_ccw(d);                
            }            
            //curpos is currently on the turning point, shift 1 step before next scan
            curpos = shift_in_dir(curpos, 1, d, m_map);
        }
        else if (res.c > res.m)//concave corners
        {
            if(res.top)         //3
            {
                // if(EeastorNorth) res.top = false;
                d = EeastorNorth? dir_cw(d): dir_ccw(d);
            }
            else if(!res.top)   //2
            {
                // if(!EeastorNorth) res.top = true;
                d = EeastorNorth? dir_ccw(d): dir_cw(d);
            }
            curpos = nextpos;
        }
        if(EN_diff_WS(d, tempd)) res.top = !res.top; //magic
    }
}

pad_id Scanner::scan_obstacle(pad_id start, scanResult &res)
{
    direction tempd = res.d;
    uint32_t steps;
    pad_id ret;
    // pad_id curpos = start, nextpos;
    bool EeastorNorth, NorthorSouth, looped = false;
    assert(res.d==EAST||res.d==WEST||res.d==NORTH||res.d==SOUTH);
    EeastorNorth = (res.d==EAST||res.d==NORTH);
    switch (res.d)
    {            
    case EAST:
        steps = scan_east(start, res);            
        break;
    case WEST:
        steps = scan_west(start, res);
        break;
    case NORTH:
        steps = scan_north(start, res);
        break;
    case SOUTH:
        steps = scan_south(start, res);
        break;
    }
    ret = shift_in_dir(start, steps, res.d, m_map);
    
    //refer to my beautiful drawing
    if(res.c < res.m)//convex corners
    {
        ret = shift_in_dir(ret, 1, res.d, m_map);
        if(res.top)         //4
        {            
            res.d = EeastorNorth? dir_ccw(res.d): dir_cw(res.d);
        }
        else if(!res.top)   //1
        {
            res.d = EeastorNorth? dir_cw(res.d): dir_ccw(res.d);                
        }
        ret = shift_in_dir(ret, 1, res.d, m_map);
    }
    else if (res.c > res.m)//concave corners
    {
        if(res.top)         //3
        {
            res.d = EeastorNorth? dir_cw(res.d): dir_ccw(res.d);
        }
        else if(!res.top)   //2
        {
            res.d = EeastorNorth? dir_ccw(res.d): dir_cw(res.d);
        }
    }
    if(EN_diff_WS(res.d, tempd)) res.top = !res.top; //magic
    return ret;
}

uint32_t Scanner::scan_east(pad_id start, scanResult &res)
{
    return scan_hori<true>(m_map, start, res);
}

uint32_t Scanner::scan_west(pad_id start, scanResult &res)
{
    return scan_hori<false>(m_map, start, res);
}

uint32_t Scanner::scan_north(pad_id start, scanResult &res)
{
    start = pad_id{m_jps->id_to_rid(jps_id{start}).id};
    //same as scan east on the r map
    return scan_hori<true>(m_rmap, start, res);
}

uint32_t Scanner::scan_south(pad_id start, scanResult &res)
{
    start = pad_id{m_jps->id_to_rid(jps_id{start}).id};
    //same as scan west on the r map
    return scan_hori<false>(m_rmap, start, res);
}

uint32_t Scanner::test_scan_single(pad_id start, bool top, char c)
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

double Scanner::vecangle(pad_id _o, pad_id _a, pad_id _b)
{
    std::pair<int, int> o, a, b, oa, ob;
    o = m_map.id_to_xy(_o);
    a = m_map.id_to_xy(_a);
    b = m_map.id_to_xy(_b);
    oa = {o.first-a.first, o.second-a.second};
    ob = {o.first-b.first, o.second-b.second};
    double cross = (oa.first * ob.second - oa.second * ob.first);
    double dot = (oa.first * ob.first + oa.second * ob.second); 
    return std::atan2(cross, dot) * 180.0/3.141592653589793238463;
}

bool Scanner::init_scan_eastwest(pad_id& start, direction in_dir)
{
    assert(
        in_dir == NORTHEAST || in_dir == NORTHWEST || in_dir == SOUTHEAST || in_dir == SOUTHWEST 
        && "Must be intercardinal direction");

    bool e, w, n, s;
    e = m_map.get(shift_in_dir(start, 1, EAST, m_map));
    w = m_map.get(shift_in_dir(start, 1, WEST, m_map));
    n = m_map.get(shift_in_dir(start, 1, NORTH, m_map));
    s = m_map.get(shift_in_dir(start, 1, SOUTH, m_map));
    //if start is on the corner of a obstacle
    if(e && w && n && s)
    {
        direction dd = (in_dir == SOUTHEAST || in_dir == SOUTHWEST) ? SOUTH : NORTH;
        start = shift_in_dir(start, 1, dd, m_map);//lol
        std::cout<<"shift ";
        if(dd == NORTH) std::cout<<"north\n";
        else if(dd == SOUTH) std::cout<<"south\n";
        return false;
    }
    return(e && w);
}

