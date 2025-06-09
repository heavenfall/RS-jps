#include <scanner.h>
using namespace warthog::domain;

Scanner::Scanner(jump::jump_point_online<>* _jps)
{
    m_map = _jps->get_map();
    m_rmap = _jps->get_rmap();
    m_jps = _jps;
};

std::vector<pad_id> Scanner::scan(pad_id start, uint32_t bx, uint32_t by)
{
    m_bx = bx; m_by = by;
    std::vector<pad_id> ret;
    scanObstcile(start, ret, EAST, true);
    return ret;
}

void Scanner::scanObstcile(pad_id start, std::vector<pad_id> &ret, direction dir, bool _top)
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
        nextpos = shiftInDir(curpos, steps, d);
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
            curpos = shiftInDir(nextpos, 1, d);
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
            curpos = shiftInDir(curpos, 1, d);
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

// std::vector<pad_id> Scanner::scanObstcile(uint32_t s, direction dir, bool _top)
// {
//     std::vector<pad_id> ret;
//     direction d = dir;
//     bool top = _top, looped = false;
//     pad_id cur = pad_id{s}, firstCorner = pad_id{s}, start = pad_id{s};
//     int midStoppingPoint;    //stopping point of traversable tiles
//     int obstaclePoint;       //stopping point of obstacle
//     uint32_t steps;
//     while (!looped)
//     {
//         switch (d)
//         {
//             case EAST:
//                 steps = scan_east(cur, top, d, midStoppingPoint, obstaclePoint);
//                 if(top)//scanned obstacle on top slider
//                 {
//                     if(midStoppingPoint > obstaclePoint)//convex corner up
//                     {
//                         d = NORTH;
//                         top = false;
//                         cur = shiftInDir(cur, steps, EAST);
//                         if(firstCorner == start) 
//                         {
//                             firstCorner = cur;
//                             break;
//                         }
//                         if(cur == firstCorner)
//                         {
//                             looped = true;
//                             break;
//                         }
//                     }
//                     if(midStoppingPoint < obstaclePoint)//concave corner down
//                     {
//                         d = SOUTH;
//                         cur = shiftInDir(cur, steps - 1, EAST);
//                     }
//                 }
//                 else//scanned obstacle on bottom slider    
//                 {
//                     if(midStoppingPoint < obstaclePoint)//concave corner up
//                     {
//                         d = NORTH;
//                         top = true;
//                         cur = shiftInDir(cur, steps - 1, EAST);
//                     }
//                     if(midStoppingPoint > obstaclePoint )//convex corner down
//                     {
//                         d = SOUTH;
//                         cur = shiftInDir(cur, steps, EAST);
//                         ret.push_back(cur);
//                     }
//                 }
//                 break;
//             case WEST:
//                 scan_west(cur, top, d, midStoppingPoint, obstaclePoint);
//                 if(top)//scanned obstacle on top slider
//                 {
//                     if(midStoppingPoint > obstaclePoint)//convex corner up
//                     {
//                         d = NORTH;
//                     }
//                     if(midStoppingPoint < obstaclePoint)//concave corner down
//                     {
//                         d = SOUTH;
//                     }
//                 }
//                 else//scanned obstacle on bottom slider    
//                 {
//                     if(midStoppingPoint < obstaclePoint)//concave corner up
//                     {
//                         d = NORTH;
//                         top = true;
//                     }
//                     if(midStoppingPoint > obstaclePoint )//convex corner down
//                     {
//                         d = SOUTH;
//                     }
//                 }
//                 break;
//             case NORTH://jump north should be west?
//                 scan_east(cur, top, d, midStoppingPoint, obstaclePoint);
//                 break;
//             case SOUTH://jump south shoud be east?
//                 scan_west(cur, top, d, midStoppingPoint, obstaclePoint);
//                 break;
//             default:
//                 break;
//         }
//     }
//     return ret;
// }

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

uint32_t Scanner::test_scan(pad_id start, bool top, char c)
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

void Scanner::printEastScan(uint64_t i)
{
    std::string s = std::bitset<64>(i).to_string();
    std::reverse(s.begin(), s.end());
    std::cout<<"->"<<s<<'\n';
}

void Scanner::printWestScan(uint64_t i)
{
    std::string s = std::bitset<64>(i).to_string();
    std::reverse(s.begin(), s.end());
    std::cout<<s<<"<-"<<'\n';
}

// uint32_t Scanner::scan_west(pad_id origin, bool& top, direction& d, int& stop, int& turn)
// {
//     gridmap_slider slider = gridmap_slider::from_bittable(m_map, origin);
//     slider.adj_bytes(-7);
//     std::array<uint64_t, 3> neis;
//     uint32_t steps = 0;
//     //[0]middle
//     //[1]above
//     //[2]below
//     neis = slider.get_neighbours_64bit_le();
//     //get the comparing word based on if the obsticle is on top or bottom 
//     uint64_t comp = top ? neis[1] : neis[2];
//     uint64_t mid = ~neis[0];
//     printWestScan(mid);  
//     maskzero<WEST>(mid, (7-slider.width8_bits));
//     maskzero<WEST>(comp, (7-slider.width8_bits));
//     //XOR mask the starting bit in comp to 0 since we dont want to check it because :
//     //cur could be a turning point making the first bit in comp 1 and the adjacent bit
//     //in comp will always be 0 unless its a turning point
//     // comp ^ (1ULL << 63);
//     // std::cout<<std::bitset<64>(neis[0]>>slider.width8_bits)<<'\n'<<std::bitset<64>(comp>>slider.width8_bits)<<'\n'<<'\n';
//     std::cout<<"offset: "<<slider.width8_bits<<'\n';
//     printWestScan(mid);
//     printWestScan(comp);
//     if(comp || mid)
//     {
//         stop = std::countl_zero(mid);
//         turn = std::countl_zero(comp);
//         steps += std::min(stop, turn);
//         return steps - (7-slider.width8_bits) -1;
//     }
//     slider.adj_bytes(-7);
//     steps += 63-(7-slider.width8_bits)-1;
//     while (true)
//     {      
//         assert(false && "not implemented");  
//         //[0]middle
//         //[1]above
//         //[2]below
//         neis = slider.get_neighbours_64bit_le();
//         //get the comparing word based on if the obsticle is on top or bottom 
//         uint64_t comp = top ? neis[1] : neis[2];
//         uint64_t mid = ~neis[0];     
//         maskzero<WEST>(mid, slider.width8_bits);
//         maskzero<WEST>(comp, slider.width8_bits);
//         //mid:  flipped bits, 0 is traversable, if a wall is encountered comp will be > 0
//         //comp: wall is 0 and travasable is 1, if the wall turns neis[0] wii be > 0
//         if(comp || mid)
//         {
//         stop = std::countr_zero(mid);
//         turn = std::countr_zero(comp);
//         steps += std::min(stop, turn);
//         return steps-1;
//         }
//         //no turning points, keep scanning in same direction
//         slider.adj_bytes(8);
//         steps+=63;
//     }
// }