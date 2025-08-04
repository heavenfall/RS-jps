#pragma once
#include <rjps.h>
#include <Log.h>

using namespace jps;
using namespace warthog::domain;


struct scanResult
{
    uint32_t c =    {}; //zero count of the comp stride
    uint32_t m =    {}; //zero count of the mid stride
    bool top   =    {};
    bool on_concave {false};//if the scan terminated on a concave point
    direction d=    {};
};

class Scanner
{
public:
    Scanner(std::shared_ptr<Tracer> _tracer, jump::jump_point_online<>* _jps);
    ~Scanner(){};

    // returns true if initial scan should be east and west, false if north or south
    // @warning will shift start up or donw if starts on a corner of an obstacle
    bool init_scan_eastwest(pad_id& start, direction in_dir);
    // template<ScanAttribute::Orientation O>
    // direction init_scan_dir(pad_id start, direction p_dir);

    //returns a all convex points of a obstacle
    void scan_obstacle(pad_id start, std::vector<pad_id> &ret, direction dir, bool _top);
    //returns the first poi along obstacle edge
    pad_id scan_obstacle(pad_id start, scanResult &res);

    //scan an obstacle in CW or CCW orientation, returns the first point
    //which results in chang of orientation
    template<ScanAttribute::Orientation o>
    void scan(pad_id parent, pad_id start, pad_id &ret);

    template<SolverTraits ST>
    pad_id find_turning_point(pad_id start, scanResult &scan_res, direction terminate_d, uint32_t xbound, uint32_t ybound);

    template<bool East>
    uint32_t scan_hori(gridmap::bittable _map, pad_id start, scanResult &res);    

    std::vector<pad_id> test_scan_full(pad_id start, uint32_t bx, uint32_t by);
    uint32_t test_scan_single(pad_id start, bool top, char c);
    
    double vecangle(pad_id o, pad_id a, pad_id b);

    uint32_t m_bx;  //x coord of the bounding box
    uint32_t m_by;  //y coord of the bounding box
private:
    uint32_t scan_east(pad_id start, scanResult &res);
    uint32_t scan_west(pad_id start, scanResult &res);
    uint32_t scan_north(pad_id start, scanResult &res);
    uint32_t scan_south(pad_id start, scanResult &res);

    jump::jump_point_online<>* m_jps;
    std::shared_ptr<Tracer> m_tracer;    
    gridmap::bittable m_map  = {};
	gridmap::bittable m_rmap = {};
    
};

template<bool East>
uint32_t Scanner::scan_hori(gridmap::bittable _map, pad_id start, scanResult &res)
{
    gridmap_slider slider = gridmap_slider::from_bittable(_map, start);//.id?
    std::array<uint64_t, 3> neis;
    uint32_t steps = 0;
    //[0]middle
    //[1]above
    //[2]below
    if constexpr(!East) 
    {
        slider.adj_bytes(-7);
        slider.width8_bits = (7-slider.width8_bits);
    }
    neis = slider.get_neighbours_64bit_le();
    //get the comparing word based on if the obsticle is on top or bottom 
    uint64_t comp = res.top ? neis[1] : neis[2];
    uint64_t mid = ~neis[0];
    maskzero<East>(mid, slider.width8_bits);
    maskzero<East>(comp, slider.width8_bits);
    if constexpr(East)
    {
        std::cout<<"mid:    ";
        printEastScan(mid);
        std::cout<<"comp:   ";
        printEastScan(comp);
    }
    else
    {
        std::cout<<"mid:    ";
        printWestScan(mid);
        std::cout<<"comp:   ";
        printWestScan(comp);
    }
    if(comp || mid)
    {
        res.m = East? std::countr_zero(mid) : std::countl_zero(mid);
        res.c = East? std::countr_zero(comp) : std::countl_zero(comp);
        steps += std::min(res.m, res.c);
        return steps - slider.width8_bits -1;
    }
    steps += 63 - slider.width8_bits - 1;
    slider.adj_bytes( East? 7:-7);
    slider.width8_bits = 7;
    while (true)
    {
        neis = slider.get_neighbours_64bit_le();
        //get the comparing word based on if the obsticle is on top or bottom 
        comp = res.top ? neis[1] : neis[2];
        mid = ~neis[0];
        maskzero<East>(mid, slider.width8_bits);
        maskzero<East>(comp, slider.width8_bits);
        if constexpr(East)
        {
            std::cout<<"mid:    ";
            printEastScan(mid);
            std::cout<<"comp:   ";
            printEastScan(comp);
        }
        else
        {
            std::cout<<"mid:    ";
            printWestScan(mid);
            std::cout<<"comp:   ";
            printWestScan(comp);
        }
        if(comp || mid)
        {
            res.m = East? std::countr_zero(mid) : std::countl_zero(mid);
            res.c = East? std::countr_zero(comp) : std::countl_zero(comp);
            steps += std::min(res.m, res.c);
            return steps - slider.width8_bits;
        }
        slider.adj_bytes(East? 7 : -7);
        steps += 63 - slider.width8_bits - 1;
    }
}

template<ScanAttribute::Orientation o>
void Scanner::scan(pad_id parent, pad_id first, pad_id &ret)
{
    pad_id second;
    scanResult res;
    res.d = SOUTH;
    res.top = true;
    second = scan_obstacle(first, res);
    double angle = vecangle(parent, first, second);
    if constexpr (o == ScanAttribute::CCW) 
    {
        while (angle <= 0)
        {
            first = second;
            second = scan_obstacle(first, res);          
            angle = vecangle(parent, first, second);
        } 
    }
    else if constexpr (o == ScanAttribute::CW) 
    {
        while (angle >= 0)
        {
            first = second;
            second = scan_obstacle(first, res);
            angle = vecangle(parent, first, second);
        } 
    }      
    ret = first;
}

//Returns the first poi, which can be:
//1. pad_id::None, if the scan leaves the bounding space
//2. The first convex point which results in scanning in opposite direction
//3. If poi is concave, return the next first convext point by continuing scan
template<SolverTraits ST>
pad_id Scanner::find_turning_point(pad_id start, scanResult &scan_res, direction terminate_d, uint32_t xbound, uint32_t ybound)
{
    auto nextpos = pad_id{}, curpos = start;
    bool in_bound = true, east_or_north;
    scan_res.on_concave = false;
    direction tempd;
    // uint32_t dir_ind = std::countr_zero<uint8_t>(p_dir)-4;
    // scan_res.top = init_scan_top[dir_ind][(scan_dir == EAST || scan_dir == WEST)];
    uint32_t steps;
    while (in_bound)
    {
        assert(scan_res.d==EAST||scan_res.d==WEST||scan_res.d==NORTH||scan_res.d==SOUTH);
        east_or_north = (scan_res.d==EAST||scan_res.d==NORTH);
        tempd = scan_res.d;
        switch (scan_res.d)
        {            
        case EAST:
            steps = scan_east(curpos, scan_res);            
            break;
        case WEST:
            steps = scan_west(curpos, scan_res);
            break;
        case NORTH:
            steps = scan_north(curpos, scan_res);
            break;
        case SOUTH:
            steps = scan_south(curpos, scan_res);
            break;
        }
        nextpos = shift_in_dir(curpos, steps, scan_res.d, m_map);
        auto c = m_map.id_to_xy(curpos), n = m_map.id_to_xy(nextpos);
        if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(c, n, "green", "scanning");
        if (scan_res.d==EAST||scan_res.d==WEST)
        {
            if(between2(xbound, c.first, n.first)) 
            {
                // std::cout<<"overstepped x bound\n";
                return pad_id::none();
            }
        }
        else 
        {
            if(between2(ybound, c.second, n.second)) 
            {
                // std::cout<<"overstepped y bound\n";
                return pad_id::none();
            }
        }
        //refer to my beautiful drawing
        if(scan_res.c < scan_res.m)//convex corners
        {
            //shift 1 because the scan stops at a step before the poi
            curpos = shift_in_dir(nextpos, 1, scan_res.d, m_map);
            if(scan_res.top)         //4
            {  
                scan_res.d = east_or_north? dir_ccw(scan_res.d): dir_cw(scan_res.d);
            }
            else if(!scan_res.top)   //1
            {
                scan_res.d = east_or_north? dir_cw(scan_res.d): dir_ccw(scan_res.d);                
            }            

            if(EN_diff_WS(scan_res.d, tempd)) scan_res.top = !scan_res.top; //magic

            if (scan_res.d == terminate_d || scan_res.on_concave) return curpos;
            //curpos is currently on the turning point, shift 1 step before next scan
            else curpos = shift_in_dir(curpos, 1, scan_res.d, m_map);
        }
        else if (scan_res.c >= scan_res.m)//concave corners
        {
            if(scan_res.top)         //3
            {
                scan_res.d = east_or_north? dir_cw(scan_res.d): dir_ccw(scan_res.d);
            }
            else if(!scan_res.top)   //2
            {
                scan_res.d = east_or_north? dir_ccw(scan_res.d): dir_cw(scan_res.d);
            }
            curpos = nextpos;
            //if first poi is on a concave point, return the next convex point
            //"switches on" the condition
            scan_res.on_concave |= (scan_res.d == terminate_d);
            if(EN_diff_WS(scan_res.d, tempd)) scan_res.top = !scan_res.top; //magic
        }
    }
    return pad_id{};
}
