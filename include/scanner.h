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
    direction_id d=    {};
};

class Scanner
{
public:
    Scanner(std::shared_ptr<Tracer> _tracer, jump::jump_point_online* _jps, gridmap_rotate_table_convs _map);
    ~Scanner(){};

    // returns true if initial scan should be east and west, false if north or south
    // @warning will shift start up or donw if starts on a corner of an obstacle
    bool init_scan_eastwest(grid_id& start, direction_id in_dir);
    // template<ScanAttribute::Orientation O>
    // direction_id init_scan_dir(grid_id start, direction_id p_dir);

    //returns a all convex points of a obstacle
    void scan_obstacle(grid_id start, std::vector<grid_id> &ret, direction_id dir, bool _top);
    //returns the first poi along obstacle edge
    grid_id scan_obstacle(grid_id start, scanResult &res);

    //scan an obstacle in CW or CCW orientation, returns the first point
    //which results in chang of orientation
    template<ScanAttribute::Orientation o>
    void scan(grid_id parent, grid_id start, grid_id &ret);

    template<SolverTraits ST>
    grid_id find_turning_point(grid_id start, scanResult &scan_res, direction_id terminate_d, uint32_t xbound, uint32_t ybound);

    template<bool East>
    uint32_t scan_hori(gridmap::bittable _map, grid_id start, scanResult &res);    

    std::vector<grid_id> test_scan_full(grid_id start, uint32_t bx, uint32_t by);
    uint32_t test_scan_single(grid_id start, bool top, char c);
    
    // double vecangle(grid_id o, grid_id a, grid_id b);
    static constexpr int32_t cross(point o, point a, point b) ///< use with is_cw, is_ccw, is_colin
    {
        auto da = point_signed_diff(o, a);
        auto db = point_signed_diff(o, a);
        // x1y2 - x2y1
        return db.first * da.second - da.first * db.second;
    }
    static constexpr bool is_cw(int32_t crs) ///< cross result a to b around o is cw
    {
        return crs > 0;
    }
    static constexpr bool is_ccw(int32_t crs) ///< cross result a to b around o is ccw
    {
        return crs < 0;
    }
    static constexpr bool is_colin(int32_t crs) ///< cross result a, b, and o are on same line (colin)
    {
        return crs == 0;
    }

    uint32_t m_bx;  //x coord of the bounding box
    uint32_t m_by;  //y coord of the bounding box
private:
    uint32_t scan_east(grid_id start, scanResult &res);
    uint32_t scan_west(grid_id start, scanResult &res);
    uint32_t scan_north(grid_id start, scanResult &res);
    uint32_t scan_south(grid_id start, scanResult &res);

    jump::jump_point_online* m_jps;
    std::shared_ptr<Tracer> m_tracer;
    gridmap_rotate_table_convs m_map;
};

template<bool East>
uint32_t Scanner::scan_hori(gridmap::bittable _map, grid_id start, scanResult &res)
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
    // if constexpr(East)
    // {
    //     std::cout<<"mid:    ";
    //     printEastScan(mid);
    //     std::cout<<"comp:   ";
    //     printEastScan(comp);
    // }
    // else
    // {
    //     std::cout<<"mid:    ";
    //     printWestScan(mid);
    //     std::cout<<"comp:   ";
    //     printWestScan(comp);
    // }
    if(comp || mid)
    {
        res.m = East? std::countr_zero(mid) : std::countl_zero(mid);
        res.c = East? std::countr_zero(comp) : std::countl_zero(comp);
        steps += std::min(res.m, res.c);
        assert(steps > 0);
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
        // if constexpr(East)
        // {
        //     std::cout<<"mid:    ";
        //     printEastScan(mid);
        //     std::cout<<"comp:   ";
        //     printEastScan(comp);
        // }
        // else
        // {
        //     std::cout<<"mid:    ";
        //     printWestScan(mid);
        //     std::cout<<"comp:   ";
        //     printWestScan(comp);
        // }
        if(comp || mid)
        {
            res.m = East? std::countr_zero(mid) : std::countl_zero(mid);
            res.c = East? std::countr_zero(comp) : std::countl_zero(comp);
            steps += std::min(res.m, res.c);
            assert(steps > 0);
            return steps - slider.width8_bits;
        }
        slider.adj_bytes(East? 7 : -7);
        steps += 63 - slider.width8_bits;
    }
}

template<ScanAttribute::Orientation o>
void Scanner::scan(grid_id parent, grid_id first, grid_id &ret)
{
    grid_id second;
    scanResult res;
    res.d = SOUTH_ID;
    res.top = true;
    second = scan_obstacle(first, res);
    point po = m_map.id_to_point(parent);
    point pa = m_map.id_to_point(first);
    point pb = m_map.id_to_point(second);
    if constexpr (o == ScanAttribute::CCW) 
    {
        while (is_ccw(cross(po, pa, pb)))
        {
            first = second;
            second = scan_obstacle(first, res);
            pa = pb;
            pb = m_map.id_to_point(second);
        }
    }
    else if constexpr (o == ScanAttribute::CW)
    {
        while (is_cw(cross(po, pa, pb)))
        {
            first = second;
            second = scan_obstacle(first, res);
            pa = pb;
            pb = m_map.id_to_point(second);
        }
    }
    ret = first;
}

//Returns the first poi, which can be:
//1. grid_id::None, if the scan leaves the bounding space
//2. The first convex point which results in scanning in opposite direction
//3. If poi is concave, return the next first convext point by continuing scan
template<SolverTraits ST>
grid_id Scanner::find_turning_point(grid_id start, scanResult &scan_res, direction_id terminate_d, uint32_t xbound, uint32_t ybound)
{
    auto nextpos = grid_id{}, curpos = start;
    bool in_bound = true, east_or_north;
    scan_res.on_concave = false;
    direction_id tempd;
    // uint32_t dir_ind = std::countr_zero<uint8_t>(p_dir)-4;
    // scan_res.top = init_scan_top[dir_ind][(scan_dir == EAST_ID || scan_dir == WEST_ID)];
    uint32_t steps;
    while (in_bound)
    {
        assert(scan_res.d==EAST_ID||scan_res.d==WEST_ID||scan_res.d==NORTH_ID||scan_res.d==SOUTH_ID);
        east_or_north = (scan_res.d==EAST_ID||scan_res.d==NORTH_ID);
        tempd = scan_res.d;
        switch (scan_res.d)
        {            
        case EAST_ID:
            steps = scan_east(curpos, scan_res);            
            break;
        case WEST_ID:
            steps = scan_west(curpos, scan_res);
            break;
        case NORTH_ID:
            steps = scan_north(curpos, scan_res);
            break;
        case SOUTH_ID:
            steps = scan_south(curpos, scan_res);
            break;
        }
        nextpos = shift_in_dir(curpos, steps, scan_res.d, m_map.table());
        auto c = m_map.id_to_point(curpos), n = m_map.id_to_point(nextpos);
        // if constexpr(ST == SolverTraits::OutputToPosthoc) m_tracer->trace_ray(c, n, "green", "scanning");
        if (scan_res.d==EAST_ID||scan_res.d==WEST_ID)
        {
            if(between2(xbound, c.x, n.x)) 
            {
                // std::cout<<"overstepped x bound\n";
                return grid_id::none();
            }
        }
        else 
        {
            if(between2(ybound, c.y, n.y)) 
            {
                // std::cout<<"overstepped y bound\n";
                return grid_id::none();
            }
        }
        //refer to my beautiful drawing
        if(scan_res.c < scan_res.m)//convex corners
        {
            //shift 1 because the scan stops at a step before the poi
            curpos = shift_in_dir(nextpos, 1, scan_res.d, m_map.table());
            if(scan_res.top)         //4
            {  
                scan_res.d = east_or_north? dir_id_ccw90(scan_res.d): dir_id_cw90(scan_res.d);
            }
            else if(!scan_res.top)   //1
            {
                scan_res.d = east_or_north? dir_id_cw90(scan_res.d): dir_id_ccw90(scan_res.d);                
            }            

            if(EN_diff_WS(scan_res.d, tempd)) scan_res.top = !scan_res.top; //magic

            if (scan_res.d == terminate_d || scan_res.on_concave) return curpos;
            //curpos is currently on the turning point, shift 1 step before next scan
            else curpos = shift_in_dir(curpos, 1, scan_res.d, m_map.table());
        }
        else if (scan_res.c >= scan_res.m)//concave corners
        {
            if(scan_res.top)         //3
            {
                scan_res.d = east_or_north? dir_id_cw90(scan_res.d): dir_id_ccw90(scan_res.d);
            }
            else if(!scan_res.top)   //2
            {
                scan_res.d = east_or_north? dir_id_ccw90(scan_res.d): dir_id_cw90(scan_res.d);
            }
            curpos = nextpos;
            //if first poi is on a concave point, return the next convex point
            //"switches on" the condition
            scan_res.on_concave |= (scan_res.d == terminate_d);
            if(EN_diff_WS(scan_res.d, tempd)) scan_res.top = !scan_res.top; //magic
        }
    }
    return grid_id{};
}
