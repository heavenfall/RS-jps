#pragma once
#include <rjps.h>
#include <bitset>
#include <algorithm>
#include <Log.h>

using namespace jps;
using namespace warthog::domain;


struct scanResult
{
    uint32_t c = {}; //zero count of the comp stride
    uint32_t m = {}; //zero count of the mid stride
    bool top   = {};
    direction d= {};
};


static void printEastScan(uint64_t i)
{
    std::string s = std::bitset<64>(i).to_string();
    std::reverse(s.begin(), s.end());
    std::cout<<"->"<<s<<'\n';
}
static void printWestScan(uint64_t i)
{
    std::string s = std::bitset<64>(i).to_string();
    std::reverse(s.begin(), s.end());
    std::cout<<s<<"<-"<<'\n';
}

//rotates a direction by 1/8 in CW or CCW
template<ScanAttribute::Orientation O>
direction rotate_eighth(direction in_dir)
{
    auto d = direction{};
    if constexpr (O == ScanAttribute::CCW)
    {
        switch (in_dir)
        {
        case NORTHEAST:
            d = NORTH;
            break;
        case NORTHWEST:
            d = WEST;
            break;
        case SOUTHEAST:
            d = EAST;
            break;
        case SOUTHWEST:
            d = SOUTH;
            break;
        }
    }
    else if constexpr (O == ScanAttribute::CW)
    {
        switch (in_dir)
        {
        case NORTHEAST:
            d = EAST;
            break;
        case NORTHWEST:
            d = NORTH;
            break;
        case SOUTHEAST:
            d = SOUTH;
            break;
        case SOUTHWEST:
            d = WEST;
            break;
        }
    }
    return d;
}

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

    pad_id find_turning_point(pad_id start, scanResult scan_res, direction terminate_d, uint32_t xbound, uint32_t ybound);

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
    
    std::cout<<"offset: "<<slider.width8_bits<<'\n';
    if constexpr(East)
    {
        maskzero<East>(mid, slider.width8_bits);
        maskzero<East>(comp, slider.width8_bits);
        // std::cout<<"mid:    ";
        // printEastScan(mid);
        // std::cout<<"comp:   ";
        // printEastScan(comp);
    }
    else
    {
        maskzero<East>(mid, slider.width8_bits);
        maskzero<East>(comp, slider.width8_bits);
        // std::cout<<"mid:    ";
        // printWestScan(mid);
        // std::cout<<"comp:   ";
        // printWestScan(comp);
    }
    if(comp || mid)
    {
        res.m = East? std::countr_zero(mid) : std::countl_zero(mid);
        res.c = East? std::countr_zero(comp) : std::countl_zero(comp);
        steps += std::min(res.m, res.c);
        return steps - slider.width8_bits -1;
    }
    slider.adj_bytes( East? 7:-7);
    steps += 63-slider.width8_bits-1;
    while (true)
    {
    assert(false && "not implemented");
    }    
    return 0;
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
