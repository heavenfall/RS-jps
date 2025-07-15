//a colletion of definitions/utils used in rjps algorithm
#pragma once
#include <jps/jump/jump_point_online.h>
#include <bitset>
#include <algorithm>
#include <map>
#include <stack>

using namespace warthog::domain;
using namespace jps;
struct rjps_node
{
    pad_id id;
    rjps_node* parent;
    direction dir;
    direction quad_mask = NONE;
    
    uint32_t xbound; 
    uint32_t ybound;
    double gval = DBL_MAX;
    double hval;
    // bool closed = false;
    constexpr inline const size_t get_key(){return (size_t)dir << 32 | (unsigned int)id;};
    const inline bool quad_expanded(const direction& quad){return quad_mask & quad;};
    inline void close_quad(const direction& quad){quad_mask = (direction)(quad|quad_mask);};

    rjps_node(pad_id _i, rjps_node* _p, std::pair<uint32_t, uint32_t> bounds, direction _dir) : 
        id(_i), parent(_p), hval(0), dir(_dir), xbound(bounds.first), ybound(bounds.second){};    
    rjps_node(pad_id _i, rjps_node* _p, uint32_t _xb, uint32_t _yb, direction _dir) : 
        id(_i), parent(_p), hval(0), dir(_dir), xbound(_xb), ybound(_yb){};
    rjps_node(pad_id _i, rjps_node* _p) : id(_i), parent(_p){};
    rjps_node(){};
};

namespace ScanAttribute
{
enum Orientation
{
    CW,
    CCW
};
}

// auto dir_ind =std::countr_zero<uint8_t>(p_dir) - 4;

// NORTHEAST 0
// NORTHWEST 1
// SOUTHEAST 2
// SOUTHWEST 3
// 0:vert, 1:hori
static constexpr std::array<std::array<direction, 2>, 4>d_init_scan_CW{{
{SOUTH, EAST}, 
{NORTH, EAST}, 
{SOUTH, WEST}, 
{NORTH, WEST}}};
// NORTHEAST 0
// NORTHWEST 1
// SOUTHEAST 2
// SOUTHWEST 3
// 0:vert, 1:hori
static constexpr std::array<std::array<direction, 2>, 4>d_init_scan_CCW{{
{NORTH, WEST}, 
{SOUTH, WEST}, 
{NORTH, EAST},
{SOUTH, EAST}}};
// NORTHEAST 0
// NORTHWEST 1
// SOUTHEAST 2
// SOUTHWEST 3
// 0:CW, 1:CCW
// direction of jps scan based on scan quadrant and scan orientation
static constexpr std::array<std::array<direction, 2>, 4>d_jps{{
{EAST, NORTH}, 
{NORTH, WEST}, 
{SOUTH, EAST}, 
{WEST, SOUTH}}};
// NORTHEAST 0
// NORTHWEST 1
// SOUTHEAST 2
// SOUTHWEST 3
// 0:CW, 1:CCW
static constexpr std::array<std::array<direction, 2>, 4>d_scan{{
{SOUTH, WEST}, 
{EAST, SOUTH}, 
{WEST, NORTH}, 
{NORTH, EAST}}};

using namespace std;
static std::map<std::string, direction>quad{
    {to_string(NORTHEAST) + to_string(NORTH) + to_string(true),  NORTHWEST},
    {to_string(NORTHEAST) + to_string(NORTH) + to_string(false), NORTHEAST},
    {to_string(NORTHEAST) + to_string(EAST)  + to_string(true),  NORTHEAST},
    {to_string(NORTHEAST) + to_string(EAST) + to_string(false),  SOUTHEAST},

    {to_string(NORTHWEST) + to_string(NORTH) + to_string(true),  NORTHWEST},
    {to_string(NORTHWEST) + to_string(NORTH) + to_string(false), NORTHEAST},
    {to_string(NORTHWEST) + to_string(WEST)  + to_string(true),  NORTHWEST},
    {to_string(NORTHWEST) + to_string(WEST) + to_string(false),  SOUTHWEST},

    {to_string(SOUTHEAST) + to_string(SOUTH) + to_string(true),  SOUTHWEST},
    {to_string(SOUTHEAST) + to_string(SOUTH) + to_string(false), SOUTHEAST},
    {to_string(SOUTHEAST) + to_string(EAST)  + to_string(true),  NORTHEAST},
    {to_string(SOUTHEAST) + to_string(EAST) + to_string(false),  SOUTHEAST},

    {to_string(SOUTHWEST) + to_string(SOUTH) + to_string(true),  SOUTHWEST},
    {to_string(SOUTHWEST) + to_string(SOUTH) + to_string(false), SOUTHEAST},
    {to_string(SOUTHWEST) + to_string(WEST)  + to_string(true),  NORTHWEST},
    {to_string(SOUTHWEST) + to_string(WEST) + to_string(false),  SOUTHWEST}
};

// NORTHEAST 0
// NORTHWEST 1
// SOUTHEAST 2
// SOUTHWEST 3
// 0:x  1:y
static constexpr std::array<std::array<int, 2>, 4> adj{{
{1, -1},
{-1, -1},
{1, 1},
{-1, 1}}};

// NORTHEAST 0
// NORTHWEST 1
// SOUTHEAST 2
// SOUTHWEST 3
// 0:vert, 1:hori
static constexpr std::array<std::array<bool, 2>, 4> init_scan_top{{
{false, true},
{true, true},
{false, false},
{true, false}}};

//check if i is inbtween a and b, doesnt matter a or b is larger or smaller
static inline bool between2(uint32_t i, uint32_t a, uint32_t b)
{
    return(i <= std::max(a, b) && i >= std::min(a, b));
}

//mask all bit before the offest(inclusive) to 0, offset is 1-based
template<bool East>
static inline void maskzero(uint64_t &num, uint32_t offset)
{
    if constexpr(East) 
    {
        num &= ((~0ull) << offset);
    }
    else
    {
        num &= ((~0ull) >> offset);
    }
}

//return JPS_ID after mooving from a JPS_ID n moves in direction d
static inline pad_id shift_in_dir(pad_id id, uint32_t n_moves, direction d, gridmap::bittable m)
{
    switch (d)
    {        
    default:
    case direction::EAST:
        return pad_id(id.id + n_moves);
        break;
    case direction::WEST:
        return pad_id(id.id - n_moves);
        break;        
    case direction::NORTH:
        return pad_id(id.id - n_moves * m.width());
        break;
    case direction::SOUTH:
        return pad_id(id.id + n_moves * m.width());
        break;
    case direction::NORTHEAST:
        return pad_id((id.id - n_moves * m.width()) + n_moves);
        break;
    case direction::NORTHWEST:
        return pad_id((id.id - n_moves * m.width()) - n_moves);            
        break;
    case direction::SOUTHEAST:
        return pad_id((id.id + n_moves * m.width()) + n_moves);            
        break;
    case direction::SOUTHWEST:
        return pad_id((id.id + n_moves * m.width()) - n_moves);            
        break;
    }
}

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

//if d1 and d2 are in d set of: {E, W} OR {N, S}
static inline bool EW_or_NS(direction d1, direction d2)
{
    auto c = uint8_t{0};
    c |= d1; c |= d2;
    return d1 == d2 || c == 12 || c == 3;
}

//if d1 and d2 are in different d set of: {E, N}, {W, S}
static inline bool EN_diff_WS(direction d1, direction d2)
{
    uint8_t a = std::countr_zero(static_cast<uint8_t>(d1));
    uint8_t b = std::countr_zero(static_cast<uint8_t>(d2));
    return (std::max(a, b) - std::min(a, b)) != 2;
}

