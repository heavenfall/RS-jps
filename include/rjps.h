//a colletion of definitions/utils used in rjps algorithm
#pragma once
#include <jps/jump/jump_point_online.h>
#include <jps/domain/rotate_gridmap.h>
#include <bitset>
#include <algorithm>
#include <map>
#include <stack>
#include <string>


#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/pairing_heap.hpp>

// typedef typename boost::heap::fibonacci_heap<>::handle_type handle_t;
using namespace warthog::domain;
using namespace jps::domain;
using namespace jps;

enum Domain
{
    Travasable,
    Obstacle
};

enum class SolverTraits
{
    Default,
    OutputToPosthoc
};

struct rjps_state
{
    grid_id      id = grid_id::none();
    direction_id   dir = {};
    // direction_id   quad_mask = {};

    rjps_state(grid_id _v) : id(_v){};
    rjps_state(grid_id _v, direction_id _d) : id(_v), dir(_d){};
    rjps_state(){};
};

struct search_node
{
    rjps_state      state;
    bool            closed = false;
    search_node*    parent = nullptr;
    double          gval = 0;
    double          hval = DBL_MAX;
    boost::heap::pairing_heap<search_node>::handle_type handle;
    const inline uint64_t get_key()
    {
        return static_cast<uint64_t>(state.id) + (static_cast<uint64_t>(state.dir) << 32);
    }
    search_node(rjps_state _state) : state(_state){};
    search_node(){};

    //WARNING: < is set to > for min-heap, not for comparisons
    bool operator<( search_node const& rhs ) const
    {
        return (gval + hval) > (rhs.gval + rhs.hval);
    }
};

struct cmp_min_f
{
    bool operator()(const search_node &a, const search_node &b) const
    {
        return (a.gval + a.hval) > (b.gval + b.hval);
    }
};

namespace ScanAttribute
{
enum Orientation
{
    CW,
    CCW
};

enum Octants 
{
    NNW,
    NNE,
    SSW, 
    SSE,
    ENE,
    ESE,
    WNW,
    WSW
};

template <direction_id D>
static inline bool on_left_octant(point f, point t)
{
    static_assert(
    D == NORTHEAST_ID || D == NORTHWEST_ID || D == SOUTHEAST_ID || D == SOUTHWEST_ID,
    "D must be inter-cardinal.");
    int dx = std::abs((int)t.x - (int)f.x), dy = std::abs((int)t.y - (int)f.y);
    using ScanAttribute::Octants;
    if      constexpr(D == NORTHEAST_ID || D == SOUTHWEST_ID)
    {
        return(dy > dx);
    }
    else if constexpr(D == NORTHWEST_ID || D == SOUTHEAST_ID)
    {
        return(dx > dy);
    }
}

template <direction_id D>
constexpr Octants get_left_octant()
{
    static_assert(
	    D == NORTHEAST_ID || D == NORTHWEST_ID || D == SOUTHEAST_ID || D == SOUTHWEST_ID,
	    "D must be inter-cardinal.");
    using ScanAttribute::Octants;
    if      (D == NORTHEAST_ID) return Octants::NNE;
    else if (D == NORTHWEST_ID) return Octants::WNW;
    else if (D == SOUTHEAST_ID) return Octants::ESE;
    else if (D == SOUTHWEST_ID) return Octants::SSW;
}

template <direction_id D>
constexpr Octants get_right_octant()
{
    static_assert(
	    D == NORTHEAST_ID || D == NORTHWEST_ID || D == SOUTHEAST_ID || D == SOUTHWEST_ID,
	    "D must be inter-cardinal.");
    using ScanAttribute::Octants;
    if      (D == NORTHEAST_ID) return Octants::ENE;
    else if (D == NORTHWEST_ID) return Octants::NNW;
    else if (D == SOUTHEAST_ID) return Octants::SSE;
    else if (D == SOUTHWEST_ID) return Octants::WSW;
}

template<ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
constexpr inline direction_id get_subseq_dir()
{
    using ScanAttribute::Octants;
    if constexpr (O == ScanAttribute::CW)
    {
        if constexpr(Octant == NNW || Octant == NNE)
        {
            return EAST_ID;
        }
        else if constexpr(Octant == ENE || Octant == ESE)
        {
            return SOUTH_ID;
        }
        else if constexpr(Octant == SSE || Octant == SSW)
        {
            return WEST_ID;
        }
        else if constexpr(Octant == WSW || Octant == WNW)
        {
            return NORTH_ID;
        }
    }
    else
    {
        if constexpr(Octant == NNW || Octant == NNE)
        {
            return WEST_ID;
        }
        else if constexpr(Octant == ENE || Octant == ESE)
        {
            return NORTH_ID;
        }
        else if constexpr(Octant == SSE || Octant == SSW)
        {
            return EAST_ID;
        }
        else if constexpr(Octant == WSW || Octant == WNW)
        {
            return SOUTH_ID;
        }
    }
}

template<ScanAttribute::Octants Octant>
constexpr inline direction_id get_jps_dir()
{
    using ScanAttribute::Octants;
    if constexpr(Octant == NNW || Octant == NNE)
    {
       return NORTH_ID;
    }
    else if constexpr(Octant == ENE || Octant == ESE)
    {
       return EAST_ID;
    }
    else if constexpr(Octant == SSE || Octant == SSW)
    {
       return SOUTH_ID;
    }
    else if constexpr(Octant == WSW || Octant == WNW)
    {
       return WEST_ID;
    }
}

template<ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
constexpr inline direction_id get_terminate_dir()
{
    using ScanAttribute::Octants;
    if constexpr (O == ScanAttribute::CW)
    {
        if constexpr(Octant == NNE || Octant == NNW)
        {
            return WEST_ID;
        }
        else if constexpr(Octant == ENE || Octant == ESE)
        {
            return NORTH_ID;
        }
        else if constexpr(Octant == SSE || Octant == SSW)
        {
            return EAST_ID;
        }
        else if constexpr(Octant == WSW || Octant == WNW)
        {
            return SOUTH_ID;
        }
    }
    else
    {
        if constexpr(Octant == NNE || Octant == NNW)
        {
            return EAST_ID;
        }
        else if constexpr(Octant == ENE || Octant == ESE)
        {
            return SOUTH_ID;
        }
        else if constexpr(Octant == SSE || Octant == SSW)
        {
            return WEST_ID;
        }
        else if constexpr(Octant == WSW || Octant == WNW)
        {
            return NORTH_ID;
        }
    }
}

//helper function to determine if a comp stride is the top or not for scanning base on scan direction and octant
template<ScanAttribute::Octants Octant>
inline bool get_init_scan_top(direction_id scan_dir)
{
    using ScanAttribute::Octants;
    if constexpr(Octant == NNE || Octant == ENE)        //NORTHEAST_ID
    {
       return (scan_dir == EAST_ID || scan_dir == WEST_ID);
    }
    else if constexpr(Octant == NNW || Octant == WNW)   //northwest
    {
       return true;
    }
    else if constexpr(Octant == ESE || Octant == SSE)   //southeast
    {
       return false;
    }
    else if constexpr(Octant == WSW || Octant == SSW)   //southwest
    {
       return (scan_dir == NORTH_ID || scan_dir == SOUTH_ID);
    }
}

inline constexpr bool horizontally_bound(Octants O)
{
    return (O == Octants::NNE || O == Octants::NNW|| O == Octants::SSE|| O == Octants::SSW);
}


}

// auto dir_ind =std::countr_zero<uint8_t>(p_dir) - 4;

// NORTHEAST_ID 0
// NORTHWEST_ID 1
// SOUTHEAST_ID 2
// SOUTHWEST_ID 3
// 0:vert, 1:hori
inline constexpr std::array<std::array<direction_id, 2>, 4>d_init_scan_CW{{
{SOUTH_ID, EAST_ID}, 
{NORTH_ID, EAST_ID}, 
{SOUTH_ID, WEST_ID}, 
{NORTH_ID, WEST_ID}
}};
// NORTHEAST_ID 0
// NORTHWEST_ID 1
// SOUTHEAST_ID 2
// SOUTHWEST_ID 3
// 0:vert, 1:hori
inline constexpr std::array<std::array<direction_id, 2>, 4>d_init_scan_CCW{{
{NORTH_ID, WEST_ID}, 
{SOUTH_ID, WEST_ID}, 
{NORTH_ID, EAST_ID},
{SOUTH_ID, EAST_ID}
}};
// NORTHEAST_ID 0
// NORTHWEST_ID 1
// SOUTHEAST_ID 2
// SOUTHWEST_ID 3
// 0:CW, 1:CCW
// direction of jps scan based on scan quadrant and scan orientation
inline constexpr std::array<std::array<direction_id, 2>, 4>d_jps{{
{EAST_ID, NORTH_ID}, 
{NORTH_ID, WEST_ID}, 
{SOUTH_ID, EAST_ID}, 
{WEST_ID, SOUTH_ID}
}};
// NORTHEAST_ID 0
// NORTHWEST_ID 1
// SOUTHEAST_ID 2
// SOUTHWEST_ID 3
// 0:CW, 1:CCW
inline constexpr std::array<std::array<direction_id, 2>, 4>d_scan{{
{SOUTH_ID, WEST_ID}, 
{EAST_ID, SOUTH_ID}, 
{WEST_ID, NORTH_ID}, 
{NORTH_ID, EAST_ID}
}};

inline uint32_t get_succ_sector(direction_id parent_s, direction_id jps, bool top)
{
    assert(is_intercardinal_id(parent_s) && (dir_intercardinal_hori(parent_s)==jps || dir_intercardinal_vert(parent_s)==jps));
    return (((uint32_t)parent_s - 4) << 2) |
           ((uint32_t)(dir_intercardinal_hori(parent_s) == jps) << 1) |
           (uint32_t)top;
    // return ((int)top <<20 |(int) parent_s <<10 | (int)jps);
}

// is like direction_id[d1=4][d2=2][top=2], d1 is parent_s, d2=is_hori, top is top
inline constexpr std::array<direction_id, 16>quad{{
    NORTHWEST_ID, // {get_succ_sector(NORTHEAST_ID, NORTH_ID, true),  NORTHWEST_ID},
    NORTHEAST_ID, // {get_succ_sector(NORTHEAST_ID, NORTH_ID, false), NORTHEAST_ID},
    NORTHEAST_ID, // {get_succ_sector(NORTHEAST_ID, EAST_ID, true),  NORTHEAST_ID},
    SOUTHEAST_ID, // {get_succ_sector(NORTHEAST_ID, EAST_ID, false),  SOUTHEAST_ID},

    NORTHWEST_ID, // {get_succ_sector(NORTHWEST_ID, NORTH_ID, true),  NORTHWEST_ID},
    NORTHEAST_ID, // {get_succ_sector(NORTHWEST_ID, NORTH_ID, false), NORTHEAST_ID},
    NORTHWEST_ID, // {get_succ_sector(NORTHWEST_ID, WEST_ID, true),  NORTHWEST_ID},
    SOUTHWEST_ID, // {get_succ_sector(NORTHWEST_ID, WEST_ID, false),  SOUTHWEST_ID},

    SOUTHWEST_ID, // {get_succ_sector(SOUTHEAST_ID, SOUTH_ID, true),  SOUTHWEST_ID},
    SOUTHEAST_ID, // {get_succ_sector(SOUTHEAST_ID, SOUTH_ID, false), SOUTHEAST_ID},
    NORTHEAST_ID, // {get_succ_sector(SOUTHEAST_ID, EAST_ID, true),  NORTHEAST_ID},
    SOUTHEAST_ID, // {get_succ_sector(SOUTHEAST_ID, EAST_ID, false),  SOUTHEAST_ID},

    SOUTHWEST_ID, // {get_succ_sector(SOUTHWEST_ID, SOUTH_ID, true),  SOUTHWEST_ID},
    SOUTHEAST_ID, // {get_succ_sector(SOUTHWEST_ID, SOUTH_ID, false), SOUTHEAST_ID},
    NORTHWEST_ID, // {get_succ_sector(SOUTHWEST_ID, WEST_ID, true),  NORTHWEST_ID},
    SOUTHWEST_ID, // {get_succ_sector(SOUTHWEST_ID, WEST_ID, false),  SOUTHWEST_ID}
}};

// NORTHEAST_ID 0
// NORTHWEST_ID 1
// SOUTHEAST_ID 2
// SOUTHWEST_ID 3
// 0:x  1:y
static constexpr std::array<point, 4> adj{{
{1u, (uint16_t)-1u},
{(uint16_t)-1u, (uint16_t)-1u},
{1u, 1u},
{(uint16_t)-1u, 1u}
}};

// NORTHEAST_ID 0
// NORTHWEST_ID 1
// SOUTHEAST_ID 2
// SOUTHWEST_ID 3
// 0:vert, 1:hori
static constexpr std::array<std::array<bool, 2>, 4> init_scan_top{{
{false, true},
{true, true},
{false, false},
{true, false}
}};

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
static inline grid_id shift_in_dir(grid_id id, uint32_t n_moves, direction_id d, gridmap::bittable m)
{
    return grid_id(id.id + warthog::grid::dir_id_adj(d, m.width()) * n_moves);
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
direction_id rotate_eighth(direction_id in_dir)
{
    static_assert(O == ScanAttribute::CCW || O == ScanAttribute::CW, "O must be CW|CCW");
    if constexpr (O == ScanAttribute::CCW)
    {
        return warthog::grid::dir_id_ccw45(in_dir);
    }
    else // if constexpr (O == ScanAttribute::CCW)
    {
        return warthog::grid::dir_id_cw45(in_dir);
    }
}

// //if d1 and d2 are in d set of: {E, W} OR {N, S}
// static inline bool EW_or_NS(direction_id d1, direction_id d2)
// {
//     auto c = uint8_t{0};
//     c |= d1; c |= d2;
//     return d1 == d2 || c == 12 || c == 3;
// }

//if d1 and d2 are in different d set of: {E, N}, {W, S}
static inline bool EN_diff_WS(direction_id d1, direction_id d2)
{
    uint8_t a = static_cast<uint8_t>(d1);
    uint8_t b = static_cast<uint8_t>(d2);
    return (std::max(a, b) - std::min(a, b)) != 2;
}

