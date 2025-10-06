//a colletion of definitions/utils used in rjps algorithm
#pragma once
#include <jps/jump/jump_point_online.h>
#include <bitset>
#include <algorithm>
#include <map>
#include <stack>
#include <string>


#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/pairing_heap.hpp>

// typedef typename boost::heap::fibonacci_heap<>::handle_type handle_t;
using namespace warthog::domain;
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
    pad_id      id = pad_id::none();
    direction   dir = direction::NONE;
    direction   quad_mask = direction::NONE;

    rjps_state(pad_id _v) : id(_v){};
    rjps_state(pad_id _v, direction _d) : id(_v), dir(_d){};
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
    const inline std::string get_key()
    {
        return std::string(std::to_string((uint64_t)state.id) +':'+ std::to_string(state.dir));
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

template <direction D>
static inline bool on_left_octant(std::pair<uint32_t, uint32_t> f, std::pair<uint32_t, uint32_t> t)
{
    static_assert(
    D == NORTHEAST || D == NORTHWEST || D == SOUTHEAST || D == SOUTHWEST,
    "D must be inter-cardinal.");
    int dx = std::abs((int)t.first - (int)f.first), dy = std::abs((int)t.second - (int)f.second);
    using ScanAttribute::Octants;
    if      constexpr(D == NORTHEAST || D == SOUTHWEST)
    {
        return(dy > dx);
    }
    else if constexpr(D == NORTHWEST || D == SOUTHEAST)
    {
        return(dx > dy);
    }
}

template <direction D>
constexpr Octants get_left_octant()
{
    static_assert(
	    D == NORTHEAST || D == NORTHWEST || D == SOUTHEAST || D == SOUTHWEST,
	    "D must be inter-cardinal.");
    using ScanAttribute::Octants;
    if      (D == NORTHEAST) return Octants::NNE;
    else if (D == NORTHWEST) return Octants::WNW;
    else if (D == SOUTHEAST) return Octants::ESE;
    else if (D == SOUTHWEST) return Octants::SSW;
}

template <direction D>
constexpr Octants get_right_octant()
{
    static_assert(
	    D == NORTHEAST || D == NORTHWEST || D == SOUTHEAST || D == SOUTHWEST,
	    "D must be inter-cardinal.");
    using ScanAttribute::Octants;
    if      (D == NORTHEAST) return Octants::ENE;
    else if (D == NORTHWEST) return Octants::NNW;
    else if (D == SOUTHEAST) return Octants::SSE;
    else if (D == SOUTHWEST) return Octants::WSW;
}

template<ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
constexpr inline direction get_subseq_dir()
{
    using ScanAttribute::Octants;
    if constexpr (O == ScanAttribute::CW)
    {
        if constexpr(Octant == NNW || Octant == NNE)
        {
            return EAST;
        }
        else if constexpr(Octant == ENE || Octant == ESE)
        {
            return SOUTH;
        }
        else if constexpr(Octant == SSE || Octant == SSW)
        {
            return WEST;
        }
        else if constexpr(Octant == WSW || Octant == WNW)
        {
            return NORTH;
        }
    }
    else
    {
        if constexpr(Octant == NNW || Octant == NNE)
        {
            return WEST;
        }
        else if constexpr(Octant == ENE || Octant == ESE)
        {
            return NORTH;
        }
        else if constexpr(Octant == SSE || Octant == SSW)
        {
            return EAST;
        }
        else if constexpr(Octant == WSW || Octant == WNW)
        {
            return SOUTH;
        }
    }
}

template<ScanAttribute::Octants Octant>
constexpr inline direction get_jps_dir()
{
    using ScanAttribute::Octants;
    if constexpr(Octant == NNW || Octant == NNE)
    {
       return NORTH;
    }
    else if constexpr(Octant == ENE || Octant == ESE)
    {
       return EAST;
    }
    else if constexpr(Octant == SSE || Octant == SSW)
    {
       return SOUTH;
    }
    else if constexpr(Octant == WSW || Octant == WNW)
    {
       return WEST;
    }
}

template<ScanAttribute::Orientation O, ScanAttribute::Octants Octant>
constexpr inline direction get_terminate_dir()
{
    using ScanAttribute::Octants;
    if constexpr (O == ScanAttribute::CW)
    {
        if constexpr(Octant == NNE || Octant == NNW)
        {
            return WEST;
        }
        else if constexpr(Octant == ENE || Octant == ESE)
        {
            return NORTH;
        }
        else if constexpr(Octant == SSE || Octant == SSW)
        {
            return EAST;
        }
        else if constexpr(Octant == WSW || Octant == WNW)
        {
            return SOUTH;
        }
    }
    else
    {
        if constexpr(Octant == NNE || Octant == NNW)
        {
            return EAST;
        }
        else if constexpr(Octant == ENE || Octant == ESE)
        {
            return SOUTH;
        }
        else if constexpr(Octant == SSE || Octant == SSW)
        {
            return WEST;
        }
        else if constexpr(Octant == WSW || Octant == WNW)
        {
            return NORTH;
        }
    }
}

//helper function to determine if a comp stride is the top or not for scanning base on scan direction and octant
template<ScanAttribute::Octants Octant>
inline bool get_init_scan_top(direction scan_dir)
{
    using ScanAttribute::Octants;
    if constexpr(Octant == NNE || Octant == ENE)        //northeast
    {
       return (scan_dir == EAST || scan_dir == WEST);
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
       return (scan_dir == NORTH || scan_dir == SOUTH);
    }
}

inline constexpr bool horizontally_bound(Octants O)
{
    return (O == Octants::NNE || O == Octants::NNW|| O == Octants::SSE|| O == Octants::SSW);
}


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

static inline int get_succ_sector(direction parent_s, direction jps, bool top)
{
    return ((int)top <<20 |(int) parent_s <<10 | (int)jps);
}

using namespace std;
static const std::unordered_map<int, direction>quad{
    {get_succ_sector(NORTHEAST, NORTH, true),  NORTHWEST},
    {get_succ_sector(NORTHEAST, NORTH, false), NORTHEAST},
    {get_succ_sector(NORTHEAST, EAST, true),  NORTHEAST},
    {get_succ_sector(NORTHEAST, EAST, false),  SOUTHEAST},

    {get_succ_sector(NORTHWEST, NORTH, true),  NORTHWEST},
    {get_succ_sector(NORTHWEST, NORTH, false), NORTHEAST},
    {get_succ_sector(NORTHWEST, WEST, true),  NORTHWEST},
    {get_succ_sector(NORTHWEST, WEST, false),  SOUTHWEST},

    {get_succ_sector(SOUTHEAST, SOUTH, true),  SOUTHWEST},
    {get_succ_sector(SOUTHEAST, SOUTH, false), SOUTHEAST},
    {get_succ_sector(SOUTHEAST, EAST, true),  NORTHEAST},
    {get_succ_sector(SOUTHEAST, EAST, false),  SOUTHEAST},

    {get_succ_sector(SOUTHWEST, SOUTH, true),  SOUTHWEST},
    {get_succ_sector(SOUTHWEST, SOUTH, false), SOUTHEAST},
    {get_succ_sector(SOUTHWEST, WEST, true),  NORTHWEST},
    {get_succ_sector(SOUTHWEST, WEST, false),  SOUTHWEST}
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

