
#pragma once

#include "misc/Common.hpp"

struct Coord {
    nook::U16 x, y;
    
    Coord() = default;
    Coord(nook::U16 x, nook::U16 y) : x(x), y(y) {}
    bool operator==(Coord c) { return x == c.x && y == c.y; }
    bool operator!=(Coord c) { return x != c.x || y != c.y; }
    
    nook::Point2 point() const { return nook::Point2(x, y); }
};
