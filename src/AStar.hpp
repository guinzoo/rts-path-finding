
#pragma once

#include "Coord.hpp"

class AStar {
public:
    AStar();
    
    void find(Coord start, Coord end, nook::Array<nook::vec2>& path);
    
private:
    enum Direction {
        NONE = 0,
        NORTH = 1,
        SOUTH = 2,
        EAST = 4,
        WEST = 8,
        NORTHEAST = 16,
        NORTHWEST = 32,
        SOUTHEAST = 64,
        SOUTHWEST = 128
    };
    
    nook::Size m_size;
    bool* m_map;
    nook::U32 m_maxIter;
    nook::PriorityQueue<Coord, nook::U32> m_queue;
    nook::Array<Coord> m_cameFrom;
    nook::Array<nook::U32> m_cost;
};
