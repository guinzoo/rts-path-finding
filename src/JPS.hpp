
#pragma once

#include "Coord.hpp"

class JPS {
public:
    JPS();
    void find(Coord start, Coord end, nook::Array<nook::vec2>& path);
    
private:
    int jumpN(Coord from, Coord goal);
    int jumpS(Coord from, Coord goal);
    int jumpW(Coord from, Coord goal);
    int jumpE(Coord from, Coord goal);
    void jumpNW(Coord from, Coord goal);
    void jumpNE(Coord from, Coord goal);
    void jumpSW(Coord from, Coord goal);
    void jumpSE(Coord from, Coord goal);
    
    nook::Size m_size;
    bool* m_map;
    nook::PriorityQueue<Coord, nook::U32> m_queue;
    nook::Array<Coord> m_cameFrom;
    nook::Array<nook::U32> m_cost;
    
    Coord m_best;
    nook::U32 m_bestC;
    nook::U32 m_bestH;
};
