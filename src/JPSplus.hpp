
#pragma once

#include "Coord.hpp"

class JPSplus {
public:
    JPSplus();
    
    void update();
    void find(Coord start, Coord end, nook::Array<nook::vec2>& path);
    
private:
    struct JP {
        nook::U16 n;
        nook::U16 ne;
        nook::U16 e;
        nook::U16 se;
        nook::U16 s;
        nook::U16 sw;
        nook::U16 w;
        nook::U16 nw;
    };
    
    nook::U16 getJumpN(Coord from);
    nook::U16 getJumpE(Coord from);
    nook::U16 getJumpS(Coord from);
    nook::U16 getJumpW(Coord from);
    nook::U16 getJumpNE(Coord from);
    nook::U16 getJumpSE(Coord from);
    nook::U16 getJumpSW(Coord from);
    nook::U16 getJumpNW(Coord from);
    
    void jumpN(Coord from, Coord goal);
    void jumpE(Coord from, Coord goal);
    void jumpS(Coord from, Coord goal);
    void jumpW(Coord from, Coord goal);
    void jumpNE(Coord from, Coord goal);
    void jumpSE(Coord from, Coord goal);
    void jumpSW(Coord from, Coord goal);
    void jumpNW(Coord from, Coord goal);
    
    nook::Size m_size;
    bool* m_map;
    JP* m_jps;
    nook::PriorityQueue<Coord, nook::U32> m_queue;
    nook::Array<Coord> m_cameFrom;
    nook::Array<nook::U32> m_cost;
    
    Coord m_best;
    nook::U32 m_bestC;
    nook::U32 m_bestH;
};
