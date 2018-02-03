
#pragma once

#include "AStar.hpp"
#include "JPS.hpp"
#include "JPSplus.hpp"
#include "WallTracing.hpp"

#include "visual/3D/RenderObject.hpp"

class PathFinder {
public:
    static PathFinder* s_instance;
    
    PathFinder();
    ~PathFinder();
    
    void findRough(nook::vec2 start, nook::vec2 end, nook::Array<nook::vec2>& path);
    void showPath(nook::Array<nook::vec2>& path);
    
    int index(int x, int y) { return y * m_size.width + x; }
    
    nook::U32 heuristic(Coord c1, Coord c2) {
//        int x = (int)c1.x - (int)c2.x;
//        int y = (int)c1.y - (int)c2.y;
//        return x * x + y * y;
        int dx = std::abs((int)c1.x - (int)c2.x);
        int dy = std::abs((int)c1.y - (int)c2.y);
        return nook::max2(dx, dy);
    }
    
    WallTracing* wallTracing() { return m_wallTracing; }
    
private:
    nook::Size m_size;
    AStar* m_astar;
    JPS* m_jps;
    JPSplus* m_jpsPlus;
    WallTracing* m_wallTracing;
    
    nook::RenderObject m_pathObject;
    nook::mat4 m_pathTransform;
};

inline PathFinder& pathFinder() {
    return *PathFinder::s_instance;
}
