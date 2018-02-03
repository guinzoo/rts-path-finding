
#pragma once

#include "Coord.hpp"

class WallTracing {
public:
    WallTracing();
    ~WallTracing();
    
    void addObstacle(nook::Circle o);
    void removeObstacle(nook::Circle o);
    
    void find(nook::vec2 start, nook::vec2 end, float radius, nook::Array<nook::vec2>& path);
    
private:
    static constexpr int RegionSize = 4;
    
    struct Corner {
        nook::U32 checked;
        nook::U32 request;
        
        nook::vec2 pos;
        nook::vec2 normal;
        Coord coord;
        bool outer;
        
        Corner* left;
        Corner* right;
        
        bool operator==(const Corner& c) { return coord == c.coord && normal == c.normal; }
    };
    
    struct Obstacle {
        nook::U32 checked;
        nook::U32 request;
        
        nook::vec2 pos;
        float radius;
    };
    
    struct Region {
        nook::List1<Corner*> corners;
        nook::List1<Obstacle*> obstacles;
    };
    
    struct Next {
        nook::U8 type; // 1-corner, 2-obstacle
        nook::U8 dir;  // 1-left, 2-right
        bool last;
        float cost;
        nook::vec2 pos;
        Next* from;
        union {
            Corner* corner;
            Obstacle* obstacle;
        };
    };
    
    Coord getRegion(Corner* c) {
        int rx = c->coord.x / RegionSize;
        int ry = c->coord.y / RegionSize;
        rx -= (c->coord.x % RegionSize == 0) & (c->outer ^ (c->normal.x < 0));
        ry -= (c->coord.y % RegionSize == 0) & (c->outer ^ (c->normal.y < 0));
        return Coord(rx, ry);
    }
    
    // dir: 1-left. 2-right, 4-up, 8-down
    Corner getCorner(Coord coord, nook::U32 cell, int dir);
    
    Obstacle* getObstacle(nook::vec2 pos);
    Obstacle* findObstacle(nook::vec2 pos);
    Obstacle* findObstacle(nook::vec2 pos, float radius);
    nook::vec2 getLeftObstacle(nook::vec2 from, nook::Circle o);
    nook::vec2 getRightObstacle(nook::vec2 from, nook::Circle o);
    void pushNextCorner(Next* n, Corner* nc);
    void pushNextObstacle(Next* n);
    void pushNextCollision(Next* n);
    
    Next findCollision(nook::vec2 from, nook::vec2 to, bool checkCorners);
    
    nook::Size m_size;
    nook::Point2 m_regionCount;
    Region* m_regions;
    nook::PagePool<Corner> m_cornerPool;
    nook::PagePool<Obstacle> m_obstaclePool;
    nook::PagePool<Next> m_nextPool;
    nook::PriorityQueue<Next*, float> m_queue;
    nook::List<nook::vec2> m_path;
    
    nook::U32 m_curCheck;
    nook::U32 m_curRequest;
    
    Obstacle m_dummyObstacle;
    Obstacle* m_curObstacle;
    Obstacle* m_endObstacle;
    Obstacle* m_lastObstacle;
    nook::vec2 m_lastOLine;
    
    Next* m_best;
    float m_bestH;
    float m_curRadius;
    int m_iter;
    nook::vec2 m_end;
};
