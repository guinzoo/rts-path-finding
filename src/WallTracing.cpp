
#include "WallTracing.hpp"
#include "Map.hpp"

using namespace nook;

namespace {
    const float kr2 = 1.08f;
    const int kMaxIter = 20;

    inline U32 getCell(int x, int y) {
        U32 cell = 0;
        cell |= map()->getCell(x - 1, y - 1)->walkable;
        cell |= map()->getCell(x, y - 1)->walkable << 1;
        cell |= map()->getCell(x, y)->walkable << 2;
        cell |= map()->getCell(x - 1, y)->walkable << 3;
        return cell;
    }

    inline float heuristics(vec2 p1, vec2 p2) {
        return (p1 - p2).length();
    }
}

WallTracing::WallTracing() {
    m_curCheck = 1;
    m_curRequest = 1;
    
    m_dummyObstacle.radius = 0.2f;
    
    m_size = map()->size();
    m_regionCount.x = ceilDiv(m_size.width, RegionSize);
    m_regionCount.y = ceilDiv(m_size.height, RegionSize);
    int rc = m_regionCount.x * m_regionCount.y;
    
    m_regions = memoryManager().allocOnStack<Region>(rc);
    for (int i = 0; i < rc; i++)
        new (m_regions + i) Region();
    
    const U32 maxQueue = 20;
    m_queue.init(maxQueue, memoryManager().allocOnStack<PriorityQueue<WallTracing::Next*, float>::Item>(maxQueue));
    
    List1<Corner*> corners;
    nook::Size hs = map()->size() / 2;
    
    auto pushCorner = [&](Coord c, vec2 n, bool o) {
        Corner* corner = m_cornerPool.alloc();
        corner->coord = c;
        corner->outer = o;
        corner->pos = vec2((int)c.x - hs.width, (int)c.y - hs.height);
        corner->normal = n;
        corner->right = corner->left = nullptr;
        corner->checked = m_curCheck;
        corner->request = m_curRequest;
        
        Coord rc = getRegion(corner);
        m_regions[rc.y * m_regionCount.x + rc.x].corners.push(corner);
        corners.push(corner);
    };
    
    for (int y = 1; y < m_size.height; y++)
        for (int x = 1; x < m_size.width; x++) {
            U32 cell = getCell(x, y);
            switch (cell) {
                case 1:  // ▜
                    pushCorner(Coord(x, y), vec2(-1.0f, -1.0f), false);
                    break;
                case 2:  // ▛
                    pushCorner(Coord(x, y), vec2(1.0f, -1.0f), false);
                    break;
                case 4:  // ▙
                    pushCorner(Coord(x, y), vec2(1.0f, 1.0f), false);
                    break;
                case 8:  // ▟
                    pushCorner(Coord(x, y), vec2(-1.0f, 1.0f), false);
                    break;
                case 5:  // ▚
                    pushCorner(Coord(x, y), vec2(-1.0f, -1.0f), false);
                    pushCorner(Coord(x, y), vec2(1.0f, 1.0f), false);
                    break;
                case 10: // ▞
                    pushCorner(Coord(x, y), vec2(1.0f, -1.0f), false);
                    pushCorner(Coord(x, y), vec2(-1.0f, 1.0f), false);
                    break;
                case 7:  // ┛
                    pushCorner(Coord(x, y), vec2(1.0f, -1.0f), true);
                    break;
                case 11: // ┗
                    pushCorner(Coord(x, y), vec2(-1.0f, -1.0f), true);
                    break;
                case 13: // ┏
                    pushCorner(Coord(x, y), vec2(-1.0f, 1.0f), true);
                    break;
                case 14: // ┓
                    pushCorner(Coord(x, y), vec2(1.0f, 1.0f), true);
                    break;
                default:
                    break;
            }
        }
    
    for (Corner* corner : corners) {
        int dx = 0;
        int dy = 0;
        int dir = 0;
        U32 wall = 0;
        
        if (corner->outer) {
            if (corner->normal.x == 1.0f) {
                if (corner->normal.y == 1.0f) {
                    dx = -1;
                    wall = 12;
                    dir = 2;
                }
                else {
                    dy = 1;
                    wall = 6;
                    dir = 8;
                }
            }
            else {
                if (corner->normal.y == 1.0f) {
                    dy = -1;
                    wall = 9;
                    dir = 4;
                }
                else {
                    dx = 1;
                    wall = 3;
                    dir = 1;
                }
            }
        }
        else { // inner
            if (corner->normal.x == 1.0f) {
                if (corner->normal.y == 1.0f) {
                    dy = 1;
                    wall = 6;
                    dir = 8;
                }
                else {
                    dx = 1;
                    wall = 3;
                    dir = 1;
                }
            }
            else {
                if (corner->normal.y == 1.0f) {
                    dx = -1;
                    wall = 12;
                    dir = 2;
                }
                else {
                    dy = -1;
                    wall = 9;
                    dir = 4;
                }
            }
        }
        
        Coord p(corner->coord.x + dx, corner->coord.y + dy);
        U32 cell = getCell(p.x, p.y);
        while (cell == wall) {
            p.x += dx;
            p.y += dy;
            cell = getCell(p.x, p.y);
        }
        
        Corner pc = getCorner(p, cell, dir);
        Coord prc = getRegion(&pc);
        Region& pr = m_regions[prc.y * m_regionCount.x + prc.x];
        
        for (Corner* cp : pr.corners) {
            if (*cp == pc) {
                corner->right = cp;
                cp->left = corner;
                break;
            }
        }
        
        Coord r = getRegion(corner);
        if (dx) {
            for (int x = r.x; x != prc.x; x += dx)
                m_regions[r.y * m_regionCount.x + x + dx].corners.push(corner);
        }
        else {
            for (int y = r.y; y != prc.y; y += dy)
                m_regions[(y + dy) * m_regionCount.x + r.x].corners.push(corner);
        }
    }
}

WallTracing::~WallTracing() {
    int rc = m_regionCount.x * m_regionCount.y;
    for (int i = 0; i < rc; i++)
        m_regions[i].~Region();
}

void WallTracing::addObstacle(Circle o) {
    nook::Size hs = map()->size() / 2;
    Point2 bl(o.pos.x - o.radius + hs.width, o.pos.y - o.radius + hs.height);
    Point2 tr(o.pos.x + o.radius + hs.width, o.pos.y + o.radius + hs.height);
    bl /= RegionSize;
    tr /= RegionSize;
    
    Obstacle* ob = m_obstaclePool.alloc();
    ob->pos = o.pos;
    ob->radius = o.radius;
    ob->checked = m_curCheck;
    ob->request = m_curRequest;
    
    for (int y = bl.y; y <= tr.y; y++)
        for (int x = bl.x; x <= tr.x; x++)
            m_regions[y * m_regionCount.x + x].obstacles.push(ob);
}

void WallTracing::removeObstacle(Circle o) {
    nook::Size hs = map()->size() / 2;
    Point2 bl(o.pos.x - o.radius + hs.width, o.pos.y - o.radius + hs.height);
    Point2 tr(o.pos.x + o.radius + hs.width, o.pos.y + o.radius + hs.height);
    bl /= RegionSize;
    tr /= RegionSize;
    
    Obstacle* ob = nullptr;
    Region& r = m_regions[bl.y * m_regionCount.x + bl.x];
    for (Obstacle* ro : r.obstacles)
        if (ro->pos == o.pos) {
            ob = ro;
            break;
        }
    ASSERT(ob);
    
    for (int y = bl.y; y <= tr.y; y++)
        for (int x = bl.x; x <= tr.x; x++)
            m_regions[y * m_regionCount.x + x].obstacles.remove(ob);
    m_obstaclePool.free(ob);
}

void WallTracing::find(vec2 start, vec2 end, float radius, Array<vec2>& path) {
    m_curCheck++;
    m_queue.clear();
    m_path.clear();
    m_nextPool.clear();
    m_curRadius = radius;
    m_end = end;
    m_iter = kMaxIter;
    
    m_curObstacle = getObstacle(start);
    m_lastObstacle = nullptr;
    m_endObstacle = findObstacle(end);
    if (m_endObstacle == &m_dummyObstacle) {
        m_lastObstacle = findObstacle(end, radius);
        if (m_lastObstacle != &m_dummyObstacle) {
            vec2 v = end - m_lastObstacle->pos;
            v *= 10.0f / v.length();
            m_lastOLine = m_lastObstacle->pos + v;
        }
    }
    
    m_best = nullptr;
    
    Next col = findCollision(start, end, true);
    if (col.type) {
        if (col.type == 2)
            col.obstacle->checked = m_curCheck;
        
        float h = heuristics(end, col.pos);
        float p = col.cost + h;
        
        Next* left = m_nextPool.alloc();
        left->type = col.type;
        left->dir = 1;
        left->last = col.last;
        left->cost = col.cost;
        left->pos = col.pos;
        left->from = nullptr;
        left->corner = col.corner;
        m_queue.insert(left, p);
        
        Next* right = m_nextPool.alloc();
        right->type = col.type;
        right->dir = 2;
        right->last = col.last;
        right->cost = col.cost;
        right->pos = col.pos;
        right->from = nullptr;
        right->corner = col.corner;
        m_queue.insert(right, p);
        
        m_best = left;
        m_bestH = h;
    }
    else
        m_path.push(end);
    
    while (m_queue.count()) {
        Next* r = m_queue.pop();
        if (r->last) {
            m_best = r;
            break;
        }
        if (!m_iter) {
            debugLog("max iter")
            break;
        }
        m_iter--;
        
        if (r->type == 1) {
            Corner* c = r->corner;
            vec2 cpos = c->pos + c->normal * radius;
            if (r->pos == cpos) {
                pushNextCorner(r, r->dir == 1 ? c->left : c->right);
                
                vec2 to = end - cpos;
                vec2 d = r->dir == 1 ? vec2(c->normal.y, -c->normal.x) : vec2(-c->normal.y, c->normal.x);
                if (c->outer && to.x * d.x >= 0.0f && to.y * d.y >= 0.0f) {
                    c->request = m_curRequest + 1;
                    c->left->request = m_curRequest + 1;
                    pushNextCollision(r);
                }
            }
            else {
                if (r->dir == 1) {
                    c->checked = m_curCheck - 1;
                    pushNextCorner(r, c);
                }
                else
                    pushNextCorner(r, c->right);
            }
        }
        else
            pushNextObstacle(r);
    }
    
    while (m_best) {
        m_path.push(m_best->pos);
        m_best = m_best->from;
    }
    m_path.push(start);
    
    List<vec2>::Node* node = m_path.root()->prev;
    for (int i = m_path.size() - 2; i > 0; i--) {
        vec2 from = node->elem;
        vec2 to = node->prev->prev->elem;
        float dist = (from - to).length();
        Next col = findCollision(from, to, true);
        if (col.cost < dist - 0.001f)
            node = node->prev;
        else
            m_path.remove(node->prev);
    }
    
    if (m_path.size() == 2 && m_path.first() == m_path.last())
        m_path.remove(m_path.root()->prev);
    
    for (vec2 p : m_path)
        path.push(p);
}

WallTracing::Corner WallTracing::getCorner(Coord coord, U32 cell, int dir) {
    Corner corner;
    corner.coord = coord;
    switch (cell) {
        case 1:  // ▜
            corner.normal = nook::vec2(-1.0f, -1.0f);
            corner.outer = false;
            break;
        case 2:  // ▛
            corner.normal = nook::vec2(1.0f, -1.0f);
            corner.outer = false;
            break;
        case 4:  // ▙
            corner.normal = nook::vec2(1.0f, 1.0f);
            corner.outer = false;
            break;
        case 8:  // ▟
            corner.normal = nook::vec2(-1.0f, 1.0f);
            corner.outer = false;
            break;
        case 5:  // ▚
            corner.normal = dir & 9 ? nook::vec2(-1.0f, -1.0f) : nook::vec2(1.0f, 1.0f);
            corner.outer = false;
            break;
        case 10: // ▞
            corner.normal = dir & 5 ? nook::vec2(-1.0f, 1.0f) : nook::vec2(1.0f, -1.0f);
            corner.outer = false;
            break;
        case 7:  // ┛
            corner.normal = nook::vec2(1.0f, -1.0f);
            corner.outer = true;
            break;
        case 11: // ┗
            corner.normal = nook::vec2(-1.0f, -1.0f);
            corner.outer = true;
            break;
        case 13: // ┏
            corner.normal = nook::vec2(-1.0f, 1.0f);
            corner.outer = true;
            break;
        case 14: // ┓
            corner.normal = nook::vec2(1.0f, 1.0f);
            corner.outer = true;
            break;
        default:
            break;
    }
    return corner;
}

WallTracing::Obstacle* WallTracing::getObstacle(vec2 pos) {
    nook::Size hs = map()->size() / 2;
    U32 rx = (U32)(pos.x + hs.width) / RegionSize;
    U32 ry = (U32)(pos.y + hs.height) / RegionSize;
    Region& r = m_regions[ry * m_regionCount.x + rx];
    
    for (Obstacle* ro : r.obstacles)
        if (ro->pos == pos)
            return ro;
    return &m_dummyObstacle;
}

WallTracing::Obstacle* WallTracing::findObstacle(vec2 pos) {
    nook::Size hs = map()->size() / 2;
    U32 rx = (U32)(pos.x + hs.width) / RegionSize;
    U32 ry = (U32)(pos.y + hs.height) / RegionSize;
    Region& r = m_regions[ry * m_regionCount.x + rx];
    
    for (Obstacle* ro : r.obstacles)
        if ((ro->pos - pos).length() <= ro->radius)
            return ro;
    return &m_dummyObstacle;
}

WallTracing::Obstacle* WallTracing::findObstacle(vec2 pos, float radius) {
    nook::Size hs = map()->size() / 2;
    U32 rx = (U32)(pos.x + hs.width) / RegionSize;
    U32 ry = (U32)(pos.y + hs.height) / RegionSize;
    Region& r = m_regions[ry * m_regionCount.x + rx];
    
    for (Obstacle* ro : r.obstacles) {
        if (ro == m_curObstacle)
            continue;
        if ((ro->pos - pos).length() <= (ro->radius + radius) * kr2)
            return ro;
    }
    return &m_dummyObstacle;
}

void WallTracing::pushNextCorner(Next* n, Corner* nc) {
    if (nc->checked == m_curCheck)
        return;
    
    vec2 cpos = nc->pos + nc->normal * m_curRadius;
    Next col = findCollision(n->pos, cpos, false);
    
    if (col.type) { // can be only obstacle
        if (col.obstacle->checked != m_curCheck) {
            col.obstacle->checked = m_curCheck;
            
            float h = heuristics(col.pos, m_end);
            float cost = n->cost + col.cost;
            
            Next* next = m_nextPool.alloc();
            next->type = 2;
            next->dir = n->dir;
            next->last = col.last;
            next->cost = cost;
            next->pos = col.pos;
            next->from = n;
            next->obstacle = col.obstacle;
            m_queue.insert(next, h + cost);
            
            if (h < m_bestH) {
                m_bestH = h;
                m_best = next;
            }
        }
    }
    else {
        nc->checked = m_curCheck;
        
        float h = heuristics(cpos, m_end);
        float cost = n->cost + col.cost;
        
        Next* next = m_nextPool.alloc();
        next->type = 1;
        next->dir = n->dir;
        next->last = false;
        next->cost = cost;
        next->pos = cpos;
        next->from = n;
        next->corner = nc;
        m_queue.insert(next, h + cost);
        
        if (h < m_bestH) {
            m_bestH = h;
            m_best = next;
        }
    }
}

void WallTracing::pushNextObstacle(Next* n) {
    Obstacle* o = n->obstacle;
    o->request = m_curRequest + 1;
    vec2 cp = o->pos - n->pos;
    vec2 ne = m_end - n->pos;
    
    float d2 = cp.length2();
    float r = o->radius + m_curRadius + 0.01f;
    float r2 = r * r;
    float tr = std::sqrt(r2 * (kr2 * kr2 - 1.0f));
    float s = (int)(n->dir & 2) - 1;
    vec2 dir;
    
    if (d2 <= r2) {
        float trr = tr / r * s;
        dir = vec2(cp.y * trr, -cp.x * trr);
    }
    else {
        float t2 = d2 - r2;
        float t = std::sqrt(t2);
        float rd2 = 1.0f / d2;
        vec2 tcp = cp * t;
        vec2 rcp = cp * r;
        float ux = (tcp.x + rcp.y * s) * rd2;
        float uy = (tcp.y - rcp.x * s) * rd2;
        dir = vec2(ux, uy) * (t + tr);
    }
    
    if (dir.cross(ne) * s <= 0.0f) {
        pushNextCollision(n);
    }
    else {
        vec2 to = n->pos + dir;
        bool last = false;
        if (o == m_lastObstacle)
            last = lineIntersect(n->pos, to, o->pos, m_lastOLine, to);
        
        Next col = findCollision(n->pos, to, true);
        if (col.type == 1) {
            U32 ch = n->dir == 1 ? col.corner->checked : col.corner->right->checked;
            if (ch == m_curCheck)
                return;
            
            float h = heuristics(m_end, col.pos);
            float cost = n->cost + col.cost;
            
            Next* next = m_nextPool.alloc();
            next->type = 1;
            next->dir = n->dir;
            next->last = col.last;
            next->cost = cost;
            next->pos = col.pos;
            next->from = n;
            next->obstacle = col.obstacle;
            m_queue.insert(next, h + cost);
            
            if (h < m_bestH) {
                m_bestH = h;
                m_best = next;
            }
        }
        else if (col.type == 2) {
            if (col.obstacle->checked == m_curCheck)
                return;
            col.obstacle->checked = m_curCheck;
            
            float h = heuristics(m_end, col.pos);
            float cost = n->cost + col.cost;
            
            Next* next = m_nextPool.alloc();
            next->type = 2;
            next->dir = n->dir;
            next->last = col.last;
            next->cost = cost;
            next->pos = col.pos;
            next->from = n;
            next->obstacle = col.obstacle;
            m_queue.insert(next, h + cost);
            
            if (h < m_bestH) {
                m_bestH = h;
                m_best = next;
            }
        }
        else {
            float h = heuristics(m_end, to);
            float cost = n->cost + col.cost;
            
            Next* next = m_nextPool.alloc();
            next->type = 2;
            next->dir = n->dir;
            next->last = last;
            next->cost = cost;
            next->pos = to;
            next->from = n;
            next->obstacle = o;
            m_queue.insert(next, h + cost);
            
            if (h < m_bestH) {
                m_bestH = h;
                m_best = next;
            }
        }
    }
}

void WallTracing::pushNextCollision(Next* n) {
    Next col = findCollision(n->pos, m_end, true);
    if (col.type == 1) {
        float h = heuristics(m_end, col.pos);
        float cost = n->cost + col.cost;
        float p = cost + h;
        
        if (col.corner->checked != m_curCheck) {
            Next* left = m_nextPool.alloc();
            left->type = 1;
            left->dir = 1;
            left->last = col.last;
            left->cost = cost;
            left->pos = col.pos;
            left->from = n;
            left->corner = col.corner;
            m_queue.insert(left, p);
            
            if (h < m_bestH) {
                m_best = left;
                m_bestH = h;
            }
        }
        if (col.corner->right->checked != m_curCheck) {
            Next* right = m_nextPool.alloc();
            right->type = 1;
            right->dir = 2;
            right->last = col.last;
            right->cost = cost;
            right->pos = col.pos;
            right->from = n;
            right->corner = col.corner;
            m_queue.insert(right, p);
            
            if (h < m_bestH) {
                m_best = right;
                m_bestH = h;
            }
        }
    }
    else if (col.type == 2) {
        if (col.obstacle->checked != m_curCheck) {
            float h = heuristics(m_end, col.pos);
            float cost = n->cost + col.cost;
            float p = cost + h;
            
            Next* left = m_nextPool.alloc();
            left->type = col.type;
            left->dir = 1;
            left->last = col.last;
            left->cost = cost;
            left->pos = col.pos;
            left->from = n;
            left->obstacle = col.obstacle;
            m_queue.insert(left, p);
            
            Next* right = m_nextPool.alloc();
            right->type = col.type;
            right->dir = 2;
            right->last = col.last;
            right->cost = cost;
            right->pos = col.pos;
            right->from = n;
            right->obstacle = col.obstacle;
            m_queue.insert(right, p);
            
            if (h < m_bestH) {
                m_best = left;
                m_bestH = h;
            }
        }
    }
    else {
        Next* next = m_nextPool.alloc();
        next->last = true;
        next->pos = m_end;
        next->from = n;
        m_queue.insert(next, n->cost + col.cost);
    }
}

WallTracing::Next WallTracing::findCollision(vec2 from, vec2 to, bool checkCorners) {
    m_curRequest++;
    m_curObstacle->request = m_curRequest;
    
    Next col;
    col.type = 0;
    col.pos = to;
    col.last = false;
    col.cost = (from - to).length2();
    vec2 dir = to - from;
    
    nook::Size hs = map()->size() / 2;
    Point2 bl, tr;
    if (from.x < to.x) {
        bl.x = from.x - m_curRadius + hs.width;
        tr.x = to.x + m_curRadius + hs.width;
    }
    else {
        bl.x = to.x - m_curRadius + hs.width;
        tr.x = from.x + m_curRadius + hs.width;
    }
    if (from.y < to.y) {
        bl.y = from.y - m_curRadius + hs.height;
        tr.y = to.y + m_curRadius + hs.height;
    }
    else {
        bl.y = to.y - m_curRadius + hs.height;
        tr.y = from.y + m_curRadius + hs.height;
    }
    bl /= RegionSize;
    tr /= RegionSize;
    
    if (checkCorners) {
        for (int y = bl.y; y <= tr.y; y++)
            for (int x = bl.x; x <= tr.x; x++) {
                Region& r = m_regions[y * m_regionCount.x + x];
                for (Corner* c : r.corners)
                    if (c->request != m_curRequest) {
                        c->request = m_curRequest;
                        
                        Corner* right = c->right;
                        vec2 l1 = c->pos + c->normal * m_curRadius;
                        vec2 l2 = right->pos + right->normal * m_curRadius;
                        vec2 p;
                        
                        if (from == l1 || from == l2) {
                            if (from == l2) {
                                right->request = m_curRequest;
                                c = right;
                            }
                            else
                                c->left->request = m_curRequest;
                            
                            if (c->outer) {
                                if (dir.x * c->normal.x < 0.0f && dir.y * c->normal.y < 0.0f) {
                                    col.type = 1;
                                    col.cost = 0.0f;
                                    col.pos = from;
                                    col.corner = c;
                                    return col;
                                }
                                else
                                    continue;
                            }
                            else {
                                if (dir.x * c->normal.x < 0.0f || dir.y * c->normal.y < 0.0f) {
                                    col.type = 1;
                                    col.cost = 0.0f;
                                    col.pos = from;
                                    col.corner = c;
                                    return col;
                                }
                                else
                                    continue;
                            }
                        }
                        
                        if (lineIntersect(from, to, l1, l2, p)) {
                            if (p == from) {
                                if (c->coord.x == right->coord.x) {
                                    if (c->normal.x * dir.x >= 0.0f)
                                        continue;
                                }
                                else {
                                    if (c->normal.y * dir.y >= 0.0f)
                                        continue;
                                }
                                col.type = 1;
                                col.cost = 0.0f;
                                col.pos = from;
                                col.corner = c;
                                return col;
                            }
                            
                            float dist2 = (p - from).length2();
                            if (dist2 < col.cost) {
                                col.type = 1;
                                col.cost = dist2;
                                col.pos = p;
                                col.corner = c;
                            }
                        }
                    }
            }
    }
    
    for (int y = bl.y; y <= tr.y; y++)
        for (int x = bl.x; x <= tr.x; x++) {
            Region& r = m_regions[y * m_regionCount.x + x];
            for (Obstacle* o : r.obstacles) {
                if (o->request != m_curRequest) {
                    o->request = m_curRequest;
                    
                    Circle c(o->pos, o->radius + m_curRadius);
                    vec2 p;
                    if (circleLineIntersectOutside(c, from, to, p)) {
                        if (p == from) {
                            vec2 vc = c.pos - from;
                            if (dir.dot(vc) <= 0.0f)
                                continue;
                            col.type = 2;
                            col.cost = 0.0f;
                            col.last = o == m_endObstacle;
                            col.pos = from;
                            col.obstacle = o;
                            return col;
                        }
                        
                        float dist2 = (p - from).length2();
                        if (dist2 < col.cost) {
                            col.type = 2;
                            col.cost = dist2;
                            col.pos = p;
                            col.obstacle = o;
                        }
                    }
                }
            }
        }
    
    col.cost = std::sqrt(col.cost);
    
    if (col.type == 2) {
        float rd = (col.obstacle->radius + m_curRadius) * (kr2 - 1.0f);
        if (col.cost > rd)
            col.pos += (from - col.pos) * (rd / col.cost);
        col.last = col.obstacle == m_endObstacle;
    }
    
    return col;
}
