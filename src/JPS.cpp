
#include "JPS.hpp"
#include "PathFinder.hpp"
#include "Map.hpp"

using namespace nook;

JPS::JPS() {
    m_size = map()->size();
    
    int count = m_size.width * m_size.height;
    int queueSize = m_size.width;
    m_map = memoryManager().allocOnStack<bool>(count);
    m_queue.init(queueSize, memoryManager().allocOnStack<PriorityQueue<Coord, U32>::Item>(queueSize));
    m_cameFrom.init(count, memoryManager().allocOnStack<Coord>(count));
    m_cost.init(count, memoryManager().allocOnStack<U32>(count));
    m_cameFrom.setCount(count);
    m_cost.setCount(count);
    
    for (int y = 0; y < m_size.height; y++)
        for (int x = 0; x < m_size.width; x++)
            m_map[pathFinder().index(x, y)] = map()->getCell(x, y)->walkable;
}

void JPS::find(Coord start, Coord end, Array<vec2>& path) {
    m_queue.clear();
    std::memset(m_cost.buf(), 0xff, m_cost.count() * sizeof(int));
    
    if (start == end) {
        path.push(map()->getPos(start.point()));
        return;
    }
    
    int startIdx = pathFinder().index(start.x, start.y);
    m_cameFrom[startIdx] = start;
    m_cost[startIdx] = 0;
    
    m_best = start;
    m_bestC = 0;
    m_bestH = pathFinder().heuristic(start, end);
    int iter = 0;
    
    jumpN(start, end);
    jumpS(start, end);
    jumpW(start, end);
    jumpE(start, end);
    jumpNW(start, end);
    jumpNE(start, end);
    jumpSW(start, end);
    jumpSE(start, end);
    
    while (m_queue.count()) {
        Coord cur = m_queue.pop();
        if (cur == end) {
            m_best = end;
            break;
        }
        
        int ci = pathFinder().index(cur.x, cur.y);
        iter++;
        
        Coord from = m_cameFrom[ci];
        int bi = ci - m_size.width;
        int ti = ci + m_size.width;
        
        if (cur.y == from.y) {
            if (cur.x > from.x) {
                if (m_map[bi] && !m_map[bi - 1]) {
                    jumpS(cur, end);
                    jumpSE(cur, end);
                }
                if (m_map[ti] && !m_map[ti - 1]) {
                    jumpN(cur, end);
                    jumpNE(cur, end);
                }
                jumpE(cur, end);
            }
            else {
                if (m_map[bi] && !m_map[bi + 1]) {
                    jumpS(cur, end);
                    jumpSW(cur, end);
                }
                if (m_map[ti] && !m_map[ti + 1]) {
                    jumpN(cur, end);
                    jumpNW(cur, end);
                }
                jumpW(cur, end);
            }
        }
        else if (cur.y < from.y) {
            if (cur.x == from.x) {
                if (m_map[ci - 1] && !m_map[ti - 1]) {
                    jumpW(cur, end);
                    jumpSW(cur, end);
                }
                if (m_map[ci + 1] && !m_map[ti + 1]) {
                    jumpE(cur, end);
                    jumpSE(cur, end);
                }
                jumpS(cur, end);
            }
            else if (cur.x > from.x)
                jumpSE(cur, end);
            else
                jumpSW(cur, end);
        }
        else { // cur.y > from.y
            if (cur.x == from.x) {
                if (m_map[ci - 1] && !m_map[bi - 1]) {
                    jumpW(cur, end);
                    jumpNW(cur, end);
                }
                if (m_map[ci + 1] && !m_map[bi + 1]) {
                    jumpE(cur, end);
                    jumpNE(cur, end);
                }
                jumpN(cur, end);
            }
            else if (cur.x > from.x)
                jumpNE(cur, end);
            else
                jumpNW(cur, end);
        }
    }
    
    // Create Path
    Coord c = m_best;
    while (c != start) {
        path.push(map()->getPos(c.point()));
        c = m_cameFrom[pathFinder().index(c.x, c.y)];
    }
    path.push(map()->getPos(start.point()));
}

int JPS::jumpN(Coord from, Coord goal) {
    int dist = 0;
    int fromi = pathFinder().index(from.x, from.y);
    int curi = fromi;
    int nexti = curi + m_size.width;
    Coord next(from.x, from.y + 1);
    
    while (true) {
        if (!m_map[nexti])
            break;
        
        dist++;
        U32 cost = m_cost[fromi] + dist;
        
        if (next == goal) {
            if (cost < m_cost[nexti]) {
                m_queue.insert(next, cost);
                m_cameFrom[nexti] = from;
                m_cost[nexti] = cost;
            }
            return dist;
        }
        
        U32 h = pathFinder().heuristic(next, goal);
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestH = h;
            m_bestC = cost;
            m_best = next;
            m_cameFrom[nexti] = from;
        }
        
        if ((m_map[nexti + 1] && !m_map[curi + 1]) || (m_map[nexti - 1] && !m_map[curi - 1])) {
            if (cost < m_cost[nexti]) {
                m_queue.insert(next, h + cost);
                m_cameFrom[nexti] = from;
                m_cost[nexti] = cost;
            }
            return dist;
        }
        
        curi = nexti;
        nexti += m_size.width;
        next.y++;
    }
    return dist;
}

int JPS::jumpE(Coord from, Coord goal) {
    int dist = 0;
    int fromi = pathFinder().index(from.x, from.y);
    int curi = fromi;
    int nexti = curi + 1;
    Coord next(from.x + 1, from.y);
    
    while (true) {
        if (!m_map[nexti])
            break;
        
        dist++;
        U32 cost = m_cost[fromi] + dist;
        
        if (next == goal) {
            if (cost < m_cost[nexti]) {
                m_queue.insert(next, cost);
                m_cameFrom[nexti] = from;
                m_cost[nexti] = cost;
            }
            return dist;
        }
        
        U32 h = pathFinder().heuristic(next, goal);
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestH = h;
            m_bestC = cost;
            m_best = next;
            m_cameFrom[nexti] = from;
        }
        
        if ((m_map[nexti + m_size.width] && !m_map[curi + m_size.width]) ||
            (m_map[nexti - m_size.width] && !m_map[curi - m_size.width])) {
            if (cost < m_cost[nexti]) {
                m_queue.insert(next, h + cost);
                m_cameFrom[nexti] = from;
                m_cost[nexti] = cost;
            }
            return dist;
        }
        
        curi = nexti;
        nexti++;
        next.x++;
    }
    return dist;
}

int JPS::jumpS(Coord from, Coord goal) {
    int dist = 0;
    int fromi = pathFinder().index(from.x, from.y);
    int curi = fromi;
    int nexti = curi - m_size.width;
    Coord next(from.x, from.y - 1);
    
    while (true) {
        if (!m_map[nexti])
            break;
        
        dist++;
        U32 cost = m_cost[fromi] + dist;
        
        if (next == goal) {
            if (cost < m_cost[nexti]) {
                m_queue.insert(next, cost);
                m_cameFrom[nexti] = from;
                m_cost[nexti] = cost;
            }
            return dist;
        }
        
        U32 h = pathFinder().heuristic(next, goal);
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestH = h;
            m_bestC = cost;
            m_best = next;
            m_cameFrom[nexti] = from;
        }
        
        if ((m_map[nexti + 1] && !m_map[curi + 1]) || (m_map[nexti - 1] && !m_map[curi - 1])) {
            if (cost < m_cost[nexti]) {
                m_queue.insert(next, h + cost);
                m_cameFrom[nexti] = from;
                m_cost[nexti] = cost;
            }
            return dist;
        }
        
        curi = nexti;
        nexti -= m_size.width;
        next.y--;
    }
    return dist;
}

int JPS::jumpW(Coord from, Coord goal) {
    int dist = 0;
    int fromi = pathFinder().index(from.x, from.y);
    int curi = fromi;
    int nexti = curi - 1;
    Coord next(from.x - 1, from.y);
    
    while (true) {
        if (!m_map[nexti])
            break;
        
        dist++;
        U32 cost = m_cost[fromi] + dist;
        
        if (next == goal) {
            if (cost < m_cost[nexti]) {
                m_queue.insert(next, cost);
                m_cameFrom[nexti] = from;
                m_cost[nexti] = cost;
            }
            return dist;
        }
        
        U32 h = pathFinder().heuristic(next, goal);
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestH = h;
            m_bestC = cost;
            m_best = next;
            m_cameFrom[nexti] = from;
        }
        
        if ((m_map[nexti + m_size.width] && !m_map[curi + m_size.width]) ||
            (m_map[nexti - m_size.width] && !m_map[curi - m_size.width])) {
            if (cost < m_cost[nexti]) {
                m_queue.insert(next, h + cost);
                m_cameFrom[nexti] = from;
                m_cost[nexti] = cost;
            }
            return dist;
        }
        
        curi = nexti;
        nexti--;
        next.x--;
    }
    return dist;
}

void JPS::jumpNE(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    int curi = fromi;
    int nexti = curi + m_size.width + 1;
    Coord next(from.x + 1, from.y + 1);
    U32 cost = m_cost[fromi] + 1;
    
    if (!m_map[curi + 1] || !m_map[nexti - 1])
        return;
    
    while (true) {
        if (!m_map[nexti])
            break;
        
        if (cost < m_cost[nexti]) {
            m_cost[nexti] = cost;
            m_cameFrom[nexti] = from;
            
            U32 h = pathFinder().heuristic(next, goal);
            if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
                m_bestH = h;
                m_bestC = cost;
                m_best = next;
            }
        }
        
        if (next == goal) {
            if (cost == m_cost[nexti])
                m_queue.insert(next, cost);
            break;
        }
        
        bool n = jumpN(next, goal);
        bool e = jumpE(next, goal);
        if (!(n & e))
            break;
        
        curi = nexti;
        nexti += m_size.width + 1;
        next = Coord(next.x + 1, next.y + 1);
        cost++;
    }
}

void JPS::jumpSE(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    int curi = fromi;
    int nexti = curi - m_size.width + 1;
    Coord next(from.x + 1, from.y - 1);
    U32 cost = m_cost[fromi] + 1;
    
    if (!m_map[curi + 1] || !m_map[nexti - 1])
        return;
    
    while (true) {
        if (!m_map[nexti])
            break;
        
        if (cost < m_cost[nexti]) {
            m_cost[nexti] = cost;
            m_cameFrom[nexti] = from;
            
            U32 h = pathFinder().heuristic(next, goal);
            if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
                m_bestH = h;
                m_bestC = cost;
                m_best = next;
            }
        }
        
        if (next == goal) {
            if (cost == m_cost[nexti])
                m_queue.insert(next, cost);
            break;
        }
        
        bool s = jumpS(next, goal);
        bool e = jumpE(next, goal);
        if (!(s & e))
            break;
        
        curi = nexti;
        nexti -= m_size.width - 1;
        next = Coord(next.x + 1, next.y - 1);
        cost++;
    }
}

void JPS::jumpSW(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    int curi = fromi;
    int nexti = curi - m_size.width - 1;
    Coord next(from.x - 1, from.y - 1);
    U32 cost = m_cost[fromi] + 1;
    
    if (!m_map[curi - 1] || !m_map[nexti + 1])
        return;
    
    while (true) {
        if (!m_map[nexti])
            break;
        
        if (cost < m_cost[nexti]) {
            m_cost[nexti] = cost;
            m_cameFrom[nexti] = from;
            
            U32 h = pathFinder().heuristic(next, goal);
            if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
                m_bestH = h;
                m_bestC = cost;
                m_best = next;
            }
        }
        
        if (next == goal) {
            if (cost == m_cost[nexti])
                m_queue.insert(next, cost);
            break;
        }
        
        bool s = jumpS(next, goal);
        bool w = jumpW(next, goal);
        if (!(s & w))
            break;
        
        curi = nexti;
        nexti -= m_size.width + 1;
        next = Coord(next.x - 1, next.y - 1);
        cost++;
    }
}

void JPS::jumpNW(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    int curi = fromi;
    int nexti = curi + m_size.width - 1;
    Coord next(from.x - 1, from.y + 1);
    U32 cost = m_cost[fromi] + 1;
    
    if (!m_map[curi - 1] || !m_map[nexti + 1])
        return;
    
    while (true) {
        if (!m_map[nexti])
            break;
        
        if (cost < m_cost[nexti]) {
            m_cost[nexti] = cost;
            m_cameFrom[nexti] = from;
            
            U32 h = pathFinder().heuristic(next, goal);
            if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
                m_bestH = h;
                m_bestC = cost;
                m_best = next;
            }
        }
        
        if (next == goal) {
            if (cost == m_cost[nexti])
                m_queue.insert(next, cost);
            break;
        }
        
        bool n = jumpN(next, goal);
        bool w = jumpW(next, goal);
        if (!(n & w))
            break;
        
        curi = nexti;
        nexti += m_size.width - 1;
        next = Coord(next.x - 1, next.y + 1);
        cost++;
    }
}
