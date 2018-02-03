
#include "JPSplus.hpp"
#include "PathFinder.hpp"
#include "Map.hpp"

using namespace nook;

JPSplus::JPSplus() {
    m_size = map()->size();
    
    int count = m_size.width * m_size.height;
    int queueSize = m_size.width;
    m_map = memoryManager().allocOnStack<bool>(count);
    m_jps = memoryManager().allocOnStack<JP>(count);
    m_queue.init(queueSize, memoryManager().allocOnStack<PriorityQueue<Coord, U32>::Item>(queueSize));
    m_cameFrom.init(count, memoryManager().allocOnStack<Coord>(count));
    m_cost.init(count, memoryManager().allocOnStack<U32>(count));
    m_cameFrom.setCount(count);
    m_cost.setCount(count);
    
    update();
}

void JPSplus::update() {
    for (int y = 0; y < m_size.height; y++)
        for (int x = 0; x < m_size.width; x++)
            m_map[pathFinder().index(x, y)] = map()->getCell(x, y)->walkable;
    
    for (int y = 1; y < m_size.height - 1; y++)
        for (int x = 1; x < m_size.width - 1; x++) {
            JP& jp = m_jps[pathFinder().index(x, y)];
            Coord c(x, y);
            jp.n = getJumpN(c);
            jp.e = getJumpE(c);
            jp.s = getJumpS(c);
            jp.w = getJumpW(c);
            jp.ne = getJumpNE(c);
            jp.se = getJumpSE(c);
            jp.sw = getJumpSW(c);
            jp.nw = getJumpNW(c);
        }
}

void JPSplus::find(Coord start, Coord end, nook::Array<nook::vec2>& path) {
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
    
    jumpN(start, end);
    jumpE(start, end);
    jumpS(start, end);
    jumpW(start, end);
    jumpNE(start, end);
    jumpSE(start, end);
    jumpSW(start, end);
    jumpNW(start, end);
    
    while (m_queue.count()) {
        Coord cur = m_queue.pop();
        if (cur == end) {
            m_best = end;
            break;
        }
        
        int ci = pathFinder().index(cur.x, cur.y);
        
        Coord from = m_cameFrom[ci];
        int bi = ci - m_size.width;
        int ti = ci + m_size.width;
        
        if (cur.y == from.y) {
            if (cur.x > from.x) {
                if (m_map[bi] & !m_map[bi - 1]) {
                    jumpS(cur, end);
                    jumpSE(cur, end);
                }
                if (m_map[ti] & !m_map[ti - 1]) {
                    jumpN(cur, end);
                    jumpNE(cur, end);
                }
                jumpE(cur, end);
            }
            else {
                if (m_map[bi] & !m_map[bi + 1]) {
                    jumpS(cur, end);
                    jumpSW(cur, end);
                }
                if (m_map[ti] & !m_map[ti + 1]) {
                    jumpN(cur, end);
                    jumpNW(cur, end);
                }
                jumpW(cur, end);
            }
        }
        else if (cur.y < from.y) {
            if (cur.x == from.x) {
                if (m_map[ci - 1] & !m_map[ti - 1]) {
                    jumpW(cur, end);
                    jumpSW(cur, end);
                }
                if (m_map[ci + 1] & !m_map[ti + 1]) {
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
                if (m_map[ci - 1] & !m_map[bi - 1]) {
                    jumpW(cur, end);
                    jumpNW(cur, end);
                }
                if (m_map[ci + 1] & !m_map[bi + 1]) {
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

U16 JPSplus::getJumpN(Coord from) {
    int curi = pathFinder().index(from.x, from.y);
    int nexti = curi + m_size.width;
    U16 d = 0;
    
    while (m_map[nexti]) {
        d++;
        if ((m_map[nexti + 1] && !m_map[curi + 1]) || (m_map[nexti - 1] && !m_map[curi - 1]))
            return d | BIT(15);
        curi = nexti;
        nexti += m_size.width;
    }
    return d;
}

U16 JPSplus::getJumpE(Coord from) {
    int curi = pathFinder().index(from.x, from.y);
    int nexti = curi + 1;
    U16 d = 0;
    
    while (m_map[nexti]) {
        d++;
        if ((m_map[nexti + m_size.width] && !m_map[curi + m_size.width]) || (m_map[nexti - m_size.width] && !m_map[curi - m_size.width]))
            return d | BIT(15);
        curi = nexti;
        nexti++;
    }
    return d;
}

U16 JPSplus::getJumpS(Coord from) {
    int curi = pathFinder().index(from.x, from.y);
    int nexti = curi - m_size.width;
    U16 d = 0;
    
    while (m_map[nexti]) {
        d++;
        if ((m_map[nexti + 1] && !m_map[curi + 1]) || (m_map[nexti - 1] && !m_map[curi - 1]))
            return d | BIT(15);
        curi = nexti;
        nexti -= m_size.width;
    }
    return d;
}

U16 JPSplus::getJumpW(Coord from) {
    int curi = pathFinder().index(from.x, from.y);
    int nexti = curi - 1;
    U16 d = 0;
    
    while (m_map[nexti]) {
        d++;
        if ((m_map[nexti + m_size.width] && !m_map[curi + m_size.width]) || (m_map[nexti - m_size.width] && !m_map[curi - m_size.width]))
            return d | BIT(15);
        curi = nexti;
        nexti--;
    }
    return d;
}

U16 JPSplus::getJumpNE(Coord from) {
    Coord next = from;
    int nexti = pathFinder().index(from.x, from.y);
    U16 d = 0;
    while (true) {
        if (!m_map[nexti + m_size.width] || !m_map[nexti + 1])
            break;
        nexti += m_size.width + 1;
        if (!m_map[nexti])
            break;
        d++;
        next.x++;
        next.y++;
    }
    return d;
}

U16 JPSplus::getJumpSE(Coord from) {
    Coord next = from;
    int nexti = pathFinder().index(from.x, from.y);
    U16 d = 0;
    while (true) {
        if (!m_map[nexti - m_size.width] || !m_map[nexti + 1])
            break;
        nexti -= m_size.width - 1;
        if (!m_map[nexti])
            break;
        d++;
        next.x++;
        next.y--;
    }
    return d;
}

U16 JPSplus::getJumpSW(Coord from) {
    Coord next = from;
    int nexti = pathFinder().index(from.x, from.y);
    U16 d = 0;
    while (true) {
        if (!m_map[nexti - m_size.width] || !m_map[nexti - 1])
            break;
        nexti -= m_size.width + 1;
        if (!m_map[nexti])
            break;
        d++;
        next.x--;
        next.y--;
    }
    return d;
}

U16 JPSplus::getJumpNW(Coord from) {
    Coord next = from;
    int nexti = pathFinder().index(from.x, from.y);
    U16 d = 0;
    while (true) {
        if (!m_map[nexti + m_size.width] || !m_map[nexti - 1])
            break;
        nexti += m_size.width - 1;
        if (!m_map[nexti])
            break;
        d++;
        next.x--;
        next.y++;
    }
    return d;
}

void JPSplus::jumpN(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    U16 d = m_jps[fromi].n;
    U16 dist = d & ~(BIT(15));
    U16 endy = from.y + dist;
    
    if (from.x == goal.x && inRange(from.y, endy, goal.y)) {
        int goali = pathFinder().index(goal.x, goal.y);
        U32 cost = m_cost[fromi] + goal.y - from.y;
        if (cost < m_cost[goali]) {
            m_queue.insert(goal, cost);
            m_cameFrom[goali] = from;
            m_cost[goali] = cost;
        }
        else if (cost == m_cost[goali])
            m_queue.insert(goal, cost);
        return;
    }
    
    if (d & BIT(15)) {
        U32 cost = m_cost[fromi] + dist;
        Coord end(from.x, endy);
        int endi = pathFinder().index(end.x, end.y);
        if (cost < m_cost[endi]) {
            U32 h = pathFinder().heuristic(end, goal);
            m_queue.insert(end, h + cost);
            m_cameFrom[endi] = from;
            m_cost[endi] = cost;
        }
    }
    
    if (goal.y > from.y) {
        Coord c(from.x, min2(goal.y, endy));
        U32 h = pathFinder().heuristic(c, goal);
        U32 cost = m_cost[fromi] + c.y - from.y;
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestC = cost;
            m_bestH = h;
            m_best = c;
            int ci = pathFinder().index(c.x, c.y);
            if (cost < m_cost[ci]) {
                m_cost[ci] = cost;
                m_cameFrom[ci] = from;
            }
        }
    }
}

void JPSplus::jumpE(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    U16 d = m_jps[fromi].e;
    U16 dist = d & ~(BIT(15));
    U16 endx = from.x + dist;
    
    if (from.y == goal.y && inRange(from.x, endx, goal.x)) {
        int goali = pathFinder().index(goal.x, goal.y);
        U32 cost = m_cost[fromi] + goal.x - from.x;
        if (cost < m_cost[goali]) {
            m_queue.insert(goal, cost);
            m_cameFrom[goali] = from;
            m_cost[goali] = cost;
        }
        else if (cost == m_cost[goali])
            m_queue.insert(goal, cost);
        return;
    }
    
    if (d & BIT(15)) {
        U32 cost = m_cost[fromi] + dist;
        Coord end(endx, from.y);
        int endi = pathFinder().index(end.x, end.y);
        if (cost < m_cost[endi]) {
            U32 h = pathFinder().heuristic(end, goal);
            m_queue.insert(end, h + cost);
            m_cameFrom[endi] = from;
            m_cost[endi] = cost;
        }
    }
    
    if (goal.x > from.x) {
        Coord c(min2(goal.x, endx), from.y);
        U32 h = pathFinder().heuristic(c, goal);
        U32 cost = m_cost[fromi] + c.x - from.x;
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestC = cost;
            m_bestH = h;
            m_best = c;
            int ci = pathFinder().index(c.x, c.y);
            if (cost < m_cost[ci]) {
                m_cost[ci] = cost;
                m_cameFrom[ci] = from;
            }
        }
    }
}

void JPSplus::jumpS(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    U16 d = m_jps[fromi].s;
    U16 dist = d & ~(BIT(15));
    U16 endy = from.y - dist;
    
    if (from.x == goal.x && inRange(endy, from.y, goal.y)) {
        int goali = pathFinder().index(goal.x, goal.y);
        U32 cost = m_cost[fromi] + from.y - goal.y;
        if (cost < m_cost[goali]) {
            m_queue.insert(goal, cost);
            m_cameFrom[goali] = from;
            m_cost[goali] = cost;
        }
        else if (cost == m_cost[goali])
            m_queue.insert(goal, cost);
        return;
    }
    
    if (d & BIT(15)) {
        U32 cost = m_cost[fromi] + dist;
        Coord end(from.x, endy);
        int endi = pathFinder().index(end.x, end.y);
        if (cost < m_cost[endi]) {
            U32 h = pathFinder().heuristic(end, goal);
            m_queue.insert(end, h + cost);
            m_cameFrom[endi] = from;
            m_cost[endi] = cost;
        }
    }
    
    if (goal.y < from.y) {
        Coord c(from.x, max2(goal.y, endy));
        U32 h = pathFinder().heuristic(c, goal);
        U32 cost = m_cost[fromi] + from.y - c.y;
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestC = cost;
            m_bestH = h;
            m_best = c;
            int ci = pathFinder().index(c.x, c.y);
            if (cost < m_cost[ci]) {
                m_cost[ci] = cost;
                m_cameFrom[ci] = from;
            }
        }
    }
}

void JPSplus::jumpW(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    U16 d = m_jps[fromi].w;
    U16 dist = d & ~(BIT(15));
    U16 endx = from.x - dist;
    
    if (from.y == goal.y && inRange(endx, from.x, goal.x)) {
        int goali = pathFinder().index(goal.x, goal.y);
        U32 cost = m_cost[fromi] + from.x - goal.x;
        if (cost < m_cost[goali]) {
            m_queue.insert(goal, cost);
            m_cameFrom[goali] = from;
            m_cost[goali] = cost;
        }
        else if (cost == m_cost[goali])
            m_queue.insert(goal, cost);
        return;
    }
    
    if (d & BIT(15)) {
        U32 cost = m_cost[fromi] + dist;
        Coord end(endx, from.y);
        int endi = pathFinder().index(end.x, end.y);
        if (cost < m_cost[endi]) {
            U32 h = pathFinder().heuristic(end, goal);
            m_queue.insert(end, h + cost);
            m_cameFrom[endi] = from;
            m_cost[endi] = cost;
        }
    }
    
    if (goal.x < from.x) {
        Coord c(max2(goal.x, endx), from.y);
        U32 h = pathFinder().heuristic(c, goal);
        U32 cost = m_cost[fromi] + from.x - c.x;
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestC = cost;
            m_bestH = h;
            m_best = c;
            int ci = pathFinder().index(c.x, c.y);
            if (cost < m_cost[ci]) {
                m_cost[ci] = cost;
                m_cameFrom[ci] = from;
            }
        }
    }
}

void JPSplus::jumpNE(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    U16 dist = m_jps[fromi].ne;
    Coord next = from;
    int nexti = fromi;
    U32 cost = m_cost[fromi];
    
    for (int i = 0; i < dist; i++) {
        next.x++;
        next.y++;
        nexti += m_size.width + 1;
        cost++;
        if (cost >= m_cost[nexti])
            break;
        m_cameFrom[nexti] = from;
        m_cost[nexti] = cost;
        jumpN(next, goal);
        jumpE(next, goal);
        
        U32 h = pathFinder().heuristic(next, goal);
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestC = cost;
            m_bestH = h;
            m_best = next;
        }
    }
}

void JPSplus::jumpSE(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    U16 dist = m_jps[fromi].se;
    Coord next = from;
    int nexti = fromi;
    U32 cost = m_cost[fromi];
    
    for (int i = 0; i < dist; i++) {
        next.x++;
        next.y--;
        nexti -= m_size.width - 1;
        cost++;
        if (cost >= m_cost[nexti])
            break;
        m_cameFrom[nexti] = from;
        m_cost[nexti] = cost;
        jumpS(next, goal);
        jumpE(next, goal);
        
        U32 h = pathFinder().heuristic(next, goal);
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestC = cost;
            m_bestH = h;
            m_best = next;
        }
    }
}

void JPSplus::jumpSW(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    U16 dist = m_jps[fromi].sw;
    Coord next = from;
    int nexti = fromi;
    U32 cost = m_cost[fromi];
    
    for (int i = 0; i < dist; i++) {
        next.x--;
        next.y--;
        nexti -= m_size.width + 1;
        cost++;
        if (cost >= m_cost[nexti])
            break;
        m_cameFrom[nexti] = from;
        m_cost[nexti] = cost;
        jumpS(next, goal);
        jumpW(next, goal);
        
        U32 h = pathFinder().heuristic(next, goal);
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestC = cost;
            m_bestH = h;
            m_best = next;
        }
    }
}

void JPSplus::jumpNW(Coord from, Coord goal) {
    int fromi = pathFinder().index(from.x, from.y);
    U16 dist = m_jps[fromi].nw;
    Coord next = from;
    int nexti = fromi;
    U32 cost = m_cost[fromi];
    
    for (int i = 0; i < dist; i++) {
        next.x--;
        next.y++;
        nexti += m_size.width - 1;
        cost++;
        if (cost >= m_cost[nexti])
            break;
        m_cameFrom[nexti] = from;
        m_cost[nexti] = cost;
        jumpN(next, goal);
        jumpW(next, goal);
        
        U32 h = pathFinder().heuristic(next, goal);
        if (h < m_bestH || (h == m_bestH && cost < m_bestC)) {
            m_bestC = cost;
            m_bestH = h;
            m_best = next;
        }
    }
}
