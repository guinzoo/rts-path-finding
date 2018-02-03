
#include "AStar.hpp"
#include "PathFinder.hpp"
#include "Map.hpp"

using namespace nook;

AStar::AStar() {
    m_size = map()->size();
    
    int count = m_size.width * m_size.height;
    m_maxIter = m_size.width * 4;
    m_map = memoryManager().allocOnStack<bool>(count);
    m_queue.init(m_maxIter, memoryManager().allocOnStack<PriorityQueue<Coord, U32>::Item>(m_maxIter));
    m_cameFrom.init(count, memoryManager().allocOnStack<Coord>(count));
    m_cost.init(count, memoryManager().allocOnStack<U32>(count));
    m_cameFrom.setCount(count);
    m_cost.setCount(count);
    
    for (int y = 0; y < m_size.height; y++)
        for (int x = 0; x < m_size.width; x++)
            m_map[pathFinder().index(x, y)] = map()->getCell(x, y)->walkable;
}

void AStar::find(Coord start, Coord end, Array<vec2>& path) {
    Point2 neighbours[] = {
        { -1, 0 },
        { 0, -1 },
        { 1, 0 },
        { 0, 1 }
    };
    
    m_queue.clear();
    std::memset(m_cost.buf(), 0, m_cost.count() * sizeof(int));
    
    m_queue.insert(start, 0);
    int startIdx = pathFinder().index(start.x, start.y);
    m_cameFrom[startIdx] = start;
    m_cost[startIdx] = 0;
    
    Coord bestCoord = start;
    U32 nearest = pathFinder().heuristic(start, end);
    int iter = 0;
    
    while (m_queue.count()) {
        Coord cur = m_queue.pop();
        int curIdx = pathFinder().index(cur.x, cur.y);
        iter++;
        
        if (cur == end) {
            bestCoord = end;
            break;
        }
        if (iter == m_maxIter)
            break;
        
        for (Point2 n : neighbours) {
            Coord next(cur.x + n.x, cur.y + n.y);
            int nextIdx = pathFinder().index(next.x, next.y);
            if (m_map[nextIdx] && !m_cost[nextIdx]) {
                U32 cost = m_cost[curIdx] + 1;
                m_cost[nextIdx] = cost;
                U32 dist = pathFinder().heuristic(next, end);
                if (dist < nearest) {
                    nearest = dist;
                    bestCoord = next;
                }
                U32 priority = cost + dist;
                m_queue.insert(next, priority);
                m_cameFrom[nextIdx] = cur;
            }
        }
    }
    
    // Create Path
    while (bestCoord != start) {
        path.push(map()->getPos(bestCoord.point()));
        bestCoord = m_cameFrom[pathFinder().index(bestCoord.x, bestCoord.y)];
    }
    path.push(map()->getPos(start.point()));
}
