# rpg-path-finding
C++ implementation of path finding algorithms for RPG-games with grid map.

## How to use

This code is used in my RPG-like game with custom in-house engine. I copied it as is, without implementation of many classes, like vec2, Array, MemoryManager etc. So if somebody wants to use it, he needs to replace them.

Algorithms work on grid map with maximum side size 2^16. If cell is not walkable there is a wall. Also there is a possibility to add round obstacles with arbitrary radius. Borders of the input map must be unwalkable because of there is no check if coordinates are out of borders for optimization reasons.

There are 2 phases of finding path:
1. Rough - position of joints restricted to center of cells. Path can be found using one of 3 algorithms: A*, JPS or JSP+. Obstables have no influence on resulting path.
2. Precise - works in continuous space using "Wall Tracing"

That scheme was gotten from game "Dota 2". First you search rough path to destination point, then precise path from current position to some point on rough path. After a while when distance to that point become short enough, you search precise path to another point on rough path.

Unlike many other implementations of A* and JPS, that can find closest path to some unreachable location. JPS uses improved algorithm that doesn't cut edges and was published in 2012 (http://harabor.net/data/papers/harabor-grastien-socs12.pdf). JPS+ cache distances to jump points.

Search time to unreachable locations with grid size 256x256 and low number of walls:
A*     ~26 ms
JSP    ~7.5 ms
JPS+   ~0.5 ms

## Wall Tracing

The most hard part is "Wall Tracing" algorithm. I was impressed with "Dota 2" path finding system and started to search how to realize it. But everything I found was only 1 mention [here](http://liquipedia.net/dota2/Pathfinding) without any explanation. So I started to invent it myself.

This implementation doesn't use trigonometric functions and works very fast for short distances (< 0.2 ms). But found paths are not perfect.
