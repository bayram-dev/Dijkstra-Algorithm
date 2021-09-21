#include "dijkstra.h"

SearchResult Dijkstra::findPath(Input i)
{
    auto t = std::chrono::high_resolution_clock::now();
    SearchResult result;
    /*
     * start.g = 0
     * OPEN = {start}
     * CLOSED = {}
     * while OPEN is not empty
     *  current = node from OPEN with minimal g-value
     *  move current from OPEN to CLOSED
     *  if current is goal
     *      return reconstructPath(current)
     *  neighbors = get neighbors of current node
     *  for each neighbor in neighbors:
     *      if neighbor in CLOSED:
     *          continue
     *      neighbor.g = current.g + cost(neighbor, current)
     *      neighbor.parent = current
     *      insert or update neighbor into OPEN
     * return path not found
     */
    i.start.g = 0;
    i.start.f = 0;
    ol.addNode(i.start);
    while(ol.getSize() != 0)
    {
        Node c = ol.getMin();
        cl.addClose(c);
        ol.popMin();
        if((c.x == i.goal.x) && (c.y == i.goal.y))
        {
            result.cost = c.g;
            result.pathfound = true;
            result.path = Dijkstra::reconstructPath(c);
            break;
        }
        std::list<Node> neighbours = i.map.getValidMoves(c);
        for (auto n : neighbours)
        {
            if (cl.inClose(n.x, n.y))
                continue;
            n.g = c.g + i.map.getCost(n, c);
            n.f = n.g;
            n.parent = cl.getPointer(c.x, c.y);
            ol.addNode(n);
        }
    }
    result.createdNodes = cl.getSize() + ol.getSize();
    result.steps = cl.getSize();
    result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t).count();
    return result;
}

std::list<Node> Dijkstra::reconstructPath(Node current)
{
    std::list<Node> path;
    while(current.parent != nullptr)
    {
        path.push_front(current);
        current = *current.parent;
    }
    path.push_front(current);
    return path;
}
