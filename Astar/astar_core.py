"""The actual A* algorithm, shared by astar.py (console) and astar_gui.py
(GUI) so the GUI runs the real pathfinder instead of a separate
reimplementation.
"""

import heapq
import math

Point = tuple  # (x, y)


def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


_DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def a_star(walls, width, height, start, goal, explored=None):
    """8-directional A* over a grid.

    walls: a set of (x, y) tuples that are blocked.
    explored: optional set that, if given, has every node the search
    actually expands added to it - lets a caller (the GUI) visualize what
    A* looked at, without changing the return value callers already rely on.
    Returns the path from start to goal inclusive as a list of (x, y)
    tuples, or an empty list if the goal is unreachable.
    """
    open_heap = [(heuristic(start, goal), start)]
    came_from = {}
    g_score = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if explored is not None:
            explored.add(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cx, cy = current
        for dx, dy in _DIRS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if (nx, ny) in walls:
                continue

            step_cost = math.sqrt(2) if dx and dy else 1.0
            tentative_g = g_score[current] + step_cost

            neighbor = (nx, ny)
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f, neighbor))

    return []
