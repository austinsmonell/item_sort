"""The actual 3D A* algorithm, shared by astar.py (console) and astar_gui.py
(GUI) so the GUI runs the real pathfinder instead of a separate
reimplementation.
"""

import heapq
import itertools
import math

Point = tuple  # (x, y, z)


def heuristic(a, b):
    """Straight-line distance between two points of any dimension."""
    return math.dist(a, b)


# Moves: the 6 face neighbors, or all 26 neighbors (faces, edges, corners).
_DIRS_6 = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
_DIRS_26 = [d for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]


def _search(walls, bounds, start, goal, dirs, explored):
    """A* over an N-dimensional grid. bounds is the size along each axis;
    dirs is the list of allowed moves. Move cost is the Euclidean length of
    the move, so the heuristic (straight-line distance) stays admissible."""
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

        for d in dirs:
            neighbor = tuple(c + m for c, m in zip(current, d))
            if not all(0 <= n < b for n, b in zip(neighbor, bounds)):
                continue
            if neighbor in walls:
                continue

            tentative_g = g_score[current] + math.sqrt(sum(m * m for m in d))

            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f, neighbor))

    return []


def a_star(walls, width, height, depth, start, goal, explored=None,
           diagonals=False):
    """A* over a 3D grid.

    walls: a set of (x, y, z) tuples that are blocked.
    diagonals=False allows 6-directional moves (along an axis only);
    diagonals=True allows all 26 neighbors (edge move cost sqrt(2), corner
    move cost sqrt(3)).
    explored: optional set that, if given, has every node the search
    actually expands added to it - lets a caller (the GUI) visualize what
    A* looked at, without changing the return value.
    Returns the path from start to goal inclusive as a list of (x, y, z)
    tuples, or an empty list if the goal is unreachable.
    """
    dirs = _DIRS_26 if diagonals else _DIRS_6
    return _search(walls, (width, height, depth), start, goal, dirs, explored)
