"""The actual 3D A* algorithm, used by astar_gui.py so the GUI runs the real
pathfinder instead of a separate reimplementation.

The arena is defined in centimetres (ARENA_*_CM) and searched on a lattice
of nodes spaced RESOLUTION_CM apart. Node (i, j, k) sits at
(i, j, k) * RESOLUTION_CM cm, and the lattice includes both arena edges, so
each axis has size/resolution + 1 nodes. Obstacles are axis-aligned boxes
given in cm (see BoxObstacles) - at this resolution the arena has tens of
millions of nodes, far too many to list blocked nodes one by one.
"""

import heapq
import itertools
import math

ARENA_X_CM = 180.0
ARENA_Y_CM = 160.0
ARENA_Z_CM = 35.0
RESOLUTION_CM = 0.25


def _nodes(size_cm, res):
    return int(round(size_cm / res)) + 1


def arena_nodes(res=RESOLUTION_CM):
    """Number of nodes along (x, y, z) for the arena at resolution res."""
    return (_nodes(ARENA_X_CM, res), _nodes(ARENA_Y_CM, res),
            _nodes(ARENA_Z_CM, res))


def cm_to_node(p, res=RESOLUTION_CM):
    """(x, y, z) in cm -> nearest lattice node (i, j, k)."""
    return tuple(int(round(v / res)) for v in p)


def node_to_cm(n, res=RESOLUTION_CM):
    """Lattice node (i, j, k) -> (x, y, z) in cm."""
    return tuple(v * res for v in n)


class BoxObstacles:
    """A set of axis-aligned boxes that behaves like a set of blocked nodes
    (supports `node in obstacles`), so a_star() can take it as `walls`.

    Each box is (x0, y0, z0, x1, y1, z1) in cm; every node inside it,
    edges included, is blocked.
    """

    def __init__(self, boxes=(), res=RESOLUTION_CM, margin=0.0):
        self.res = res
        self.margin = margin  # cm each box is grown by when blocking nodes
        self.boxes = []
        self._ranges = []
        for b in boxes:
            self.add(b)

    def add(self, box):
        x0, y0, z0, x1, y1, z1 = box
        box = (min(x0, x1), min(y0, y1), min(z0, z1),
               max(x0, x1), max(y0, y1), max(z0, z1))
        self.boxes.append(box)
        r = self.res
        eps = 1e-9
        m = self.margin
        self._ranges.append(tuple(
            (math.ceil((box[a] - m) / r - eps),
             math.floor((box[a + 3] + m) / r + eps))
            for a in range(3)))

    def remove(self, index):
        del self.boxes[index]
        del self._ranges[index]

    def __contains__(self, node):
        x, y, z = node
        for (xa, xb), (ya, yb), (za, zb) in self._ranges:
            if xa <= x <= xb and ya <= y <= yb and za <= z <= zb:
                return True
        return False

    def __len__(self):
        return len(self.boxes)


# Moves: the 6 face neighbors, or all 26 neighbors (faces, edges, corners),
# each with its Euclidean length in lattice steps.
def _with_cost(dirs):
    return [(dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz))
            for dx, dy, dz in dirs]


_DIRS_6 = _with_cost([(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                      (0, 0, 1), (0, 0, -1)])
_DIRS_26 = _with_cost(d for d in itertools.product((-1, 0, 1), repeat=3)
                      if any(d))


_SQRT2 = math.sqrt(2)
_SQRT3 = math.sqrt(3)


def _h_diag(a, b):
    """Exact open-space cost between two nodes with 26-direction moves
    (3D octile distance). Admissible and consistent, and much tighter than
    straight-line distance, so the search stays near the optimal route."""
    d1, d2, d3 = sorted((abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2])))
    return _SQRT3 * d1 + _SQRT2 * (d2 - d1) + (d3 - d2)


def _h_axis(a, b):
    """Exact open-space cost with 6-direction moves (Manhattan distance)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def a_star(walls, width, height, depth, start, goal, explored=None,
           diagonals=True, weight=1.0):
    """A* over a 3D lattice of width x height x depth nodes.

    walls: anything supporting `node in walls` for (i, j, k) nodes - a set
    of nodes, or a BoxObstacles.
    diagonals=True (default) allows all 26 neighbors (edge move cost
    sqrt(2), corner move cost sqrt(3), in lattice steps); False allows only
    the 6 axis moves. Diagonals are much faster on a big lattice: they keep
    the search close to the straight line instead of flooding the space.
    explored: optional set that, if given, has every node the search
    actually expands added to it - lets a caller (the GUI) visualize what
    A* looked at, without changing the return value.
    weight: heuristic multiplier. 1.0 gives a shortest path; larger values
    (weighted A*) expand far fewer nodes and return a path at most `weight`
    times longer than optimal.
    Returns the path from start to goal inclusive as a list of (i, j, k)
    nodes, or an empty list if the goal is unreachable.
    """
    dirs = _DIRS_26 if diagonals else _DIRS_6
    h = _h_diag if diagonals else _h_axis

    if start in walls or goal in walls:
        return []

    # heap entries are (f, h, node): among equal f, expand the node closest
    # to the goal first, which avoids flooding the many equal-cost routes.
    # f is rounded so float noise doesn't break those ties.
    h0 = h(start, goal)
    open_heap = [(weight * h0, h0, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue  # stale duplicate entry
        closed.add(current)

        if explored is not None:
            explored.add(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cx, cy, cz = current
        g = g_score[current]
        for dx, dy, dz, cost in dirs:
            nx, ny, nz = cx + dx, cy + dy, cz + dz
            if not (0 <= nx < width and 0 <= ny < height and 0 <= nz < depth):
                continue
            neighbor = (nx, ny, nz)
            if neighbor in closed or neighbor in walls:
                continue

            tentative_g = g + cost
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                hn = h(neighbor, goal)
                heapq.heappush(open_heap, (round(tentative_g + weight * hn, 9), hn, neighbor))

    return []


class _Corridor:
    """Blocks every node whose nearest coarse node is not in `cells`, plus
    everything blocked by `obstacles`. Used to confine the fine search to a
    band around the coarse path."""

    def __init__(self, obstacles, cells, factor):
        self.obstacles = obstacles
        self.cells = cells
        self.k = factor

    def __contains__(self, node):
        k = self.k
        cell = (round(node[0] / k), round(node[1] / k), round(node[2] / k))
        return cell not in self.cells or node in self.obstacles


def plan_path(boxes, start_cm, goal_cm, res=RESOLUTION_CM, coarse_res=2.0,
              diagonals=True, explored=None, corridor=1, fine_weight=2.0):
    """Plan a path through the arena, in cm, at resolution `res`.

    A search over all ~64 million nodes at 0.25 cm is far too slow in
    Python whenever an obstacle forces a detour, so this plans in two
    passes:
      1. a coarse A* at `coarse_res` cm (a whole-arena search of only tens
         of thousands of nodes; boxes are grown by half a coarse cell so
         nothing thin slips between coarse nodes),
      2. a fine A* at `res` cm restricted to a band `corridor` coarse
         cells around the coarse path (widened if that fails), run as
         weighted A* (`fine_weight`) so it doesn't flood the corridor.
    boxes: list of (x0, y0, z0, x1, y1, z1) in cm.
    explored: optional set that receives every expanded node as (x, y, z)
    cm, from both passes.
    Returns a list of (x, y, z) cm points from start to goal, or [] if no
    path was found. The result is near-optimal rather than guaranteed
    optimal (at most fine_weight times the best path in the corridor), and
    gaps much narrower than coarse_res may be missed.
    """
    factor = round(coarse_res / res)
    if factor < 1 or abs(factor * res - coarse_res) > 1e-9:
        raise ValueError("coarse_res must be a whole multiple of res")

    fine_n = arena_nodes(res)
    coarse_n = arena_nodes(coarse_res)
    start = cm_to_node(start_cm, res)
    goal = cm_to_node(goal_cm, res)
    cstart = cm_to_node(start_cm, coarse_res)
    cgoal = cm_to_node(goal_cm, coarse_res)

    def note(nodes, r):
        if explored is not None:
            explored.update(node_to_cm(n, r) for n in nodes)

    for margin in (coarse_res / 2, 0.0):
        seen = set()
        coarse = a_star(BoxObstacles(boxes, coarse_res, margin), *coarse_n,
                        cstart, cgoal, seen, diagonals)
        note(seen, coarse_res)
        if coarse:
            break
    else:
        return []

    # the fine endpoints must lie inside the corridor even if the coarse
    # start/goal snapped away from them
    coarse = [cstart] + coarse + [cgoal]
    fine_obstacles = BoxObstacles(boxes, res)
    for r in (corridor, corridor * 2, corridor * 4):
        cells = set()
        for cx, cy, cz in coarse:
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    for dz in range(-r, r + 1):
                        cells.add((cx + dx, cy + dy, cz + dz))
        seen = set()
        path = a_star(_Corridor(fine_obstacles, cells, factor), *fine_n,
                      start, goal, seen, diagonals, fine_weight)
        note(seen, res)
        if path:
            return [node_to_cm(n, res) for n in path]
    return []
