# 3D A* Pathfinding

A* pathfinder for a 3D arena defined in centimetres. The algorithm lives in `astar_core.py`; `astar_gui.py` is a tkinter GUI that calls `plan_path()`.

## Arena and resolution

Set at the top of `astar_core.py`:

```python
ARENA_X_CM = 180.0
ARENA_Y_CM = 160.0
ARENA_Z_CM = 35.0
RESOLUTION_CM = 0.25
```

The search runs on a lattice of nodes 0.25 cm apart. Node `(i, j, k)` is at `(i, j, k) * 0.25` cm, and both arena edges are included, so the lattice is 721 x 641 x 141 nodes (about 65 million). Obstacles are axis-aligned boxes given in cm (`BoxObstacles`). Listing blocked nodes one by one would not fit in memory at this resolution. There is no robot-size clearance: a path can touch the surface of a box.

## Files

| File | Purpose |
|------|---------|
| `astar_core.py` | Arena constants, `BoxObstacles`, `a_star` (lattice search), `plan_path` (the two-pass planner the GUI uses) |
| `astar_gui.py` | GUI with a top-down map and a rotatable 3D view |

## GUI use

Run `python astar_gui.py`.

- **Height z** slider chooses the height you are working at.
- **Box**: drag a rectangle on the map. Its height range comes from the *Box z* fields (default 0 to 35, a full-height pillar).
- **Start / Goal**: click; placed at the current height.
- **Erase**: click a box to delete it.
- **Diagonals**: 26-direction moves (on by default).
- **Show explored (3D)**: shows the cells the search expanded, as faint dots. It is off by default because recording them uses extra memory.

On the map, boxes reaching the current height are solid and others are dashed outlines. The path is a thin line, drawn thick where it is within 1 cm of the current height. The right panel shows the whole arena in 3D: **drag to rotate, wheel to zoom**, with the current height as a highlighted plane. Heights are stretched 2.5x there so the 35 cm depth is readable.

## How the algorithm works (`astar_core.py`)

### `a_star(walls, width, height, depth, start, goal, explored=None, diagonals=True, weight=1.0)`

Standard A* on the node lattice, choosing the next node by lowest `f = g + weight * h`:

- **g**: the cheapest known cost from `start`, where a move costs its Euclidean length in lattice steps (1, `sqrt(2)` or `sqrt(3)`).
- **h**: the exact open-space cost to the goal. With 26 moves that is the 3D octile distance, `sqrt(3)*d1 + sqrt(2)*(d2-d1) + (d3-d2)` for sorted axis distances `d1 <= d2 <= d3`. With 6 moves it is Manhattan distance. Both are admissible and consistent.
- Ties on `f` go to the node with the smaller `h` (`f` is rounded to 9 decimals so float noise doesn't break ties). Without this, the many equal-cost routes across a 3D lattice make the search flood millions of nodes even in an empty arena.
- A closed set skips stale duplicate heap entries.
- `weight > 1` is weighted A*: far fewer expansions, at most `weight` times the optimal length.

`walls` can be any object supporting `node in walls`, such as a set of nodes or a `BoxObstacles`.

### `plan_path(boxes, start_cm, goal_cm, res=0.25, coarse_res=2.0, diagonals=True, explored=None, corridor=1, fine_weight=2.0)`

Even with the tie-breaking, a real obstacle forces the search to flood the shadow behind it, and plain A* over 65 million nodes takes minutes in Python. So planning is two-pass:

1. **Coarse pass:** A* at 2 cm (about 90 x 80 x 18 nodes) over the whole arena. Boxes are grown by half a coarse cell so thin obstacles cannot slip between coarse nodes. If that finds nothing, it retries without the growth.
2. **Fine pass:** weighted A* (`fine_weight` 2.0) at 0.25 cm, confined to a corridor of `corridor` coarse cells around the coarse path. If it fails, the corridor is widened (2x, then 4x).

Returns cm points from start to goal, or `[]`. Typical scenes take under a second, and scenes with a thin obstacle to go around took about 2 s in testing. An unreachable goal took about 4 s.

Trade-offs of the two-pass approach:

- The path is near-optimal, not guaranteed shortest.
- A gap much narrower than `coarse_res` (2 cm) may be missed, so the planner can report `no path` for a goal that is technically reachable through it.
- The fine path is a staircase of 26-direction moves and is not smoothed.

## Pseudocode

```
a_star(walls, W, H, D, start, goal, explored, diagonals, weight):
    if start in walls or goal in walls: return []
    moves = 26 directions if diagonals else 6 directions   # each with its length
    h(n)  = octile3D(n, goal) if diagonals else manhattan(n, goal)

    open   = min-heap of (f, h, node)      # ordered by f, then by h
    push (weight*h(start), h(start), start)
    g[start] = 0
    came_from = {}
    closed = {}

    while open is not empty:
        pop node `cur` with lowest (f, h)
        if cur in closed: continue         # stale duplicate entry
        add cur to closed (and to explored, if given)

        if cur == goal:
            walk came_from from goal back to start, reverse it, return it

        for each move (dx, dy, dz, cost):
            nb = cur + (dx, dy, dz)
            skip if nb is outside 0..W-1, 0..H-1, 0..D-1
            skip if nb in closed or nb in walls

            new_g = g[cur] + cost
            if new_g < g[nb] (unset counts as infinity):
                g[nb] = new_g
                came_from[nb] = cur
                push (round(new_g + weight*h(nb), 9), h(nb), nb)

    return []                              # goal unreachable
```

```
plan_path(boxes, start_cm, goal_cm, res, coarse_res, diagonals, explored,
          corridor, fine_weight):
    factor = coarse_res / res                        # 2.0 / 0.25 = 8
    start, goal   = start_cm, goal_cm as nodes at res
    cstart, cgoal = start_cm, goal_cm as nodes at coarse_res

    # 1. coarse pass over the whole arena
    for margin in (coarse_res / 2, 0):               # grown boxes first, then exact
        coarse = a_star(boxes grown by margin, coarse-lattice size,
                        cstart, cgoal, diagonals)
        if coarse is not empty: break
    if still empty: return []

    # 2. fine pass inside a corridor around the coarse path
    for r in (corridor, 2*corridor, 4*corridor):     # widen if the search fails
        cells = every coarse node within r steps of any node on the coarse path
        blocked(node) = nearest coarse node of `node` is not in cells
                        or node is inside a box
        path = a_star(blocked, fine-lattice size, start, goal,
                      diagonals, weight = fine_weight)
        if path is not empty:
            return path with every node converted to cm
    return []
```
