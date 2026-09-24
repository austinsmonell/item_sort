# 3D A* Pathfinding

A grid-based A* pathfinder in Python on a 3D grid. The algorithm lives in `astar_core.py`; `astar.py` is a console wrapper and `astar_gui.py` is a tkinter GUI. Both call the same `a_star()`.

## Files

| File | Purpose |
|------|---------|
| `astar_core.py` | The A* implementation (`a_star`, `heuristic`) |
| `astar.py` | Console front end: parses a grid from stdin, prints the path |
| `astar_gui.py` | GUI, shows one layer at a time (`python astar_gui.py`) |

## Console use

```
python astar.py [--diag] < grid.txt
```

Input is `width height depth`, then `depth` layers (z = 0, 1, ...) of `height` rows each, separated by blank lines.

| Char | Meaning |
|------|---------|
| `.`  | open cell |
| `#`  | wall |
| `S`  | start |
| `G`  | goal |

Short rows are padded with walls. The grid must contain both `S` and `G`. Output is the path from `S` to `G` inclusive, one `x,y,z` per line (`x` right, `y` down, `z` layer), or `no path`. Example:

```
3 3 3
S..
...
...

.#.
.#.
.#.

...
...
..G
```

## GUI use

Pick Wall / Start / Goal / Erase, click or drag on the left grid, use the **Layer** slider to change z level, tick **Diagonals** for 26-neighbor movement, then **Run A\***. On the left grid the path on the current layer is a solid green square; path cells on other layers are small dark dots, and a start or goal on another layer shows as an outline.

The right panel is a 3D view of the whole grid: **drag to rotate, mouse wheel to zoom**. Layers are drawn as stacked planes (the one you're editing is highlighted), walls are see-through squares, the path is a thick green line through all layers, and S/G are marked. **Show explored (3D)** adds the cells A* expanded as faint dots.

## How the algorithm works (`astar_core.py`)

```python
a_star(walls, width, height, depth, start, goal, explored=None, diagonals=False) -> list[(x, y, z)]
```

- `walls`: set of blocked `(x, y, z)` cells
- `explored`: optional set that gets every expanded node added to it, so a caller can visualize the search. It does not change the return value.
- `diagonals`: `False` allows 6 axis moves (cost 1). `True` allows all 26 neighbors (edge moves cost `sqrt(2)`, corner moves `sqrt(3)`). Corner cutting between walls is not checked.
- Returns the path from `start` to `goal`, or `[]` if unreachable.

A* picks the next cell to expand by lowest `f = g + h`:

- **g**: the cheapest known cost from `start` to that cell.
- **h**: straight-line (Euclidean) distance to `goal`, via `math.dist`.
- **f**: total estimated cost of a path through that cell.

Data structures: `open_heap` (a `heapq` min-heap of `(f, cell)`), `g_score` (best known cost per cell), `came_from` (parent per cell, used to rebuild the path).

Steps:

1. Push `start` with `f = h(start)`; `g_score[start] = 0`.
2. Pop the lowest-`f` cell. Record it in `explored` if given.
3. If it is the goal, walk `came_from` backwards, reverse, and return the path.
4. Otherwise try each allowed move. Skip neighbors that are out of bounds or in `walls`.
5. `tentative_g = g_score[current] + move_length`. If that beats the neighbor's stored `g`, record it and `came_from`, and push the neighbor with `f = tentative_g + h(neighbor)`.
6. If the heap empties without reaching the goal, return `[]`.

Notes:

- **Lazy deletion:** there is no closed set. A cell found via a cheaper route is pushed again and the older entry stays in the heap. It may be popped later but cannot overwrite better `g` values, so the result stays correct.
- **Optimality:** move cost is the Euclidean length of the move, so the Euclidean heuristic never overestimates. It is admissible and consistent, and the returned path is a shortest one.
- **Ties:** heap entries are `(f, cell)` tuples, so equal `f` values are ordered by cell coordinates.
