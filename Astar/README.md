# A* Pathfinding

A grid-based A* pathfinder in Python. The algorithm lives in `astar_core.py`; `astar.py` is a console wrapper around it. (`astar_gui.py` is a GUI that calls the same core function and is not covered here.)

## Files

| File | Purpose |
|------|---------|
| `astar_core.py` | The A* implementation (`a_star`, `heuristic`) |
| `astar.py` | Console front end: parses a grid from stdin, prints the path |

## Running the console version

```
python astar.py < grid.txt
```

Input format:

```
<width> <height>
<height rows of <width> characters>
```

| Char | Meaning |
|------|---------|
| `.`  | open cell |
| `#`  | wall |
| `S`  | start |
| `G`  | goal |

Short rows are padded with walls. The grid must contain both `S` and `G`.

Example:

```
5 4
S....
.###.
.#G..
.....
```

Output is the path from `S` to `G` inclusive, one `x,y` per line (origin at the top-left, `x` to the right, `y` down), or `no path` if the goal is unreachable.

## How the algorithm works (`astar_core.py`)

```python
a_star(walls, width, height, start, goal, explored=None) -> list[(x, y)]
```

- `walls`: set of blocked `(x, y)` cells
- `explored`: optional set that gets every expanded node added to it, so a caller can visualize the search. It does not change the return value.
- Returns the path as a list of `(x, y)` from `start` to `goal`, or `[]` if unreachable.

### Core ideas

A* picks the next cell to expand by lowest `f = g + h`:

- **g**: the cheapest known cost from `start` to that cell.
- **h**: the heuristic estimate of remaining cost to `goal`. Here it is the straight-line (Euclidean) distance, `math.hypot`.
- **f**: total estimated cost of a path going through that cell.

### Data structures

- `open_heap`: a min-heap (`heapq`) of `(f, cell)`, the frontier still to be expanded.
- `g_score`: dict of the best known cost to reach each cell.
- `came_from`: dict mapping each cell to the cell it was reached from, used to rebuild the path.

### Steps

1. Push `start` onto the heap with `f = h(start)`; `g_score[start] = 0`.
2. Pop the cell with the lowest `f`. Record it in `explored` if one was given.
3. If it is the goal, walk `came_from` backwards, reverse, and return the path.
4. Otherwise look at each neighbor. Skip it if it is out of bounds or in `walls`.
5. Compute `tentative_g = g_score[current] + step_cost`. If that beats the neighbor's stored `g` (missing means infinity), record the new `g` and `came_from`, then push it with `f = tentative_g + h(neighbor)`.
6. If the heap empties without reaching the goal, return `[]`.

### Notes and behavior

- **Lazy deletion:** there is no closed set. When a cell is found via a cheaper route it is pushed again, and the older, worse entry stays in the heap. It can be popped later and re-expanded, but it will not overwrite better `g` values, so the result stays correct. This is simpler than a decrease-key heap at a small cost in extra work.
- **Optimality:** the Euclidean heuristic never overestimates the true cost on this grid, so it is admissible (and consistent). The returned path is therefore the shortest one.
- **Movement:** `_DIRS` currently lists only the four cardinal directions, so paths move up/down/left/right at cost 1.0. The function's docstring says "8-directional" and `step_cost` already handles diagonals (cost `sqrt(2)`), but the diagonal entries are missing from `_DIRS`. To enable diagonal movement, add `(1, 1), (1, -1), (-1, 1), (-1, -1)` to `_DIRS`. Diagonal moves squeeze between two touching walls, since corner cutting is not checked.
- **Ties:** heap entries are `(f, cell)` tuples, so equal `f` values are ordered by cell coordinates.
