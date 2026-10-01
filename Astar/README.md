# 3D A* Pathfinding

A* pathfinder for a 3D arena defined in centimetres. The algorithm lives in `astar_core.py`; `arena_env.py` defines the physical environment, and `astar_gui.py` is a tkinter GUI that plans through it with `plan_path()`.

## Arena and resolution

The environment (walls, moveable area, boxes, carriage and lift) comes from `arena_env.py`; see the Arena environment section below. The only planner setting in `astar_core.py` is the resolution:

```python
RESOLUTION_CM = 0.25
```

A* plans the **tool point**: carriage x, carriage y and lift z, all in cm, over the arena's moveable area (`move_x` by `move_y` by `move_z`). The search runs on a lattice of nodes 0.25 cm apart. Node `(i, j, k)` is at `(i, j, k) * 0.25` cm, and both edges of the moveable area are included, so with a 154 x 104 x 30 cm area the lattice is 617 x 417 x 121 nodes (about 31 million). Listing blocked nodes one by one would not fit in memory at this resolution, so obstacles stay as boxes (`BoxObstacles`).

### How the environment is passed in

The carriage and lift plate have size, so the planner does not just avoid the boxes: `Arena.configuration_boxes()` grows every box into the region of tool-point positions where the mechanism would touch it. The path is then a plain point path through the leftover free space.

- **Carriage:** it fills the full arena height, so a box blocks every lift z wherever the carriage footprint would overlap it.
- **Lift plate:** it sits off one face of the carriage, so a box blocks the tool point over an offset region, and only for the z range where the plate would overlap it vertically. The plate's height above the floor is z plus `lift_floor_margin`.
- **Touching is allowed**, matching `Arena.collisions()`. `plan_path(..., clearance=cm)` adds extra room around every box.
- **Walls** are not included: the moveable area already keeps the mechanism inside them.

I checked this against `Arena.collisions()` at 20,000 random lattice points with no mismatches, and a planned path across the default layout is collision-free at every point.

## Files

| File | Purpose |
|------|---------|
| `arena_env.py` | The environment (walls, boxes, carriage, lift), with a standalone jog/geometry viewer |
| `astar_core.py` | `BoxObstacles`, `a_star` (lattice search), `plan_path` (the two-pass planner the GUI uses) |
| `esp_link.py` | WiFi link that streams a planned path to the ESP32 |
| `astar_gui.py` | GUI: plans through the arena, with a top-down map and a rotatable 3D view |

## GUI use

Run `python astar_gui.py`. It loads the default arena and box layout from `arena_env.py`; to change either, edit that file.

- **Start / Goal**: pick one, then click the map (x right, y up, origin bottom left of the moveable area). It is placed at the **Lift z** slider's value. The carriage and plate are drawn where you click, and turn red if they overlap a box.
- **Run A\***: plans the tool point around all the boxes. It refuses to run if the start or goal is in collision, and after planning it checks every path point against the arena and reports whether the path is collision-free. Planning gives up after the **Timeout (s)** in the toolbar (default `PLAN_TIMEOUT_S` in `astar_core.py`) and says it timed out.
- **Path** slider: steps the carriage and lift along the planned path, in both views.
- **Diagonals**: 26-direction moves (on by default).
- **Show explored (3D)**: shows the cells the search expanded, as faint dots. It is off by default because recording them uses extra memory.

### Picking up and placing a box

1. Set the box type's **pick sequence** (default `y-2, z3` for both types) and **lift height**, which is how far above the box's bottom its lift point is on its back face (default 8 cm for large, 6.5 cm for small), and **lift y offset**, which moves that lift point in y from the back face: + is away from the box, − is into it (default 0). Then click the box, **Run A\***, then **Send path**. Pick and drop only take effect once a path has actually run: on the machine, or, when not connected, in the simulation that **Send path** plays instead (it moves Start to the path's end; **STOP** halts it).
2. The box sits still through the pick sequence (the plate may overlap it). Only once the whole sequence has run is the box **on the lift**, held where it was at that moment. It moves with the tool point, and A* plans around the walls, floor and other boxes for it as well. A box with another box stacked on it is not picked up.
3. **Click or drag on the map** (or click the floor in the 3D view) to choose where the carried box goes. A click centres the box on that point, and you can drag the box or its outline to adjust it. The dashed outline shows where it will land: on the nearest surface below it, meaning the top of the highest box under that footprint, or the floor (its top height is shown). The outline turns red if the box would overlap something or the carriage can't reach the spot. **Run A\*** plans a path to just above that spot, then lowers the box straight down as far as it will go, until its bottom meets the surface. If anything else stops the lowering first, the box isn't placed. The box then sits still while the pick sequence runs in reverse to drop it, and the status line says how many drop steps there are. The sequence reversed is the one that picked the box up. If that was empty, the drop uses the one currently set for the box type.
4. **Place box here** cancels a box move: it plans setting the carried box down where it is now, lowered onto the surface under it, followed by the drop sequence, ready to **Send path**. If a path is running, press **STOP** first.
5. **Boxes: Load on lift** puts a new box of the chosen type on the lift, where a finished pick would have left it with the lift at Start (the type's pick sequence undone from Start). Use it for a box that's already on the real lift, or to try placing without picking first. It's refused if a box is already on the lift or the new box would overlap something, so move the lift somewhere clear first. **Remove all boxes** empties the arena, including a box on the lift.
6. While you are carrying a box, clicking another box is refused. When not connected, **Clear** puts the carried box back where it was picked from (a box loaded onto the lift is removed instead).

On the map, boxes show their top height in cm (stacked boxes draw on top of those below). The right panel shows the whole arena in 3D: **drag to rotate, wheel to zoom**. The path is drawn at the plate's underside height.

## Driving the ESP32 (`esp_link.py`)

The GUI's machine bar sends the planned path to the `esp32_4axis` controller over WiFi, using the same WebSocket (`ws://<host>:81`) as `stepper_gui.html` (needs `pip install websocket-client`).

1. Enter the board's hostname (`stepper.local`) or IP and **Connect**. All three axes must be homed and the drives on.
2. **Start is always the machine's current position** (carriage x, y and lift z), updated live while connected; clicks on the map set the Goal only. Click a Goal and **Run A\***. If the machine moves after planning, run A* again, because the path must begin where the machine is.
3. Set **Speed** and press **Send path**. **STOP** sends `S`. The map follows the commanded position.
4. **Home** plans A* from where the mechanism is to the position in the box next to it (default x 20, y 0, z 10 cm; `HOME_POSE` in `astar_gui.py`) and sends the path straight away, or simulates it when not connected. A box on the lift goes along.

The firmware runs each axis independently, so the host plans the motion and streams it. **The tool runs through the corners of the path without stopping.** `plan_profile()` plans one speed profile for each run: it speeds up from rest, cruises at the chosen speed (capped at 70% of each axis's `run` speed, for its share of the motion), and slows down for each corner and for the end. Each corner is taken just slowly enough that no axis has to change speed by more than it can within `CORNER_TOL_CM` (0.1 cm) of the line. The plan uses only 50% of each axis's `acc`, keeping the rest spare to absorb corners and lag, so shallow corners are taken almost at full speed and sharp ones slowly. Every 50 ms the host sends each moving axis an `MS` command: its share of the planned speed, aimed at the end of its current one-direction stretch. FastAccelStepper takes a new `MS` mid-move without stopping, and each axis lands exactly where its stretch ends. Each 250 ms status line nudges every axis back toward where the plan has it (`TRACK_GAIN`). Against a simulated controller with the live axis settings, the tool stayed within 0.13 cm of the planned line, well inside the 0.5 cm planning clearance. The machine still comes to rest at the start and the end, and at every point of a pick or drop sequence, where the plate is engaged with a box and the moves must be exact. `smooth_path()` first pulls the 0.25 cm lattice staircase into a few straight, collision-checked segments. Paths that leave a firmware soft limit are refused before anything moves.

**Check `AXIS_MAP` in `esp_link.py` before first use.** x = axis 3, y = axis 1 (the gantry pair), z = axis 4 are guesses from the travel lengths in the ESP32 README, and it assumes step 0 after homing is tool coordinate 0. Set `sign` / `offset_cm` per axis if a direction is reversed or the zero is elsewhere. Axis 4 is open loop, so a stall there is invisible.

The corners of the path are followed with a little tracking lag, and `plan_path` allows touching a box. The GUI's **Clearance** field (default 0.5 cm) keeps the path off the boxes.

## How the algorithm works (`astar_core.py`)

### `a_star(walls, width, height, depth, start, goal, explored=None, diagonals=True, weight=1.0)`

Standard A* on the node lattice, choosing the next node by lowest `f = g + weight * h`:

- **g**: the cheapest known cost from `start`, where a move costs its Euclidean length in lattice steps (1, `sqrt(2)` or `sqrt(3)`).
- **h**: the exact open-space cost to the goal. With 26 moves that is the 3D octile distance, `sqrt(3)*d1 + sqrt(2)*(d2-d1) + (d3-d2)` for sorted axis distances `d1 <= d2 <= d3`. With 6 moves it is Manhattan distance. Both are admissible and consistent.
- Ties on `f` go to the node with the smaller `h` (`f` is rounded to 9 decimals so float noise doesn't break ties). Without this, the many equal-cost routes across a 3D lattice make the search flood millions of nodes even in an empty arena.
- A closed set skips stale duplicate heap entries.
- `weight > 1` is weighted A*: far fewer expansions, at most `weight` times the optimal length.

`walls` can be any object supporting `node in walls`, such as a set of nodes or a `BoxObstacles`.

### `plan_path(arena, start_cm, goal_cm, res=0.25, coarse_res=2.0, diagonals=True, explored=None, corridor=1, fine_weight=2.0, clearance=0.0)`

Even with the tie-breaking, a real obstacle forces the search to flood the shadow behind it, and plain A* over tens of millions of nodes takes minutes in Python. So planning is two-pass:

1. **Coarse pass:** A* at 2 cm (about 78 x 53 x 16 nodes) over the whole arena. Boxes are grown by half a coarse cell so thin obstacles cannot slip between coarse nodes. If that finds nothing, it retries without the growth.
2. **Fine pass:** weighted A* (`fine_weight` 2.0) at 0.25 cm, confined to a corridor of `corridor` coarse cells around the coarse path. If it fails, the corridor is widened (2x, then 4x).

Returns cm points from start to goal, or `[]`. Paths across the default layout took about 0.2 s in testing.

Trade-offs of the two-pass approach:

- The path is near-optimal, not guaranteed shortest.
- A gap much narrower than `coarse_res` (2 cm) may be missed, so the planner can report `no path` for a goal that is technically reachable through it.
- The fine path is a staircase of 26-direction moves and is not smoothed.

## Arena environment (`arena_env.py`)

`Arena` models the physical setup: the outer walls, the moveable area, the carriage and the lift (rod and plate). Run it standalone to check the geometry and jog the mechanism by hand:

```
python arena_env.py
```

The window shows top, front (x-z) and side (y-z) views. Jog with the +/- buttons, the arrow keys (x/y), PgUp/PgDn or w/s (z), or type a position and press Go. `h` or Home puts the carriage at x = y = 0 with the lift raised. Motion is clamped to the travel limits, and any overlap between the moving parts and a wall or obstacle is drawn red and listed in the status line.

**All wall, carriage and lift dimensions in `ArenaDims` are placeholders**, so edit them from the CAD. The carriage is a vertical box that fills the full arena height (`wall_height`) and moves in x and y only. The lift plate rides up and down its `+y` face (`plate_side`), with no rod, and its z is the lift's own reading, 0 at the low position and `move_z` at the high one. `lift_floor_margin` does not limit that travel: it places the arena floor, so with the lift reading 0 the plate is `lift_floor_margin` above the floor. The tool point is the carriage centre plus the plate's underside height. The wall clearance is worked out from how far the carriage and plate reach past that point, plus a separate gap for each wall (`wall_gap_x_min`, `wall_gap_x_max`, `wall_gap_y_min`, `wall_gap_y_max`).

There is also an **overhead wall** across the whole x span of the arena. It is `wall_thickness` thick and runs from `overhead_wall_bottom` (31 cm) above the floor up to `wall_height`. Its +y face is at y = `overhead_wall_y` (-3 cm), in the lift's y coordinate, and it extends toward -y from there. Unlike the outer walls it is within the mechanism's reach. Near y = 0 the plate slides under it, so there the lift can only go up to 24.75, and A* plans around it. Home (x = y = 0) therefore raises the lift only as far as it will go without touching anything. The map draws the wall hatched.

The standalone window starts with a default layout of floor-standing boxes: 3 large (32 x 20 x 15 cm) and 4 small (15.5 x 21 x 14 cm). Sizes are in `BOX_TYPES` and positions in `DEFAULT_LAYOUT` (`(type, x, y)` or `(type, x, y, z)` for a box stacked at height z; x, y is each box's corner nearest the origin), both in `arena_env.py`.

From code: `Arena().jog("x", 5)`, `.move_to(x, y, z)`, `.collisions()`, and `Arena(obstacles=[(x0, y0, z0, x1, y1, z1)])`. It is used by `astar_core.plan_path()` and `astar_gui.py`.

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
plan_path(arena, start_cm, goal_cm, res, coarse_res, diagonals, explored,
          corridor, fine_weight, clearance):
    boxes = arena.configuration_boxes(clearance)     # obstacles grown by the mechanism
    size  = the arena's moveable area (move_x, move_y, move_z)
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
