# A\* rectangle sorting study

A slide-block puzzle with standard-size square boxes in a rectangular arena. You
pick a box and a destination; A\* works out the cheapest sequence of slides that
gets it there, **including which other boxes have to be shoved out of the way**,
under a cost function you tune.

```bash
python main.py
```

Standard library only — tkinter for the GUI, no third-party dependencies.

## Using it

| Action | Control |
| --- | --- |
| Choose the floor | **small / medium / large** counts, then **Regenerate** |
| Empty the arena | **Clear floor** (counts are kept, so Regenerate refills) |
| Rearrange by hand | tick **Drag boxes with the mouse**, then left-drag |
| Select a box | left-click it |
| Set its goal | right-click anywhere, or left-click empty floor |
| Nudge the goal | **arrow keys** (hold shift for five cells at a time) |
| Choose a planner | **Relaxed** (fast) or **Exhaustive** (optimal) |
| Choose a planner | **Conflict-based** (fast) or **Exhaustive A\*** (optimal) |
| Search | **Plan move** — press again to stop optimising and keep the best plan |
| Watch it | **Play / Step / Rewind**, speed slider |
| Start over | **Reset layout** |

Arrow keys act on the canvas, so click it once to give it focus (clicking a box
does that anyway). The first arrow press puts the goal one cell off the selected
box's current position; after that it walks from wherever the goal already is,
and stops at the arena edge.

The layout you end up with is the layout you plan from — after playback, or
after dragging a box by hand, the current arrangement becomes the new starting
layout, so moves chain one after another. **Reset layout** returns to that
arrangement, not to the original random one.

## The boxes

Three standard sizes, all squares: **small 0.10 m**, **medium 0.20 m**, **large
0.30 m**, coloured green / blue / red. You give counts per type and the
generator drops them at random legal positions on the movement grid, biggest
first — a large box dropped into an arena that is already half full is the case
that fails, so it gets first pick while the floor is empty.

If the request cannot be honoured the layout is left alone and the panel says
why: nothing requested, impossible by area, a box larger than the arena, or
rejection sampling failing to find room (which is the interesting one — it comes
with the floor coverage so you can see how tight you are asking it to pack).

Sizes and counts live in `study_config.py`; the arena is `ARENA` there too.

## The gripper

Two L-shaped jaws — one fixed, one travelling along the diagonal towards it —
close on **opposite corners** of a box. Each jaw is an L: two arms
`GRIPPER_THICKNESS` (0.01 m) thick, running `GRIPPER_REACH` (0.05 m) along each
edge from the corner.

The jaws sit *outside* the box, so picking one up needs empty floor at its
corners. That makes some boxes simply unliftable — one wedged into an arena
corner has no room for a jaw on either side, and no amount of space to slide it
afterwards helps. Either diagonal will do, though, which is why this is modelled
properly rather than by just inflating every box by the jaw thickness: a box with
a neighbour hard against its bottom-left corner is still perfectly grippable from
the other diagonal.

Generated layouts respect it. `LayoutGenerator` places a box only if the jaws can
reach it *and* every box already down is still reachable — dropping a box beside
another can take away the last diagonal that one had, stranding it on the floor
forever. It matters more than it sounds:

| layout | boxes pickable without the check | with it |
| --- | --- | --- |
| 9 boxes, 0.05 grid | 7 / 9 | **9 / 9** |
| 16 boxes, 0.05 grid | 10 / 16 | **16 / 16** |
| 8 large, 0.02 grid | 7 / 8 | **8 / 8** |

The jaws are drawn on the canvas. Standing still they preview the grip the
selected box would get, in red if neither diagonal is free and that box cannot be
picked up at all. **During playback they ride the box being carried**, on the
diagonal the plan actually recorded, so you can watch the machine change grip as
it moves from one box to the next. A faint outline around them shows the open
position — the room the plan had to reserve so the jaws can let go again.

### The solver plans for the mechanism, not just the boxes

Give `Board` a gripper and a move is legal only if the machine can make it. Three
things follow, and they are what `Board.can_carry` and `Board.carry_path` enforce:

- **The jaws travel with the box.** The body that has to fit through a gap is the
  box *plus its jaws*, so a carried box cannot always follow a route the bare box
  would slide along.
- **They are modelled open, not closed.** The travelling jaw is backed off by
  `GRIPPER_STROKE` along its diagonal, because the machine has to be able to let
  go where it arrives as well as take hold where it started.
- **A grip cannot change mid-carry.** Which diagonal is held is part of
  `PuzzleState`, chosen when a box is picked up and fixed until it is put down.
  `Board.neighbors` offers both diagonals on a fresh pick and only the held one
  while carrying.

Every planner goes through the same hook, so all three respect it: A\* through
`neighbors`, decomposition and repair through `carry_path`. Decomposition's
routing masks also carry a margin wide enough for the jaws whichever way round
they end up — without it, it routes through gaps the real check then rejects and
throws the whole plan away.

The cost is reach. On a 9-box floor the optimal plan goes from 1.550 to 1.900
before repair, and a 6-large-box layout on a 0.02 m grid that solves without the
gripper finds no route at all with it — the jaws simply do not fit through the
gaps between large boxes at that spacing.

The clearance costs packing density, and at the top end it costs a lot. Over 20
seeds, 12 large boxes on a 0.02 m grid went from 14 layouts to 8, and 32 boxes at
56% floor coverage went from 20 to **none** — random placement cannot find a
legal arrangement that tight. That is a limit of rejection sampling rather than a
proof that no such layout exists, but it does mean the densest benchmarks in this
README are no longer generatable with the gripper switched on.

## The cost function

Two weights, both live on the panel:

- **Distance** — cost per metre travelled, by any box. This is the "total
  distance of all moved boxes" term.
- **Re-grip** — cost charged each time the mechanism has to start moving a
  *different* box than the one it moved last. This is the "number of boxes
  required to move" term: shoving three boxes aside pays it three extra times,
  and a box pushed, released, then pushed again pays it twice.

Raise the re-grip weight and the planner will drive a box the long way around
rather than clear a path through two others. Raise the distance weight and it
does the opposite.

**Heuristic weight** is not part of the cost. At `1.0` A\* returns a provably
optimal plan; above `1.0` it searches greedily — far fewer nodes, and a plan
guaranteed to be at most that factor above optimal. It is the knob to reach for
when a search is slow, alongside the **grid step** (the biggest lever of all:
halving it roughly quadruples the reachable layouts).

One non-obvious interaction: setting the **re-grip weight to zero makes searches
much harder**, not easier. Free grips mean a huge number of box shuffles cost
exactly the same, so A\* has an enormous plateau of equally good layouts to sift
through — and the heuristic loses its grip terms at the same time.

## Performance

The heuristic does the heavy lifting. It costs the target's Manhattan distance
plus a grip, *and* — for every box sitting on the goal footprint — that box's
minimum escape distance plus a grip of its own. Those boxes provably have to
move, and every term belongs to a different box and a different grip, so the
estimate never overshoots and the plan stays optimal.

Measured against the same planner using a distance-only heuristic, at default
weights, identical plan costs in every case both could solve:

| case | grid | before | after |
| --- | --- | --- | --- |
| A → C's spot | 0.04 | 11,913 nodes / 0.81 s | **322 / 0.02 s** |
| A → F's spot | 0.04 | 19,525 nodes / 1.40 s | **378 / 0.02 s** |
| A → C's spot | 0.02 | *150k budget blown* | **2,271 / 0.14 s** |
| D → C's corner | 0.02 | *150k budget blown* | **40,366 / 3.6 s** |

Two smaller changes ride along: box overlap in the search's hot path is decided
by integer configuration-space ranges instead of building rectangles (worth
about 1.3× per node — `Board.can_place`, checked against the float geometry by
`Board._verify_cspace` at construction), and the open heap breaks ties toward
larger `g` so the search drives at the goal rather than fanning across a plateau.

### Decomposition: finding *a* plan

A\* searches the joint space of every box position at once. That is what makes it
optimal and what makes it hopeless on a crowded floor — and it is not a heuristic
problem. An obstruction-counting heuristic bought 1.4× on boards that already
worked and nothing at all on the ones that did not.

`DecompositionPlanner` asks a smaller question instead (the shape of Stilman &
Kuffner's work on navigation among movable obstacles). The important part is
*when* it asks:

1. **Route** — where the target would go if the other boxes were not there. One
   box, a few hundred cells, breadth-first — microseconds.
2. **Drive** — move it along that route as far as it legally can *right now*.
3. **Clear** — it is now nose-to-nose with one box. Shove that box out of the way
   of the route the target has *left to drive*.
4. **Repeat** — drive on, re-evaluating after every shove. A box that will not
   budge at all becomes scenery and the target routes around it.

Driving before clearing is what makes it robust, and the reason is worth being
precise about. Clearing the whole corridor up front demands that every box in the
way find a home outside the *entire* route, which on a packed floor often has no
solution. Clearing as it goes asks for much less: the keep-out region shrinks
every time the target advances, cells the target has already driven past are free
to park in — a box can be shoved into the space right behind it — and the floor a
box escapes into is the real one at that moment rather than a worst-case
snapshot. Requiring the full path to be clear before the target moves was the
single biggest limit on what this could solve.

Two subtleties, both of which cost real solutions when they were wrong:

- A box only has to clear the region actually in its way, and that region differs
  per box. A box on the target's route must clear the *target's* corridor; a box
  pinning one of those must clear *that box's* escape route. Conflate them and a
  box already outside the target's corridor looks "done" while being the very
  thing blocking its neighbour.
- A box named as a pin has to go to the **front** of the queue. Left where it
  was, whatever is waiting on it re-derives the same pin forever — a livelock
  indistinguishable from a hopeless layout, and immune to raising the iteration
  cap.

Cost tracks the number of *obstructions* — usually a handful — not the number of
boxes on the floor, which is why it survives densities that stop A\* outright.

| board | plain / anytime A\* | decomposition |
| --- | --- | --- |
| 24 boxes, 42%, middle | 1.800 in 0.39 s | **1.850 in 0.8 ms** |
| 24 boxes, 42%, far corner | 11.150 after 2.3 s, 7.050 after 42 s | **8.600 in 5 ms** |
| 32 boxes, 56%, middle | nothing, at any weight | **4.450 in 3 ms** |
| 32 boxes, 56%, far corner | nothing | **15.350 in 15 ms** |
| 40 boxes, 69%, middle | nothing | **14.850 in 9 ms** |

The plans are cheaper to find, not cheap: driving and clearing in steps costs
grips that clearing the whole corridor up front does not. That is what the repair
pass below is for.

### Repairing the route

There are two ways to improve a plan, and they are not interchangeable.
`AnytimePlanner` searches for a *different* plan and keeps it if it is cheaper —
it never looks at the route it was given except as a price to beat. On a crowded
floor that search gets nowhere, so it hands back what it started with, unchanged.

`PlanRepairer` edits the route it is given:

| edit | what it buys |
| --- | --- |
| **drop** | take out everything one box did — if the plan still works, that box never needed moving: its travel *and* its grip |
| **group** | slide a leg next to another leg of the same box. A grip is only a *change* of which box is moving, so A,B,A costs three and A,A,B costs two — same moves, different order |
| **front-load** | put every shove ahead of the target's first step so it drives in one run — one grip instead of seven |
| **straighten** | re-plan a single leg between its own endpoints against the floor as it actually stands then |
| **collapse** | where a box was shoved twice, send it straight to where it ended up |

**group** is the one that matters when re-grips are what you are paying for.
Decomposition interleaves by nature — drive, shove, drive, shove — and every
alternation is a grip that reordering can often remove outright.

Every edit is applied blindly and then *replayed* move by move against
`Board.can_place`; anything that does not survive is thrown away. That is what
makes guessing safe, and it is why the pass is a few lines per edit rather than a
proof obligation.

Measured over 437 feasible plans: **all legal before and after, 46% improved**,
mean gain 15.4% where it helps, best 36.5%, and **24% fewer grips overall**. On
the 24-box far corner it turns 8.600 into 6.100 (19 grips down to 9) in 9 ms; on
32 boxes at 56% it recovers 3.700 exactly — the same plan the old
clear-the-whole-corridor approach produced, on a board that approach could not
solve.

The three stages run in order on one button press: decomposition finds a route,
repair tidies it, A\* searches for better still and supplies the bound.

### Do not set the distance weight to exactly zero

If only re-grips matter to you, a *small* distance weight still beats zero — at
zero the planner gets worse at minimising grips, which is not obvious.

Twelve large boxes on a 0.02 m grid, corner to opposite corner:

| distance weight | grips after repair | moves |
| --- | --- | --- |
| 0.0 | 19 | 330 |
| 0.05 | **13** | **252** |

Two things break at zero. **straighten** goes inert — shortening a leg cannot
lower a cost that ignores distance, so the plan keeps every detour, and those
detours are what make legs collide and block the reordering that removes grips.
And A\* is left with a heuristic of about 0.25 against a true remaining cost near
5, on a landscape where moving one box any distance is free: an enormous plateau
of equal-cost states. It expanded 40,000 nodes in 8.5 s and improved nothing, at
either weight — with distance ignored, decomposition and repair are doing all the
work.

A box may be shoved **more than once** — if it gets in the way again later it is
simply cleared again, from wherever it now stands. And the target is picked up as
many times as the driving needs. Neither is free (each costs a re-grip) but both
buy feasibility, and the optimiser is there to trade them back down.

Two strategies run, because which one wins is not predictable from the layout:
clearing the whole route up front is cheaper when it works, since the target then
drives in a single leg, but the placements it commits to can box in whatever is
met later. Each takes milliseconds, so both run — with their own reroute
sequences, so one strategy's dead end cannot send the other down the wrong path —
and the cheaper plan wins.

Measured over 600 goals spanning box counts, grids, seeds and weight settings:
**98% solved, every plan legal, slowest 110 ms.** The target is gripped more than
once in 278 of them and some box moves twice in 212 — the interleaving is not a
rare fallback, it is how most of the harder plans get made.

It is a feasibility finder, not an optimiser — which is exactly how it is used.
The plan it returns goes straight on the board and then seeds `AnytimePlanner`,
so the search is priced against a real plan from its first node. On dense boards
that is the difference between a plan and none at all: over 16 goals at 20–24
boxes with a 4 s leash, the cold optimiser found **nothing on 7 of them** while
the seeded one always had something. (Not strictly dominant — on one goal the
cold run ended up cheaper, because the seeded run spends its budget differently.)

What it will not resolve is a **cycle**: a cluster of boxes that pin each other
in a loop, where L8 needs L7 moved, L7 needs L6, and L6 needs L8 back. Breaking
one of those needs a box parked somewhere temporary and then moved again in a
coordinated way, which a greedy chase with no backtracking cannot find. It
detects the cycle and gives up rather than spinning.

### Anytime planning

Plain A\* is all-or-nothing. On a crowded floor it either returns the optimal
plan or, after half a minute, nothing at all — and you cannot tell which it will
be until it is over. That is the wrong shape for something a person is sitting in
front of.

`AnytimePlanner` runs the same A\*, repeatedly, with the heuristic inflated by a
weight that starts high and comes down:

| weight | behaviour |
| --- | --- |
| 25 | greedy — drives at the goal, ignores most alternatives, usually has *a* legal plan quickly. May cost up to 25× the best possible. |
| 10, 5, 3, 2 | progressively less greedy, better plans, more search |
| 1.0 | plain admissible A\*: optimal, and slow when the floor is full |

Every run that beats the plan in hand hands it straight to the GUI, so **the
board shows the best plan so far while the search carries on** — the dashed route
and the outlined end positions update as it improves, and the panel prints the
guarantee beside the cost. Pressing **Plan move** again stops the optimiser and
keeps whatever is in hand; it is always legal and its bound is always known.

It opens at 25 rather than something modest because the first round has one job:
return *a* plan. On a 24-box floor at 42% fill, reaching a far corner took 6.8 s
at weight 5 and 2.0 s at weight 25 — worse plans, far sooner, with the later
rounds there to fix the cost.

Two things stop the later rounds wasting effort. **Incumbent pruning**: once a
plan costing C is in hand, any node whose `g + h` already reaches C cannot lead
anywhere better, because `h` never overestimates — so it is dropped on sight, and
later rounds are far cheaper than running them cold. And a **shared node budget**,
so a hopeless final round cannot run forever.

Restarting each round rather than repairing the previous search tree (what ARA\*
does) throws work away. It also keeps this to one readable loop, and with
incumbent pruning the repeated rounds are cheap enough that the trade reads well
for a study.

Measured on a 24-box floor at 42% fill:

| goal | plain A\* | anytime |
| --- | --- | --- |
| middle | 1.800 in 0.39 s | **first plan in 12 ms** (2.100), optimal 1.800 by 0.44 s |
| far corner | **fails** after 120,000 nodes / 60 s | **first plan in 2.0 s** (11.150), improving to 8.550 then 7.050 |

Across 200 goals over two re-grip weights: **199 solved, 370 plans streamed,
every one legal and each strictly cheaper than the last, no bound ever
violated**, final answer matching exhaustive A\* on 193 and solving 2 that
exhaustive A\* could not.

### What is still hard

Decomposition raised the ceiling a long way but did not remove it. A far corner
at 69% fill and anything at 83% still come back with nothing. The chase there
runs deep, shoving boxes repeatedly, and ends on a mutually-pinned cluster of
large boxes — not on a tuning limit; raising the iteration caps changes none of
those cases. With 0.36 m² of free floor at 83% fill and a large box needing
0.09 m², there is often genuinely nowhere to put what has to move.

Worth keeping in mind: "no plan found" and "no plan exists" are still different
things here, and nothing in the study distinguishes them.

Memory is worth knowing about: a 120,000-node search on a crowded floor peaks in
the hundreds of MB, because every node stores the whole layout. Large node
budgets on crowded floors will use a lot of RAM.

## Layout of the code

| File | Holds |
| --- | --- |
| `geometry.py` | `Rect` — the only place overlap and containment are defined |
| `gripper.py` | `Gripper` — the two L-shaped jaws, and what clearance they need |
| `layout.py` | `LayoutGenerator` — the standard box catalogue and random placement |
| `puzzle_state.py` | `PuzzleState` — the dynamic half: a grid cell per box, plus the box last moved |
| `board.py` | `Board` — the static half: arena, box sizes, grid; owns the legality rule and generates successor states |
| `cost_model.py` | `CostModel` — the tunable weights and the matching A\* heuristic |
| `astar_planner.py` | `AStarPlanner` — exhaustive A\* over whole layouts; `Plan`, `Move` |
| `decomposition.py` | `DecompositionPlanner` — finds *a* plan fast by clearing obstructions |
| `plan_repair.py` | `PlanRepairer` — rewrites that plan into a cheaper one |
| `anytime_planner.py` | `AnytimePlanner` — searches for better still, with a bound |
| `puzzle_gui.py` | `PuzzleGUI` — canvas, controls, playback; runs searches on a worker thread |
| `study_config.py` | Arena, boxes, grid step, default weights, slider ranges, colours |
| `main.py` | Wires the four together and runs the window |

A search node is a **whole layout**, not a position — that is what makes moving
an obstruction a first-class part of the plan. Edges come from `Board.neighbors`
(one box, one grid step, four-connected) and are priced by `CostModel`.
`last_moved` rides along in the state so the re-grip charge can be counted.

## Simplifications, and where they give way

The mechanism is currently idealised: it can grab any box from any side and slide
it in any direction, and it occupies no space itself. A move is legal when the
box stays inside the arena and overlaps no other box.

That rule lives in exactly one method — `Board.can_place`. Modelling the real
gantry's geometry (swept volume of the paddle, an approach direction implied by
the push direction, a stand-off clearance) is a new test inside `_fits` plus
whatever extra state it needs; the search, the cost model and the GUI do not
change. The same goes for costing a move by mechanism travel rather than box
travel — that is `CostModel.move_cost`, with a matching relaxation in
`CostModel.heuristic` to stay admissible.

Other deliberate limits:

- Box corners snap to the grid, and the grid slider re-snaps the *current*
  arrangement rather than regenerating it. When the boxes are packed tightly a
  coarser step would push two of them into each other, so the change is refused
  and the slider reverts — regenerate on the new step if you want it anyway.
  With 0.10 / 0.20 / 0.30 m boxes the steps that always re-snap cleanly are
  0.02, 0.05 and 0.10.
- Moves are four-connected, so every step costs the same distance. Diagonals
  would need the heuristic to change with them.
- One box moves at a time, one cell at a time.
