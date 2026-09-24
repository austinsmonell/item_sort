"""The arena environment: outer walls, moveable area, carriage and lift.

All lengths are cm. World frame: origin at the corner of the moveable area,
x to the right, y up the page (top view), z up from the floor, so x = 0,
y = 0 is the bottom left of the top view.

  * The arena floor is z = 0 and the walls are `wall_height` tall.
  * The *carriage* is a vertical box that occupies the full height of the
    arena (floor to `wall_height`). It moves in x and y only and never moves
    in z. Its centre is the reference point (x, y) that moves over the
    moveable area: x in 0..move_x, y in 0..move_y (astar_core.py uses the
    same 180 x 160 x 35).
  * The *lift plate* is attached to one face of the carriage (`plate_side`)
    and travels up and down along it, with no rod. The lift coordinate z is
    the lift's own reading: 0 at its low position and `move_z` at its high
    position. `lift_floor_margin` locates the arena floor relative to the
    lift: with the lift reading 0, the plate underside is that far above the
    floor, so its height above the floor is z + lift_floor_margin. It does
    not limit the lift's travel.
  * The *tool point* is (carriage x, carriage y, plate z).
  * The *outer walls* enclose the moveable area. The clearance on each side is
    worked out from how far the carriage and plate stick out of the reference
    point, plus that side's wall gap (`wall_gap_x_min`, `wall_gap_x_max`,
    `wall_gap_y_min`, `wall_gap_y_max`), so the mechanism can reach every edge
    of the moveable area without touching a wall.

EVERY MECHANISM AND WALL DIMENSION IN ArenaDims BELOW IS A PLACEHOLDER.
Fill them in from the CAD, then run this file to check the geometry:

    python arena_env.py

The window shows top, front (x-z) and side (y-z) views and lets you jog the
carriage (x, y) and lift (z) by hand. Anything that overlaps is drawn red
and listed in the status line.

Use it from other code the same way:

    arena = Arena()
    arena.jog("x", +5)            # clamped to the travel limits
    arena.move_to(90, 80, 20)
    arena.collisions()            # names of overlapping solid pairs
"""

import tkinter as tk
from dataclasses import dataclass, field
from typing import List, NamedTuple, Optional, Sequence, Tuple

Box = Tuple[float, float, float, float, float, float]  # x0 y0 z0 x1 y1 z1

AXES = ("x", "y", "z")


@dataclass(frozen=True)
class ArenaDims:
    # ---- moveable area: the range the carriage centre / plate z reach ---
    move_x: float = 154.0
    move_y: float = 104.0
    move_z: float = 30.0

    # ---- outer walls (PLACEHOLDERS) -------------------------------------
    wall_thickness: float = 2.0
    wall_height: float = 40.0
    # clearance between the mechanism and the inside of each wall, measured
    # with the carriage at the edge of the moveable area. min = the wall at
    # x = 0 / y = 0 side, max = the wall at x = move_x / y = move_y side.
    wall_gap_x_min: float = 3.0
    wall_gap_x_max: float = 3.0
    wall_gap_y_min: float = 30.0
    wall_gap_y_max: float = 10.0
    floor_thickness: float = 2.0   # drawn below z = 0; z = 0 is the floor top

    # ---- carriage (PLACEHOLDERS): vertical box on the floor -------------
    carriage_w: float = 2.0       # size along x
    carriage_d: float = 2.5       # size along y
    # (its height is the wall height: it fills the whole arena height)

    # ---- lift plate (PLACEHOLDERS): rides along one carriage face -------
    plate_side: str = "-y"         # which face: "+x", "-x", "+y" or "-y"
    plate_len: float = 2.0         # how far it sticks out of that face
    plate_width: float = 7.5      # size across the face
    plate_t: float = 2.2           # thickness (vertical)
    lift_floor_margin: float = 4.0  # plate underside above the floor at z = 0

    def __post_init__(self):
        if self.plate_side not in ("+x", "-x", "+y", "-y"):
            raise ValueError("plate_side must be '+x', '-x', '+y' or '-y'")
        if self.lift_floor_margin < 0:
            raise ValueError("lift_floor_margin must be at least 0")
        if (self.lift_floor_margin + self.move_z + self.plate_t
                > self.wall_height):
            raise ValueError("the plate would travel above the top of the "
                             "carriage: lift_floor_margin + move_z + plate_t "
                             "> wall_height")


# Box types that sit on the floor. Sizes are (x, y, z) in cm.
BOX_TYPES = {
    "large": (32.0, 20.0, 15.0),
    "small": (15.5, 21.0, 14.0),
}

# Default layout: (box type, x, y) or (box type, x, y, z), with (x, y) the
# box's corner nearest the origin in the world frame and z the height of its
# underside (default 0, the floor). Edit freely.
DEFAULT_LAYOUT = [
    ("large", 8.0, 8.0), ("large", 50.0, 8.0), ("large", 92.0, 8.0),
    ("small", 8.0, 45.0), ("small", 33.5, 45.0), ("small", 59.0, 45.0),
    ("small", 84.5, 45.0),
    # stacked on top of the boxes above (z = height of the box below)
    ("large", 8.0, 8.0, 15.0),        # large on large
    ("small", 58.25, 7.5, 15.0),      # small centred on the second large
    ("small", 84.5, 45.0, 14.0),      # small on small
]


def make_boxes(layout=DEFAULT_LAYOUT) -> List["Solid"]:
    """Turn a layout into named box solids."""
    counts = {}
    solids = []
    for kind, x, y, *rest in layout:
        z = rest[0] if rest else 0.0
        w, d, h = BOX_TYPES[kind]
        counts[kind] = counts.get(kind, 0) + 1
        solids.append(Solid(f"{kind} box {counts[kind]}",
                            (x, y, z, x + w, y + d, z + h), kind))
    return solids


class Solid(NamedTuple):
    name: str
    box: Box
    kind: str  # "wall", "obstacle", a BOX_TYPES name, "carriage" or "plate"


def _overlap(a: Box, b: Box, eps: float = 1e-9) -> bool:
    """True if two boxes share volume (touching faces do not count)."""
    return all(a[i] < b[i + 3] - eps and b[i] < a[i + 3] - eps
               for i in range(3))


class Arena:
    """The static environment plus the carriage/lift pose."""

    def __init__(self, dims: Optional[ArenaDims] = None,
                 obstacles: Sequence = ()):
        """obstacles: boxes (x0, y0, z0, x1, y1, z1) in cm, or Solids such as
        the ones make_boxes() returns."""
        self.dims = dims or ArenaDims()
        self.obstacles: List[Solid] = [
            o if isinstance(o, Solid) else Solid(f"obstacle {i + 1}", tuple(o), "obstacle")
            for i, o in enumerate(obstacles)]
        self.pos = [0.0, 0.0, self.dims.move_z]  # x, y, z (lift starts up)

    # ---- limits and motion ---------------------------------------------

    @property
    def limits(self) -> Tuple[Tuple[float, float], ...]:
        d = self.dims
        return ((0.0, d.move_x), (0.0, d.move_y),
                (0.0, d.move_z))

    def move_to(self, x: float, y: float, z: float) -> bool:
        """Move the tool point, clamped to the travel limits. Returns True if
        it got there exactly, False if any axis was clamped."""
        exact = True
        for i, v in enumerate((x, y, z)):
            lo, hi = self.limits[i]
            c = min(max(v, lo), hi)
            exact = exact and c == v
            self.pos[i] = c
        return exact

    def jog(self, axis: str, delta: float) -> bool:
        """Move one axis by delta cm. Returns False if it hit a limit."""
        i = AXES.index(axis)
        target = list(self.pos)
        target[i] += delta
        return self.move_to(*target)

    def home(self):
        """x = y = 0 with the lift fully raised."""
        self.move_to(0.0, 0.0, self.dims.move_z)

    # ---- geometry ---------------------------------------------------------

    def plate_box(self, x: float, y: float, z: float) -> Box:
        """The lift plate at lift reading z, carriage centre (x, y). The box is
        in world coordinates, so it sits lift_floor_margin higher than z."""
        d = self.dims
        z = z + d.lift_floor_margin
        sign = 1 if d.plate_side[0] == "+" else -1
        if d.plate_side[1] == "x":
            near = d.carriage_w / 2
            xa, xb = sorted((x + sign * near, x + sign * (near + d.plate_len)))
            ya, yb = y - d.plate_width / 2, y + d.plate_width / 2
        else:
            near = d.carriage_d / 2
            ya, yb = sorted((y + sign * near, y + sign * (near + d.plate_len)))
            xa, xb = x - d.plate_width / 2, x + d.plate_width / 2
        return (xa, ya, z, xb, yb, z + d.plate_t)

    def carriage_box(self, x: float, y: float) -> Box:
        d = self.dims
        return (x - d.carriage_w / 2, y - d.carriage_d / 2, 0.0,
                x + d.carriage_w / 2, y + d.carriage_d / 2, d.wall_height)

    def inner_box(self) -> Tuple[float, float, float, float]:
        """(x0, y0, x1, y1) of the floor inside the walls: the moveable area
        plus however far the carriage and plate reach past the reference
        point on each side, plus that side's wall gap."""
        d = self.dims
        c, p = self.carriage_box(0, 0), self.plate_box(0, 0, 0)
        reach = [max(-c[0], -p[0]), max(-c[1], -p[1]),
                 max(c[3], p[3]), max(c[4], p[4])]
        return (-reach[0] - d.wall_gap_x_min, -reach[1] - d.wall_gap_y_min,
                d.move_x + reach[2] + d.wall_gap_x_max,
                d.move_y + reach[3] + d.wall_gap_y_max)

    def walls(self) -> List[Solid]:
        d = self.dims
        ix0, iy0, ix1, iy1 = self.inner_box()
        t, h = d.wall_thickness, d.wall_height
        return [
            Solid("floor", (ix0 - t, iy0 - t, -d.floor_thickness, ix1 + t,
                            iy1 + t, 0), "wall"),
            Solid("wall -y", (ix0 - t, iy0 - t, 0, ix1 + t, iy0, h), "wall"),
            Solid("wall +y", (ix0 - t, iy1, 0, ix1 + t, iy1 + t, h), "wall"),
            Solid("wall -x", (ix0 - t, iy0, 0, ix0, iy1, h), "wall"),
            Solid("wall +x", (ix1, iy0, 0, ix1 + t, iy1, h), "wall"),
        ]

    def static_solids(self) -> List[Solid]:
        return self.walls() + self.obstacles

    def moving_solids(self) -> List[Solid]:
        """Carriage body and lift plate at the current pose."""
        x, y, z = self.pos
        return [Solid("carriage", self.carriage_box(x, y), "carriage"),
                Solid("lift plate", self.plate_box(x, y, z), "plate")]

    def collisions(self) -> List[Tuple[str, str]]:
        """Pairs (moving, static) of names that overlap at the current pose."""
        hits = []
        for m in self.moving_solids():
            for s in self.static_solids():
                if _overlap(m.box, s.box):
                    hits.append((m.name, s.name))
        return hits

    def overall_size(self) -> Tuple[float, float, float]:
        """Outside dimensions of the walls (x, y) and their height."""
        ix0, iy0, ix1, iy1 = self.inner_box()
        t = self.dims.wall_thickness
        return (ix1 - ix0 + 2 * t, iy1 - iy0 + 2 * t, self.dims.wall_height)

    def describe(self) -> str:
        d = self.dims
        ix0, iy0, ix1, iy1 = self.inner_box()
        ox, oy, oz = self.overall_size()
        return "\n".join([
            "Outer walls   : %g x %g x %g cm (thickness %g)" % (ox, oy, oz, d.wall_thickness),
            "Inside floor  : %g x %g cm" % (ix1 - ix0, iy1 - iy0),
            "Moveable area : %g x %g cm, lift z 0 to %g" % (
                d.move_x, d.move_y, d.move_z),
            "Lift low pos. : plate %g cm above the floor" % d.lift_floor_margin,
            "Carriage      : %g x %g x %g cm (full arena height)" % (
                d.carriage_w, d.carriage_d, d.wall_height),
            "Lift plate    : %g out x %g wide x %g thick, on %s face" % (
                d.plate_len, d.plate_width, d.plate_t, d.plate_side),
        ])


# --------------------------------------------------------------------------
# Standalone viewer / jog panel
# --------------------------------------------------------------------------

KIND_COLOR = {"wall": "#6b6b6b", "obstacle": "#8a6a3a", "large": "#4c9a6a",
              "small": "#c9a23a", "carriage": "#3b6ea8",
              "plate": "#e08a2c"}
HIT_COLOR = "#e03030"


class View:
    """One orthographic projection. h/v are the world axis indices shown
    horizontally and vertically; vertical z is drawn pointing up."""

    def __init__(self, parent, title, h, v, size=(520, 300), flip_h=False):
        self.title, self.h, self.v = title, h, v
        self.flip_h = flip_h  # mirror left and right
        self.w, self.hgt = size
        self.canvas = tk.Canvas(parent, width=size[0], height=size[1],
                                background="#1a1a1a", highlightthickness=0)

    def draw(self, arena: Arena, hits):
        c = self.canvas
        c.delete("all")
        d = arena.dims
        ix0, iy0, ix1, iy1 = arena.inner_box()
        t = d.wall_thickness
        lo = [ix0 - t, iy0 - t, -d.floor_thickness]
        hi = [ix1 + t, iy1 + t, d.wall_height]
        pad = 28
        span_h = hi[self.h] - lo[self.h]
        span_v = hi[self.v] - lo[self.v]
        s = min((self.w - 2 * pad) / span_h, (self.hgt - 2 * pad) / span_v)
        ox = (self.w - span_h * s) / 2
        oy = (self.hgt - span_v * s) / 2
        # vertical axis points up on screen for z and (top view) for y

        def sx(a):
            if self.flip_h:
                return ox + (hi[self.h] - a) * s
            return ox + (a - lo[self.h]) * s

        def sy(a):
            return oy + (hi[self.v] - a) * s

        def rect(b, **kw):
            xa, xb = sx(b[self.h]), sx(b[self.h + 3])
            ya, yb = sy(b[self.v]), sy(b[self.v + 3])
            c.create_rectangle(min(xa, xb), min(ya, yb), max(xa, xb), max(ya, yb), **kw)

        # moveable area (where the tool point can go)
        area = [0, 0, d.lift_floor_margin, d.move_x, d.move_y,
                d.lift_floor_margin + d.move_z]
        rect(area, outline="#4a7db5", dash=(4, 3))

        hit_names = {n for pair in hits for n in pair}
        for sol in arena.static_solids():
            bad = sol.name in hit_names
            rect(sol.box, fill=HIT_COLOR if bad else KIND_COLOR[sol.kind],
                 outline="#2a2a2a")
        for sol in arena.moving_solids():
            bad = sol.name in hit_names
            rect(sol.box, fill=HIT_COLOR if bad else KIND_COLOR[sol.kind],
                 outline="white", stipple="gray50" if sol.kind == "carriage" else "")

        c.create_text(6, 4, anchor="nw", fill="#bbbbbb", text=self.title)


class ArenaApp:
    def __init__(self, root, arena: Arena):
        self.root = root
        self.arena = arena
        root.title("Arena geometry / jog")

        self.step = tk.StringVar(value="5")
        self.goto = [tk.StringVar(value="0"), tk.StringVar(value="0"),
                     tk.StringVar(value=f"{arena.dims.move_z:g}")]
        self.readout = tk.StringVar()
        self.status = tk.StringVar()

        left = tk.Frame(root)
        left.pack(side="left", padx=8, pady=8)
        right = tk.Frame(root)
        right.pack(side="left", padx=(0, 8), pady=8, anchor="n")

        self.views = [
            View(left, "Top (x right, y up)", 0, 1, (520, 470)),
            View(right, "Front (x right, z up)", 0, 2, (520, 220)),
            View(right, "Side (y right, z up)", 1, 2, (520, 220)),
        ]
        self.views[0].canvas.pack()
        self.views[1].canvas.pack(pady=(0, 6))
        self.views[2].canvas.pack()

        panel = tk.Frame(right)
        panel.pack(fill="x", pady=8)

        tk.Label(panel, text="Jog step (cm):").grid(row=0, column=0, sticky="w")
        tk.Entry(panel, textvariable=self.step, width=6).grid(row=0, column=1, sticky="w")

        for r, axis in enumerate(AXES, start=1):
            tk.Label(panel, text=f"{axis.upper()}").grid(row=r, column=0, sticky="w")
            tk.Button(panel, text="-", width=4,
                      command=lambda a=axis: self.jog(a, -1)).grid(row=r, column=1)
            tk.Button(panel, text="+", width=4,
                      command=lambda a=axis: self.jog(a, +1)).grid(row=r, column=2)

        tk.Button(panel, text="Home", width=6, command=self.home).grid(
            row=1, column=4, padx=(16, 0))

        tk.Label(panel, text="Go to x y z:").grid(row=4, column=0, sticky="w", pady=(6, 0))
        for i, var in enumerate(self.goto):
            tk.Entry(panel, textvariable=var, width=6).grid(row=4, column=1 + i, pady=(6, 0))
        tk.Button(panel, text="Go", command=self.go).grid(row=4, column=4, pady=(6, 0))

        tk.Label(right, textvariable=self.readout, font=("TkFixedFont", 10),
                 anchor="w", justify="left").pack(fill="x")
        tk.Label(right, textvariable=self.status, anchor="w", justify="left",
                 wraplength=520).pack(fill="x", pady=4)
        tk.Label(right, text=arena.describe(), font=("TkFixedFont", 9),
                 anchor="w", justify="left", fg="#555555").pack(fill="x", pady=(8, 0))
        tk.Label(right, text="Keys: arrows = x/y, PgUp/PgDn (or w/s) = z, h = home",
                 anchor="w", fg="#555555").pack(fill="x")

        for key, (axis, sign) in {"<Left>": ("x", -1), "<Right>": ("x", 1),
                                  "<Up>": ("y", 1), "<Down>": ("y", -1),
                                  "<Prior>": ("z", 1), "<Next>": ("z", -1),
                                  "w": ("z", 1), "s": ("z", -1)}.items():
            root.bind(key, lambda e, a=axis, sg=sign: self.jog(a, sg))
        root.bind("h", lambda e: self.home())

        self.refresh()

    def step_cm(self) -> float:
        try:
            return abs(float(self.step.get()))
        except ValueError:
            return 5.0

    def jog(self, axis, sign):
        # ignore keystrokes typed into an entry box
        if isinstance(self.root.focus_get(), tk.Entry):
            return
        ok = self.arena.jog(axis, sign * self.step_cm())
        self.refresh(None if ok else f"{axis.upper()} at travel limit")

    def home(self):
        self.arena.home()
        self.refresh()

    def go(self):
        try:
            x, y, z = (float(v.get()) for v in self.goto)
        except ValueError:
            self.status.set("Go to: enter three numbers.")
            return
        ok = self.arena.move_to(x, y, z)
        self.refresh(None if ok else "Target clamped to the travel limits")

    def refresh(self, note=None):
        a = self.arena
        hits = a.collisions()
        x, y, z = a.pos
        self.readout.set(f"tool point  x {x:8.2f}   y {y:8.2f}   z {z:8.2f} cm")
        msgs = []
        if note:
            msgs.append(note)
        if hits:
            msgs.append("COLLISION: " + "; ".join(f"{m} / {s}" for m, s in hits))
        self.status.set("   ".join(msgs) if msgs else "No collisions.")
        for v in self.views:
            v.draw(a, hits)


if __name__ == "__main__":
    root = tk.Tk()
    ArenaApp(root, Arena(obstacles=make_boxes()))
    root.mainloop()
