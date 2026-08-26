"""The gripper: two L-shaped jaws that close diagonally on a box's corners.

One jaw is fixed, the other travels along the diagonal towards it, so the pair
always clamps two *opposite* corners.  Each jaw is an L — two thin arms meeting
at the corner, one running along each edge of the box.

                 reach
              ◄────────►
        ┌─────────────────────────┐
        │▀▀▀▀▀▀▀▀             ▛▀▀▀│ ▲
        │▌                       ▐│ │ arms are `thickness` thick
        │▌       the box          │ │
        │▌                       ▐│ │
        │▙▄▄▄                    ▐│ │
        └─────────────────────────┘ ▼
         bottom-left jaw    top-right jaw

Because the jaws sit *outside* the box, gripping needs empty floor there — which
is the whole reason this module exists.  A box wedged against a wall, or packed
tight against its neighbours on both diagonals, cannot be picked up at all no
matter how much room there is to slide it afterwards.

Either diagonal will do, so a box is grippable when at least one of the two
pairs is clear.  That is what makes the model worth having rather than just
inflating every box by the jaw thickness: a box with a neighbour hard against
its bottom-left corner is still perfectly grippable from the other diagonal.

Lengths are metres, like everything else.  `reach` is how far each arm runs
along the edge from the corner; it is clamped to the box's own side length, so
the jaws never overhang a box smaller than the reach.
"""

from typing import Iterable, List, Sequence, Tuple

from geometry import Rect

# The two ways round: which pair of opposite corners the jaws take.
DIAGONALS: Tuple[Tuple[str, str], ...] = (("bottom-left", "top-right"),
                                          ("bottom-right", "top-left"))


class Gripper:
    """Jaw geometry, and the clearance a box needs in order to be picked up."""

    def __init__(self, thickness: float = 0.01, reach: float = 0.05,
                 stroke: float = 0.02):
        if thickness <= 0.0:
            raise ValueError("gripper thickness must be positive")
        self.thickness = thickness
        self.reach = reach
        # How far the travelling jaw retracts along the diagonal to open.  The
        # machine has to open before it can take a box and after it lets one go,
        # so this is the widest the mechanism ever is.
        self.stroke = stroke

    # ------------------------------------------------------------------ shape

    def jaw(self, rect: Rect, corner: str) -> List[Rect]:
        """The two arms of the L that wraps `corner` of `rect`.

        The arms overlap at the corner itself, which is what makes it an L
        rather than two loose bars.
        """
        t = self.thickness
        across = min(self.reach, rect.w)
        up = min(self.reach, rect.h)
        left, bottom = rect.x, rect.y
        right, top = rect.x + rect.w, rect.y + rect.h

        if corner == "bottom-left":
            return [Rect(left - t, bottom - t, across + t, t),
                    Rect(left - t, bottom - t, t, up + t)]
        if corner == "bottom-right":
            return [Rect(right - across, bottom - t, across + t, t),
                    Rect(right, bottom - t, t, up + t)]
        if corner == "top-left":
            return [Rect(left - t, top, across + t, t),
                    Rect(left - t, top - up, t, up + t)]
        if corner == "top-right":
            return [Rect(right - across, top, across + t, t),
                    Rect(right, top - up, t, up + t)]
        raise ValueError(f"unknown corner {corner!r}")

    def jaws(self, rect: Rect, diagonal: int = 0) -> List[Rect]:
        """All four arms for one diagonal — both jaws, two arms each."""
        near, far = DIAGONALS[diagonal]
        return self.jaw(rect, near) + self.jaw(rect, far)

    def open_jaws(self, rect: Rect, diagonal: int = 0) -> List[Rect]:
        """The four arms with the travelling jaw backed off along the diagonal.

        The fixed jaw stays put; only the far one retracts, outwards along the
        diagonal it closes on.  This is the shape the mechanism has to fit into
        in order to take hold of a box or let go of one.
        """
        near, far = DIAGONALS[diagonal]
        away = self._retreat(far)
        arms = self.jaw(rect, near)
        for arm in self.jaw(rect, far):
            arms.append(Rect(arm.x + away[0] * self.stroke,
                             arm.y + away[1] * self.stroke, arm.w, arm.h))
        return arms

    def carried(self, rect: Rect, diagonal: int = 0) -> List[Rect]:
        """Everything that moves when a gripped box moves: box plus open jaws.

        The jaws travel with the box, so they are as much a part of the moving
        body as the box is — that is what stops a carried box being threaded
        through a gap the box alone would fit.  Modelled open rather than
        closed, because the machine must still be able to let go where it
        arrives.
        """
        return [rect] + self.open_jaws(rect, diagonal)

    @staticmethod
    def _retreat(corner: str) -> Tuple[int, int]:
        """Unit direction the travelling jaw backs off in, away from the box."""
        return {"bottom-left": (-1, -1), "bottom-right": (1, -1),
                "top-left": (-1, 1), "top-right": (1, 1)}[corner]

    def envelope(self, rect: Rect) -> Rect:
        """Everything the jaws could ever need, whichever diagonal is used.

        A cheap bounding test: if nothing intrudes on this, the box is certainly
        grippable and the per-diagonal checks can be skipped.
        """
        t = self.thickness
        return Rect(rect.x - t, rect.y - t, rect.w + 2 * t, rect.h + 2 * t)

    # ------------------------------------------------------------------ rules

    def diagonal_is_clear(self, rect: Rect, diagonal: int,
                          obstacles: Sequence[Rect], arena: Rect) -> bool:
        """Can the jaws take this diagonal without fouling anything?"""
        for arm in self.jaws(rect, diagonal):
            if not arena.contains(arm):
                return False
            for other in obstacles:
                if arm.overlaps(other):
                    return False
        return True

    def can_grip(self, rect: Rect, obstacles: Iterable[Rect],
                 arena: Rect) -> bool:
        """True when at least one diagonal is free.

        `obstacles` must not include `rect` itself.
        """
        others = [o for o in obstacles]
        if arena.contains(self.envelope(rect)) and not any(
                self.envelope(rect).overlaps(o) for o in others):
            return True                  # nothing near it at all
        return any(self.diagonal_is_clear(rect, d, others, arena)
                   for d in range(len(DIAGONALS)))

    def usable_diagonal(self, rect: Rect, obstacles: Iterable[Rect],
                        arena: Rect):
        """Which diagonal the jaws would use, or None if the box is stuck."""
        others = [o for o in obstacles]
        for d in range(len(DIAGONALS)):
            if self.diagonal_is_clear(rect, d, others, arena):
                return d
        return None
