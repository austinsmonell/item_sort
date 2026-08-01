"""Axis-aligned rectangle geometry.

Everything in this study is an axis-aligned rectangle in metres: the arena, the
boxes, and the goal footprint.  This module is the only place that knows how to
test overlap and containment, so the tolerance used for "touching is allowed,
overlapping is not" lives in exactly one place.

`Rect` is a NamedTuple rather than a dataclass because the search builds one per
candidate move — tuple construction is several times cheaper than a frozen
dataclass's, and that shows up directly in plan times.
"""

from typing import NamedTuple, Tuple

# Boxes are allowed to touch edge-to-edge.  Overlap tests therefore need a small
# tolerance, otherwise floating point noise in a grid position turns a flush
# contact into a collision.
EPS = 1e-9


class Rect(NamedTuple):
    """Rectangle anchored at its lower-left corner (x, y) with size (w, h)."""

    x: float
    y: float
    w: float
    h: float

    @property
    def x2(self) -> float:
        return self.x + self.w

    @property
    def y2(self) -> float:
        return self.y + self.h

    @property
    def center(self) -> Tuple[float, float]:
        return (self.x + self.w / 2.0, self.y + self.h / 2.0)

    def overlaps(self, other: "Rect", eps: float = EPS) -> bool:
        """True when the two rectangles share area (a shared edge does not)."""
        return (
            self.x < other.x + other.w - eps
            and other.x < self.x + self.w - eps
            and self.y < other.y + other.h - eps
            and other.y < self.y + self.h - eps
        )

    def contains(self, other: "Rect", eps: float = EPS) -> bool:
        """True when `other` lies wholly inside this rectangle."""
        return (
            other.x >= self.x - eps
            and other.y >= self.y - eps
            and other.x + other.w <= self.x + self.w + eps
            and other.y + other.h <= self.y + self.h + eps
        )

    def contains_point(self, px: float, py: float) -> bool:
        return (self.x <= px <= self.x + self.w
                and self.y <= py <= self.y + self.h)
