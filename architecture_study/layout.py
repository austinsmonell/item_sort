"""Standard box catalogue and random layout generation.

Boxes come in a few standard square sizes rather than arbitrary rectangles, so a
layout is described by *how many of each type* you want.  This module turns those
counts into concrete non-overlapping placements on the movement grid.

Placement is rejection sampling, biggest boxes first — a large box dropped into
an arena that is already half full is the case that fails, so it gets first pick
while the floor is empty.  When it cannot be done the generator says so rather
than returning a layout that overlaps.
"""

import math
import random
from typing import Dict, List, Sequence, Tuple

from geometry import Rect

_SNAP_EPS = 1e-6


class LayoutFull(RuntimeError):
    """Raised when the requested boxes cannot be placed in the arena."""


class LayoutGenerator:
    """Turns per-type box counts into a random, legal starting layout."""

    def __init__(self, arena: Rect, types: Sequence[Tuple[str, float]],
                 attempts: int = 500, seed=None):
        """`types` is a sequence of (type name, side length) in metres."""
        self.arena = arena
        self.types = list(types)
        self.attempts = attempts
        self.rng = random.Random(seed)

    def fill_fraction(self, counts: Dict[str, int]) -> float:
        """Share of the arena floor the requested boxes would cover."""
        area = sum(counts.get(name, 0) * size * size for name, size in self.types)
        return area / (self.arena.w * self.arena.h)

    def generate(self, counts: Dict[str, int], step: float) -> List[tuple]:
        """Return (name, x, y, w, h) placements on a `step` grid.

        Raises `LayoutFull` if the boxes will not fit.
        """
        wanted = []
        for type_name, size in self.types:
            wanted.extend([(type_name, size)] * max(0, counts.get(type_name, 0)))
        if not wanted:
            raise LayoutFull("no boxes requested — set at least one count above zero")

        fill = self.fill_fraction(counts)
        if fill > 1.0:
            raise LayoutFull(
                f"those boxes cover {fill * 100:.0f}% of the arena floor — "
                "they cannot possibly fit")

        wanted.sort(key=lambda item: -item[1])
        placed: List[Rect] = []
        boxes: List[tuple] = []
        tally: Dict[str, int] = {}

        for type_name, size in wanted:
            max_x = int(math.floor((self.arena.w - size) / step + _SNAP_EPS))
            max_y = int(math.floor((self.arena.h - size) / step + _SNAP_EPS))
            if max_x < 0 or max_y < 0:
                raise LayoutFull(
                    f"a {type_name} box ({size:.2f} m) does not fit the arena")

            rect = None
            for _ in range(self.attempts):
                candidate = Rect(self.arena.x + self.rng.randint(0, max_x) * step,
                                 self.arena.y + self.rng.randint(0, max_y) * step,
                                 size, size)
                if not any(candidate.overlaps(other) for other in placed):
                    rect = candidate
                    break
            if rect is None:
                raise LayoutFull(
                    f"placed only {len(boxes)} of {len(wanted)} boxes after "
                    f"{self.attempts} tries each ({fill * 100:.0f}% floor "
                    "coverage) — reduce the counts or use a finer grid")

            tally[type_name] = tally.get(type_name, 0) + 1
            placed.append(rect)
            boxes.append((f"{type_name[0].upper()}{tally[type_name]}",
                          rect.x, rect.y, size, size))
        return boxes
