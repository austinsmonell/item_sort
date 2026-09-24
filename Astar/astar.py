"""Simple A* pathfinding on a 3D grid - console version.

Grid is read from stdin:
  first line: width height depth
  then `depth` layers (z = 0, 1, ...), each `height` rows of `width`
  characters, layers separated by a blank line:
    '.' open, '#' wall, 'S' start, 'G' goal

Pass --diag as an argument to allow diagonal moves (26 neighbors instead
of 6).

Prints the path from S to G (inclusive) as "x,y,z" per line, or
"no path" if the goal is unreachable.

The actual algorithm lives in astar_core.py, shared with astar_gui.py so
the GUI runs this same code rather than a separate reimplementation.
"""

import sys

from astar_core import a_star


def main():
    diagonals = "--diag" in sys.argv[1:]
    lines = sys.stdin.read().splitlines()
    if not lines:
        return 1

    width, height, depth = map(int, lines[0].split())
    rows = [ln for ln in lines[1:] if ln.strip()]  # blank separators optional

    walls = set()
    start = goal = None
    for z in range(depth):
        layer = rows[z * height:(z + 1) * height]
        for y in range(height):
            row = (layer[y] if y < len(layer) else "").ljust(width, '#')
            for x, ch in enumerate(row[:width]):
                if ch == '#':
                    walls.add((x, y, z))
                elif ch == 'S':
                    start = (x, y, z)
                elif ch == 'G':
                    goal = (x, y, z)

    if start is None or goal is None:
        print("grid must contain both S and G", file=sys.stderr)
        return 1

    path = a_star(walls, width, height, depth, start, goal,
                  diagonals=diagonals)

    if not path:
        print("no path")
        return 0

    for x, y, z in path:
        print(f"{x},{y},{z}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
