"""Simple A* pathfinding on a 2D grid - console version.

Grid is read from stdin:
  first line: width height
  next height lines: width characters each
    '.' open, '#' wall, 'S' start, 'G' goal

Prints the path from S to G (inclusive) as "x,y" per line, or
"no path" if the goal is unreachable.

The actual algorithm lives in astar_core.py, shared with astar_gui.py so
the GUI runs this same code rather than a separate reimplementation.
"""

import sys

from astar_core import a_star


def main():
    lines = sys.stdin.read().splitlines()
    if not lines:
        return 1

    width, height = map(int, lines[0].split())
    grid = lines[1:1 + height]

    walls = set()
    start = goal = None
    for y, row in enumerate(grid):
        row = row.ljust(width, '#')
        for x, ch in enumerate(row):
            if ch == '#':
                walls.add((x, y))
            elif ch == 'S':
                start = (x, y)
            elif ch == 'G':
                goal = (x, y)

    if start is None or goal is None:
        print("grid must contain both S and G", file=sys.stderr)
        return 1

    path = a_star(walls, width, height, start, goal)

    if not path:
        print("no path")
        return 0

    for x, y in path:
        print(f"{x},{y}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
