"""Run the A* rectangle-sorting study.

    python main.py

Set how many small/medium/large boxes go on the floor, pick one, place its goal,
tune the cost weights, and watch A* work out which boxes have to get out of the
way.
"""

import sys

import study_config as cfg
from board import Board
from cost_model import CostModel
from layout import LayoutFull, LayoutGenerator
from puzzle_gui import PuzzleGUI


def main():
    generator = LayoutGenerator(cfg.ARENA, cfg.BOX_TYPES, cfg.PLACEMENT_ATTEMPTS,
                                cfg.RANDOM_SEED)
    try:
        boxes = generator.generate(cfg.DEFAULT_COUNTS, cfg.GRID_STEP)
    except LayoutFull as exc:
        print(f"cannot build the starting layout: {exc}", file=sys.stderr)
        return 1

    board = Board(cfg.ARENA, boxes, cfg.GRID_STEP)
    cost = CostModel(cfg.DISTANCE_WEIGHT, cfg.REGRIP_WEIGHT, cfg.HEURISTIC_WEIGHT)
    PuzzleGUI(cfg, board, cost, generator).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
