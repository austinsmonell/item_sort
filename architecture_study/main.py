"""Run the A* rectangle-sorting study.

    python main.py

Pick a box, drop a goal, tune the cost weights, and watch A* work out which
boxes have to get out of the way.
"""

import study_config as cfg
from board import Board
from cost_model import CostModel
from puzzle_gui import PuzzleGUI


def main():
    board = Board(cfg.ARENA, cfg.BOXES, cfg.GRID_STEP)
    cost = CostModel(cfg.DISTANCE_WEIGHT, cfg.REGRIP_WEIGHT, cfg.HEURISTIC_WEIGHT)
    PuzzleGUI(cfg, board, cost).run()


if __name__ == "__main__":
    main()
