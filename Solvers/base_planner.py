"""
base_planner.py - Common Base Interface for Backward Bend Sequence Solvers
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from Core.backward_graph import BackwardGraph, BendState


class BasePlanner(ABC):
    """
    Abstract Base Class for all BendSeq sequence search planners.
    Ensures a consistent interface across different planning strategies (A*, Greedy, Monte Carlo, etc.).
    """

    def __init__(self, graph: BackwardGraph):
        self.graph = graph

    @abstractmethod
    def solve(self, max_iterations: int = 5000) -> Dict[str, Any]:
        """
        Execute the solver algorithm to find a collision-free backward unfolding sequence.

        :param max_iterations: Maximum node expansions allowed before aborting.
        :return: Dictionary containing 'success', 'goal_state', 'nodes_explored', 'error'.
        """
        pass

    def compute_heuristic(self, state: BendState) -> float:
        """
        Admissible & consistent heuristic for backward planning.

        Estimates remaining cost from current state down to fully flat target state:
        1. Number of remaining bends (each bend requires at least 1 stroke).
        2. Flange clearance heuristic (outer flanges should be unfolded before inner/enclosed bends).
        """
        num_remaining = len(state.remaining_bends)
        if num_remaining == 0:
            return 0.0

        base_h = float(num_remaining)

        # Additional domain heuristic: count remaining bends with potential box enclosure
        enclosure_penalty = 0.0
        for bend_id in state.remaining_bends:
            bend_info = self.graph.feature_map.get(bend_id, {})
            # Longer bends or bends with large angles incur higher un-nesting complexity
            if bend_info.get("length", 0) > 200.0:
                enclosure_penalty += 0.2

        return base_h + enclosure_penalty
