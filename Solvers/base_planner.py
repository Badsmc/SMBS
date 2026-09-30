"""
base_planner.py - Base Interface for Backward Bend Sequence Planners (SMBS Phase 8)

SPEC v1.0 Requirement (Section 17):
Common base interface for all sequence search planners.
Uses heuristic estimation (without unproven claims of mathematical admissibility).
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from Core.fold_state import FoldState
from Physics.validator import PhysicalValidator


class BasePlanner(ABC):
    """
    Abstract Base Class for sequence search planners operating on FoldState & Physical Oracle.
    """

    def __init__(self, validator: PhysicalValidator):
        self.validator = validator

    @abstractmethod
    def solve(self, initial_state: FoldState, max_iterations: int = 5000) -> Dict[str, Any]:
        """
        Execute search algorithm to find a verified collision-free fold sequence.

        :param initial_state: Root FoldState.
        :param max_iterations: Maximum node expansions allowed.
        :return: Planning result dictionary.
        """
        pass

    def compute_heuristic(self, state: FoldState) -> float:
        """
        Domain heuristic estimate for remaining unfolding cost.
        Returns number of remaining folded bends.
        """
        num_remaining = len(state.remaining_bends)
        if num_remaining == 0:
            return 0.0
        return float(num_remaining)
