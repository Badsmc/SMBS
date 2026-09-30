"""
dfs_backward.py - Reference Backward DFS Solver with Physical Oracle (SMBS Phase 6)

SPEC v1.0 Requirement (Section 20 & 43):
First production/reference solver: Backward DFS + Backtracking operating directly on Physical Oracle.
Proves completeness and physical correctness without relying on unproven heuristic admissibility assumptions.
"""

from typing import Dict, Any, List, Optional, Set
from Core.fold_state import FoldState
from Core.bend_graph import BendGraph, BendRecord
from Core.panel_graph import PanelGraph
from Physics.validator import PhysicalValidator, ValidationResult
from .base_planner import BasePlanner


class DFSBackwardPlanner:
    """
    Reference Backward Depth-First Search (DFS) Planner with Backtracking.
    """

    def __init__(
        self,
        bend_graph: BendGraph,
        validator: PhysicalValidator,
        panel_graph: Optional[PanelGraph] = None
    ):
        self.bend_graph = bend_graph
        self.validator = validator
        self.panel_graph = panel_graph
        self.rejection_records: List[Dict[str, Any]] = []

    def solve(
        self,
        initial_state: FoldState,
        max_depth: int = 500
    ) -> Dict[str, Any]:
        """
        Execute backward DFS search with backtracking.

        :param initial_state: Root FoldState (fully bent 3D part).
        :param max_depth: Maximum recursion search depth limit.
        :return: Result dictionary containing 'success', 'goal_state', 'explored_count', 'rejections'.
        """
        explored_count = 0
        self.rejection_records.clear()

        def _dfs(current: FoldState, depth: int) -> Optional[FoldState]:
            nonlocal explored_count
            explored_count += 1

            if current.is_goal():
                return current

            if depth >= max_depth:
                return None

            # Evaluate all remaining candidate bends in current state
            candidates = sorted(list(current.remaining_bends))

            for bend_id in candidates:
                bend_rec = self.bend_graph.get_bend(bend_id)
                if bend_rec is None:
                    continue

                # Query Physical Oracle gate for step validity
                val_result: ValidationResult = self.validator.validate_step(
                    current_state=current,
                    bend_record=bend_rec,
                    panel_graph=self.panel_graph
                )

                if not val_result.valid:
                    self.rejection_records.append({
                        "bend_id": bend_id,
                        "stage": val_result.stage,
                        "reason": val_result.reason,
                        "depth": depth
                    })
                    continue  # INVALID transition -> backtrack to next candidate

                # VALID transition -> Recurse into next FoldState
                goal = _dfs(val_result.next_state, depth + 1)
                if goal is not None:
                    return goal

            return None  # Dead end -> backtrack

        goal_state = _dfs(initial_state, 0)

        if goal_state is not None:
            return {
                "success": True,
                "goal_state": goal_state,
                "explored_count": explored_count,
                "rejections_count": len(self.rejection_records)
            }

        return {
            "success": False,
            "error": f"DFS search exhausted after exploring {explored_count} states without finding a valid path.",
            "explored_count": explored_count,
            "rejections": self.rejection_records
        }
