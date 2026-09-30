"""
greedy_unfold.py - Fast Greedy Backward Unfold Planner (SMBS Phase 8/9)

Selects the first physically valid, collision-free bend that minimizes local heuristic cost.
Provides ultra-fast sequence finding operating on canonical FoldState and Physical Oracle gate.
"""

from typing import Dict, Any, Optional
from Core.fold_state import FoldState
from Core.bend_graph import BendGraph
from Core.panel_graph import PanelGraph
from Core.state_cache import StateCache
from Physics.validator import PhysicalValidator, ValidationResult
from .base_planner import BasePlanner


class GreedyUnfoldPlanner(BasePlanner):
    """
    Greedy Solver operating on FoldState & Physical Oracle Gate.
    """

    def __init__(
        self,
        bend_graph: BendGraph,
        validator: PhysicalValidator,
        panel_graph: Optional[PanelGraph] = None,
        state_cache: Optional[StateCache] = None
    ):
        super().__init__(validator)
        self.bend_graph = bend_graph
        self.panel_graph = panel_graph
        self.cache = state_cache or StateCache()

    def solve(
        self,
        initial_state: FoldState,
        max_iterations: int = 5000
    ) -> Dict[str, Any]:
        """
        Execute greedy backward search.
        """
        current = initial_state
        nodes_explored = 0

        while not current.is_goal() and nodes_explored < max_iterations:
            nodes_explored += 1

            best_successor = None
            best_cost = float('inf')

            candidates = sorted(list(current.remaining_bends))

            for bend_id in candidates:
                bend_rec = self.bend_graph.get_bend(bend_id)
                if bend_rec is None:
                    continue

                val_result: ValidationResult = self.validator.validate_step(
                    current_state=current,
                    bend_record=bend_rec,
                    panel_graph=self.panel_graph
                )

                if not val_result.valid or val_result.next_state is None:
                    continue

                successor = val_result.next_state
                local_h = self.compute_heuristic(successor)

                if local_h < best_cost:
                    best_cost = local_h
                    best_successor = successor

            if best_successor is None:
                return {
                    "success": False,
                    "error": f"Greedy planner hit dead-end at {len(current.remaining_bends)} remaining bends.",
                    "nodes_explored": nodes_explored
                }

            current = best_successor

        if current.is_goal():
            return {
                "success": True,
                "goal_state": current,
                "nodes_explored": nodes_explored,
                "cache_stats": self.cache.stats
            }

        return {
            "success": False,
            "error": "Greedy planner exceeded maximum iteration limit.",
            "nodes_explored": nodes_explored
        }

