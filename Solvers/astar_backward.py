"""
astar_backward.py - Backward A* Search Planner with Physical Oracle (SMBS Phase 8)

SPEC v1.0 Requirement (Section 19 & 48):
A* search planner operating directly on canonical FoldState and Physical Oracle gate.
Priority queue ranks state nodes by f(n) = g(n) + h(n).
Every candidate transition is validated by PhysicalValidator.
"""

import heapq
from typing import Dict, Any, List, Optional, Set, Tuple
from Core.fold_state import FoldState
from Core.bend_graph import BendGraph, BendRecord
from Core.panel_graph import PanelGraph
from Core.state_cache import StateCache
from Physics.validator import PhysicalValidator, ValidationResult
from .base_planner import BasePlanner


class AStarBackwardPlanner(BasePlanner):
    """
    Backward A* Search Planner with Physical Oracle Gate.
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
        Execute backward A* search.
        """
        initial_state.h_cost = self.compute_heuristic(initial_state)

        counter = 0
        open_set: List[Tuple[float, int, FoldState]] = [(initial_state.f_cost, counter, initial_state)]
        g_scores: Dict[str, float] = {initial_state.fingerprint(): initial_state.g_cost}

        nodes_explored = 0

        while open_set and nodes_explored < max_iterations:
            _, _, current = heapq.heappop(open_set)
            nodes_explored += 1

            if current.is_goal():
                return {
                    "success": True,
                    "goal_state": current,
                    "nodes_explored": nodes_explored,
                    "cache_stats": self.cache.stats
                }

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
                    continue  # INVALID transition -> reject branch

                successor = val_result.next_state
                if successor is None:
                    continue

                fingerprint = successor.fingerprint()
                tentative_g = successor.g_cost

                if fingerprint not in g_scores or tentative_g < g_scores[fingerprint]:
                    g_scores[fingerprint] = tentative_g
                    successor.h_cost = self.compute_heuristic(successor)
                    self.cache.put(fingerprint, successor)

                    counter += 1
                    heapq.heappush(open_set, (successor.f_cost, counter, successor))

        return {
            "success": False,
            "error": f"A* search exhausted after exploring {nodes_explored} nodes without reaching flat state.",
            "nodes_explored": nodes_explored,
            "cache_stats": self.cache.stats
        }
