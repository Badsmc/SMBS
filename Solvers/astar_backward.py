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

    def compute_heuristic(self, state: FoldState) -> float:
        """
        Optimized heuristic for sheet metal backward planning:
        Penalize states that still have outer bends (small moving subtrees) remaining.
        This aggressively guides the search toward "center-first" sequences (unfolding outer bends first).
        """
        num_remaining = len(state.remaining_bends)
        if num_remaining == 0 or not self.panel_graph:
            return float(num_remaining)
            
        penalty = 0.0
        for b_id in state.remaining_bends:
            # The smaller the subtree, the more "outer" the bend is.
            # If outer bends are still in remaining_bends, it means we haven't unfolded them yet.
            # We want to unfold them first, so we penalize them being left behind.
            moving_count = len(self.panel_graph.moving_subtree(b_id))
            total_count = len(self.panel_graph.panels)
            min_moving = min(moving_count, total_count - moving_count)
            penalty += 2.0 / (min_moving + 1)
            
        return float(num_remaining) + penalty

    def solve(
        self,
        initial_state: FoldState,
        max_iterations: int = 5000,
        timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Execute backward A* search.
        """
        import time
        start_time = time.time()
        initial_fp = initial_state.fingerprint()
        initial_h = self.compute_heuristic(initial_state)
        counter = 0
        open_set: List[Tuple[float, int, FoldState]] = [(initial_h, counter, initial_state)]
        g_scores: Dict[str, float] = {initial_fp: 0.0}
        came_from: Dict[str, Tuple[str, Dict[str, Any]]] = {}

        nodes_explored = 0

        while open_set and nodes_explored < max_iterations:
            if timeout is not None and (time.time() - start_time) > timeout:
                break
                
            _, _, current = heapq.heappop(open_set)
            nodes_explored += 1
            current_fp = current.fingerprint()

            if current.is_goal():
                # Reconstruct path
                sequence = []
                curr_fp = current_fp
                step_num = 1
                while curr_fp in came_from:
                    parent_fp, action = came_from[curr_fp]
                    action_copy = dict(action)
                    action_copy["step_number"] = step_num
                    sequence.append(action_copy)
                    curr_fp = parent_fp
                    step_num += 1
                
                return {
                    "success": True,
                    "sequence": sequence,
                    "nodes_explored": nodes_explored,
                    "cache_stats": self.cache.stats
                }

            def score_candidate(b_id: str) -> int:
                """Cheap topological score: prioritize outer bends (fewer moving panels)."""
                if self.panel_graph:
                    moving_count = len(self.panel_graph.moving_subtree(b_id))
                    total_count = len(self.panel_graph.panels)
                    return min(moving_count, total_count - moving_count)
                return 0

            candidates = sorted(list(current.remaining_bends), key=score_candidate)

            for bend_id in candidates:
                bend_rec = self.bend_graph.get_bend(bend_id)
                if bend_rec is None:
                    continue

                # Predictive check: skip OCC if target state is already reached via a better/equal path
                next_fp = current.predict_successor_fingerprint(bend_id)
                tentative_g = g_scores[current_fp] + 1.0
                if next_fp in g_scores and tentative_g >= g_scores[next_fp]:
                    continue

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
                # A* step cost is 1.0 per bend for now
                tentative_g = g_scores[current_fp] + 1.0

                if fingerprint not in g_scores or tentative_g < g_scores[fingerprint]:
                    g_scores[fingerprint] = tentative_g
                    h_cost = self.compute_heuristic(successor)
                    f_cost = tentative_g + h_cost
                    self.cache.put(fingerprint, successor)

                    moving_panels = list(self.panel_graph.moving_subtree(bend_id)) if self.panel_graph else []
                    action = {
                        "bend_id": bend_id,
                        "unfolded_angle": 0.0,
                        "moving_panels": moving_panels
                    }
                    came_from[fingerprint] = (current_fp, action)

                    counter += 1
                    heapq.heappush(open_set, (f_cost, counter, successor))

            # Memory Optimization: Free the shape after expanding all successors
            # Since FoldState tree keeps parents alive, this prevents OOM.
            if hasattr(current, "shape") and current.shape is not None:
                if hasattr(current.shape, "nullify"):
                    try:
                        current.shape.nullify()
                    except Exception:
                        pass
                current.shape = None

        return {
            "success": False,
            "error": f"A* search exhausted after exploring {nodes_explored} nodes without reaching flat state.",
            "nodes_explored": nodes_explored,
            "cache_stats": self.cache.stats
        }
