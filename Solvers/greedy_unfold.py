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
        max_iterations: int = 5000,
        timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Execute greedy backward search.
        """
        import time
        start_time = time.time()
        current = initial_state
        nodes_explored = 0
        current_path = []

        while not current.is_goal() and nodes_explored < max_iterations:
            if timeout is not None and (time.time() - start_time) > timeout:
                break
                
            nodes_explored += 1

            best_successor = None
            best_cost = float('inf')
            best_action = None

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
                    # Free the previous best_successor shape if we are replacing it
                    if best_successor is not None and hasattr(best_successor, "shape") and best_successor.shape is not None:
                        if hasattr(best_successor.shape, "nullify"):
                            try:
                                best_successor.shape.nullify()
                            except Exception:
                                pass
                        best_successor.shape = None

                    best_cost = local_h
                    best_successor = successor
                    
                    moving_panels = list(self.panel_graph.moving_subtree(bend_id)) if self.panel_graph else []
                    best_action = {
                        "bend_id": bend_id,
                        "unfolded_angle": 0.0,
                        "moving_panels": moving_panels,
                        "step_number": len(current.remaining_bends)
                    }
                else:
                    # Free memory for non-selected branch
                    if hasattr(successor, "shape") and successor.shape is not None:
                        if hasattr(successor.shape, "nullify"):
                            try:
                                successor.shape.nullify()
                            except Exception:
                                pass
                        successor.shape = None

            if best_successor is None:
                return {
                    "success": False,
                    "error": f"Greedy planner hit dead-end at {len(current.remaining_bends)} remaining bends.",
                    "nodes_explored": nodes_explored
                }

            # Free previous current state's shape (now that we move to next)
            if current != initial_state and hasattr(current, "shape") and current.shape is not None:
                if hasattr(current.shape, "nullify"):
                    try:
                        current.shape.nullify()
                    except Exception:
                        pass
                current.shape = None

            current_path.append(best_action)
            current = best_successor

        if current.is_goal():
            forward_sequence = list(reversed(current_path))
            for i, step in enumerate(forward_sequence):
                step["step_number"] = i + 1
            
            return {
                "success": True,
                "sequence": forward_sequence,
                "nodes_explored": nodes_explored,
                "cache_stats": self.cache.stats
            }

        return {
            "success": False,
            "error": "Greedy planner exceeded maximum iteration limit.",
            "nodes_explored": nodes_explored
        }

