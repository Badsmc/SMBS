"""
greedy_unfold.py - Fast Greedy Backward Unfold Planner

Selects the first physically valid, collision-free bend that minimizes local step cost.
Provides ultra-fast sequence finding for standard sheet metal parts (boxes, brackets, covers).
"""

from typing import Dict, Any
from Core.backward_graph import BackwardGraph, BendState
from .base_planner import BasePlanner


class GreedyUnfoldPlanner(BasePlanner):
    """
    Greedy Solver operating in inverted state space.
    """

    def solve(self, max_iterations: int = 5000) -> Dict[str, Any]:
        """
        Execute greedy backward search.
        """
        current = self.graph.root_state
        current.h_cost = self.compute_heuristic(current)

        nodes_explored = 0

        while not current.is_goal() and nodes_explored < max_iterations:
            nodes_explored += 1
            
            best_successor = None
            best_cost = float('inf')

            # Expand all candidate backward transitions from current state
            for successor, step_cost in self.graph.expand_successors(current):
                local_h = self.compute_heuristic(successor)
                total_local_cost = step_cost + local_h

                if total_local_cost < best_cost:
                    best_cost = total_local_cost
                    best_successor = successor

            if best_successor is None:
                # Dead-end reached: no collision-free bend can be unfolded from current state
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
                "nodes_explored": nodes_explored
            }

        return {
            "success": False,
            "error": "Greedy planner exceeded maximum iteration limit.",
            "nodes_explored": nodes_explored
        }
