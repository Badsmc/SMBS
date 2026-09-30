"""
astar_backward.py - Backward A* Search Planner for Sheet Metal Unfolding

Traverses the state space backwards from the Fully Bent 3D model (Root) down to the Flat Sheet (Goal).
Priority queue ranks nodes by f(n) = g(n) + h(n) + P(n), where:
- g(n): Cumulative path cost (flange rotations, part handling).
- h(n): Admissible heuristic (remaining bends + enclosure estimate).
- P(n): Penalty terms (tool collision risk, unnecessary part flips).
"""

import heapq
from typing import Dict, Any, List, Set, Tuple
from Core.backward_graph import BackwardGraph, BendState
from .base_planner import BasePlanner


class AStarBackwardPlanner(BasePlanner):
    """
    A* Solver operating in inverted state space tree.
    """

    def solve(self, max_iterations: int = 5000) -> Dict[str, Any]:
        """
        Execute backward A* search.
        """
        root = self.graph.root_state
        root.h_cost = self.compute_heuristic(root)

        # Min-heap priority queue storing tuples: (f_cost, counter, state)
        counter = 0
        open_set: List[Tuple[float, int, BendState]] = [(root.f_cost, counter, root)]
        
        # Track best g_cost for visited geometric state hashes
        g_scores: Dict[str, float] = {root.hash_key: root.g_cost}

        nodes_explored = 0

        while open_set and nodes_explored < max_iterations:
            _, _, current = heapq.heappop(open_set)
            nodes_explored += 1

            # Check for goal (0 bends remaining -> fully flat pattern)
            if current.is_goal():
                return {
                    "success": True,
                    "goal_state": current,
                    "nodes_explored": nodes_explored
                }

            # Lazy expansion of valid, collision-free successor states
            for successor, step_cost in self.graph.expand_successors(current):
                tentative_g = current.g_cost + step_cost
                hash_key = successor.hash_key

                if hash_key not in g_scores or tentative_g < g_scores[hash_key]:
                    g_scores[hash_key] = tentative_g
                    successor.g_cost = tentative_g
                    successor.h_cost = self.compute_heuristic(successor)
                    successor.parent = current

                    counter += 1
                    heapq.heappush(open_set, (successor.f_cost, counter, successor))

        return {
            "success": False,
            "error": f"A* search exhausted after exploring {nodes_explored} nodes without reaching flat state.",
            "nodes_explored": nodes_explored
        }
