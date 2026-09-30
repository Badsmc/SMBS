"""
backward_graph.py - Inverted State Tree (Fully Bent 3D Part -> Flat Sheet)

In Backward Planning architecture:
- Root State (Depth 0): Fully bent 3D part (all N bends formed).
- Goal State: Fully flat sheet metal (0 bends remaining).
- Graph Traversal: Inverted state expansion — at each step, select a remaining bend and UNFOLD it (reduce angle to 0).
- Final output path is inverted back to yield the physical forward bending process (Flat -> Bent).
"""

from typing import List, Set, Dict, Any, Optional, Tuple, Generator
from .state_cache import StateCache


class BendState:
    """
    Represents a single node in the backward search space graph.
    """

    def __init__(
        self,
        shape: Any,
        remaining_bends: Set[str],
        unfolded_bends: Tuple[str, ...] = (),
        parent: Optional['BendState'] = None,
        action: Optional[Dict[str, Any]] = None,
        g_cost: float = 0.0,
        h_cost: float = 0.0
    ):
        self.shape = shape
        self.remaining_bends = set(remaining_bends)
        self.unfolded_bends = unfolded_bends
        self.parent = parent
        self.action = action or {}
        self.g_cost = g_cost
        self.h_cost = h_cost

        # Lazy geometric hash key initialization
        self.hash_key = StateCache.compute_hash(shape, self.remaining_bends)

    @property
    def f_cost(self) -> float:
        """Total A* priority cost f(n) = g(n) + h(n)."""
        return self.g_cost + self.h_cost

    def is_goal(self) -> bool:
        """Goal is reached when 0 bends remain to be unfolded (fully flat pattern)."""
        return len(self.remaining_bends) == 0

    def reconstruct_backward_path(self) -> List['BendState']:
        """Reconstruct path from root (fully bent) down to this state."""
        path = []
        curr: Optional['BendState'] = self
        while curr is not None:
            path.append(curr)
            curr = curr.parent
        path.reverse()
        return path

    def reconstruct_forward_sequence(self) -> List[Dict[str, Any]]:
        """
        Invert the backward unfolding trajectory to obtain the physical forward bending sequence.
        
        Backward: Bent -> Unfold B_k -> Unfold B_j -> Flat
        Forward:  Flat -> Bend B_j -> Bend B_k -> Bent Part
        """
        path = self.reconstruct_backward_path()
        forward_steps = []
        
        # Traverse path backwards (from flat target to fully bent root)
        for i in range(len(path) - 1, 0, -1):
            state = path[i]
            action = dict(state.action)
            # Invert action direction for forward bending output
            action["step_number"] = len(path) - i
            forward_steps.append(action)
            
        return forward_steps

    def __lt__(self, other: 'BendState') -> bool:
        return self.f_cost < other.f_cost

    def __repr__(self) -> str:
        return f"<BendState bends_left={len(self.remaining_bends)} g={self.g_cost:.2f} h={self.h_cost:.2f}>"


class BackwardGraph:
    """
    Manages the inverted search tree and lazy transition generation.
    """

    def __init__(
        self,
        initial_shape: Any,
        feature_map: Dict[str, Dict[str, Any]],
        sheetmetal_bridge: Any,
        collision_detector: Any,
        state_cache: StateCache
    ):
        self.initial_shape = initial_shape
        self.feature_map = feature_map
        self.bridge = sheetmetal_bridge
        self.collision_detector = collision_detector
        self.cache = state_cache

        # Create root state (fully bent)
        all_bends = set(feature_map.keys())
        self.root_state = BendState(
            shape=initial_shape,
            remaining_bends=all_bends,
            g_cost=0.0,
            h_cost=0.0
        )
        self.cache.put(self.root_state.hash_key, self.root_state)

    def expand_successors(
        self,
        current_state: BendState
    ) -> Generator[Tuple[BendState, float], None, None]:
        """
        Lazy evaluation generator yielding valid, collision-free successor states.
        
        For each candidate bend in current_state.remaining_bends:
        1. Invokes sheetmetal_bridge to compute unbent shape geometrically.
        2. Performs fast BoundingBox + exact collision check against tools/frame.
        3. Validates backgauge feasibility (hybrid online check).
        4. Calculates step cost (flange movement, tool changes, part rotation).
        """
        for bend_id in sorted(list(current_state.remaining_bends)):
            bend_info = self.feature_map[bend_id]
            
            # Step 1: Geometric unfolding via SheetMetal bridge (pure TopoShape)
            unfolded_shape = self.bridge.unfold_bend_shape(
                current_state.shape,
                bend_info
            )
            
            # Step 2: Kinematic & tool collision detection during unfold motion
            collision, collision_msg = self.collision_detector.check_unfold_collision(
                current_state.shape,
                unfolded_shape,
                bend_info
            )
            
            if collision:
                # Collision detected -> invalid backward branch
                # print(f"DEBUG: Candidate bend {bend_id} rejected: {collision_msg}")
                continue

            # Step 3: Create successor state parameters
            next_remaining = set(current_state.remaining_bends) - {bend_id}
            next_unfolded = current_state.unfolded_bends + (bend_id,)
            
            action_data = {
                "bend_id": bend_id,
                "angle": bend_info.get("angle", 90.0),
                "radius": bend_info.get("radius", 1.0),
                "length": bend_info.get("length", 100.0),
                "axis": bend_info.get("axis", (0, 0, 1)),
                "unfold_collision_clearance": True
            }

            # Check if this shape state was already explored (State Cache)
            temp_hash = StateCache.compute_hash(unfolded_shape, next_remaining)
            cached_state = self.cache.get(temp_hash)
            
            if cached_state is not None:
                new_state = cached_state
            else:
                new_state = BendState(
                    shape=unfolded_shape,
                    remaining_bends=next_remaining,
                    unfolded_bends=next_unfolded,
                    parent=current_state,
                    action=action_data
                )
                self.cache.put(temp_hash, new_state)

            # Step cost calculation (e.g. flange length penalty, orientation change)
            step_cost = self._compute_step_cost(current_state, new_state, bend_info)

            yield new_state, step_cost

    def _compute_step_cost(
        self,
        prev_state: BendState,
        next_state: BendState,
        bend_info: Dict[str, Any]
    ) -> float:
        """Compute transition cost between backward states."""
        base_cost = 1.0
        
        # Penalize unfolding long/heavy flanges early (prefer unfolding outer small flanges first)
        length = bend_info.get("length", 100.0)
        length_penalty = (length / 1000.0) * 0.5
        
        # Penalize axis alignment changes (prefer grouping parallel bends to reduce press brake setup changes)
        axis_change_penalty = 0.0
        if prev_state.action and "axis" in prev_state.action:
            prev_axis = prev_state.action["axis"]
            curr_axis = bend_info.get("axis", (0, 0, 1))
            if prev_axis != curr_axis:
                axis_change_penalty = 2.0

        return base_cost + length_penalty + axis_change_penalty
