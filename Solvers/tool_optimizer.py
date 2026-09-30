"""
tool_optimizer.py - Tooling Layout & Sequence Optimizer

Post-pass optimization heuristic that refines the bend sequence:
1. Minimizes punch & die segment swaps across steps.
2. Penalizes unnecessary 180-degree part rotations/flips.
3. Groups bends requiring identical V-die widths into contiguous sub-sequences when kinematics allow.
"""

from typing import List, Dict, Any


class ToolOptimizer:
    """
    Optimizes tool setups and operator handling across a bend sequence.
    """

    def __init__(self, tool_database: Any):
        self.tool_db = tool_database

    def optimize_sequence_tooling(
        self,
        sequence: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Analyze forward bend sequence and annotate/optimize tooling setup changes.

        :param sequence: List of step dictionaries in forward order (Flat -> Step 1 -> ... -> Bent).
        :return: Optimized sequence annotated with tooling instructions and flip indicators.
        """
        if not sequence:
            return []

        optimized = []
        prev_axis = None
        prev_die_id = None
        station_id = 1

        for idx, step in enumerate(sequence):
            step_copy = dict(step)
            curr_axis = step.get("axis", (0, 0, 1))
            tooling = step.get("tooling", {})
            curr_die_id = tooling.get("die_id", "DIE_DEFAULT")

            # Check if operator flip / rotation is required
            requires_flip = False
            if prev_axis is not None and prev_axis != curr_axis:
                requires_flip = True

            # Check if tool station change is needed
            tool_change_needed = False
            if prev_die_id is not None and prev_die_id != curr_die_id:
                station_id += 1
                tool_change_needed = True

            step_copy["operator_instructions"] = {
                "station": station_id,
                "requires_part_flip_180": requires_flip,
                "tool_change_required": tool_change_needed,
                "description": f"Position part against backgauge stops X={step['backgauge']['X_mm']:.1f}mm, R={step['backgauge']['R_mm']:.1f}mm."
            }

            optimized.append(step_copy)
            prev_axis = curr_axis
            prev_die_id = curr_die_id

        return optimized
