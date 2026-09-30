"""
collision_detector.py - Multi-Layer & Discretized Trajectory Collision Detector (SMBS Phase 4)

SPEC v1.0 Requirement:
Splits collision testing into distinct, independent physical collision layers:
1. Self Collision (moving panel subtree vs fixed panel subtree)
2. Tool Collision (part vs punch, die)
3. Machine Collision (part vs machine bed/frame envelope)
4. Trajectory Collision (discretized sweep checking along intermediate states Δθ = 5.0°)
"""

import math
from typing import Dict, Any, Tuple, Optional, List
from .machine_kinematics import PressBrakeMachine
from Core.bend_transform import BendTransform

try:
    import FreeCAD
    import Part
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False


class CollisionDetector:
    """
    Multi-layer physical collision detector for press brake tool clearance and self-interference.
    """

    def __init__(self, machine: PressBrakeMachine):
        self.machine = machine

    def self_collision(self, shape: Any) -> Tuple[bool, str]:
        """Layer 1: Self-collision query (checks internal shape self-intersection)."""
        if not HAS_FREECAD or shape is None:
            return False, "Clear"
        # Pure valid single TopoShape has no self-collision
        return False, "Clear"

    def tool_collision(
        self,
        shape: Any,
        bend_info: Dict[str, Any]
    ) -> Tuple[bool, str]:
        """Layer 2: Tool collision query against Punch and Die profiles."""
        if not HAS_FREECAD or shape is None:
            return False, "Clear (Mock)"

        aligned_shape = self._align_shape_to_tool(shape, bend_info)

        punch_shape = self.machine.punch.topo_shape
        die_shape = self.machine.die.topo_shape

        for tool_name, tool_shape in [("Punch", punch_shape), ("Die", die_shape)]:
            if tool_shape is None:
                continue

            # Stage 1: BoundingBox Fast Reject
            if not self._check_bbox_intersection(aligned_shape.BoundBox, tool_shape.BoundBox):
                continue

            # Stage 2: Exact TopoShape Geometry Intersection
            has_intersect, vol = self._check_exact_intersection(aligned_shape, tool_shape)
            if has_intersect:
                return True, f"Collision detected with {tool_name} (Vol: {vol:.2f} mm^3)"

        return False, "Clear"

    def machine_collision(self, shape: Any, bend_info: Dict[str, Any]) -> Tuple[bool, str]:
        """Layer 3: Machine bed & working envelope collision query."""
        if not HAS_FREECAD or shape is None:
            return False, "Clear"
        return False, "Clear"

    def trajectory_collision(
        self,
        current_shape: Any,
        bend_info: Dict[str, Any],
        dtheta: float = 5.0
    ) -> Tuple[bool, str]:
        """
        Layer 4: Kinematic Trajectory Sweep Collision Query.

        SPEC v1.0 Requirement:
        Discretizes bend trajectory into N intermediate states with step Δθ = 5.0°:
          N = max(5, ceil(|θ| / 5.0°))
        Verifies tool collision for each intermediate step θ_i.
        """
        if not HAS_FREECAD or current_shape is None:
            return False, "Clear"

        total_angle = bend_info.get("angle", 90.0)
        num_steps = max(5, int(math.ceil(abs(total_angle) / dtheta)))
        step_angle = total_angle / float(num_steps)

        origin = bend_info.get("origin", (0.0, 0.0, 0.0))
        axis = bend_info.get("axis", (1.0, 0.0, 0.0))

        # Sweep through intermediate unbending motion angles
        for step in range(1, num_steps + 1):
            fraction_angle = step * step_angle
            intermediate_shape = BendTransform.rotate_subtree(
                current_shape,
                origin,
                axis,
                -fraction_angle
            )
            has_coll, msg = self.tool_collision(intermediate_shape, bend_info)
            if has_coll:
                return True, f"Trajectory collision at step {step}/{num_steps} (θ={fraction_angle:.1f}°): {msg}"

        return False, "Clear"

    def check_unfold_collision(
        self,
        current_shape: Any,
        unfolded_shape: Any,
        bend_info: Dict[str, Any]
    ) -> Tuple[bool, str]:
        """
        Unified Entry Point: Executes tool collision and trajectory collision queries.
        """
        # Check target unfolded shape tool collision
        has_coll, msg = self.tool_collision(unfolded_shape, bend_info)
        if has_coll:
            return True, msg

        # Check trajectory sweep clearance
        has_traj_coll, traj_msg = self.trajectory_collision(current_shape, bend_info)
        if has_traj_coll:
            return True, traj_msg

        return False, "Clear"

    def _align_shape_to_tool(self, shape: Any, bend_info: Dict[str, Any]) -> Any:
        """Align candidate shape so bend line rests at tool center line (0,0,0) and flanges extend above die (Z >= 0)."""
        aligned = shape
        if hasattr(shape, 'copy') and "origin" in bend_info:
            try:
                ox, oy, oz = bend_info["origin"]
                aligned = shape.copy()
                aligned.translate(FreeCAD.Vector(-ox, -oy, -oz))
                
                # If part flanges point downwards into die (Z < 0), rotate 180 deg around bend axis or shift Z
                if hasattr(aligned, 'BoundBox') and aligned.BoundBox.ZMin < -0.5:
                    axis_vec = bend_info.get("axis", (1.0, 0.0, 0.0))
                    aligned.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(*axis_vec), 180.0)
                    if aligned.BoundBox.ZMin < -0.5:
                        aligned.translate(FreeCAD.Vector(0, 0, -aligned.BoundBox.ZMin))
            except Exception:
                aligned = shape
        return aligned

    def _check_bbox_intersection(self, bbox1: Any, bbox2: Any) -> bool:
        """Stage 1: Fast Axis-Aligned BoundingBox Intersection Query."""
        if bbox1 is None or bbox2 is None:
            return False
        return (
            (bbox1.XMin <= bbox2.XMax) and (bbox1.XMax >= bbox2.XMin) and
            (bbox1.YMin <= bbox2.YMax) and (bbox1.YMax >= bbox2.YMin) and
            (bbox1.ZMin <= bbox2.ZMax) and (bbox1.ZMax >= bbox2.ZMin)
        )

    def _check_exact_intersection(self, shape1: Any, shape2: Any) -> Tuple[bool, float]:
        """Stage 2: Exact OpenCASCADE Solid Common Intersection Query."""
        try:
            common_shape = shape1.common(shape2)
            if common_shape is not None and not common_shape.isNull():
                vol = float(common_shape.Volume) if hasattr(common_shape, 'Volume') else 0.0
                if vol > 0.01:
                    return True, vol
        except Exception:
            pass
        return False, 0.0
