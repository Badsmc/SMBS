"""
validator.py - Single Authoritative Physical Oracle for Step Validation (SMBS Phase 5)

SPEC v1.0 Requirement (LAW-1 — Physical Oracle):
No solver shall contain its own physical model.
Solver proposes: candidate bend -> PhysicalValidator.validate_step(...) -> VALID / INVALID.
This component is the SINGLE AUTHORITATIVE GATE for physical step validity.
"""

from typing import Dict, Any, Optional, Tuple
from Core.fold_state import FoldState
from Core.bend_graph import BendRecord
from Core.panel_graph import PanelGraph
from Core.bend_transform import BendTransform
from Physics.collision_detector import CollisionDetector
from Physics.machine_kinematics import PressBrakeMachine

try:
    import FreeCAD
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False

class ValidationResult:
    """Encapsulates the result of a physical step validation query."""

    def __init__(
        self,
        valid: bool,
        reason: str,
        stage: str,
        next_state: Optional[FoldState] = None,
        diagnostics: Optional[Dict[str, Any]] = None
    ):
        self.valid = valid
        self.reason = reason
        self.stage = stage
        self.next_state = next_state
        self.diagnostics = diagnostics or {}

    def __repr__(self) -> str:
        status = "VALID" if self.valid else f"INVALID[{self.stage}:{self.reason}]"
        return f"<ValidationResult {status}>"


class PhysicalValidator:
    """
    Unified Physical Oracle for sheet metal bending sequence validation.
    """

    def __init__(self, machine: PressBrakeMachine):
        self.machine = machine
        self.detector = CollisionDetector(self.machine)
        self.invalid_transitions: Dict[Tuple[str, str], ValidationResult] = {}

    def validate_step(
        self,
        current_state: FoldState,
        bend_record: BendRecord,
        panel_graph: Optional[PanelGraph] = None,
        tooling_override: Optional[Dict[str, Any]] = None
    ) -> ValidationResult:
        """
        Validate a candidate backward unbending step from current_state for bend_record.

        Evaluation Order (SPEC Section 37):
        1. Candidate consistency check
        2. Subtree kinematics calculation
        3. Layer 1 Self Collision check
        4. Layer 2 Tool Collision check
        5. Layer 3 Machine Collision check
        6. Layer 4 Trajectory Sweep Collision check
        7. Construct & return next FoldState
        """
        bend_id = bend_record.id
        current_fp = current_state.fingerprint()
        
        # Check transition cache first (fast rejection)
        cache_key = (current_fp, bend_id)
        if cache_key in self.invalid_transitions:
            return self.invalid_transitions[cache_key]

        # Stage 1: Candidate consistency & approach check
        if bend_id not in current_state.remaining_bends:
            return ValidationResult(
                valid=False,
                reason=f"Bend {bend_id} is not in remaining_bends set.",
                stage="APPROACH"
            )
            
        bend_info = bend_record.to_dict()
        app_coll, app_msg = self.detector.approach_collision(current_state.shape, bend_info)
        if app_coll:
            if HAS_FREECAD:
                FreeCAD.Console.PrintMessage(f"BendSeq: Reject [{bend_id}] stage=APPROACH: {app_msg}\n")
            res = ValidationResult(valid=False, reason=app_msg, stage="APPROACH")
            self.invalid_transitions[cache_key] = res
            return res

        # Stage 2: Kinematics subtree resolution and transformation
        moving_panels = panel_graph.moving_subtree(bend_id) if panel_graph else set()
        unfolded_shape = BendTransform.rotate_subtree(
            current_state.shape,
            bend_record.axis_origin,
            bend_record.axis_direction,
            -bend_record.signed_angle,
            moving_panels,
            panel_graph
        )

        # Stage 3: SELF Collision Check
        self_coll, self_msg = self.detector.self_collision(unfolded_shape, bend_id, panel_graph)
        if self_coll:
            if HAS_FREECAD:
                FreeCAD.Console.PrintMessage(f"BendSeq: Reject [{bend_id}] stage=SELF: {self_msg}\n")
            res = ValidationResult(valid=False, reason=self_msg, stage="SELF")
            self.invalid_transitions[cache_key] = res
            return res

        # Stage 4: TOOL Collision Check
        tool_coll, tool_msg = self.detector.tool_collision(unfolded_shape, bend_info, bend_id, panel_graph)
        if tool_coll:
            if HAS_FREECAD:
                FreeCAD.Console.PrintMessage(f"BendSeq: Reject [{bend_id}] stage=TOOL: {tool_msg}\n")
            res = ValidationResult(valid=False, reason=tool_msg, stage="TOOL")
            self.invalid_transitions[cache_key] = res
            return res

        # Stage 5: MACHINE Collision Check
        mach_coll, mach_msg = self.detector.machine_collision(unfolded_shape, bend_info, bend_id, panel_graph)
        if mach_coll:
            if HAS_FREECAD:
                FreeCAD.Console.PrintMessage(f"BendSeq: Reject [{bend_id}] stage=MACHINE: {mach_msg}\n")
            res = ValidationResult(valid=False, reason=mach_msg, stage="MACHINE")
            self.invalid_transitions[cache_key] = res
            return res

        # Stage 6: BEND Trajectory Sweep Collision Check (Δθ = 5.0°)
        traj_coll, traj_msg = self.detector.trajectory_collision(current_state.shape, bend_info, dtheta=5.0)
        if traj_coll:
            if HAS_FREECAD:
                FreeCAD.Console.PrintMessage(f"BendSeq: Reject [{bend_id}] stage=BEND_TRAJECTORY: {traj_msg}\n")
            res = ValidationResult(valid=False, reason=traj_msg, stage="BEND_TRAJECTORY")
            self.invalid_transitions[cache_key] = res
            return res

        # Stage 7: RETRACT Collision Check
        ret_coll, ret_msg = self.detector.retract_collision(unfolded_shape, bend_info)
        if ret_coll:
            if HAS_FREECAD:
                FreeCAD.Console.PrintMessage(f"BendSeq: Reject [{bend_id}] stage=RETRACT: {ret_msg}\n")
            res = ValidationResult(valid=False, reason=ret_msg, stage="RETRACT")
            self.invalid_transitions[cache_key] = res
            return res

        # Step is VALID -> Produce next FoldState
        next_fold_state = current_state.apply_unfold(
            bend_id=bend_id,
            unfolded_shape=unfolded_shape,
            moving_panels=moving_panels
        )

        return ValidationResult(
            valid=True,
            reason="Step validated physically.",
            stage="valid",
            next_state=next_fold_state,
            diagnostics={"bend_id": bend_id, "signed_angle": bend_record.signed_angle}
        )
