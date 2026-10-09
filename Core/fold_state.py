"""
fold_state.py - Full Canonical Physical State Model for Sheet Metal Folding (SMBS Phase 3)

SPEC v1.0 Requirement (LAW-3 — FoldState):
State MUST NOT be modeled merely as shape + remaining_bends.
Canonical FoldState contains:
├── shape
├── folded_bends
├── remaining_bends
├── panel_transforms
├── bend_transforms
├── orientation
├── tooling_state
└── diagnostics
"""

import hashlib
import json
from typing import Dict, Any, List, Set, Tuple, Optional
from Core.bend_graph import BendRecord, BendGraph
from Core.panel_graph import PanelGraph


class SearchState:
    """
    Lightweight Search Node representation for graph search algorithms.
    Does NOT hold heavy OCC TopoShapes.
    """
    def __init__(
        self,
        state_id: str,
        parent_id: Optional[str],
        folded_bends: Set[str],
        remaining_bends: Set[str],
        bend_transforms: Dict[str, float],
        g_cost: float = 0.0,
        action: Optional[Dict[str, Any]] = None
    ):
        self.state_id = state_id
        self.parent_id = parent_id
        self.folded_bends = set(folded_bends)
        self.remaining_bends = set(remaining_bends)
        self.bend_transforms = dict(bend_transforms)
        self.g_cost = g_cost
        self.action = action or {}

    def is_goal(self) -> bool:
        return len(self.remaining_bends) == 0


class FoldState:
    """
    Canonical physical state of a sheet metal part during folding/unfolding search.
    """

    def __init__(
        self,
        shape: Any,
        folded_bends: Set[str],
        remaining_bends: Set[str],
        panel_transforms: Optional[Dict[str, Tuple[float, ...]]] = None,
        bend_transforms: Optional[Dict[str, float]] = None,
        orientation: Optional[Tuple[float, float, float]] = None,
        tooling_state: Optional[Dict[str, Any]] = None,
        diagnostics: Optional[Dict[str, Any]] = None,
        parent_id: Optional[str] = None
    ):
        self.shape = shape
        self.folded_bends = set(folded_bends)
        self.remaining_bends = set(remaining_bends)
        self.panel_transforms = panel_transforms or {}
        self.bend_transforms = bend_transforms or {}
        self.orientation = orientation or (0.0, 0.0, 0.0)
        self.tooling_state = tooling_state or {"die_id": "DEFAULT_DIE", "punch_id": "DEFAULT_PUNCH"}
        self.diagnostics = diagnostics or {}
        self.parent_id = parent_id

        self._cached_fingerprint: Optional[str] = None

    def clear_shape(self) -> None:
        """Explicitly release OCC TopoShape reference to free C++ memory immediately."""
        self.shape = None

    @classmethod
    def initial(
        cls,
        part_shape: Any,
        bend_graph: BendGraph,
        panel_graph: Optional[PanelGraph] = None,
        tooling_state: Optional[Dict[str, Any]] = None
    ) -> 'FoldState':
        """
        Construct initial Root state (Fully Bent 3D Part, 0 bends unfolded).
        """
        all_bends = {b.id for b in bend_graph.all_bends()}
        bend_angles = {b.id: b.signed_angle for b in bend_graph.all_bends()}
        panel_ids = list(panel_graph.panels.keys()) if panel_graph else []
        initial_panel_transforms = {pid: (0.0, 0.0, 0.0) for pid in panel_ids}

        return cls(
            shape=part_shape,
            folded_bends=all_bends,
            remaining_bends=all_bends,
            panel_transforms=initial_panel_transforms,
            bend_transforms=bend_angles,
            tooling_state=tooling_state
        )

    def is_goal(self) -> bool:
        """Goal is reached when 0 bends remain in folded state (fully flat sheet)."""
        return len(self.remaining_bends) == 0

    def fingerprint(self) -> str:
        """
        SPEC v1.0 Requirement: Canonical physical state fingerprint.
        Incorporates folded_bends, remaining_bends, canonical panel_transforms,
        bend_transforms, orientation, and tooling_state.
        Does NOT depend on OCC Shape objects for predictive hashing and state deduplication.
        """
        if self._cached_fingerprint is not None:
            return self._cached_fingerprint

        folded_sig = "_".join(sorted(list(self.folded_bends)))
        remaining_sig = "_".join(sorted(list(self.remaining_bends)))

        bend_trans_sig = "_".join(f"{k}:{self.bend_transforms[k]:.1f}" for k in sorted(self.bend_transforms.keys()))
        
        # Canonical panel transforms signature with fixed rounding precision (0.01 mm / deg)
        panel_trans_parts = []
        for p_id in sorted(self.panel_transforms.keys()):
            val = self.panel_transforms[p_id]
            if isinstance(val, (tuple, list)):
                formatted_val = ",".join(f"{v:.2f}" for v in val)
            else:
                formatted_val = f"{val}"
            panel_trans_parts.append(f"{p_id}:({formatted_val})")
        panel_trans_sig = "_".join(panel_trans_parts)

        orient_sig = f"{self.orientation[0]:.1f}_{self.orientation[1]:.1f}_{self.orientation[2]:.1f}"
        tooling_sig = f"{self.tooling_state.get('punch_id')}_{self.tooling_state.get('die_id')}"

        raw_key = (
            f"folded[{folded_sig}]_rem[{remaining_sig}]_bends[{bend_trans_sig}]_"
            f"panels[{panel_trans_sig}]_orient[{orient_sig}]_tool[{tooling_sig}]"
        )
        self._cached_fingerprint = hashlib.sha256(raw_key.encode('utf-8')).hexdigest()
        return self._cached_fingerprint

    def predict_successor_fingerprint(self, bend_id: str) -> str:
        """
        Predict the fingerprint of the successor state if bend_id were unfolded.
        This allows checking caches/dead_states BEFORE executing expensive OCC geometry.
        """
        new_folded = set(self.folded_bends) - {bend_id}
        new_remaining = set(self.remaining_bends) - {bend_id}
        
        folded_sig = "_".join(sorted(list(new_folded)))
        remaining_sig = "_".join(sorted(list(new_remaining)))

        # Next bend transforms will have bend_id = 0.0
        new_bend_transforms = dict(self.bend_transforms)
        new_bend_transforms[bend_id] = 0.0
        bend_trans_sig = "_".join(f"{k}:{new_bend_transforms[k]:.1f}" for k in sorted(new_bend_transforms.keys()))
        
        panel_trans_parts = []
        for p_id in sorted(self.panel_transforms.keys()):
            val = self.panel_transforms[p_id]
            if isinstance(val, (tuple, list)):
                formatted_val = ",".join(f"{v:.2f}" for v in val)
            else:
                formatted_val = f"{val}"
            panel_trans_parts.append(f"{p_id}:({formatted_val})")
        panel_trans_sig = "_".join(panel_trans_parts)

        orient_sig = f"{self.orientation[0]:.1f}_{self.orientation[1]:.1f}_{self.orientation[2]:.1f}"
        tooling_sig = f"{self.tooling_state.get('punch_id')}_{self.tooling_state.get('die_id')}"

        raw_key = (
            f"folded[{folded_sig}]_rem[{remaining_sig}]_bends[{bend_trans_sig}]_"
            f"panels[{panel_trans_sig}]_orient[{orient_sig}]_tool[{tooling_sig}]"
        )
        return hashlib.sha256(raw_key.encode('utf-8')).hexdigest()

    def apply_unfold(
        self,
        bend_id: str,
        unfolded_shape: Any,
        moving_panels: Set[str]
    ) -> 'FoldState':
        """
        Create successor FoldState after unfolding bend_id to 0 degrees.
        """
        new_folded = set(self.folded_bends) - {bend_id}
        new_remaining = set(self.remaining_bends) - {bend_id}

        new_bend_transforms = dict(self.bend_transforms)
        new_bend_transforms[bend_id] = 0.0  # Unbent to 0 degrees

        new_panel_transforms = dict(self.panel_transforms)
        for pid in moving_panels:
            if pid in new_panel_transforms:
                current_transform = new_panel_transforms[pid]
                # Update panel transform record state
                if isinstance(current_transform, tuple) and len(current_transform) == 3:
                    new_panel_transforms[pid] = (
                        current_transform[0],
                        current_transform[1],
                        current_transform[2]
                    )

        return FoldState(
            shape=unfolded_shape,
            folded_bends=new_folded,
            remaining_bends=new_remaining,
            panel_transforms=new_panel_transforms,
            bend_transforms=new_bend_transforms,
            orientation=self.orientation,
            tooling_state=self.tooling_state,
            parent_id=self.fingerprint()
        )

    def __repr__(self) -> str:
        return f"<FoldState rem={len(self.remaining_bends)} folded={len(self.folded_bends)}>"
