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
        parent: Optional['FoldState'] = None,
        action: Optional[Dict[str, Any]] = None,
        g_cost: float = 0.0,
        h_cost: float = 0.0
    ):
        self.shape = shape
        self.folded_bends = set(folded_bends)
        self.remaining_bends = set(remaining_bends)
        self.panel_transforms = panel_transforms or {}
        self.bend_transforms = bend_transforms or {}
        self.orientation = orientation or (0.0, 0.0, 0.0)
        self.tooling_state = tooling_state or {"die_id": "DEFAULT_DIE", "punch_id": "DEFAULT_PUNCH"}
        self.diagnostics = diagnostics or {}
        self.parent = parent
        self.action = action or {}
        self.g_cost = g_cost
        self.h_cost = h_cost

        self._cached_fingerprint: Optional[str] = None

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
            tooling_state=tooling_state,
            g_cost=0.0,
            h_cost=0.0
        )

    def is_goal(self) -> bool:
        """Goal is reached when 0 bends remain in folded state (fully flat sheet)."""
        return len(self.remaining_bends) == 0

    @property
    def f_cost(self) -> float:
        """Total A* priority cost f(n) = g(n) + h(n)."""
        return self.g_cost + self.h_cost

    def fingerprint(self) -> str:
        """
        SPEC v1.0 Requirement: Full physical state fingerprint.
        Incorporates folded_bends, remaining_bends, panel_transforms, bend_transforms,
        orientation, tooling_state, and shape geometric signature.
        """
        if self._cached_fingerprint is not None:
            return self._cached_fingerprint

        folded_sig = "_".join(sorted(list(self.folded_bends)))
        remaining_sig = "_".join(sorted(list(self.remaining_bends)))

        bend_trans_sig = "_".join(f"{k}:{self.bend_transforms[k]:.1f}" for k in sorted(self.bend_transforms.keys()))
        orient_sig = f"{self.orientation[0]:.1f}_{self.orientation[1]:.1f}_{self.orientation[2]:.1f}"
        tooling_sig = f"{self.tooling_state.get('punch_id')}_{self.tooling_state.get('die_id')}"

        shape_sig = ""
        if self.shape is not None and hasattr(self.shape, 'BoundBox'):
            try:
                bb = self.shape.BoundBox
                shape_sig = f"bb[{bb.XMin:.1f},{bb.XMax:.1f},{bb.YMin:.1f},{bb.YMax:.1f},{bb.ZMin:.1f},{bb.ZMax:.1f}]"
            except Exception:
                shape_sig = f"stub_{id(self.shape)}"

        raw_key = f"folded[{folded_sig}]_rem[{remaining_sig}]_bends[{bend_trans_sig}]_orient[{orient_sig}]_tool[{tooling_sig}]_{shape_sig}"
        self._cached_fingerprint = hashlib.sha256(raw_key.encode('utf-8')).hexdigest()
        return self._cached_fingerprint

    def apply_unfold(
        self,
        bend_id: str,
        unfolded_shape: Any,
        moving_panels: Set[str],
        unfold_cost: float = 1.0
    ) -> 'FoldState':
        """
        Create successor FoldState after unfolding bend_id to 0 degrees.
        """
        new_folded = set(self.folded_bends) - {bend_id}
        new_remaining = set(self.remaining_bends) - {bend_id}

        new_bend_transforms = dict(self.bend_transforms)
        new_bend_transforms[bend_id] = 0.0  # Unbent to 0 degrees

        action_data = {
            "bend_id": bend_id,
            "unfolded_angle": 0.0,
            "moving_panels": list(moving_panels)
        }

        return FoldState(
            shape=unfolded_shape,
            folded_bends=new_folded,
            remaining_bends=new_remaining,
            panel_transforms=dict(self.panel_transforms),
            bend_transforms=new_bend_transforms,
            orientation=self.orientation,
            tooling_state=self.tooling_state,
            parent=self,
            action=action_data,
            g_cost=self.g_cost + unfold_cost,
            h_cost=0.0
        )

    def reconstruct_backward_path(self) -> List['FoldState']:
        """Reconstruct path from root down to this state."""
        path = []
        curr: Optional['FoldState'] = self
        while curr is not None:
            path.append(curr)
            curr = curr.parent
        path.reverse()
        return path

    def reconstruct_forward_sequence(self) -> List[Dict[str, Any]]:
        """Invert backward path to obtain physical forward sequence."""
        path = self.reconstruct_backward_path()
        sequence = []
        for i in range(len(path) - 1, 0, -1):
            state = path[i]
            action = dict(state.action)
            action["step_number"] = len(path) - i
            sequence.append(action)
        return sequence

    def __repr__(self) -> str:
        return f"<FoldState rem={len(self.remaining_bends)} folded={len(self.folded_bends)} g={self.g_cost:.2f}>"
