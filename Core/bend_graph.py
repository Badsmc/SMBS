"""
bend_graph.py - Canonical Bend Record & Bend Relationship Graph for SMBS Phase 1

SPEC v1.0 Requirement:
Represents the canonical model of sheet metal bend features.
Uses signed_angle (-180° to +180°) instead of unsigned absolute angle heuristics.
Raises IncompleteBendError if required geometric parameters cannot be extracted.
"""

from typing import Dict, Any, List, Optional, Tuple
import math


class IncompleteBendError(ValueError):
    """Raised when a BendRecord lacks mandatory geometric parameters."""
    pass


class BendRecord:
    """
    Canonical representation of a single sheet metal bend feature.
    """

    def __init__(
        self,
        bend_id: str,
        feature_id: str,
        parent_panel_id: str,
        child_panel_id: str,
        axis_origin: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle: float,
        radius: float,
        thickness: float,
        bend_sign: int = 1,
        flat_axis: Optional[Tuple[float, float, float]] = None,
        flat_center: Optional[Tuple[float, float, float]] = None,
        length: float = 100.0,
        metadata: Optional[Dict[str, Any]] = None
    ):
        self.id = bend_id
        self.feature_id = feature_id
        self.parent_panel_id = parent_panel_id
        self.child_panel_id = child_panel_id
        self.axis_origin = axis_origin
        self.axis_direction = self._normalize_vector(axis_direction)
        self.signed_angle = float(signed_angle)
        self.radius = float(radius)
        self.thickness = float(thickness)
        self.bend_sign = 1 if bend_sign >= 0 else -1
        self.flat_axis = flat_axis or self.axis_direction
        self.flat_center = flat_center or axis_origin
        self.length = float(length)
        self.metadata = metadata or {}

        # Validate mandatory canonical fields
        self.validate()

    def validate(self) -> None:
        """Validate that all mandatory canonical fields are populated and bounded."""
        if not self.id or not self.parent_panel_id or not self.child_panel_id:
            raise IncompleteBendError(f"Bend {self.id}: Missing parent/child panel identification.")

        if self.radius <= 0.0 or self.thickness <= 0.0 or self.length <= 0.0:
            raise IncompleteBendError(f"Bend {self.id}: Non-positive physical dimensions (R={self.radius}, T={self.thickness}, L={self.length}).")

        if not (-180.0 <= self.signed_angle <= 180.0) or math.isnan(self.signed_angle):
            raise IncompleteBendError(f"Bend {self.id}: Invalid signed_angle {self.signed_angle}.")

    @staticmethod
    def _normalize_vector(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Return unit normalized vector."""
        mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        if mag < 1e-9:
            raise IncompleteBendError(f"Zero magnitude vector provided for bend axis: {v}")
        return (v[0] / mag, v[1] / mag, v[2] / mag)

    @property
    def unsigned_angle(self) -> float:
        """Return absolute bend angle magnitude in degrees."""
        return abs(self.signed_angle)

    def to_dict(self) -> Dict[str, Any]:
        """Convert BendRecord to standard dictionary representation."""
        return {
            "id": self.id,
            "feature_id": self.feature_id,
            "parent_panel_id": self.parent_panel_id,
            "child_panel_id": self.child_panel_id,
            "axis_origin": list(self.axis_origin),
            "axis_direction": list(self.axis_direction),
            "signed_angle": self.signed_angle,
            "angle": self.unsigned_angle,
            "radius": self.radius,
            "thickness": self.thickness,
            "bend_sign": self.bend_sign,
            "length": self.length,
            "axis": list(self.axis_direction),
            "origin": list(self.axis_origin)
        }

    def __repr__(self) -> str:
        return f"<BendRecord {self.id} angle={self.signed_angle:+.1f}° ({self.parent_panel_id}->{self.child_panel_id})>"


class BendGraph:
    """
    Graph container managing all canonical BendRecords for a sheet metal model.
    """

    def __init__(self):
        self._bends: Dict[str, BendRecord] = {}

    def add_bend(self, bend: BendRecord) -> None:
        """Add a validated BendRecord to the graph."""
        bend.validate()
        self._bends[bend.id] = bend

    def get_bend(self, bend_id: str) -> Optional[BendRecord]:
        """Retrieve BendRecord by ID."""
        return self._bends.get(bend_id)

    def all_bends(self) -> List[BendRecord]:
        """Return list of all registered BendRecords."""
        return list(self._bends.values())

    def __len__(self) -> int:
        return len(self._bends)

    def __contains__(self, bend_id: str) -> bool:
        return bend_id in self._bends
