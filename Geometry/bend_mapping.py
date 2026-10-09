"""
bend_mapping.py - Data structure for matching solver bends to 2D flat pattern lines.

SPEC v1.0 Requirement (Stage 2):
Minimal data structure for matching Bend ID to Geometry Reference.
Separates Bend ID (physical bend), Operation Number (sequence index), and Geometry Reference (2D line).
"""

from typing import Optional, Tuple, Any, Dict

class BendMatchStatus:
    UNMATCHED = "UNMATCHED"
    MATCHED = "MATCHED"
    AMBIGUOUS = "AMBIGUOUS"

class BendMappingRecord:
    """
    Holds the confirmed mapping between a solver Bend ID and a FreeCAD SheetMetal 2D Sketch line.
    """
    def __init__(
        self,
        bend_id: str,
        length: float,
        signed_angle: Optional[float] = None,
        bend_sign: Optional[int] = None,
        # 3D Source geometry
        source_center: Optional[Tuple[float, float, float]] = None,
        source_axis: Optional[Tuple[float, float, float]] = None,
        # 2D Flat geometry reference
        flat_sketch_name: Optional[str] = None,
        flat_line_index: Optional[int] = None,
        flat_start_point: Optional[Tuple[float, float, float]] = None,
        flat_end_point: Optional[Tuple[float, float, float]] = None,
        # Matching metadata
        match_status: str = BendMatchStatus.UNMATCHED,
        match_method: Optional[str] = None,
        error_reason: Optional[str] = None
    ):
        self.bend_id = bend_id
        self.length = length
        self.signed_angle = signed_angle
        self.bend_sign = bend_sign
        
        self.source_center = source_center
        self.source_axis = source_axis
        
        self.flat_sketch_name = flat_sketch_name
        self.flat_line_index = flat_line_index
        self.flat_start_point = flat_start_point
        self.flat_end_point = flat_end_point
        
        self.match_status = match_status
        self.match_method = match_method
        self.error_reason = error_reason

    def to_dict(self) -> Dict[str, Any]:
        return {
            "bend_id": self.bend_id,
            "length": self.length,
            "signed_angle": self.signed_angle,
            "bend_sign": self.bend_sign,
            "source_center": self.source_center,
            "source_axis": self.source_axis,
            "flat_sketch_name": self.flat_sketch_name,
            "flat_line_index": self.flat_line_index,
            "flat_start_point": self.flat_start_point,
            "flat_end_point": self.flat_end_point,
            "match_status": self.match_status,
            "match_method": self.match_method,
            "error_reason": self.error_reason
        }
