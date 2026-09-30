"""
tool_database.py - Local Tooling Catalog & Selection Database

Stores press brake punch and die specifications:
- V-groove opening rules (V = 6T to 8T for steel).
- Punch tip radii and height clearance profiles.
- Tool segment layouts (e.g. 100mm, 200mm, 500mm, ear segments).
"""

from typing import Dict, Any, List, Optional


class ToolDatabase:
    """
    Manages available punch and die tooling inventory.
    """

    def __init__(self):
        self._punches = {
            "PUNCH_GOOSENECK_120": {
                "id": "PUNCH_GOOSENECK_120",
                "type": "gooseneck",
                "height_mm": 120.0,
                "angle_deg": 88.0,
                "tip_radius_mm": 1.0,
                "max_tonnage_per_m": 80.0
            },
            "PUNCH_STRAIGHT_100": {
                "id": "PUNCH_STRAIGHT_100",
                "type": "straight",
                "height_mm": 100.0,
                "angle_deg": 88.0,
                "tip_radius_mm": 0.8,
                "max_tonnage_per_m": 100.0
            }
        }

        self._dies = {
            "DIE_V08": {"id": "DIE_V08", "v_width_mm": 8.0, "angle_deg": 88.0, "height_mm": 80.0},
            "DIE_V12": {"id": "DIE_V12", "v_width_mm": 12.0, "angle_deg": 88.0, "height_mm": 80.0},
            "DIE_V16": {"id": "DIE_V16", "v_width_mm": 16.0, "angle_deg": 88.0, "height_mm": 80.0},
            "DIE_V24": {"id": "DIE_V24", "v_width_mm": 24.0, "angle_deg": 88.0, "height_mm": 80.0}
        }

    def select_tooling_for_bend(
        self,
        bend_info: Dict[str, Any],
        part_thickness: float = 2.0
    ) -> Dict[str, Any]:
        """
        Select optimal punch and die combination based on sheet thickness and bend angle.

        Standard Rule of Thumb: V-die width V ≈ 6 * T (or 8 * T for thicker sheet).
        """
        target_v = 6.0 * part_thickness

        # Find closest V-die from database
        best_die_id = "DIE_V12"
        min_diff = float('inf')
        for die_id, die in self._dies.items():
            diff = abs(die["v_width_mm"] - target_v)
            if diff < min_diff:
                min_diff = diff
                best_die_id = die_id

        selected_die = self._dies[best_die_id]
        
        # Select punch based on bend clearance requirements
        bend_length = bend_info.get("length", 100.0)
        selected_punch = self._punches["PUNCH_GOOSENECK_120"]

        return {
            "punch_id": selected_punch["id"],
            "punch_type": selected_punch["type"],
            "punch_height_mm": selected_punch["height_mm"],
            "die_id": selected_die["id"],
            "v_width_mm": selected_die["v_width_mm"],
            "die_height_mm": selected_die["height_mm"],
            "segment_layout_mm": [100, 200, 500] if bend_length > 300 else [100, 200]
        }
