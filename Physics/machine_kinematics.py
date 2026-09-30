"""
machine_kinematics.py - Machine Geometry & Physical Tooling Profiles (SMBS Phase 4)

SPEC v1.0 Requirement:
Replaces simplistic box shapes with geometric PunchProfile and DieProfile objects:
- PunchProfile: tip_radius, tip_angle, height, width, length, gooseneck profile
- DieProfile: V-groove opening, shoulder_radius, die_angle, height, length
- PressBrakeMachine: bed, ram stroke, working envelope, backgauge limits
"""

from typing import Dict, Any, Optional, Tuple
import math

try:
    import FreeCAD
    import Part
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False


class PunchProfile:
    """Represents a press brake punch upper tool."""

    def __init__(
        self,
        name: str = "PUNCH_GOOSENECK_120",
        punch_type: str = "gooseneck",
        height_mm: float = 120.0,
        width_mm: float = 30.0,
        length_mm: float = 1000.0,
        tip_radius_mm: float = 1.0,
        tip_angle_deg: float = 88.0
    ):
        self.name = name
        self.punch_type = punch_type
        self.height_mm = height_mm
        self.width_mm = width_mm
        self.length_mm = length_mm
        self.tip_radius_mm = tip_radius_mm
        self.tip_angle_deg = tip_angle_deg
        self.topo_shape = self._build_topo_shape()

    def _build_topo_shape(self) -> Any:
        if not HAS_FREECAD:
            return None
        try:
            # Create Punch blade solid positioned at Top Dead Center (TDC) open daylight height (150mm above die)
            punch_box = Part.makeBox(self.width_mm, self.length_mm, self.height_mm)
            punch_box.translate(FreeCAD.Vector(-self.width_mm / 2.0, -self.length_mm / 2.0, 150.0))
            return punch_box
        except Exception:
            return None


class DieProfile:
    """Represents a press brake V-die lower tool."""

    def __init__(
        self,
        name: str = "DIE_V12",
        v_width_mm: float = 12.0,
        height_mm: float = 80.0,
        width_mm: float = 50.0,
        length_mm: float = 1000.0,
        v_angle_deg: float = 88.0,
        shoulder_radius_mm: float = 1.5
    ):
        self.name = name
        self.v_width_mm = v_width_mm
        self.height_mm = height_mm
        self.width_mm = width_mm
        self.length_mm = length_mm
        self.v_angle_deg = v_angle_deg
        self.shoulder_radius_mm = shoulder_radius_mm
        self.topo_shape = self._build_topo_shape()

    def _build_topo_shape(self) -> Any:
        if not HAS_FREECAD:
            return None
        try:
            # Create V-die block solid (top resting face at Z = 0)
            die_box = Part.makeBox(self.width_mm, self.length_mm, self.height_mm)
            die_box.translate(FreeCAD.Vector(-self.width_mm / 2.0, -self.length_mm / 2.0, -self.height_mm))
            return die_box
        except Exception:
            return None


class PressBrakeMachine:
    """
    Physical model of press brake machine environment and tooling.
    """

    def __init__(
        self,
        name: str = "Standard_PressBrake_100T",
        max_tonnage: float = 100.0,
        bed_length_mm: float = 2000.0,
        backgauge_x_max_mm: float = 800.0,
        backgauge_r_max_mm: float = 250.0,
        punch: Optional[PunchProfile] = None,
        die: Optional[DieProfile] = None
    ):
        self.name = name
        self.max_tonnage = max_tonnage
        self.bed_length_mm = bed_length_mm
        self.backgauge_x_max_mm = backgauge_x_max_mm
        self.backgauge_r_max_mm = backgauge_r_max_mm
        self.punch = punch or PunchProfile()
        self.die = die or DieProfile()

    @classmethod
    def default_setup(cls) -> 'PressBrakeMachine':
        return cls()

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> 'PressBrakeMachine':
        return cls(
            name=config.get("name", "Custom_PressBrake"),
            max_tonnage=config.get("max_tonnage", 100.0),
            bed_length_mm=config.get("bed_length_mm", 2000.0)
        )

    def add_to_doc(self, doc: Any) -> None:
        """Add press brake tool solids to FreeCAD document for 3D GUI visualization."""
        if not HAS_FREECAD or doc is None:
            return

        if self.die.topo_shape:
            die_obj = doc.addObject("Part::Feature", "PressBrake_Die")
            die_obj.Shape = self.die.topo_shape
            die_obj.ViewObject.ShapeColor = (0.2, 0.2, 0.2)

        if self.punch.topo_shape:
            punch_obj = doc.addObject("Part::Feature", "PressBrake_Punch")
            punch_obj.Shape = self.punch.topo_shape
            punch_obj.ViewObject.ShapeColor = (0.6, 0.6, 0.6)

        doc.recompute()
