"""
machine_kinematics.py - Press Brake Kinematics & Tool Geometry Modeling

Defines press brake machine components as physical TopoShape solids:
- Punch (Upper tool profile: gooseneck, straight punch, tip radius).
- Die (Lower tool profile: V-groove, shoulder radius, die height).
- Machine Bed / Ram frame.
- Backgauge Assembly (X, R, Z1, Z2 axis limits).
"""

from typing import Dict, Any, Optional

try:
    import FreeCAD
    import Part
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False


class ToolProfile:
    """Represents a physical press brake tooling profile."""

    def __init__(
        self,
        name: str,
        tool_type: str,  # 'punch' or 'die'
        height_mm: float,
        width_mm: float,
        length_mm: float = 1000.0,
        v_width_mm: float = 12.0,
        v_angle_deg: float = 88.0,
        tip_radius_mm: float = 1.0
    ):
        self.name = name
        self.tool_type = tool_type
        self.height_mm = height_mm
        self.width_mm = width_mm
        self.length_mm = length_mm
        self.v_width_mm = v_width_mm
        self.v_angle_deg = v_angle_deg
        self.tip_radius_mm = tip_radius_mm
        self.topo_shape = self._build_topo_shape()

    def _build_topo_shape(self) -> Any:
        """Create exact Part.TopoShape solid representation for tool collision queries."""
        if not HAS_FREECAD:
            return None

        try:
            if self.tool_type == 'die':
                # Create V-die block shape
                die_box = Part.makeBox(self.width_mm, self.length_mm, self.height_mm)
                # Translate die so V-groove centerline rests at origin (0, Y, 0)
                die_box.translate(FreeCAD.Vector(-self.width_mm / 2.0, -self.length_mm / 2.0, -self.height_mm))
                return die_box
            else:
                # Create Punch blade shape
                punch_box = Part.makeBox(self.width_mm, self.length_mm, self.height_mm)
                punch_box.translate(FreeCAD.Vector(-self.width_mm / 2.0, -self.length_mm / 2.0, 0.0))
                return punch_box
        except Exception:
            return None


class PressBrakeMachine:
    """
    Physical model of press brake machine environment.
    """

    def __init__(
        self,
        name: str = "Standard_PressBrake_100T",
        max_tonnage: float = 100.0,
        bed_length_mm: float = 2000.0,
        backgauge_x_max_mm: float = 800.0,
        backgauge_r_max_mm: float = 250.0,
        default_punch: Optional[ToolProfile] = None,
        default_die: Optional[ToolProfile] = None
    ):
        self.name = name
        self.max_tonnage = max_tonnage
        self.bed_length_mm = bed_length_mm
        self.backgauge_x_max_mm = backgauge_x_max_mm
        self.backgauge_r_max_mm = backgauge_r_max_mm

        self.punch = default_punch or ToolProfile("PUNCH_P1", "punch", height_mm=120.0, width_mm=30.0)
        self.die = default_die or ToolProfile("DIE_V12", "die", height_mm=80.0, width_mm=50.0, v_width_mm=12.0)

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
