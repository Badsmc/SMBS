"""
BendSeq Physics Package
Provides machine kinematics, multi-layer collision detector, and hybrid backgauge position calculator.
"""

from .machine_kinematics import PressBrakeMachine, PunchProfile, DieProfile
from .collision_detector import CollisionDetector
from .backgauge_calculator import BackgaugeCalculator

# Backward compatibility alias
ToolProfile = PunchProfile

__all__ = [
    "PressBrakeMachine",
    "PunchProfile",
    "DieProfile",
    "ToolProfile",
    "CollisionDetector",
    "BackgaugeCalculator"
]
