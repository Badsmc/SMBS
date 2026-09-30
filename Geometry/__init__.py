"""
BendSeq Geometry Package
Provides FreeCAD SheetMetal unfolding bridge and 3D CAD feature extraction.
"""

from .sheetmetal_bridge import SheetMetalBridge
from .feature_extractor import FeatureExtractor

__all__ = ["SheetMetalBridge", "FeatureExtractor"]
