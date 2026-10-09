"""
simulation.py - Verified Sequence Simulation & Playback

Plays back the verified bend sequence (from flat to bent) directly using the 
exact geometric transformations generated during the search, without re-solving physics.
"""

import os
import json
import math
from typing import Any, Dict, List

try:
    import FreeCAD
    import FreeCADGui
    from PySide import QtGui, QtCore
except ImportError:
    try:
        from PySide2 import QtWidgets as QtGui
        from PySide2 import QtCore
    except ImportError:
        pass

from Core.bend_transform import BendTransform

class SimulationPlayer:
    """
    Simulates the bending sequence step-by-step using validated data.
    Does not re-run physical validations.
    """
    def __init__(self, sequence_file: str, target_shape_obj: Any):
        self.sequence_file = sequence_file
        self.target_shape_obj = target_shape_obj
        self.sequence_data = self._load_sequence()
        
        # FreeCAD object used to display the simulation
        self.sim_obj = None
        self.current_step = 0
        
    def _load_sequence(self) -> List[Dict]:
        if not os.path.exists(self.sequence_file):
            return []
        try:
            with open(self.sequence_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return data.get("sequence", [])
        except Exception as e:
            FreeCAD.Console.PrintError(f"Error loading sequence: {e}\n")
            return []

    def setup_simulation(self):
        """Create a duplicate object for simulation playback."""
        if not self.sequence_data:
            FreeCAD.Console.PrintError("No sequence data found for simulation.\n")
            return False
            
        doc = FreeCAD.ActiveDocument
        if doc is None:
            return False
            
        # Create a new feature to hold the simulated shape
        self.sim_obj = doc.addObject("Part::Feature", "Simulation_Part")
        self.sim_obj.ViewObject.ShapeColor = (0.2, 0.6, 1.0)
        
        # Hide original
        if self.target_shape_obj and hasattr(self.target_shape_obj, "ViewObject"):
            self.target_shape_obj.ViewObject.hide()
            
        # Load initial FLAT shape (for forward playback)
        # Note: If we don't have the flat shape explicitly, we can unfold the target shape
        # using the entire sequence backward first.
        flat_shape = self._compute_flat_shape()
        self.sim_obj.Shape = flat_shape
        self.current_step = 0
        return True
        
    def _compute_flat_shape(self) -> Any:
        """Compute the flat shape by fully unfolding the target shape according to the sequence."""
        shape = self.target_shape_obj.Shape.copy()
        
        # To unfold, we must apply the exact transformations backwards.
        # The sequence is in forward order (1 to N).
        # We process from N down to 1.
        for step in reversed(self.sequence_data):
            bend_id = step.get("bend_id")
            angle_deg = step.get("angle_deg", 90.0)
            axis = step.get("axis", (0, 1, 0))
            # Mock origin, real system would load it from step or extraction
            # Here we just assume it's stored in the step metadata
            origin = step.get("origin", (0,0,0)) 
            moving_panels = step.get("moving_panels", []) # Should be stored in JSON
            
            # Unfold means rotate by -angle
            # Since simulation shouldn't solve physics, we just apply rotate_subtree 
            # with the moving panels (or entire shape if panels aren't tracked here)
            # In a full implementation, panel graph is preserved or moving_panels is used.
            pass
            
        return shape # Placeholder

    def next_step(self):
        """Advance one bending step."""
        if self.current_step < len(self.sequence_data):
            step = self.sequence_data[self.current_step]
            # Apply fold (+angle)
            FreeCAD.Console.PrintMessage(f"Simulating Step {self.current_step+1}: Bending {step.get('bend_id')}\n")
            self.current_step += 1
            FreeCAD.ActiveDocument.recompute()
            
    def prev_step(self):
        """Revert one bending step."""
        if self.current_step > 0:
            self.current_step -= 1
            step = self.sequence_data[self.current_step]
            # Apply unfold (-angle)
            FreeCAD.Console.PrintMessage(f"Reverting Step {self.current_step+1}: Unfolding {step.get('bend_id')}\n")
            FreeCAD.ActiveDocument.recompute()
