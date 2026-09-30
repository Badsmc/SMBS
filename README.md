# BendSeq - FreeCAD Workbench Addon for Sheet Metal Bending Sequence Planning

**BendSeq** is a FreeCAD workbench addon implementing **Backward Planning Architecture** (Lazy Evaluation + inverted state space search) for sheet metal bending sequence optimization using [shaise/FreeCAD_SheetMetal](https://github.com/shaise/FreeCAD_SheetMetal).

---

## Directory Structure

```
BendSeq/
├── Init.py                       # FreeCAD registration
├── InitGui.py                    # FreeCAD GUI Workbench registration
├── BendSeqWorkbench.py           # Workbench + toolbar commands
├── Core/
│   ├── backward_graph.py         # Inverted state tree (bent → flat)
│   ├── state_cache.py            # Hash table of Part.TopoShape (lazy cache)
│   └── orchestrator.py           # Main loop: select bend → cache check → SheetMetal call → collision → cost
├── Geometry/
│   ├── sheetmetal_bridge.py      # Thin wrapper over SheetMetal native unfold (virtual, no UI features)
│   └── feature_extractor.py      # Extract bends, radii, angles, K-factor from source 3D model
├── Solvers/
│   ├── base_planner.py           # Common interface
│   ├── astar_backward.py         # A* with penalty system
│   ├── greedy_unfold.py          # Fast greedy (first physically possible bend)
│   └── tool_optimizer.py         # Heuristic that penalizes tool changes
├── Physics/
│   ├── collision_detector.py     # Kinematic simulation of flange unfold + BoundingBox + exact tool geometry
│   ├── machine_kinematics.py     # Punch/die as Part.TopoShape
│   └── backgauge_calculator.py   # Hybrid Backgauge X, R, Z1, Z2 calculation
├── IO/
│   ├── bendseq_writer.py         # Invert successful path → write BendSeq file
│   └── tool_database.py          # Local tool profiles
├── ARCHITECTURE.md               # Architectural specification & trade-off rationale
└── README.md                     # Usage guide
```

---

## Installation & Setup

1. Copy or symlink the `BendSeq` folder into your FreeCAD Mod directory:
   - **macOS**: `~/Library/Application Support/FreeCAD/Mod/BendSeq`
   - **Linux**: `~/.local/share/FreeCAD/Mod/BendSeq` or `~/.FreeCAD/Mod/BendSeq`
   - **Windows**: `%APPDATA%\FreeCAD\Mod\BendSeq`

2. Ensure `FreeCAD_SheetMetal` is installed via FreeCAD Addon Manager.

3. Restart FreeCAD. Select **BendSeq Planner** from the Workbench dropdown menu.

---

## Python Console Example Usage

```python
import sys, os
sys.path.append("/path/to/BendSeq")

from Core.orchestrator import BendSeqOrchestrator
import FreeCADGui

# Select target 3D sheet metal object in FreeCAD
sel = FreeCADGui.Selection.getSelection()
shape = sel[0].Shape

# Run backward planner
orchestrator = BendSeqOrchestrator()
result = orchestrator.run_planning(shape, algorithm="astar", output_file="output_bendseq.json")

print(f"Success: {result['success']} in {result['planning_time']:.2f}s")
for step in result['sequence']:
    print(f"Step {step['step']}: Bend {step['bend_id']} | Backgauge X={step['backgauge']['X_mm']}mm")
```

---

## Backgauge Strategy Summary
BendSeq implements a **Hybrid Strategy** for rear stop backgauge calculation:
1. **Online Stage**: Fast feasibility check during graph search (`is_backgauge_feasible`) to ensure a rear reference flange exists.
2. **Post-Pass Stage**: High-precision calculation (`compute_step_backgauge`) of $X, R, Z_1, Z_2$ stop finger coordinates on the final sequence.
See [ARCHITECTURE.md](ARCHITECTURE.md) for full trade-off analysis.
