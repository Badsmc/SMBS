# BendSeq - FreeCAD Workbench Addon for Sheet Metal Bending Sequence Planning

**BendSeq** is a FreeCAD workbench addon implementing **Backward Search Architecture** (Lazy Evaluation + inverted state space traversal: Fully Bent 3D Part $\to$ Flat Pattern) for sheet metal bending sequence planning using [shaise/FreeCAD_SheetMetal](https://github.com/shaise/FreeCAD_SheetMetal).

---

## Directory Structure

```
SMBS/
├── Init.py                       # FreeCAD registration
├── InitGui.py                    # FreeCAD GUI Workbench registration
├── BendSeqWorkbench.py           # Workbench + toolbar commands
├── Core/
│   ├── bend_graph.py             # Canonical BendRecord & BendGraph model
│   ├── panel_graph.py            # Panel tree topology & moving subtree resolution
│   ├── bend_transform.py         # Single Kinematic Backend (Rodrigues matrix rotation)
│   ├── fold_state.py             # Full physical FoldState & fingerprinting
│   ├── state_cache.py            # Hash table cache of FoldStates
│   └── orchestrator.py           # Orchestrates geometry, Physical Oracle, search & post-pass
├── Geometry/
│   ├── feature_extractor.py      # Extract bends, signed angles, axes, thickness, K-factor
│   └── sheetmetal_bridge.py      # Wrapper over SheetMetal native unfold
├── Solvers/
│   ├── base_planner.py           # Abstract BasePlanner interface
│   ├── dfs_backward.py           # Reference DFS backward solver with backtracking
│   ├── astar_backward.py         # Backward A* search planner with Physical Oracle
│   ├── greedy_unfold.py          # Fast greedy backward planner
│   └── tool_optimizer.py         # Post-pass tooling layout & flip optimizer
├── Physics/
│   ├── validator.py              # Physical Oracle Gate (Single gate for step validation)
│   ├── collision_detector.py     # 4-layer collision detection (Self, Tool, Machine, Trajectory)
│   ├── machine_kinematics.py     # Punch, die & press brake machine definitions
│   └── backgauge_calculator.py   # Hybrid Backgauge X, R, Z1, Z2 calculation
├── IO/
│   ├── bendseq_writer.py         # Invert path -> write BendSeq_v1.0 JSON format
│   └── tool_database.py          # Punch & die tooling database
├── ARCHITECTURE.md               # Architectural specification (Laws LAW-1 .. LAW-5)
└── README.md                     # Usage guide
```

---

## Installation & Setup

1. Copy or symlink the `BendSeq` folder into your FreeCAD Mod directory:
   - **macOS**: `~/Library/Application Support/FreeCAD/Mod/SMBS`
   - **Linux**: `~/.local/share/FreeCAD/Mod/SMBS` or `~/.FreeCAD/Mod/SMBS`
   - **Windows**: `%APPDATA%\FreeCAD\Mod\SMBS`

2. Ensure `FreeCAD_SheetMetal` is installed via FreeCAD Addon Manager.

3. Restart FreeCAD. Select **BendSeq Planner** from the Workbench dropdown menu.

---

## Python Console Example Usage

```python
import sys, os
sys.path.append("/path/to/SMBS")

from Core.orchestrator import BendSeqOrchestrator
import FreeCADGui

# Select target 3D sheet metal object in FreeCAD
sel = FreeCADGui.Selection.getSelection()
shape = sel[0].Shape

# Run backward planner (algorithm options: "dfs", "astar", "greedy")
orchestrator = BendSeqOrchestrator()
result = orchestrator.run_planning(shape, algorithm="dfs", output_file="output_bendseq.json")

print(f"Success: {result['success']} in {result['planning_time']:.2f}s")
for step in result['sequence']:
    print(f"Step {step['step']}: Bend {step['bend_id']} | Backgauge X={step['backgauge']['X_mm']}mm, R={step['backgauge']['R_mm']}mm")
```

---

## Architectural Principles (Laws LAW-1 .. LAW-5)

1. **LAW-1 (Physical Oracle Gate)**: Single gate `PhysicalValidator.validate_step()` validates all candidate transitions.
2. **LAW-2 (Single Kinematic Backend)**: `BendTransform.rotate_subtree()` executes all panel transformations using Rodrigues matrices ($R(\theta)$).
3. **LAW-3 (Canonical FoldState)**: States maintain full physical attributes (`folded_bends`, `remaining_bends`, `panel_transforms`, `bend_transforms`, `tooling_state`).
4. **LAW-4 (Simulation)**: Replays `VerifiedFoldState[]` trajectories.
5. **LAW-5 (Binary INVALID)**: Physical invalidity returns `INVALID` classification rather than soft penalty costs.
