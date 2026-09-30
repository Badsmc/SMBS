# BendSeq Architecture Specification

## Overview
**BendSeq** is a production-grade FreeCAD workbench addon designed to solve sheet metal bending sequences using **Backward Planning Architecture**.

Instead of traditional forward folding search (which suffers from severe combinatorial explosion when generating intermediate folded states from a flat sheet), BendSeq starts from the **fully bent 3D sheet metal part** and walks **backwards** to a **fully flat sheet** by successively unfolding candidate bends.

```
Backward Search:  Fully Bent 3D Part (Root)  ──>  Unfold Bend B_k  ──>  Unfold Bend B_j  ──>  Flat Sheet (Goal)
                                                                                                    │
Forward Output:   Fully Bent 3D Part         <──  Step 2 (Bend B_k) <──  Step 1 (Bend B_j) <───────┘
```

---

## Key Design Principles

### 1. Inverted State Graph & Traversal
- **Root State ($S_0$)**: Fully bent 3D model with $N$ formed bends.
- **Goal State ($S_{\text{goal}}$)**: Flat sheet metal pattern with $0$ remaining bends.
- **Transitions**: At state $S_k$, a candidate bend $b \in S_k.\text{remaining\_bends}$ is unfolded geometrically ($\theta \to 0^\circ$).
- **Path Inversion**: Once a collision-free backward path to flat state is found, the sequence is inverted to yield the operator's physical forward bending steps.

### 2. Lazy Evaluation & Geometric State Cache (`state_cache.py`)
- The state graph is **never fully materialized in memory**.
- Intermediate 3D shape states are generated **on demand** during search node expansions.
- Each generated shape is indexed in a thread-safe `StateCache` using a unique geometric hash key calculated from:
  $$\text{Hash} = \text{SHA-256}\left(\text{Volume}, \text{Area}, \text{BoundBox}, \text{CenterOfMass}, \{\text{remaining\_bends}\}\right)$$
- If different unfolding branches lead to identical intermediate geometries (graph coalescence), `StateCache` avoids redundant collision checks and unfold calculations.

### 3. Pure Geometry SheetMetal Bridge (`sheetmetal_bridge.py`)
- **No Document Pollution**: The unfolder bridge NEVER invokes `doc.addObject()` or modifies `FreeCAD.ActiveDocument`.
- Operates purely in-memory on OpenCASCADE `Part.TopoShape` objects.
- Uses `shaise/FreeCAD_SheetMetal` native unfolder when present, or applies exact OpenCASCADE `Placement` / `Rotation` matrix transforms around neutral bend lines.

### 4. 2-Stage Collision Detector (`collision_detector.py`)
To achieve high search throughput without sacrificing physical accuracy:
1. **Stage 1 (BoundingBox Fast Reject)**: $O(1)$ axis-aligned bounding box clearance check between part shape and press brake punch/die solids. Disjoint boxes return immediately.
2. **Stage 2 (Exact TopoShape Geometry Query)**: Evaluates exact OpenCASCADE solid intersection (`shape.common(tool_shape)`). Returns collision if intersection volume exceeds micro-threshold.

---

## Critical Design Decision: Backgauge Strategy

### Options Evaluated

| Strategy | Search Speed | Kinematic Guarantee | Backgauge Precision |
| :--- | :---: | :---: | :---: |
| **1. Online / Simultaneous** | Low | High | High |
| **2. Post-Pass Only** | High | Low (Risk of un-gaugeable state) | High |
| **3. Hybrid Approach (CHOSEN)** | **High** | **High** | **High** |

### Trade-off Justification
- **Online / Simultaneous** calculates exact 2-finger stop contact points ($X, R, Z_1, Z_2$) for every intermediate search micro-step. This adds heavy raycasting overhead for thousands of un-popped nodes.
- **Post-Pass Only** defers all backgauge calculations until a path is found, but risks generating a step where no parallel rear edge is within physical backgauge stroke.
- **Hybrid Strategy (Implemented)** combines the best of both:
  - **Online Stage**: Light feasibility check during graph search (`BackgaugeCalculator.is_backgauge_feasible()`) verifies that a valid parallel reference flange exists behind the die within physical machine limits.
  - **Post-Pass Stage**: Full high-precision calculation (`BackgaugeCalculator.compute_step_backgauge()`) resolves exact $X, R, Z_1, Z_2$ coordinates, stop finger touch points, and part stability scores on the final inverted forward sequence.

---

## Solver Suite (`Solvers/`)

1. **A* Backward Planner (`astar_backward.py`)**:
   - Priority queue ordered by $f(S) = g(S) + h(S) + P(S)$.
   - Admissible heuristic $h(S) = |S.\text{remaining\_bends}| + \text{EnclosurePenalty}$.
   - Step cost $g(S)$ penalizes long flange rotations and unnecessary axis direction changes.

2. **Greedy Backward Planner (`greedy_unfold.py`)**:
   - Ultra-fast $O(N)$ solver for standard box and bracket parts.
   - Picks the first collision-free bend with lowest local step cost.

3. **Tool & Layout Optimizer (`tool_optimizer.py`)**:
   - Post-pass optimizer that annotates sequence with operator $180^\circ$ part flip warnings, minimizes V-die station swaps, and assigns punch/die segments from `ToolDatabase`.

---

## Machine-Readable Output (`bendseq_writer.py`)
Generates standardized `BendSeq_v1.0` JSON sequence files containing complete part metadata, step-by-step bend parameters, backgauge stop coordinates ($X, R, Z_1, Z_2$), and tool station assignments.
