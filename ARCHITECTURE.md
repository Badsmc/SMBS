# BendSeq Architecture Specification (SMBS v1.0)

## Overview
**BendSeq** is a production-grade FreeCAD workbench addon designed to solve sheet metal bending sequences using a **Backward Search Architecture** with a single authoritative **Physical Oracle** and **Lazy Evaluation**.

Instead of traditional forward folding search (which suffers from severe combinatorial explosion when generating intermediate folded states from a flat sheet), BendSeq starts from the **fully bent 3D sheet metal part** and walks **backwards** to a **fully flat sheet** by successively unfolding candidate bends.

```
Geometry  ──>  Canonical Part Model  ──>  FoldState  ──>  Physical Oracle  ──>  Search  ──>  Verified Sequence  ──>  IO / Simulation / UI
```

---

## Architectural Laws (SMBS v1.0)

1. **LAW-1 — Physical Oracle Gate (`validator.py`)**:
   No solver contains its own physical model or collision rules. Solvers propose candidate bend transitions to `PhysicalValidator.validate_step(...)`, which acts as the single gate executing state consistency checks, candidate updates, kinematic transforms, and multi-layer collision queries.

2. **LAW-2 — Single Kinematic Backend (`bend_transform.py`)**:
   All panel and subtree rotations execute exclusively through `BendTransform.rotate_subtree()` using 3x3 Rodrigues rotation matrices ($R(\theta)$) around the bend axis. Direct calling of `shape.rotate()` or `shape.Placement` inside solvers or UI is strictly prohibited.

3. **LAW-3 — Canonical `FoldState` (`fold_state.py`)**:
   State is represented as a comprehensive physical snapshot containing `shape`, `folded_bends`, `remaining_bends`, `panel_transforms`, `bend_transforms`, `orientation`, `tooling_state`, `g_cost`, `h_cost`, and `reconstruct_forward_sequence()`. Unique physical fingerprints are generated via `StateCache`.

4. **LAW-4 — Simulation & Verification**:
   Simulation replays the exact trajectory of `VerifiedFoldState[]` returned by the Physical Oracle and solver.

5. **LAW-5 — Binary Valid / Invalid Classification**:
   Physically impossible candidate steps return `INVALID` with stage-specific rejection reasons, rather than arbitrary numeric penalty weights.

---

## State Graph & Traversal

- **Root State ($S_0$)**: Fully bent 3D model with $N$ formed bends.
- **Goal State ($S_{\text{goal}}$)**: Flat sheet metal pattern with $0$ remaining bends.
- **Transitions**: At state $S_k$, a candidate bend $b \in S_k.\text{remaining\_bends}$ is evaluated for unfolding ($\theta \to 0^\circ$).
- **Path Inversion**: Once a collision-free backward path to flat state is found, `reconstruct_forward_sequence()` inverts the path to yield physical forward bending steps.

---

## Multi-Layer Physical Oracle (`PhysicalValidator`)

`PhysicalValidator` evaluates candidate transitions through 4 deterministic stages:

1. **Stage 1: Consistency Check**: Verifies candidate bend $b$ is present in `current_state.remaining_bends`.
2. **Stage 2: FoldState Transition**: Computes candidate `FoldState` by removing $b$ from `remaining_bends` and adding $b$ to `folded_bends`.
3. **Stage 3: Kinematic Transform**: Applies Rodrigues rotation via `BendTransform.rotate_subtree()` to compute moving panel geometry.
4. **Stage 4: Multi-Layer Collision Detection (`CollisionDetector`)**:
   - **Layer 1: Self-Collision**: Part solid self-intersection check.
   - **Layer 2: Tool Collision**: Part solid vs. punch/die intersection check.
   - **Layer 3: Machine Collision**: Part solid vs. press brake frame/bed check.
   - **Layer 4: Trajectory Sweep Collision**: Angular discretization sweep ($\Delta \theta = 5.0^\circ$) across full bend angle.

---

## Backgauge & Tooling Strategy (Post-Pass Hybrid)

- **Online Stage**: Fast feasibility check during graph search (`BackgaugeCalculator.is_backgauge_feasible()`) verifies that a valid parallel reference flange exists behind the die within physical machine limits.
- **Post-Pass Stage**: High-precision calculation (`BackgaugeCalculator.compute_step_backgauge()`) calculates exact $X, R, Z_1, Z_2$ coordinates, stop finger touch points, and part stability scores on the final inverted forward sequence.
- **Tool Optimizer (`tool_optimizer.py`)**: Annotates steps with operator $180^\circ$ part flip warnings, minimizes V-die station swaps, and assigns punch/die segments from `ToolDatabase`.

---

## Solver Suite (`Solvers/`)

1. **DFS Backward Planner (`dfs_backward.py`)**:
   - Reference backward depth-first search with backtracking.
   - Operates through `PhysicalValidator` gate.

2. **A* Backward Planner (`astar_backward.py`)**:
   - Priority queue ordered by $f(S) = g(S) + h(S)$.
   - Evaluates successors via `PhysicalValidator`.

3. **Greedy Backward Planner (`greedy_unfold.py`)**:
   - Ultra-fast greedy planner for standard box/bracket parts.

---

## Machine-Readable Export (`bendseq_writer.py`)

Exports standard `BendSeq_v1.0` JSON sequence files containing part metadata, step-by-step bend parameters, backgauge stop coordinates ($X, R, Z_1, Z_2$), operator instructions, and tool station assignments.
