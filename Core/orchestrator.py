"""
orchestrator.py - Central Coordination Loop for Backward Bend Sequence Planning

Coordinates:
1. Feature extraction from 3D model (FeatureExtractor).
2. Pure geometric unfolding bridge without document pollution (SheetMetalBridge).
3. Lazy evaluation state caching (StateCache).
4. Backward graph space construction (BackwardGraph).
5. Search execution via modular solver algorithms (Solvers package).
6. Collision detection & physical kinematics (CollisionDetector, MachineKinematics).
7. Hybrid Backgauge position calculation (BackgaugeCalculator).
8. Backward trajectory inversion and machine-readable output writing (BendSeqWriter).
"""

import time
from typing import Dict, Any, Optional, List

from .state_cache import StateCache
from .backward_graph import BackwardGraph, BendState
from Geometry.feature_extractor import FeatureExtractor
from Geometry.sheetmetal_bridge import SheetMetalBridge
from Physics.collision_detector import CollisionDetector
from Physics.machine_kinematics import PressBrakeMachine
from Physics.backgauge_calculator import BackgaugeCalculator
from Solvers.astar_backward import AStarBackwardPlanner
from Solvers.greedy_unfold import GreedyUnfoldPlanner
from Solvers.tool_optimizer import ToolOptimizer
from IO.bendseq_writer import BendSeqWriter
from IO.tool_database import ToolDatabase


class BendSeqOrchestrator:
    """
    Main orchestration engine for BendSeq.
    """

    def __init__(self, machine_config: Optional[Dict[str, Any]] = None):
        self.state_cache = StateCache()
        self.machine = PressBrakeMachine.from_config(machine_config) if machine_config else PressBrakeMachine.default_setup()
        self.collision_detector = CollisionDetector(self.machine)
        self.bridge = SheetMetalBridge()
        self.extractor = FeatureExtractor()
        self.backgauge_calculator = BackgaugeCalculator(self.machine)
        self.tool_database = ToolDatabase()
        self.tool_optimizer = ToolOptimizer(self.tool_database)

    def run_planning(
        self,
        target_shape: Any,
        algorithm: str = "astar",
        output_file: str = "part_bendseq.json",
        max_iterations: int = 5000
    ) -> Dict[str, Any]:
        """
        Execute the full backward planning workflow on a 3D sheet metal TopoShape.

        Returns planning results dictionary including inverted forward sequence,
        backgauge settings, tooling assignments, execution time, and cache statistics.
        """
        start_time = time.time()

        # Step 1: Extract sheet metal bend features
        features = self.extractor.extract_features(target_shape)
        if not features or "bends" not in features or not features["bends"]:
            return {
                "success": False,
                "error": "No valid sheet metal bend features detected in model shape."
            }

        bends_map = features["bends"]
        k_factor = features.get("k_factor", 0.40)
        thickness = features.get("thickness", 2.0)

        # Step 2: Initialize Backward Graph with StateCache & CollisionDetector
        graph = BackwardGraph(
            initial_shape=target_shape,
            feature_map=bends_map,
            sheetmetal_bridge=self.bridge,
            collision_detector=self.collision_detector,
            state_cache=self.state_cache
        )

        # Step 3: Select and execute backward solver
        if algorithm.lower() == "greedy":
            planner = GreedyUnfoldPlanner(graph=graph)
        else:
            planner = AStarBackwardPlanner(graph=graph)

        planning_result = planner.solve(max_iterations=max_iterations)

        if not planning_result.get("success"):
            return {
                "success": False,
                "error": planning_result.get("error", "Solver failed to find a valid unfold path."),
                "planning_time": time.time() - start_time,
                "cache_stats": self.state_cache.stats
            }

        goal_state: BendState = planning_result["goal_state"]

        # Step 4: Invert backward solution trajectory (Flat -> Step 1 -> Step 2 -> Fully Bent)
        forward_sequence = goal_state.reconstruct_forward_sequence()

        # Step 5: Post-Pass Hybrid Strategy — Calculate precise Backgauge (X, R, Z1, Z2) positions
        # and optimize tool segment allocations for each step in forward order.
        detailed_sequence = []
        for step in forward_sequence:
            bend_id = step["bend_id"]
            bend_info = bends_map[bend_id]

            # Calculate exact backgauge stop finger position
            bg_data = self.backgauge_calculator.compute_step_backgauge(
                step_number=step["step_number"],
                bend_info=bend_info,
                part_thickness=thickness
            )

            # Select/assign punch & die tooling from database
            tooling = self.tool_database.select_tooling_for_bend(bend_info, thickness)

            step_entry = {
                "step": step["step_number"],
                "bend_id": bend_id,
                "angle_deg": step.get("angle", 90.0),
                "radius_mm": step.get("radius", 1.0),
                "length_mm": step.get("length", 100.0),
                "axis": step.get("axis", (0, 0, 1)),
                "backgauge": bg_data,
                "tooling": tooling
            }
            detailed_sequence.append(step_entry)

        # Optimize tooling layout changes across steps
        optimized_sequence = self.tool_optimizer.optimize_sequence_tooling(detailed_sequence)

        # Step 6: Write machine-readable BendSeq file
        writer = BendSeqWriter()
        written_path = writer.write_bendseq(
            filename=output_file,
            metadata={
                "part_thickness_mm": thickness,
                "k_factor": k_factor,
                "material": features.get("material", "Steel_S235"),
                "total_bends": len(bends_map),
                "algorithm_used": algorithm,
                "planning_time_sec": round(time.time() - start_time, 3)
            },
            sequence=optimized_sequence
        )

        total_time = time.time() - start_time

        return {
            "success": True,
            "sequence": optimized_sequence,
            "bendseq_file": written_path,
            "planning_time": total_time,
            "nodes_explored": planning_result.get("nodes_explored", 0),
            "cache_stats": self.state_cache.stats
        }
