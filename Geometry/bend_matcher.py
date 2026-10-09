"""
bend_matcher.py - Algorithms for mapping Bend IDs to 2D Sketch lines.

SPEC v1.0 Requirement (Stage 3):
Calculates correct flat pattern coordinates by mathematical unfolding.
Matches bends against 2D sketch geometries using length and distance matrix.
"""

from typing import Dict, List, Tuple, Any, Optional
import math
from Core.bend_graph import BendRecord
from Core.panel_graph import PanelGraph
from Core.bend_transform import BendTransform
from Geometry.bend_mapping import BendMappingRecord, BendMatchStatus

def compute_flat_coordinates(
    bends_dict: Dict[str, Dict[str, Any]],
    panel_graph: PanelGraph,
    root_panel_id: Optional[str] = None
) -> Dict[str, Tuple[float, float, float]]:
    """
    Computes the 2D flat pattern center of each bend by traversing the PanelGraph.
    Unfolds each child panel relative to its parent by rotating it by -signed_angle.
    
    Returns a dictionary mapping bend_id -> (X, Y, Z) flat coordinate.
    """
    if not root_panel_id:
        if not panel_graph.panels:
            return {}
        root_panel_id = list(panel_graph.panels.keys())[0]

    # Queue for BFS: (current_panel_id, accumulated_transform_list)
    # accumulated_transform_list = [(axis_origin, axis_dir, angle_to_rotate), ...]
    queue = [(root_panel_id, [])]
    visited = set([root_panel_id])
    
    flat_centers: Dict[str, Tuple[float, float, float]] = {}

    while queue:
        current_panel, transforms = queue.pop(0)
        
        # Traverse edges (bends) connected to this panel
        if current_panel not in panel_graph._adj:
            continue
            
        for target_panel, bend_id in panel_graph._adj[current_panel].items():
            # Find the bend
            bend_data = bends_dict.get(bend_id)
            if not bend_data: continue

            # Apply all accumulated transforms to the bend's origin
            flat_pt = bend_data["axis_origin"]
            # Apply transforms in order from leaf back to root (local to global)
            for (cx, cy, cz), (dx, dy, dz), angle in transforms:
                flat_pt = BendTransform.transform_point(flat_pt, (cx, cy, cz), (dx, dy, dz), angle)
            
            flat_centers[bend_id] = flat_pt
            
            if target_panel not in visited:
                visited.add(target_panel)
                
                # To unfold the child panel, we must rotate it by -signed_angle around THIS bend's flat axis!
                # Wait, the transform list must store the CURRENT (folded) axis, because we rotate the folded subtree
                # all the way back to the root's frame.
                # So the transform for unfolding this bend is: rotate around bend_data["axis_origin"] by -bend_data["signed_angle"]
                # But whether it's + or - depends on if we traverse parent->child or child->parent.
                # If current_panel is parent_panel_id, we are unfolding the child.
                # In bend_transform, rotate_subtree uses -signed_angle to unfold child relative to parent.
                angle = -bend_data["signed_angle"]
                if current_panel != bend_data.get("parent_panel_id"):
                    # We are traversing backwards (child -> parent), so rotation is reversed
                    angle = -angle
                
                new_transforms = transforms.copy()
                new_transforms.insert(0, (bend_data["axis_origin"], bend_data["axis_direction"], angle))
                queue.append((target_panel, new_transforms))
                
    return flat_centers


def match_bends_to_sketch(
    flat_centers: Dict[str, Tuple[float, float, float]],
    bends_dict: Dict[str, Dict[str, Any]],
    sketch_lines: List[Dict[str, Any]],
    length_tolerance: float = 100.0,
    distance_tolerance: float = 10.0
) -> Dict[str, BendMappingRecord]:
    """
    Matches computed flat centers of bends to 2D FreeCAD sketch lines.
    sketch_lines: list of dicts {"index": int, "start": (x,y,z), "end": (x,y,z), "length": float}
    """
    results = {}
    
    # Pre-calculate centers of sketch lines
    for line in sketch_lines:
        line["center"] = (
            (line["start"][0] + line["end"][0]) / 2.0,
            (line["start"][1] + line["end"][1]) / 2.0,
            (line["start"][2] + line["end"][2]) / 2.0
        )

    # Calculate distance matrix for sketch lines
    sketch_dist_matrix = {}
    for i, line_i in enumerate(sketch_lines):
        for j, line_j in enumerate(sketch_lines):
            ci = line_i["center"]
            cj = line_j["center"]
            sketch_dist_matrix[(i, j)] = math.sqrt((ci[0]-cj[0])**2 + (ci[1]-cj[1])**2 + (ci[2]-cj[2])**2)

    # Calculate distance matrix for flat bends
    bend_ids = list(flat_centers.keys())
    bend_dist_matrix = {}
    for i, bid_i in enumerate(bend_ids):
        for j, bid_j in enumerate(bend_ids):
            ci = flat_centers[bid_i]
            cj = flat_centers[bid_j]
            bend_dist_matrix[(bid_i, bid_j)] = math.sqrt((ci[0]-cj[0])**2 + (ci[1]-cj[1])**2 + (ci[2]-cj[2])**2)

    used_lines = set()

    for bid in bend_ids:
        b_data = bends_dict[bid]
        target_len = b_data.get("length", 100.0)
        
        # 1. Filter by length
        candidates = []
        for line in sketch_lines:
            if abs(line["length"] - target_len) < length_tolerance:
                candidates.append(line)
                
        rec = BendMappingRecord(
            bend_id=bid,
            length=target_len,
            signed_angle=b_data.get("signed_angle"),
            bend_sign=b_data.get("bend_sign"),
            source_center=b_data.get("axis_origin"),
            source_axis=b_data.get("axis_direction")
        )

        if not candidates:
            rec.match_status = BendMatchStatus.UNMATCHED
            rec.error_reason = "No sketch line found with matching length."
            results[bid] = rec
            continue
            
        if len(candidates) == 1:
            idx = candidates[0]["index"]
            if idx in used_lines:
                rec.match_status = BendMatchStatus.AMBIGUOUS
                rec.error_reason = "Duplicate binding for length."
            else:
                used_lines.add(idx)
                rec.match_status = BendMatchStatus.MATCHED
                rec.flat_line_index = idx
            rec.flat_start_point = candidates[0]["start"]
            rec.flat_end_point = candidates[0]["end"]
            rec.match_method = "length"
            results[bid] = rec
            continue

        # 2. Ambiguity resolution using topology (Distance Matrix)
        # Find which candidate best matches the distance to ALL ALREADY MATCHED bends,
        # or just distances to all other bends (isomorphism).
        best_candidate = None
        best_match_score = -1
        best_error = float('inf')
        ambiguous = False
        
        for cand in candidates:
            cand_idx = cand["index"]
            if cand_idx in used_lines:
                continue
                
            my_dists = sorted([bend_dist_matrix[(bid, other)] for other in bend_ids if other != bid])
            cand_dists = sorted([sketch_dist_matrix[(sketch_lines.index(cand), j)] for j in range(len(sketch_lines)) if j != sketch_lines.index(cand)])
            
            match_score = 0
            err_sum = 0.0
            
            used_my = set()
            for cd in cand_dists:
                best_d = None
                best_diff = float('inf')
                best_k = -1
                for k, md in enumerate(my_dists):
                    if k not in used_my:
                        diff = abs(cd - md)
                        if diff < best_diff:
                            best_diff = diff
                            best_k = k
                            best_d = md
                if best_diff <= distance_tolerance:
                    match_score += 1
                    err_sum += best_diff
                    used_my.add(best_k)
                else:
                    err_sum += 1000.0
            
            if match_score > best_match_score or (match_score == best_match_score and err_sum < best_error - 1.0):
                best_match_score = match_score
                best_error = err_sum
                best_candidate = cand
            elif match_score == best_match_score and abs(err_sum - best_error) <= 1.0:
                # Tie breaker: deterministically pick the smaller index to ensure stability
                if best_candidate is None or cand["index"] < best_candidate["index"]:
                    best_candidate = cand

        if not best_candidate:
            rec.match_status = BendMatchStatus.AMBIGUOUS
            rec.error_reason = "No candidate found after distance matrix check."
        else:
            used_lines.add(best_candidate["index"])
            rec.match_status = BendMatchStatus.MATCHED
            rec.flat_line_index = best_candidate["index"]
            rec.flat_start_point = best_candidate["start"]
            rec.flat_end_point = best_candidate["end"]
            rec.match_method = "length+topology"
            
        results[bid] = rec

    return results
