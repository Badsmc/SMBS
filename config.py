# -*- coding: utf-8 -*-
"""config.py — единый конфиг BendSeq."""

SEQUENCE_CONFIG = {
    # --- Оснастка ---
    # "none"        — не резать геометрию
    # "bbox"        — обрезка BBox-параллелепипедом в локальной системе
    "tool_trim_mode": "none",
    "tool_max_cross_section_mm": 3.0,

    "tool_use_reference_point": True,
    "tool_default_role": "auto",
    "tool_approach_from_height": True,

    # --- Геометрия листа ---
    "sheet_thickness_mm": 1.5,

    # --- Коллизии ---
    "volume_epsilon": 50.0,
    "collision_volume_epsilon": 0.1,
    "bbox_margin": 0.1,
    "contact_tolerance": 0.05,

    # --- Контроль объёма OCC ---
    "volume_loss_max_rel": 0.02,
    "volume_loss_max_rel_reverse": 0.05,

    # --- Verifier ---
    "verify_tol_volume_rel":  0.025,
    "verify_tol_area_rel":    0.010,
    "verify_tol_bbox_mm":     2.0,
    "verify_tol_inertia_rel": 0.02,

    # --- Визуализация ---
    "kinematic_visualization": True,

    # --- Подвод пуансона ---
    "approach_distance": 50.0,
    "approach_samples": 3,

    # --- Клиренсы ---
    "punch_clearance_offset": 2.0,
    "die_clearance_offset": 0.5,
    "backgauge_clearance": 1.0,

    # --- Флаги проверок ---
    "check_punch": True,
    "check_die": True,
    "check_press_frame": True,
    "check_self_collision": True,
    "check_backgauge": True,

    # --- UI / Overlay ---
    "char_height": 6.0,
    "overlay_offset_mm": -3.0,
    "overlay_slide_mm": 20.0,
    "overlay_shift_x_mm": 0.0,
    "overlay_shift_y_mm": 0.0,
    "overlay_debug": True,

    # --- Кинематика ---
    "kinematics_operator_side": (0.0, -1.0, 0.0),
    "kinematics_prefer_moving": "operator",

    # --- Auto-Sequencer forward ---
    "auto_greedy_restarts": 1,
    "auto_astar_weight": 2.0,
    "auto_greedy_max_seconds": 10.0,
    "auto_astar_max_seconds": 60.0,

    # --- Auto-Sequencer backward ---
    "auto_backward_max_nodes": 20000,
    "auto_backward_max_steps": 10000,
    "auto_backward_greedy": True,
    "auto_hybrid": True,

    # --- Штрафы ---
    "auto_penalty_base":         1.0,
    "auto_penalty_flip":         2.0,
    "auto_penalty_rotation":     1.0,
    "auto_penalty_tool_change":  2.0,
    "auto_penalty_tool_length":  1.0,
    "auto_penalty_backgauge":    0.5,
    "auto_penalty_feature":      0.3,
    "auto_penalty_contact":      3.0,

    # --- Scoring ---
    "scoring_w_orientation":     2.0,
    "scoring_w_depth_ping_pong": 0.5,
    "scoring_w_leaf":            1.0,

    # --- Верификация развёртки ---
    "unfold_verify": True,
    "unfold_area_conservation_tol": 1.0,
    "unfold_adjacency_check": True,
    "unfold_verify_raise": False,

    # --- Multi-edge ---
    "multi_edge_min_volume_factor": 200.0,

    # --- Debug ---
    "debug_enabled": True,
    "debug_default_path": None,

    # --- Direction ---
    "direction_from_topology_invert": False,
}