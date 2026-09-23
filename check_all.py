#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_all.py — headless-проверки всех модулей BendSeq (Фазы 1-3.5).

Запуск:
    cd ~/Desktop/SMBS && python3 check_all.py

Ничего не создаёт на диске. Не требует FreeCAD (headless).
Проверяет: импорт модуля, наличие публичных символов, базовое поведение.
"""

import os
import sys
import traceback

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


# =====================================================================
# Мини-фреймворк
# =====================================================================

PASSED = 0
FAILED = 0
FAILED_LIST = []
SKIPPED = 0


def ok(label):
    global PASSED
    PASSED += 1
    print("  ✓ {}".format(label))


def fail(label, exc=None):
    global FAILED
    FAILED += 1
    FAILED_LIST.append(label)
    msg = "  ✗ {}".format(label)
    if exc is not None:
        msg += ": {}".format(exc)
    print(msg)


def skip(label, why=""):
    global SKIPPED
    SKIPPED += 1
    print("  ○ {} ({})".format(label, why))


def section(title):
    print()
    print("=" * 70)
    print("  {}".format(title))
    print("=" * 70)


def check(label, fn):
    """Выполнить fn(), поймать AssertionError/Exception."""
    try:
        fn()
        ok(label)
    except AssertionError as e:
        fail(label, e or "assert failed")
    except Exception as e:
        tb = traceback.format_exc().splitlines()
        short = "{}: {}".format(type(e).__name__, e)
        print("    traceback tail:")
        for line in tb[-4:]:
            print("      {}".format(line))
        fail(label, short)


# =====================================================================
# Фаза 0 — config
# =====================================================================

section("Фаза 0: config.py")


def check_config():
    from config import SEQUENCE_CONFIG
    assert "sheet_thickness_mm" in SEQUENCE_CONFIG
    assert "collision_volume_epsilon" in SEQUENCE_CONFIG
    assert "scoring_w_orientation" in SEQUENCE_CONFIG
    assert "unfold_verify" in SEQUENCE_CONFIG
    assert "auto_backward_max_nodes" in SEQUENCE_CONFIG


check("config.SEQUENCE_CONFIG содержит все ключи", check_config)


# =====================================================================
# Фаза 1 — panel_solid, panel_graph_2d, sequence_state, unfold_extractor
# =====================================================================

section("Фаза 1: core.panel_solid")


def check_panel_solid():
    from core import panel_solid
    assert hasattr(panel_solid, "PART_OK")
    assert hasattr(panel_solid, "panel_solid_world")
    assert hasattr(panel_solid, "panel_solid_from_face")
    assert hasattr(panel_solid, "transform_solid")
    assert hasattr(panel_solid, "describe_solid")
    assert hasattr(panel_solid, "_identity_u4")
    # Identity-проверка чисто вычислима
    assert panel_solid._is_identity_u4(panel_solid._identity_u4())
    not_id = [r[:] for r in panel_solid._identity_u4()]
    not_id[0][3] = 5.0
    assert not panel_solid._is_identity_u4(not_id)
    # Clean polygon
    cl = panel_solid._clean_polygon_2d(
        [(0, 0), (0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])
    assert cl == [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    # Headless — функции возвращают None
    assert panel_solid.panel_solid_world(None, None, 1.5) is None
    assert panel_solid.panel_solid_from_face(None, None) is None


check("panel_solid: symbols + identity + clean_polygon + headless",
      check_panel_solid)


section("Фаза 1: core.panel_graph_2d")


def check_panel_graph_2d():
    from core import panel_graph_2d as pg2
    assert hasattr(pg2, "SHAPELY_OK")
    assert hasattr(pg2, "PanelNode2D")
    assert hasattr(pg2, "BendAxis2D")
    assert hasattr(pg2, "PanelGraph2D")
    assert hasattr(pg2, "build_panel_graph_2d")
    assert hasattr(pg2, "polygonize_cells")
    assert hasattr(pg2, "match_cells_to_panels")

    n = pg2.PanelNode2D(3, 0,
                        exterior_2d=[(0, 0), (10, 0), (10, 5), (0, 5)],
                        centroid_2d=(5, 2.5), area_mm2=50.0, cell_z=1.5)
    assert n.face_index == 3
    assert n.area_mm2 == 50.0

    b = pg2.BendAxis2D(3, 4, 0, 1, (0, 0), (10, 0))
    assert abs(b.length_mm - 10.0) < 1e-9

    g = pg2.PanelGraph2D()
    g.cell_z = 1.5
    v = g.validate()
    assert v["panels_n"] == 0 and v["ok"] is True

    g._bend_by_pair[(3, 4)] = b
    g._bend_by_pair[(4, 3)] = b
    assert g.bend_by_face_pair(3, 4) is b
    assert g.bend_by_face_pair(4, 3) is b
    assert g.get_bend_between(3, 4) is b
    assert g.get_bend_between(4, 3) is b
    assert g.get_bend_between(3, 99) is None

    # headless — build_panel_graph_2d падает понятно
    try:
        pg2.build_panel_graph_2d(None, None, None)
        assert False, "should raise"
    except RuntimeError:
        pass


check("panel_graph_2d: symbols + PanelNode2D + PanelGraph2D + access",
      check_panel_graph_2d)


section("Фаза 1: core.sequence_state")


def check_sequence_state():
    from core import sequence_state as seq
    from core.panel_graph_2d import PanelNode2D, BendAxis2D, PanelGraph2D

    assert hasattr(seq, "FrozensetSequenceState")
    assert hasattr(seq, "ManualSequenceState")
    assert hasattr(seq, "is_free_aabb")
    assert hasattr(seq, "adjacent_pairs")
    assert hasattr(seq, "build_face_to_step_key")
    assert hasattr(seq, "build_face_pair_to_step_key_mapping")
    assert hasattr(seq, "order_to_step_keys")
    assert hasattr(seq, "order_to_face_pairs")

    # 3-panel chain
    g2 = PanelGraph2D()
    g2.cell_z = 1.5
    g2.thickness_mm = 1.5
    for i in range(3):
        g2.panels[i] = PanelNode2D(
            i, i,
            exterior_2d=[(10 * i, 0), (10 * i + 10, 0),
                         (10 * i + 10, 10), (10 * i, 10)],
            centroid_2d=(10 * i + 5, 5), area_mm2=100.0, cell_z=1.5)
        g2.panel_to_cell[i] = i
        g2.cell_to_panel[i] = i
    b01 = BendAxis2D(0, 1, 0, 1, (10, 0), (10, 10))
    b12 = BendAxis2D(1, 2, 1, 2, (20, 0), (20, 10))
    g2.bends = [b01, b12]
    g2.adjacency = {0: {1}, 1: {0, 2}, 2: {1}}
    for k in [(0, 1), (1, 0)]:
        g2._bend_by_pair[k] = b01
    for k in [(1, 2), (2, 1)]:
        g2._bend_by_pair[k] = b12

    bend_info = {(0, 1): {"angle_deg": 90.0, "sign": 1},
                 (1, 2): {"angle_deg": 90.0, "sign": 1}}
    ss = seq.FrozensetSequenceState(g2, bend_info, thickness_mm=1.5)

    assert g2.anchor_face_index == 1
    assert sorted(ss.bend_ids) == [(0, 1), (1, 2)]

    e = ss.empty_state()
    f = ss.full_state()
    assert len(e) == 0 and len(f) == 2

    s1 = ss.apply(e, (0, 1))
    assert ss.is_done(s1, (1, 0))
    s2 = ss.undo(s1, (0, 1))
    assert len(s2) == 0

    # undoable(full) = листья
    u = ss.undoable(f)
    u_canon = sorted(seq._canon(*k) for k in u)
    assert u_canon == [(0, 1), (1, 2)]

    # foldable(empty) = оба ребра (anchor в центре)
    fold = ss.foldable(e)
    fold_canon = sorted(seq._canon(*k) for k in fold)
    assert fold_canon == [(0, 1), (1, 2)]

    # T(empty) — identity
    T_empty = ss.transforms(e)
    for fi in (0, 1, 2):
        assert T_empty[fi] == seq._identity4()

    # T(full) — панель 0 повёрнута вверх
    T_full = ss.transforms(f)
    c0 = seq._mat4_apply(T_full[0], (5.0, 5.0, 1.5))
    assert abs(c0[0] - 10.0) < 1e-6
    assert abs(c0[1] - 5.0) < 1e-6
    assert abs(c0[2] - 6.5) < 1e-6

    # AABB свободно на пустом/полном с exclude
    exclude = seq.adjacent_pairs(g2)
    assert seq.is_free_aabb(ss, e, exclude_pairs=exclude)
    assert seq.is_free_aabb(ss, f, exclude_pairs=exclude)


check("sequence_state: frozenset + transforms + AABB", check_sequence_state)


section("Фаза 1: core.unfold_extractor")


def check_unfold_extractor():
    from core import unfold_extractor as ue
    assert hasattr(ue, "extract_unfold_data")
    assert hasattr(ue, "extract_unfold_data_ex")
    assert hasattr(ue, "describe_extraction")

    v = ue._min_panel_volume(1.5, 50.0)
    assert abs(v - 75.0) < 1e-9
    v = ue._min_panel_volume(0.5, 50.0)
    assert abs(v - 25.0) < 1e-9
    v = ue._min_panel_volume(None)
    assert v == 100.0

    d = ue.extract_unfold_data_ex(None, "Face1")
    assert d["ok"] is False
    assert d["panels"] == []

    p, h, u = ue.extract_unfold_data(None, "Face1")
    assert p == [] and h == [] and u is None


check("unfold_extractor: min_volume + headless API", check_unfold_extractor)


# =====================================================================
# Фаза 2 — backward, scoring, фасад
# =====================================================================

section("Фаза 2: autosequencer.backward")


def check_backward():
    from autosequencer import backward as B
    from core.sequence_state import (
        FrozensetSequenceState, is_free_aabb, adjacent_pairs,
    )
    from core.panel_graph_2d import PanelNode2D, BendAxis2D, PanelGraph2D

    assert hasattr(B, "SearchResult")
    assert hasattr(B, "search_greedy")
    assert hasattr(B, "search_astar")
    assert hasattr(B, "search_sequence")
    assert hasattr(B, "verify_sequence")

    # chain
    g2 = PanelGraph2D()
    g2.cell_z = 1.5
    g2.thickness_mm = 1.5
    for i in range(3):
        g2.panels[i] = PanelNode2D(
            i, i,
            exterior_2d=[(10 * i, 0), (10 * i + 10, 0),
                         (10 * i + 10, 10), (10 * i, 10)],
            centroid_2d=(10 * i + 5, 5), area_mm2=100.0, cell_z=1.5)
        g2.panel_to_cell[i] = i
        g2.cell_to_panel[i] = i
    b01 = BendAxis2D(0, 1, 0, 1, (10, 0), (10, 10))
    b12 = BendAxis2D(1, 2, 1, 2, (20, 0), (20, 10))
    g2.bends = [b01, b12]
    g2.adjacency = {0: {1}, 1: {0, 2}, 2: {1}}
    for k in [(0, 1), (1, 0)]:
        g2._bend_by_pair[k] = b01
    for k in [(1, 2), (2, 1)]:
        g2._bend_by_pair[k] = b12

    ss = FrozensetSequenceState(
        g2, {(0, 1): {"angle_deg": 90.0, "sign": 1},
             (1, 2): {"angle_deg": 90.0, "sign": 1}},
        thickness_mm=1.5)
    free_fn = lambda st: is_free_aabb(
        ss, st, exclude_pairs=adjacent_pairs(g2))

    res = B.search_greedy(ss, free_fn)
    assert res.success and len(res.order_forward) == 2

    res = B.search_astar(ss, free_fn)
    assert res.success and res.source == "backward_astar"

    res = B.search_sequence(ss, free_fn, strategy="greedy")
    assert res.success
    res = B.search_sequence(ss, free_fn, strategy="astar")
    assert res.success
    try:
        B.search_sequence(ss, free_fn, strategy="nonsense")
        assert False
    except ValueError:
        pass

    v = B.verify_sequence(ss, [(0, 1), (1, 2)], free_fn)
    assert v["valid"]


check("backward: greedy + astar + verify", check_backward)


section("Фаза 2: autosequencer.scoring")


def check_scoring():
    from autosequencer import scoring as S
    from core.sequence_state import FrozensetSequenceState
    from core.panel_graph_2d import PanelNode2D, BendAxis2D, PanelGraph2D

    assert hasattr(S, "ScoringWeights")
    assert hasattr(S, "score_sequence")
    assert hasattr(S, "rank_sequences")
    assert hasattr(S, "deduplicate_orders")

    # простая chain (та же, что выше)
    g2 = PanelGraph2D()
    g2.cell_z = 1.5
    g2.thickness_mm = 1.5
    for i in range(3):
        g2.panels[i] = PanelNode2D(
            i, i,
            exterior_2d=[(10 * i, 0), (10 * i + 10, 0),
                         (10 * i + 10, 10), (10 * i, 10)],
            centroid_2d=(10 * i + 5, 5), area_mm2=100.0, cell_z=1.5)
        g2.panel_to_cell[i] = i
        g2.cell_to_panel[i] = i
    b01 = BendAxis2D(0, 1, 0, 1, (10, 0), (10, 10))
    b12 = BendAxis2D(1, 2, 1, 2, (20, 0), (20, 10))
    g2.bends = [b01, b12]
    g2.adjacency = {0: {1}, 1: {0, 2}, 2: {1}}
    for k in [(0, 1), (1, 0)]:
        g2._bend_by_pair[k] = b01
    for k in [(1, 2), (2, 1)]:
        g2._bend_by_pair[k] = b12
    ss = FrozensetSequenceState(
        g2, {(0, 1): {"angle_deg": 90.0, "sign": 1},
             (1, 2): {"angle_deg": 90.0, "sign": 1}},
        thickness_mm=1.5)

    sc = S.score_sequence([(0, 1), (1, 2)], ss)
    assert sc["order_length"] == 2
    assert "total" in sc

    ranked = S.rank_sequences([[(0, 1), (1, 2)], [(1, 2), (0, 1)]], ss)
    assert len(ranked) == 2

    dedup = S.deduplicate_orders([[(0, 1)], [(1, 0)], [(1, 2)]])
    assert len(dedup) == 2

    w = S.ScoringWeights()
    from config import SEQUENCE_CONFIG
    assert w.w_orientation == SEQUENCE_CONFIG["scoring_w_orientation"]


check("scoring: metrics + rank + dedup", check_scoring)


section("Фаза 2: autosequencer.__init__ (фасад)")


def check_autoseq_facade():
    from autosequencer import (
        calculate_auto_sequence_ex, calculate_auto_sequence,
    )
    from core.panel_graph_2d import PanelNode2D, BendAxis2D, PanelGraph2D

    g2 = PanelGraph2D()
    g2.cell_z = 1.5
    g2.thickness_mm = 1.5
    for i in range(3):
        g2.panels[i] = PanelNode2D(
            i, i,
            exterior_2d=[(10 * i, 0), (10 * i + 10, 0),
                         (10 * i + 10, 10), (10 * i, 10)],
            centroid_2d=(10 * i + 5, 5), area_mm2=100.0, cell_z=1.5)
        g2.panel_to_cell[i] = i
        g2.cell_to_panel[i] = i
    b01 = BendAxis2D(0, 1, 0, 1, (10, 0), (10, 10))
    b12 = BendAxis2D(1, 2, 1, 2, (20, 0), (20, 10))
    g2.bends = [b01, b12]
    g2.adjacency = {0: {1}, 1: {0, 2}, 2: {1}}
    for k in [(0, 1), (1, 0)]:
        g2._bend_by_pair[k] = b01
    for k in [(1, 2), (2, 1)]:
        g2._bend_by_pair[k] = b12

    bend_info = {(0, 1): {"angle_deg": 90.0, "sign": 1},
                 (1, 2): {"angle_deg": 90.0, "sign": 1}}

    result = calculate_auto_sequence_ex(
        panel_graph_2d=g2, bend_info=bend_info, thickness_mm=1.5,
        strategy="backward_astar", logger=lambda m: None)
    assert result["ok"]
    assert len(result["order_face_pairs"]) == 2

    ids = calculate_auto_sequence(
        panel_graph_2d=g2, bend_info=bend_info, thickness_mm=1.5,
        strategy="backward_astar", logger=lambda m: None)
    assert isinstance(ids, list)


check("facade: calculate_auto_sequence + _ex", check_autoseq_facade)


# =====================================================================
# Фаза 3.1 — bend_feature, panel_graph, bend_kinematics, geometry_utils
# =====================================================================

section("Фаза 3.1: core.bend_feature")


def check_bend_feature():
    from core import bend_feature as bf
    assert hasattr(bf, "BendSpec") or hasattr(bf, "BendLine")
    assert hasattr(bf, "BendLine")
    assert hasattr(bf, "BendFeature")
    assert hasattr(bf, "group_bend_specs_by_feature")
    assert hasattr(bf, "describe_features")

    class FS:
        def __init__(self, bid, feat, sub):
            self.id = bid
            self.metadata = {"feature": feat, "sub_element": sub}

    specs = [FS("A", "Bend001", "Edge3"),
             FS("B", "Bend004_1", "Edge111"),
             FS("C", "Bend004_2", "Edge127"),
             FS("D", "", "Edge0")]
    g = bf.group_bend_specs_by_feature(specs)
    assert "Bend001" in g
    assert "Bend004_1" in g and "Bend004_2" in g


check("bend_feature: groups", check_bend_feature)


section("Фаза 3.1: core.panel_graph")


def check_panel_graph():
    from core import panel_graph as pg
    assert hasattr(pg, "PART_OK")
    assert hasattr(pg, "PanelGraph")
    assert hasattr(pg, "PanelNode")
    assert hasattr(pg, "PanelGraphError")

    n = pg.PanelNode("Panel_5", shape=object(), face_index=42)
    assert n.face_index == 42
    assert n.resolved_face_index() == 42

    n2 = pg.PanelNode("Panel_7", shape=object())
    assert n2.face_index is None
    assert n2.resolved_face_index() == 7

    try:
        pg.PanelGraph([], [], [])
        assert False
    except pg.PanelGraphError:
        pass


check("panel_graph: PanelNode + face_index fallback", check_panel_graph)


section("Фаза 3.1: core.bend_kinematics")


def check_bend_kinematics():
    from core import bend_kinematics as bk
    assert hasattr(bk, "BendKinematics")
    assert hasattr(bk, "_as_dir")
    assert hasattr(bk, "_to_tuple")

    kin = bk.BendKinematics(operator_side=(0.0, -1.0, 0.0))
    assert kin.is_configured()
    assert kin.operator_side is not None

    unconf = bk.BendKinematics()
    assert not unconf.is_configured()

    kin_cfg = bk.BendKinematics.from_config()
    assert isinstance(kin_cfg.is_configured(), bool)

    assert bk._as_dir(None) is None
    t = bk._to_tuple((1.5, 2.5, 3.5))
    assert t == (1.5, 2.5, 3.5)


check("bend_kinematics: config + _as_dir", check_bend_kinematics)


section("Фаза 3.1: core.geometry_utils")


def check_geometry_utils():
    from core import geometry_utils as gu
    assert hasattr(gu, "shape_in_world")
    assert hasattr(gu, "bbox_of")
    assert gu.shape_in_world(None) is None
    assert gu.bbox_of(None) is None


check("geometry_utils: headless None returns", check_geometry_utils)


# =====================================================================
# Фаза 3.2 — kinematic_engine
# =====================================================================

section("Фаза 3.2: core.kinematic_engine")


def check_kinematic_engine():
    from core import kinematic_engine as ke
    assert hasattr(ke, "KinematicEngine")
    assert hasattr(ke, "KinematicEngineError")
    for name in ("all_step_keys", "compute_step_placements",
                 "build_step_compound", "state_at",
                 "panel_face_index", "step_key_to_panel_id",
                 "build_face_pair_map", "describe_signs"):
        assert hasattr(ke.KinematicEngine, name)
    if not ke._FC_OK:
        try:
            ke.KinematicEngine(panel_graph=None, bend_features=[])
            assert False
        except ke.KinematicEngineError:
            pass


check("kinematic_engine: symbols + headless guard",
      check_kinematic_engine)


# =====================================================================
# Фаза 3.3.1 — collision_engine + verifier
# =====================================================================

section("Фаза 3.3.1: core.collision_engine + verifier")


def check_collision_verifier():
    from core import collision_engine as ce
    from core import verifier as vf

    assert hasattr(ce, "CollisionEngine")
    assert hasattr(ce, "CollisionResult")

    v = ce.CollisionEngine._resolve_volume_epsilon(None)
    assert v == 0.1
    assert ce.CollisionEngine._resolve_volume_epsilon(0.001) == 0.001

    res = ce.CollisionEngine.pair(None, None)
    assert res.check_failed
    assert len(res.details) >= 1
    assert "FreeCAD" in res.details[0]

    d = ce.CollisionEngine.describe(res)
    assert "CHECK-FAILED" in d

    class BB:
        def __init__(self, xmin, xmax, ymin, ymax, zmin, zmax):
            self.XMin, self.XMax = xmin, xmax
            self.YMin, self.YMax = ymin, ymax
            self.ZMin, self.ZMax = zmin, zmax

    a = BB(0, 10, 0, 10, 0, 10)
    b = BB(20, 30, 0, 10, 0, 10)
    c = BB(5, 15, 0, 10, 0, 10)
    assert not ce.CollisionEngine._bbox_intersects(a, b, margin=0.1)
    assert ce.CollisionEngine._bbox_intersects(a, c, margin=0.1)

    assert hasattr(vf, "verify_against_reference")
    assert hasattr(vf, "VerificationReport")
    tv, ta, tb, ti = vf._tolerances()
    assert tv == 0.025

    rep = vf.verify_against_reference(None, None)
    assert rep.ok is False
    assert any("FreeCAD" in n or "headless" in n for n in rep.notes)
    d = rep.to_dict()
    assert "volume_sim" in d


check("collision + verifier", check_collision_verifier)


# =====================================================================
# Фаза 3.3.2 — bend_simulator
# =====================================================================

section("Фаза 3.3.2: core.bend_simulator")


def check_bend_simulator():
    from core import bend_simulator as bs
    assert hasattr(bs, "simulate_bend_occ")
    assert hasattr(bs, "point_on_moving_side")
    assert hasattr(bs, "transform_bend_spec_inplace")
    assert hasattr(bs, "SliceFailure")
    assert hasattr(bs, "_normalize")
    assert hasattr(bs, "_make_bend_plane")
    assert hasattr(bs, "_auto_volume_tol")
    assert hasattr(bs, "_volumes_consistent")
    assert hasattr(bs, "_drop_tiny_fragments")
    assert hasattr(bs, "_make_compound")
    assert hasattr(bs, "_merge_group")
    assert hasattr(bs, "_make_half_space_solid")
    assert hasattr(bs, "_slice_by_plane")
    assert hasattr(bs, "_distance_to_plane")
    assert hasattr(bs, "_side_touches_plane")
    assert hasattr(bs, "_classify")
    assert hasattr(bs, "_clamp_center_inside_bbox")
    assert hasattr(bs, "_log_assembly")
    assert hasattr(bs, "_guess_sheet_thickness")

    e = bs.SliceFailure("msg", bend_id="B1", details={"a": 1})
    assert e.bend_id == "B1"
    assert e.details == {"a": 1}

    if not bs._FC_OK:
        try:
            bs.simulate_bend_occ(None, type("B", (), {"id": "B1"})())
            assert False
        except bs.SliceFailure:
            pass

        assert bs.transform_bend_spec_inplace(None, None) is False

    # _guess_sheet_thickness: config имеет 1.5
    t = bs._guess_sheet_thickness(None)
    assert t == 1.5


check("bend_simulator: helpers + public API", check_bend_simulator)


# =====================================================================
# Фаза 3.3.3 — sequence_validator
# =====================================================================

section("Фаза 3.3.3: core.sequence_validator")


def check_sequence_validator():
    from core import sequence_validator as sv
    assert hasattr(sv, "StepValidationResult")
    assert hasattr(sv, "SequenceValidator")
    assert hasattr(sv, "_clone_bend_spec")
    assert hasattr(sv, "_sheet_normal_of")
    assert hasattr(sv, "_sheet_thickness")

    r = sv.StepValidationResult(
        bend_id="B1", step_number=1, ok=True,
        shape_after=None, collision_type=None, reason_human="ok")
    assert r.details == {} and r.volume == 0.0

    t = sv._sheet_thickness(None)
    assert t == 1.0

    v = sv.SequenceValidator()
    assert v.punch_adapter is None

    class FB:
        id = "B1"
        name = "B1"
        center = (0, 0, 0)
        axis = (1, 0, 0)
        angle = 90.0
        length = 100.0
        direction = "up"
        normal = (0, 0, 1)
        up_hint = (0, 0, 1)
        metadata = {}

    if not sv._FC_OK:
        res = v.validate_step(None, FB(), 0)
        assert not res.ok
        assert res.collision_type == "CHECK_FAILED"

        results = v.validate_full_sequence(None, [FB(), FB()])
        assert len(results) == 2
        assert all(not r.ok for r in results)

        rr = v.validate_full_sequence_reverse(None, [FB()])
        assert rr == []


check("sequence_validator: symbols + headless",
      check_sequence_validator)


# =====================================================================
# Фаза 3.4 — adapters + tooling
# =====================================================================

section("Фаза 3.4: adapters + tooling")


def check_adapters_tooling():
    from adapters import freecad_adapter as fa
    from adapters import text_overlay as to
    from plugins.tooling import base_tool as bt
    from plugins.tooling import press_adapter as pa

    assert hasattr(fa, "BendSpec")
    assert hasattr(fa, "FreeCADSheetMetalAdapter")
    ad = fa.FreeCADSheetMetalAdapter(unfold_obj=None)
    assert ad.get_flat_shape() is None
    assert ad.get_bend_specs() == []
    assert ad.build_bend_features() == []

    assert hasattr(to, "SequenceTextOverlay")
    ov = to.SequenceTextOverlay(parent_unfold=None)
    ov.clear()  # no-op
    ov.draw_numbers([])  # no-op

    assert hasattr(bt, "ToolAdapter")
    ta = bt.ToolAdapter(None)
    assert ta.make_virtual_shape(None, None) is None
    assert bt._as_dir(None) is None
    assert bt._as_dir((1, 2, 3)) == (1.0, 2.0, 3.0)

    assert hasattr(pa, "PressGeometryAdapter")
    pg = pa.PressGeometryAdapter([])
    assert pg.get_virtual_shapes() == []


check("adapters + tooling: headless safe", check_adapters_tooling)


# =====================================================================
# Фаза 3.5 — topology + kinematics + solver
# =====================================================================

section("Фаза 3.5: topology + kinematics + solver")


def check_topology():
    from autosequencer import topology as tp

    class FB:
        def __init__(self, bid, feature=None, parent=None, sub=None):
            self.id = bid
            self.angle = 90.0          # <-- добавить
            self.direction = "up"       # <-- добавить
            self.length = 100.0         # <-- добавить
            self.metadata = {
                "feature": feature or "",
                "parent_feature": parent or "",
                "sub_element": sub or "",
             "sub_element": sub or "",
             }

    bends = [
        FB("B1", feature="Bend001", parent="Pad"),
        FB("B2", feature="Bend002", parent="Bend001"),
        FB("B3", feature="Bend003", parent="Bend002"),
    ]
    graph, fob = tp.feature_dependency_graph(bends)
    assert graph["Bend002"] == {"Bend001"}
    assert graph["Bend003"] == {"Bend002"}

    ordered = tp.topological_order(bends)
    ids = [b.id for b in ordered]
    assert ids.index("B1") < ids.index("B2") < ids.index("B3")

    class A:
        def get_bend_specs(self):
            return [FB("B1"), FB("B2")]

    specs = tp.collect_bend_specs(A(), logger=lambda m: None)
    assert len(specs) == 2


check("topology: dependency graph + order", check_topology)


def check_autoseq_kinematics():
    from autosequencer import kinematics as kin

    class FakeRes:
        def __init__(self, ok):
            self.ok = ok
            self.shape_after = "s" if ok else None
            self.is_contact = False
            self.bend_id = "?"
            self.step_number = 0
            self.collision_type = None if ok else "FAKE"
            self.reason_human = ""
            self.details = {}
            self.volume = 0.0
            self.min_distance = None

    class FakeValidator:
        def __init__(self):
            self.calls = []
        def validate_step(self, current_shape, bend, step_idx,
                          already_done, kinematics):
            self.calls.append((bend.id, step_idx))
            return FakeRes(True)

    class FB:
        def __init__(self, bid):
            self.id = bid
            self.metadata = {}

    fv = FakeValidator()
    eng = kin.ReverseKinematicsEngine(fv, kinematics=None,
                                       logger=lambda m: None)
    b1 = FB("B1")
    r1 = eng.can_apply("flat", b1, 0, [])
    assert r1.ok
    assert len(fv.calls) == 1
    # Cache hit
    r1b = eng.can_apply("flat2", b1, 0, [])
    assert r1b is r1
    assert len(fv.calls) == 1
    eng.clear_cache()
    assert eng.cache_size() == 0


check("autosequencer.kinematics: cache", check_autoseq_kinematics)


def check_solver():
    from autosequencer import solver as sv
    from autosequencer import kinematics as kin

    assert hasattr(sv, "greedy_sequence")
    assert hasattr(sv, "astar_sequence")

    cfg = sv._penalties()
    assert cfg["base"] == 1.0

    f = sv._axis_parallel_factor((1, 0, 0), (1, 0, 0))
    assert abs(f - 1.0) < 1e-9
    f2 = sv._axis_parallel_factor((1, 0, 0), (0, 1, 0))
    assert abs(f2 - 0.0) < 1e-9

    class FakeRes:
        def __init__(self, ok):
            self.ok = ok
            self.shape_after = "s" if ok else None
            self.is_contact = False
            self.bend_id = "?"
            self.step_number = 0
            self.collision_type = None if ok else "FAKE"
            self.reason_human = ""
            self.details = {}
            self.volume = 0.0
            self.min_distance = None

    class FakeValidator:
        def __init__(self, blocked=None):
            self.blocked = blocked or set()
        def validate_step(self, current_shape, bend, step_idx,
                          already_done, kinematics):
            if bend.id in self.blocked:
                return FakeRes(False)
            return FakeRes(True)

    class FB:
        def __init__(self, bid):
            self.id = bid
            self.metadata = {}
            self.angle = 90.0
            self.direction = "up"
            self.length = 100.0
            self.axis = (1, 0, 0)

    bends = [FB("B1"), FB("B2"), FB("B3")]

    eng = kin.ReverseKinematicsEngine(FakeValidator(), kinematics=None,
                                       logger=lambda m: None)
    seq = sv.greedy_sequence(bends, flat_shape="flat", engine=eng,
                              allow_partial=True, logger=lambda m: None)
    assert seq == ["B1", "B2", "B3"]

    eng2 = kin.ReverseKinematicsEngine(
        FakeValidator(blocked={"B2"}), kinematics=None,
        logger=lambda m: None)
    seq2 = sv.greedy_sequence(bends, flat_shape="flat", engine=eng2,
                                allow_partial=True,
                                logger=lambda m: None)
    assert "B2" not in seq2

    eng3 = kin.ReverseKinematicsEngine(FakeValidator(), kinematics=None,
                                        logger=lambda m: None)
    seq3 = sv.astar_sequence(bends, flat_shape="flat", engine=eng3,
                               max_iterations=500, max_seconds=5.0,
                               allow_partial=True, logger=lambda m: None)
    assert sorted(seq3) == ["B1", "B2", "B3"]


check("solver: greedy + astar", check_solver)


# =====================================================================
# Итог
# =====================================================================

section("ИТОГ")
print("  PASS: {}".format(PASSED))
print("  FAIL: {}".format(FAILED))
print("  SKIP: {}".format(SKIPPED))
if FAILED_LIST:
    print()
    print("  Провалившиеся проверки:")
    for lbl in FAILED_LIST:
        print("    - {}".format(lbl))

print()
if FAILED == 0:
    print("  ✓ ВСЕ ПРОВЕРКИ ПРОШЛИ")
    sys.exit(0)
else:
    print("  ✗ ЕСТЬ ОШИБКИ")
    sys.exit(1)