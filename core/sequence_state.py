# -*- coding: utf-8 -*-
"""
core/sequence_state.py
Состояния последовательности гибки.

Содержит:
  * FrozensetSequenceState — иммутабельные состояния (порт из BendSpec).
    Работает с PanelGraph2D в 2D-развёртке (плоскость Z = cell_z).
  * ManualSequenceState — линейный state для UI (SM, обёртка над
    FrozensetSequenceState).
  * build_face_to_step_key / build_face_pair_to_step_key_mapping —
    трансляция face-pairs <-> (feat_id, line_idx).
  * is_free_aabb — лёгкая AABB-проверка коллизий без OCC.

Не импортирует FreeCADGui. Требует FreeCAD.Part только для AABB
(через core.panel_solid).

Версия: BENDBEQ_SEQUENCE_STATE_V1
"""

from __future__ import annotations

import math
from collections import deque
from typing import Dict, Iterable, List, Optional, Set, Tuple, FrozenSet

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


# =====================================================================
# Типы
# =====================================================================

BendKey = Tuple[int, int]           # (parent_face, child_face) или (child, parent)
StepKey = Tuple[str, int]           # (feat_id, line_idx)
State = FrozenSet[BendKey]
U4 = List[List[float]]
Face = int


# =====================================================================
# 4x4 matrix primitives (приватные)
# =====================================================================

def _identity4() -> U4:
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def _mat4_mul(a: U4, b: U4) -> U4:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _mat4_apply(m: U4, v: Tuple[float, float, float]):
    x, y, z = v
    return (m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
            m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
            m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3])


def _rotation_around_axis_4x4(pivot, axis_dir, angle_rad: float) -> U4:
    """4x4 поворот вокруг оси (pivot + t*axis_dir), правая рука."""
    ax, ay, az = axis_dir
    L = math.sqrt(ax * ax + ay * ay + az * az)
    if L < 1e-12:
        return _identity4()
    ax, ay, az = ax / L, ay / L, az / L

    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    C = 1.0 - c

    r00 = c + ax * ax * C
    r01 = ax * ay * C - az * s
    r02 = ax * az * C + ay * s
    r10 = ay * ax * C + az * s
    r11 = c + ay * ay * C
    r12 = ay * az * C - ax * s
    r20 = az * ax * C - ay * s
    r21 = az * ay * C + ax * s
    r22 = c + az * az * C

    px, py, pz = pivot
    tx = px - (r00 * px + r01 * py + r02 * pz)
    ty = py - (r10 * px + r11 * py + r12 * pz)
    tz = pz - (r20 * px + r21 * py + r22 * pz)

    return [[r00, r01, r02, tx],
            [r10, r11, r12, ty],
            [r20, r21, r22, tz],
            [0.0, 0.0, 0.0, 1.0]]


# =====================================================================
# Вспомогательные
# =====================================================================

def _canon(a: int, b: int) -> Tuple[int, int]:
    return (a, b) if a <= b else (b, a)


def _find_key(bend_ids: Iterable[BendKey], key: BendKey
              ) -> Optional[BendKey]:
    """Найти в bend_ids канонически совпадающий ключ."""
    target = _canon(*key)
    for k in bend_ids:
        if _canon(*k) == target:
            return k
    return None


def _edge_key_any_orientation(key_set, a: int, b: int):
    if (a, b) in key_set:
        return (a, b)
    if (b, a) in key_set:
        return (b, a)
    return None


def _angle_sign(info):
    """Извлечь (angle_deg, sign) из info (dict или object)."""
    if info is None:
        return 0.0, +1
    if isinstance(info, dict):
        return (float(info.get("angle_deg", 0.0)),
                int(info.get("sign", 1)))
    a = getattr(info, "angle_deg", None)
    if a is None:
        a = getattr(info, "angle", 0.0)
    try:
        a = float(a)
    except (TypeError, ValueError):
        try:
            a = float(a.Value)
        except Exception:
            a = 0.0
    s = getattr(info, "rotation_sign", None)
    if s is None:
        s = getattr(info, "sign", 1)
    try:
        s = int(s)
        if s not in (-1, 1):
            s = 1
    except (TypeError, ValueError):
        s = 1
    return a, s


# =====================================================================
# FrozensetSequenceState
# =====================================================================

class FrozensetSequenceState(object):
    """Иммутабельные состояния гибки на frozenset.

    Attributes:
        g2: PanelGraph2D (см. core/panel_graph_2d.py)
        bend_info: dict[(parent, child) -> {"angle_deg": float, "sign": ±1}]
        thickness_mm: float
        bend_ids: list[(parent, child)] — все рёбра графа
    """

    def __init__(self, panel_graph_2d, bend_info, thickness_mm=None):
        self.g2 = panel_graph_2d
        self.bend_info = dict(bend_info) if bend_info else {}
        self.thickness_mm = (float(thickness_mm)
                             if thickness_mm is not None else 0.0)

        self._cache: Dict = {}
        self._bend_canon: Set[Tuple[int, int]] = set()
        self.bend_ids: List[BendKey] = []
        for e in panel_graph_2d.bends:
            k = (e.parent_face_index, e.child_face_index)
            self.bend_ids.append(k)
            self._bend_canon.add(_canon(*k))

        self._parent_of: Optional[Dict[int, Optional[int]]] = None

    # ------------------------------------------------------------------
    # Кэш T_p
    # ------------------------------------------------------------------

    def _cache_key(self, done_bends: State, fractions) -> tuple:
        done_canon = frozenset(_canon(a, b) for (a, b) in done_bends)
        if fractions:
            fr = frozenset(
                (_canon(k[0], k[1]), round(float(v), 6))
                for k, v in fractions.items()
            )
        else:
            fr = frozenset()
        return (done_canon, fr)

    def transforms(self, done_bends: Iterable[BendKey],
                   fractions=None) -> Dict[Face, U4]:
        """T_p для состояния. Кэшируется.

        Args:
            done_bends: iterable of (parent, child); порядок не важен.
            fractions: dict[(parent, child) -> float] или None.

        Returns:
            dict[face_index -> U4]
        """
        key = self._cache_key(done_bends, fractions)
        cached = self._cache.get(key)
        if cached is not None:
            return cached
        T = self._compute_transforms(done_bends, fractions)
        self._cache[key] = T
        return T

    def clear_cache(self) -> None:
        self._cache.clear()

    def cache_size(self) -> int:
        return len(self._cache)

    # ------------------------------------------------------------------
    # Множества состояний
    # ------------------------------------------------------------------

    def empty_state(self) -> State:
        return frozenset()

    def full_state(self) -> State:
        return frozenset(self.bend_ids)

    def is_done(self, done_bends: State, key: BendKey) -> bool:
        return _find_key(done_bends, key) is not None

    def apply(self, done_bends: State, key: BendKey) -> State:
        """Вернуть новое состояние с добавленным гибом (fraction=1.0)."""
        found = _find_key(self.bend_ids, key)
        if found is None:
            raise ValueError("bend {} not in graph".format(key))
        new_set = set(done_bends)
        new_set.add(found)
        return frozenset(new_set)

    def undo(self, done_bends: State, key: BendKey) -> State:
        """Вернуть новое состояние без указанного гиба."""
        found = _find_key(done_bends, key)
        if found is None:
            return frozenset(done_bends)
        new_set = set(done_bends)
        new_set.discard(found)
        return frozenset(new_set)

    # ------------------------------------------------------------------
    # Дерево от anchor
    # ------------------------------------------------------------------

    def _build_tree(self) -> Dict[int, Optional[int]]:
        """BFS-дерево от anchor. Кэшируется."""
        if self._parent_of is not None:
            return self._parent_of
        anchor = self.g2.anchor_face_index
        if anchor is None:
            self._parent_of = {}
            return self._parent_of
        parent_of: Dict[int, Optional[int]] = {anchor: None}
        q = deque([anchor])
        while q:
            n = q.popleft()
            for nb in sorted(self.g2.neighbors(n)):
                if nb in parent_of:
                    continue
                parent_of[nb] = n
                q.append(nb)
        self._parent_of = parent_of
        return parent_of

    def _child_side(self, key: BendKey) -> Optional[int]:
        """face_index панели-ребёнка в дереве от anchor, или None."""
        parent_of = self._build_tree()
        a, b = key
        if parent_of.get(a) == b:
            return a
        if parent_of.get(b) == a:
            return b
        return None

    # ------------------------------------------------------------------
    # Фильтры backward / forward
    # ------------------------------------------------------------------

    def undoable(self, done_bends: State) -> List[BendKey]:
        """Список гибов, которые можно отогнуть (backward)."""
        result = []
        done_canon = {_canon(*k) for k in done_bends}
        for k in self.bend_ids:
            if _canon(*k) not in done_canon:
                continue
            child = self._child_side(k)
            if child is None:
                continue
            has_other = False
            for other in done_bends:
                if _canon(*other) == _canon(*k):
                    continue
                if other[0] == child or other[1] == child:
                    has_other = True
                    break
            if not has_other:
                result.append(k)
        return result

    def foldable(self, done_bends: State) -> List[BendKey]:
        """Список гибов, которые можно выполнить (forward)."""
        result = []
        done_canon = {_canon(*k) for k in done_bends}
        parent_of = self._build_tree()
        for k in self.bend_ids:
            if _canon(*k) in done_canon:
                continue
            a, b = k
            if parent_of.get(a) == b:
                par, ch = b, a
            elif parent_of.get(b) == a:
                par, ch = a, b
            else:
                continue
            ok = True
            node = par
            while parent_of.get(node) is not None:
                up = parent_of[node]
                if _canon(up, node) not in done_canon:
                    ok = False
                    break
                node = up
            if ok:
                result.append(k)
        return result

    # ------------------------------------------------------------------
    # Диагностика
    # ------------------------------------------------------------------

    def describe_state(self, done_bends: State) -> str:
        done_canon = {_canon(*k) for k in done_bends}
        return "state: {}/{} done, undoable={}, foldable={}".format(
            len(done_canon), len(self.bend_ids),
            len(self.undoable(done_bends)),
            len(self.foldable(done_bends)))

    def neighbors_of(self, face_index: int) -> Set[int]:
        return set(self.g2.neighbors(int(face_index)))

    def all_bends(self) -> List[BendKey]:
        return list(self.bend_ids)

    # ------------------------------------------------------------------
    # Внутренний: кинематика
    # ------------------------------------------------------------------

    def _compute_transforms(self, done_bends: Iterable[BendKey],
                            fractions=None) -> Dict[Face, U4]:
        """Собственная 2D-кинематика: плоская развёртка → T_p.

        BFS от anchor. Для каждого ребёнка — поворот относительно
        оси гиба в плоскости Z = cell_z.
        """
        if fractions is None:
            fractions = {}

        done_set: Set[BendKey] = set()
        for k in done_bends:
            done_set.add((k[0], k[1]))

        anchor = self.g2.anchor_face_index
        if anchor is None:
            return {}

        # BFS-порядок и parent_of
        order: List[Face] = []
        parent_of: Dict[Face, Optional[Face]] = {anchor: None}
        q = deque([anchor])
        while q:
            p = q.popleft()
            order.append(p)
            for nb in sorted(self.g2.neighbors(p)):
                if nb in parent_of:
                    continue
                parent_of[nb] = p
                q.append(nb)

        cell_z = float(self.g2.cell_z)
        T: Dict[Face, U4] = {anchor: _identity4()}

        for p in order:
            if p == anchor:
                continue
            par = parent_of.get(p)
            if par is None:
                T[p] = _identity4()
                continue

            axis = self.g2.get_bend_between(par, p)
            if axis is None:
                T[p] = [row[:] for row in T[par]]
                continue

            info_key = _edge_key_any_orientation(
                self.bend_info.keys(),
                axis.parent_face_index, axis.child_face_index)
            info = self.bend_info.get(info_key) if info_key else None

            done_key = _edge_key_any_orientation(
                done_set,
                axis.parent_face_index, axis.child_face_index)

            if done_key is None:
                T[p] = [row[:] for row in T[par]]
                continue

            angle_deg, sign = _angle_sign(info)
            fr = float(fractions.get(done_key, 1.0))
            fr = max(0.0, min(1.0, fr))

            # SheetMetal-конвенция: angle_deg — внутренний угол гиба.
            # Относительный поворот нормалей = 180 - angle_deg.
            rel_rot_deg = (180.0 - angle_deg) if angle_deg > 1e-9 else 0.0
            theta = math.radians(rel_rot_deg) * fr * sign
            pivot = (axis.p1[0], axis.p1[1], cell_z)
            axis_dir = (axis.direction[0], axis.direction[1], 0.0)

            R_local = _rotation_around_axis_4x4(pivot, axis_dir, theta)
            T[p] = _mat4_mul(T[par], R_local)

        return T


# =====================================================================
# Лёгкий AABB-collision
# =====================================================================

def _panel_world_aabb(T_p: U4, p2d, cell_z: float,
                      thickness: float) -> Optional[tuple]:
    """AABB панели в мировых координатах."""
    ext = getattr(p2d, "exterior_2d", None)
    if not ext:
        return None
    xs, ys, zs = [], [], []
    for (x, y) in ext:
        wx, wy, wz = _mat4_apply(T_p, (float(x), float(y), float(cell_z)))
        xs.append(wx); ys.append(wy); zs.append(wz)
    if not xs:
        return None
    #pad = thickness * 0.5 + 0.05
    pad = 0.02
    return (min(xs) - pad, max(xs) + pad,
            min(ys) - pad, max(ys) + pad,
            min(zs) - pad, max(zs) + pad)


def _aabb_overlap(a, b, min_overlap: float = 0.05) -> bool:
    if a is None or b is None:
        return False
    ox = min(a[1], b[1]) - max(a[0], b[0])
    oy = min(a[3], b[3]) - max(a[2], b[2])
    oz = min(a[5], b[5]) - max(a[4], b[4])
    return ox > min_overlap and oy > min_overlap and oz > min_overlap


def is_free_aabb(ss: FrozensetSequenceState, state: State,
                 exclude_pairs: Optional[Set[Tuple[int, int]]] = None
                 ) -> bool:
    """Быстрая AABB-проверка коллизий для состояния.

    Без OCC (только матрицы + bounding box). Используется как
    is_free_fn в backward-поиске.

    Args:
        ss: FrozensetSequenceState
        state: frozenset гибов
        exclude_pairs: пары (a, b), которые игнорируются (parent-child).

    Returns:
        True если коллизий нет.
    """
    g2 = ss.g2
    thickness = ss.thickness_mm or 1.5

    T = ss.transforms(state)

    aabbs: Dict[Face, tuple] = {}
    for fi, p2d in g2.panels.items():
        if fi not in T:
            continue
        aabb = _panel_world_aabb(T[fi], p2d, g2.cell_z, thickness)
        if aabb is not None:
            aabbs[fi] = aabb

    ids = sorted(aabbs.keys())
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            key = (a, b) if a < b else (b, a)
            if exclude_pairs and key in exclude_pairs:
                continue
            if _aabb_overlap(aabbs[a], aabbs[b], min_overlap=0.1):
                return False
    return True


def adjacent_pairs(g2) -> Set[Tuple[int, int]]:
    """Множество пар parent-child — для exclude в is_free_aabb."""
    out = set()
    for e in g2.bends:
        a = min(e.parent_face_index, e.child_face_index)
        b = max(e.parent_face_index, e.child_face_index)
        out.add((a, b))
    return out


# =====================================================================
# ManualSequenceState (SM + мост)
# =====================================================================

class ManualSequenceState(object):
    """Линейное состояние последовательности для UI.

    Совместимо с SM-кодом (task_panel). Дополнительно может быть
    связано с FrozensetSequenceState для валидации (set_frozenset_state).
    """

    def __init__(self, flat_shape, all_bends: list):
        self.flat_shape = flat_shape
        self.all_bends = list(all_bends)
        self.ordered_ids: List[str] = []
        self.step_results: List = []
        # Мост к frozenset-состоянию (опционально)
        self.frozenset_state: Optional[FrozensetSequenceState] = None
        # Таблица id -> BendKey для трансляции (опционально)
        self._id_to_key: Dict[str, BendKey] = {}

    # --- SM API ---

    def append_bend(self, bend_id: str) -> None:
        if bend_id not in self.ordered_ids:
            self.ordered_ids.append(bend_id)

    def reset(self) -> None:
        self.ordered_ids.clear()
        self.step_results.clear()

    def is_complete(self) -> bool:
        return len(self.ordered_ids) == len(self.all_bends)

    def get_ordered_bends(self) -> list:
        id_map = {b.id: b for b in self.all_bends}
        return [id_map[bid] for bid in self.ordered_ids if bid in id_map]

    # --- Мост ---

    def set_frozenset_state(self,
                            ss: FrozensetSequenceState,
                            id_to_key: Optional[Dict[str, BendKey]] = None
                            ) -> None:
        """Привязать frozenset-state и таблицу id -> BendKey."""
        self.frozenset_state = ss
        if id_to_key is not None:
            self._id_to_key = dict(id_to_key)

    def to_frozenset(self) -> FrozenSet[BendKey]:
        """Текущее ordered_ids как frozenset BendKey."""
        if not self.frozenset_state:
            return frozenset()
        out = set()
        for bid in self.ordered_ids:
            key = self._id_to_key.get(bid)
            if key is not None:
                out.add(key)
        return frozenset(out)

    def from_frozenset(self, state: FrozenSet[BendKey]) -> None:
        """Перезаписать ordered_ids из frozenset."""
        if not self.frozenset_state:
            return
        rev = {v: k for k, v in self._id_to_key.items()}
        new_ids = []
        for key in state:
            bid = rev.get(key) or rev.get((key[1], key[0]))
            if bid is not None:
                new_ids.append(bid)
        self.ordered_ids = new_ids


# =====================================================================
# Трансляция face-pairs <-> step keys
# =====================================================================

def build_face_to_step_key(panel_graph) -> Dict[Face, StepKey]:
    """Из PanelGraph (SM) собрать face_index -> (feat_id, line_idx).

    Работает с SM-PanelGraph: nodes имеют .id, .parent_id, .children.
    child.feature_id / parent.children [(child_id, feat_id, line_idx)].

    Если у PanelNode есть .face_index (BS-вариант) — используем его.
    Иначе берём int(node.id.split("_")[-1]) как fallback.
    """
    mapping: Dict[Face, StepKey] = {}

    def _face_of(node) -> Optional[int]:
        fi = getattr(node, "face_index", None)
        if fi is not None:
            try:
                return int(fi)
            except (TypeError, ValueError):
                pass
        nid = getattr(node, "id", None)
        if isinstance(nid, str) and "_" in nid:
            try:
                return int(nid.split("_")[-1])
            except (TypeError, ValueError):
                pass
        return None

    nodes = getattr(panel_graph, "nodes", None) or {}
    for nid, node in nodes.items():
        if getattr(node, "feature_id", None) is None:
            continue
        parent_id = getattr(node, "parent_id", None)
        if parent_id is None:
            continue
        parent = nodes.get(parent_id)
        if parent is None:
            continue
        line_idx = 0
        feat_id = getattr(node, "feature_id", None)
        for (cid, ffid, li) in (getattr(parent, "children", []) or []):
            if cid == nid:
                if ffid:
                    feat_id = ffid
                line_idx = int(li) if li is not None else 0
                break
        fi = _face_of(node)
        if fi is None:
            continue
        mapping[fi] = (str(feat_id), line_idx)
    return mapping


def build_face_pair_to_step_key_mapping(
        panel_graph,
        panel_graph_2d
) -> Dict[Tuple[int, int], StepKey]:
    """Трансляция face-pairs (BS) -> step keys (SM).

    Для каждой пары (parent_face, child_face) в PanelGraph2D
    находим соответствующий (feat_id, line_idx) через PanelGraph.

    Ориентация пары не важна: если face ребёнка известен — берём его.
    Если лицо parent — тоже пробуем (fallback для симметричных случаев).

    Returns:
        dict[(parent_face, child_face)] -> (feat_id, line_idx).
        Неполный, если часть пар не разрешилась.
    """
    face_to_step = build_face_to_step_key(panel_graph)
    mapping: Dict[Tuple[int, int], StepKey] = {}

    for bend in (getattr(panel_graph_2d, "bends", []) or []):
        a = int(bend.parent_face_index)
        b = int(bend.child_face_index)
        if b in face_to_step:
            mapping[(a, b)] = face_to_step[b]
        elif a in face_to_step:
            mapping[(a, b)] = face_to_step[a]

    return mapping


def invert_mapping(mapping: Dict[Tuple[int, int], StepKey]
                   ) -> Dict[StepKey, Tuple[int, int]]:
    """Обратная таблица step_key -> face_pair."""
    return {v: k for k, v in mapping.items()}


# =====================================================================
# Утилиты
# =====================================================================

def order_to_step_keys(order_forward: Iterable[BendKey],
                       mapping: Dict[Tuple[int, int], StepKey]
                       ) -> List[StepKey]:
    """Перевести order_forward (face-pairs) в список step keys."""
    out = []
    for (a, b) in order_forward:
        key = mapping.get((a, b)) or mapping.get((b, a))
        if key is not None:
            out.append(key)
    return out


def order_to_face_pairs(order_steps: Iterable[StepKey],
                        mapping: Dict[Tuple[int, int], StepKey]
                        ) -> List[BendKey]:
    """Перевести step keys обратно в face-pairs."""
    rev = invert_mapping(mapping)
    out = []
    for sk in order_steps:
        fp = rev.get(sk)
        if fp is not None:
            out.append(fp)
    return out