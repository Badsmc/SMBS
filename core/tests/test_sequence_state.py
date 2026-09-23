# -*- coding: utf-8 -*-
"""Тесты FrozensetSequenceState."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from core.sequence_state import FrozensetSequenceState, _canon
from core.panel_graph_2d import PanelNode2D, BendAxis2D, PanelGraph2D


def _chain():
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
    return g2


class TestFrozensetSequenceState(unittest.TestCase):

    def setUp(self):
        self.g2 = _chain()
        self.ss = FrozensetSequenceState(
            self.g2,
            {(0, 1): {"angle_deg": 90.0, "sign": 1},
             (1, 2): {"angle_deg": 90.0, "sign": 1}},
            thickness_mm=1.5)

    def test_canon(self):
        self.assertEqual(_canon(0, 1), (0, 1))
        self.assertEqual(_canon(1, 0), (0, 1))
        self.assertEqual(_canon(2, 1), (1, 2))

    def test_empty_full(self):
        self.assertEqual(len(self.ss.empty_state()), 0)
        self.assertEqual(len(self.ss.full_state()), 2)

    def test_apply_undo(self):
        s1 = self.ss.apply(self.ss.empty_state(), (0, 1))
        self.assertEqual(len(s1), 1)
        s2 = self.ss.undo(s1, (0, 1))
        self.assertEqual(len(s2), 0)

    def test_is_done_both_orientations(self):
        s1 = self.ss.apply(self.ss.empty_state(), (0, 1))
        self.assertTrue(self.ss.is_done(s1, (1, 0)))
        self.assertTrue(self.ss.is_done(s1, (0, 1)))

    def test_undoable_full(self):
        u = self.ss.undoable(self.ss.full_state())
        u_canon = sorted(_canon(*k) for k in u)
        self.assertEqual(u_canon, [(0, 1), (1, 2)])

    def test_foldable_empty(self):
        f = self.ss.foldable(self.ss.empty_state())
        f_canon = sorted(_canon(*k) for k in f)
        self.assertEqual(f_canon, [(0, 1), (1, 2)])

    def test_transforms_empty_identity(self):
        from core.sequence_state import _identity4
        T = self.ss.transforms(self.ss.empty_state())
        for fi in (0, 1, 2):
            self.assertEqual(T[fi], _identity4())

    def test_cache(self):
        self.ss.clear_cache()
        self.ss.transforms(self.ss.empty_state())
        self.ss.transforms(self.ss.empty_state())
        self.assertEqual(self.ss.cache_size(), 1)


if __name__ == "__main__":
    unittest.main()
