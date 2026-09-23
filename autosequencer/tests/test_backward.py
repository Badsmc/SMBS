# -*- coding: utf-8 -*-
"""Тесты backward-поиска."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from autosequencer import backward as B
from core.sequence_state import (
    FrozensetSequenceState, is_free_aabb, adjacent_pairs,
)
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


class TestBackward(unittest.TestCase):
    def setUp(self):
        g2 = _chain()
        self.ss = FrozensetSequenceState(
            g2,
            {(0, 1): {"angle_deg": 90.0, "sign": 1},
             (1, 2): {"angle_deg": 90.0, "sign": 1}},
            thickness_mm=1.5)
        self.free = lambda st: is_free_aabb(
            self.ss, st, exclude_pairs=adjacent_pairs(g2))

    def test_greedy(self):
        r = B.search_greedy(self.ss, self.free)
        self.assertTrue(r.success)
        self.assertEqual(len(r.order_forward), 2)

    def test_astar(self):
        r = B.search_astar(self.ss, self.free)
        self.assertTrue(r.success)
        self.assertEqual(r.source, "backward_astar")

    def test_verify(self):
        r = B.search_astar(self.ss, self.free)
        v = B.verify_sequence(self.ss, r.order_forward, self.free)
        self.assertTrue(v["valid"])

    def test_unknown_strategy(self):
        with self.assertRaises(ValueError):
            B.search_sequence(self.ss, self.free, strategy="bogus")


if __name__ == "__main__":
    unittest.main()
