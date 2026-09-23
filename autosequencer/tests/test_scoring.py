# -*- coding: utf-8 -*-
"""Тесты scoring."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

from autosequencer import scoring as S


class TestScoringWeights(unittest.TestCase):
    def test_defaults(self):
        w = S.ScoringWeights()
        self.assertAlmostEqual(w.w_orientation, 2.0, places=6)
        self.assertAlmostEqual(w.w_depth_ping_pong, 0.5, places=6)
        self.assertAlmostEqual(w.w_leaf, 1.0, places=6)

    def test_override(self):
        w = S.ScoringWeights(w_orientation=10.0)
        self.assertEqual(w.w_orientation, 10.0)


class TestDedup(unittest.TestCase):
    def test_dedup(self):
        out = S.deduplicate_orders([[(0, 1)], [(1, 0)], [(1, 2)]])
        self.assertEqual(len(out), 2)


if __name__ == "__main__":
    unittest.main()
