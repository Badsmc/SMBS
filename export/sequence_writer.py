# -*- coding: utf-8 -*-
"""
export/sequence_writer.py
Экспорт последовательности гибки в CSV.

Публичный API:
    write_csv(report, path) -> path
    write_all(report, dir_path, base_name) -> path

Версия: BENDBEQ_SEQUENCE_WRITER_V1
"""

from __future__ import annotations

import csv
import os
from typing import Any, Dict, List


CSV_HEADER = [
    "step", "parent_face", "child_face",
    "feature_ref", "edge_ref",
    "angle_deg", "radius_mm", "direction",
    "axis_pt_x", "axis_pt_y", "axis_pt_z",
    "axis_dir_x", "axis_dir_y", "axis_dir_z",
    "note",
]


def _fmt(v, nd: int = 4):
    if v is None:
        return ""
    if isinstance(v, float):
        return "{:.{}f}".format(v, nd)
    return str(v)


def _step_to_row(s: Dict[str, Any]) -> List[str]:
    axis = s.get("bend_axis_3d") or {}
    pt = axis.get("point") or [None, None, None]
    dr = axis.get("direction") or [None, None, None]
    return [
        s.get("step", ""),
        s.get("parent_face", ""),
        s.get("child_face", ""),
        s.get("feature_ref", ""),
        s.get("edge_ref", ""),
        _fmt(s.get("angle_deg"), 2),
        _fmt(s.get("radius_mm"), 3),
        s.get("direction", ""),
        _fmt(pt[0], 4), _fmt(pt[1], 4), _fmt(pt[2], 4),
        _fmt(dr[0], 4), _fmt(dr[1], 4), _fmt(dr[2], 4),
        s.get("note", ""),
    ]


def write_csv(report: Dict[str, Any], path: str) -> str:
    with open(path, "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        for s in report.get("sequence", []) or []:
            w.writerow(_step_to_row(s))
    return path


def write_all(report: Dict[str, Any],
               dir_path: str,
               base_name: str) -> str:
    if not os.path.isdir(dir_path):
        os.makedirs(dir_path, exist_ok=True)
    return write_csv(report, os.path.join(dir_path, base_name + ".csv"))
