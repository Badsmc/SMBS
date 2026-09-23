# -*- coding: utf-8 -*-
"""
plugins/tooling/press_adapter.py
PressGeometryAdapter — список форм станины пресса в мировых координатах.

Используется в SequenceValidator как press_shapes.

Версия: BENDBEQ_PRESS_ADAPTER_V1
"""

from __future__ import annotations

from core.geometry_utils import shape_in_world


class PressGeometryAdapter:
    """Обёртка над объектами станины.

    Каждый объект может содержать несколько Solid'ов; возвращаются
    шейпы верхнего уровня, достаточно для CollisionEngine.pair().
    """

    def __init__(self, press_objects=None):
        self.press_objects = press_objects or []

    def get_virtual_shapes(self) -> list:
        """Список Part.Shape станины в мировых координатах."""
        shapes = []
        for obj in self.press_objects:
            shape = shape_in_world(obj)
            if shape is not None:
                shapes.append(shape)
        return shapes

    def __repr__(self):
        return "PressGeometryAdapter(objects={})".format(
            len(self.press_objects))