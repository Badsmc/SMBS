# -*- coding: utf-8 -*-
"""Spec → ToolAdapter (совместимый с существующим интерфейсом)."""

from __future__ import annotations

try:
    import FreeCAD as App
except ImportError:
    App = None

from .tool_library import ToolLibrary


class _ShapeProxy:
    """Обёртка, изображающая минимальный FreeCAD-объект
    (Name, Label, Shape) для ToolAdapter."""

    def __init__(self, name: str, label: str, shape):
        self.Name = name
        self.Label = label
        self.Shape = shape


def make_punch_adapter(spec, library: ToolLibrary,
                       length: float = 500.0, logger=None):
    """PunchSpec → ToolAdapter(role='punch')."""
    try:
        from plugins.tooling.base_tool import ToolAdapter
    except ImportError:
        return None
    shape = library.get_punch_shape(spec.id, length=length)
    if shape is None:
        if logger:
            logger("[ToolFactory] punch shape не получен: {}".format(spec.id))
        return None
    proxy = _ShapeProxy("PUNCH_" + spec.id, spec.name, shape)
    return ToolAdapter(proxy, role="punch")


def make_die_adapter(spec, library: ToolLibrary,
                     length: float = 500.0, logger=None):
    try:
        from plugins.tooling.base_tool import ToolAdapter
    except ImportError:
        return None
    shape = library.get_die_shape(spec.id, length=length)
    if shape is None:
        if logger:
            logger("[ToolFactory] die shape не получен: {}".format(spec.id))
        return None
    proxy = _ShapeProxy("DIE_" + spec.id, spec.name, shape)
    return ToolAdapter(proxy, role="die")


def make_adapters_from_set(set_spec, library: ToolLibrary,
                            length: float = 500.0, logger=None):
    """ToolSetSpec → (punch_adapter, die_adapter)."""
    punch = library.get_punch(set_spec.punch_id)
    die = library.get_die(set_spec.die_id)
    if punch is None or die is None:
        if logger:
            logger("[ToolFactory] не найден punch/die для сета {}"
                    .format(set_spec.id))
        return None, None
    return (make_punch_adapter(punch, library, length, logger),
            make_die_adapter(die, library, length, logger))