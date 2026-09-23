# -*- coding: utf-8 -*-
"""Глобальная библиотека инструментов."""

from __future__ import annotations
from pathlib import Path
from typing import Optional
from .tool_library import ToolLibrary


_library: Optional[ToolLibrary] = None


def default_library_path() -> Path:
    return Path.home() / "BendSeq" / "ToolLibrary"


def get_library() -> ToolLibrary:
    global _library
    if _library is None:
        _library = ToolLibrary(default_library_path())
    return _library


def set_library(lib: ToolLibrary) -> None:
    global _library
    _library = lib