# -*- coding: utf-8 -*-
"""
ui/context.py — BendSeq singleton state V2.

Добавлено относительно V1:
  * общий лог-буфер, чтобы PanelDebug видел все сообщения
    из PanelAuto и PanelSim;
  * метод log(msg) — единая точка для логирования;
  * notify-события "log", "log_clear", "results", "adapter".
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional


class BendSeqContext:
    _instance: Optional["BendSeqContext"] = None

    @classmethod
    def get(cls) -> "BendSeqContext":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    @classmethod
    def clear_singleton(cls) -> None:
        cls._instance = None

    def __init__(self):
        self._listeners: List[Callable[[str], None]] = []
        self._log_buffer: List[str] = []
        self.reset()

    def reset(self) -> None:
        # вход
        self.adapter = None
        self.unfold_obj = None
        self.flat_shape = None
        self.all_bends: list = []
        self.unfold_face_name: Optional[str] = None
        # графы
        self.pg3 = None
        self.pg2 = None
        self.bend_info: Dict = {}
        self.thickness: Optional[float] = None
        self.bends_sketch = None
        # результаты
        self.state = None
        self.step_results: list = []
        self.last_auto_result: Optional[dict] = None
        # инструменты
        self.tool_adapters: Dict[str, Any] = {
            "punch": None, "die": None, "press": None, "gauge": None,
        }
        # симуляция
        self.sim_controller = None
        self._notify("reset")

    # --- listeners ---

    def add_listener(self, fn: Callable[[str], None]) -> None:
        if fn not in self._listeners:
            self._listeners.append(fn)

    def remove_listener(self, fn: Callable[[str], None]) -> None:
        try:
            self._listeners.remove(fn)
        except ValueError:
            pass

    def _notify(self, event: str) -> None:
        for fn in list(self._listeners):
            try:
                fn(event)
            except Exception:
                pass

    # --- лог ---

    def log(self, msg: str) -> None:
        text = str(msg).rstrip("\n")
        self._log_buffer.append(text)
        try:
            import FreeCAD as App
            App.Console.PrintMessage(text + "\n")
        except Exception:
            pass
        self._notify("log")

    def log_lines(self) -> List[str]:
        return list(self._log_buffer)

    def clear_log(self) -> None:
        self._log_buffer = []
        self._notify("log_clear")

    # --- удобные сеттеры ---

    def set_adapter(self, adapter) -> None:
        self.adapter = adapter
        try:
            self.flat_shape = adapter.get_flat_shape()
            self.all_bends = adapter.get_bend_specs()
        except Exception:
            self.flat_shape = None
            self.all_bends = []
        self._notify("adapter")

    def set_step_results(self, results) -> None:
        self.step_results = list(results or [])
        self._notify("results")

    def __repr__(self):
        return "BendSeqContext(bends={}, results={})".format(
            len(self.all_bends), len(self.step_results))


__version__ = "BENDBEQ_CONTEXT_V2"
try:
    import FreeCAD as _App
    _App.Console.PrintMessage(
        "[context] Загружена версия: {}\n".format(__version__))
except Exception:
    pass