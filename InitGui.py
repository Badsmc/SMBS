# -*- coding: utf-8 -*-
"""InitGui.py — BendSeq V4.

Панели:
  Auto       — ui.panel_auto.PanelAuto     (реальная логика)
  Симуляция  — ui.panel_sim.PanelSim       (реальная логика)
  Дебаг      — ui.panel_debug.PanelDebug   (реальная логика)
  Инструмент — заглушка _TaskPanel         (в разработке)

Все определения — внутри _bendseq_init(), потому что FreeCAD
выполняет InitGui.py через exec() с раздельными globals/locals.
"""


def _bendseq_init():
    import os
    import FreeCAD as App
    import FreeCADGui as Gui

    try:
        from PySide2 import QtCore, QtWidgets
    except ImportError:
        from PySide6 import QtCore, QtWidgets

    # =================================================================
    # ПУТИ
    # =================================================================

    def _addon_root():
        try:
            base = App.getUserAppDataDir()
            candidate = os.path.join(base, "Mod", "SMBS")
            if os.path.isdir(candidate):
                return candidate
        except Exception:
            pass
        return os.getcwd()

    def _icon_dir():
        return os.path.join(_addon_root(), "Icons")

    def _icon_path(name):
        return os.path.join(_icon_dir(), name)

    # =================================================================
    # ИКОНКИ
    # =================================================================

    def _icon_svg_data():
        return {
            "BendSeq_Auto.svg": (
                '<?xml version="1.0" encoding="UTF-8"?>'
                '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">'
                '<rect x="6" y="10" width="52" height="44" fill="none" '
                'stroke="#4a4a4a" stroke-width="2.6" rx="2"/>'
                '<line x1="6" y1="22" x2="58" y2="22" stroke="#4a4a4a" stroke-width="2.6"/>'
                '<line x1="6" y1="34" x2="58" y2="34" stroke="#4a4a4a" stroke-width="1.8"/>'
                '<line x1="6" y1="46" x2="58" y2="46" stroke="#4a4a4a" stroke-width="2.6"/>'
                '<circle cx="14" cy="22" r="5.5" fill="#4a4a4a"/>'
                '<circle cx="14" cy="34" r="5.5" fill="#fff" stroke="#4a4a4a" stroke-width="2"/>'
                '<circle cx="14" cy="46" r="5.5" fill="#4a4a4a"/>'
                '<text x="14" y="25" font-size="8" font-family="sans-serif" '
                'font-weight="bold" fill="#fff" text-anchor="middle">1</text>'
                '<text x="14" y="37" font-size="8" font-family="sans-serif" '
                'font-weight="bold" fill="#4a4a4a" text-anchor="middle">2</text>'
                '<text x="14" y="49" font-size="8" font-family="sans-serif" '
                'font-weight="bold" fill="#fff" text-anchor="middle">3</text>'
                '</svg>'
            ),
            "BendSeq_Sim.svg": (
                '<?xml version="1.0" encoding="UTF-8"?>'
                '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">'
                '<path d="M8 42 L28 32 L56 42 L36 52 Z" fill="none" '
                'stroke="#4a4a4a" stroke-width="2.6" stroke-linejoin="round"/>'
                '<line x1="8" y1="42" x2="8" y2="20" stroke="#4a4a4a" stroke-width="2.6"/>'
                '<line x1="28" y1="32" x2="28" y2="10" stroke="#4a4a4a" stroke-width="2.6"/>'
                '<line x1="56" y1="42" x2="56" y2="20" stroke="#4a4a4a" stroke-width="2.6"/>'
                '<line x1="36" y1="52" x2="36" y2="30" stroke="#4a4a4a" stroke-width="2.6"/>'
                '<path d="M8 20 L28 10 L56 20 L36 30 Z" fill="none" '
                'stroke="#4a4a4a" stroke-width="2.6" stroke-linejoin="round"/>'
                '<polygon points="24,34 24,48 38,41" fill="#4a4a4a"/>'
                '</svg>'
            ),
            "BendSeq_Debug.svg": (
                '<?xml version="1.0" encoding="UTF-8"?>'
                '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">'
                '<rect x="10" y="6" width="44" height="52" fill="none" '
                'stroke="#4a4a4a" stroke-width="2.6" rx="2"/>'
                '<path d="M16 20 L20 24 L28 16" fill="none" stroke="#4a4a4a" '
                'stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"/>'
                '<path d="M16 36 L20 40 L28 32" fill="none" stroke="#4a4a4a" '
                'stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"/>'
                '<line x1="32" y1="20" x2="48" y2="20" stroke="#4a4a4a" stroke-width="2.2"/>'
                '<line x1="32" y1="36" x2="48" y2="36" stroke="#4a4a4a" stroke-width="2.2"/>'
                '<line x1="16" y1="50" x2="48" y2="50" stroke="#4a4a4a" stroke-width="2.2"/>'
                '</svg>'
            ),
            "BendSeq_Tool.svg": (
                '<?xml version="1.0" encoding="UTF-8"?>'
                '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">'
                '<polygon points="22,6 42,6 42,26 32,36 22,26" fill="none" '
                'stroke="#4a4a4a" stroke-width="2.6" stroke-linejoin="round"/>'
                '<line x1="22" y1="14" x2="42" y2="14" stroke="#4a4a4a" stroke-width="1.6"/>'
                '<path d="M8 42 L22 42 L32 54 L42 42 L56 42 L56 58 L8 58 Z" '
                'fill="none" stroke="#4a4a4a" stroke-width="2.6" stroke-linejoin="round"/>'
                '</svg>'
            ),
        }

    def _ensure_icons():
        try:
            d = _icon_dir()
            if not os.path.isdir(d):
                os.makedirs(d)
            for name, content in _icon_svg_data().items():
                path = os.path.join(d, name)
                if not os.path.isfile(path):
                    f = open(path, "w", encoding="utf-8")
                    f.write(content)
                    f.close()
                    App.Console.PrintMessage(
                        "[BendSeq] icon created: {}\n".format(name))
        except Exception as e:
            App.Console.PrintWarning(
                "[BendSeq] icon generation failed: {}\n".format(e))

    # =================================================================
    # TASK PANEL
    # =================================================================

    def _show_task_panel(panel):
        try:
            Gui.Control.showDialog(panel)
        except Exception as e:
            App.Console.PrintError(
                "[BendSeq] task panel show failed: {}\n".format(e))

    class _PlaceholderTaskPanel(QtWidgets.QWidget):
        """Заглушка для панели, которая ещё в разработке.
        Полноценные панели (Auto/Sim/Debug) реализованы в ui.panel_*.
        """

        def __init__(self, title, hint):
            QtWidgets.QWidget.__init__(self)
            self.setWindowTitle(title)

            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(8, 8, 8, 8)
            layout.setSpacing(6)

            lbl = QtWidgets.QLabel("<b>{}</b>".format(title))
            lbl.setStyleSheet("font-size: 13pt; padding: 4px;")
            layout.addWidget(lbl)

            h = QtWidgets.QLabel(hint)
            h.setWordWrap(True)
            h.setStyleSheet("color: #666; padding: 4px;")
            layout.addWidget(h)

            layout.addStretch(1)

            btn_row = QtWidgets.QHBoxLayout()
            btn_row.addStretch(1)
            btn_close = QtWidgets.QPushButton("Закрыть")
            btn_close.setMinimumWidth(90)
            btn_close.clicked.connect(self._on_close)
            btn_row.addWidget(btn_close)
            layout.addLayout(btn_row)

        def _on_close(self):
            try:
                Gui.Control.closeDialog()
            except Exception:
                pass

        @property
        def form(self):
            return self

        def getStandardButtons(self):
            try:
                return int(QtWidgets.QDialogButtonBox.Close)
            except Exception:
                return 0

        def accept(self):
            self._on_close()
            return True

        def reject(self):
            self._on_close()
            return True

    # =================================================================
    # ФАБРИКИ ПАНЕЛЕЙ
    # =================================================================

    def _panel_auto():
        from ui.panel_auto import PanelAuto
        return PanelAuto()

    def _panel_sim():
        from ui.panel_sim import PanelSim
        return PanelSim()

    def _panel_debug():
        from ui.panel_debug import PanelDebug
        return PanelDebug()

    def _panel_tool():
        return _PlaceholderTaskPanel(
            "Инструмент — выбор и редактирование",
            "Пуансон / матрица / станина / упор, генератор Cybelec. "
            "Панель в разработке.")

    # =================================================================
    # КОМАНДЫ
    # =================================================================

    class _Cmd:
        def __init__(self, name, menu, tip, icon_file, panel_fn):
            self.name = name
            self.menu_text = menu
            self.tooltip = tip
            self.icon_file = icon_file
            self.panel_fn = panel_fn

        def GetResources(self):
            return {
                "MenuText": self.menu_text,
                "ToolTip": self.tooltip,
                "Pixmap": _icon_path(self.icon_file),
            }

        def IsActive(self):
            return True

        def Activated(self):
            try:
                panel = self.panel_fn()
            except Exception as e:
                import traceback
                App.Console.PrintError(
                    "[BendSeq] panel {} failed: {}\n{}\n".format(
                        self.name, e, traceback.format_exc()))
                return
            _show_task_panel(panel)

    class CmdAuto(_Cmd):
        def __init__(self):
            _Cmd.__init__(self, "BendSeq_Auto", "BendSeq: Auto",
                          "Рассчитать последовательность гибки",
                          "BendSeq_Auto.svg", _panel_auto)

    class CmdSim(_Cmd):
        def __init__(self):
            _Cmd.__init__(self, "BendSeq_Sim", "BendSeq: Симуляция",
                          "3D-симуляция последовательности",
                          "BendSeq_Sim.svg", _panel_sim)

    class CmdDebug(_Cmd):
        def __init__(self):
            _Cmd.__init__(self, "BendSeq_Debug", "BendSeq: Дебаг",
                          "Логи и отчёты",
                          "BendSeq_Debug.svg", _panel_debug)

    class CmdTool(_Cmd):
        def __init__(self):
            _Cmd.__init__(self, "BendSeq_Tool", "BendSeq: Инструмент",
                          "Выбор и настройка оснастки",
                          "BendSeq_Tool.svg", _panel_tool)

    # =================================================================
    # WORKBENCH
    # =================================================================

    class BendSeqWorkbench(Gui.Workbench):
        MenuText = "BendSeq"
        ToolTip = "Планирование последовательности гибки"
        Icon = ""

        def __init__(self):
            self._cmds = [
                "BendSeq_Auto",
                "BendSeq_Sim",
                "BendSeq_Debug",
                "BendSeq_Tool",
            ]

        def GetClassName(self):
            return "Gui::PythonWorkbench"

        def Initialize(self):
            App.Console.PrintMessage("[BendSeq] Initialize workbench\n")
            Gui.addCommand("BendSeq_Auto", CmdAuto())
            Gui.addCommand("BendSeq_Sim", CmdSim())
            Gui.addCommand("BendSeq_Debug", CmdDebug())
            Gui.addCommand("BendSeq_Tool", CmdTool())
            self.appendToolbar("BendSeq", self._cmds)
            self.appendMenu("BendSeq", self._cmds)

        def Activated(self):
            pass

        def Deactivated(self):
            pass

        def ContextMenu(self, recipient):
            self.appendContextMenu("BendSeq", self._cmds)

    # =================================================================
    # ЗАПУСК
    # =================================================================

    App.Console.PrintMessage(
        "[BendSeq] addon root = {}\n".format(_addon_root()))

    _ensure_icons()

    # Импортируем ui.context до addWorkbench, чтобы синглтон
    # создался один раз и был доступен всем панелям.
    try:
        from ui import context as _ctx_mod
        App.Console.PrintMessage(
            "[BendSeq] context loaded: {}\n".format(
                getattr(_ctx_mod, "__version__", "?")))
    except Exception as e:
        App.Console.PrintError(
            "[BendSeq] context import failed: {}\n".format(e))

    Gui.addWorkbench(BendSeqWorkbench())

    App.Console.PrintMessage("[BendSeq] InitGui V4 done\n")


# Одна точка входа.
_bendseq_init()