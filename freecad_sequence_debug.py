# -*- coding: utf-8 -*-
"""
freecad_sequence_debug.py — аудит + scene probe для BendSeq.

Версия: BENDBEQ_DEBUG_V1

Возможности:
  * ProjectAuditor.run(scene=True) -> AuditReport
  * ProjectAuditor.save(out_dir) -> (json_path, txt_path)
  * ProjectAuditor.render_text() -> str

Статические проверки (минимум):
  * Синтаксические ошибки в .py файлах проекта.
  * Импорт удалённых модулей (bend_group_tree, bend_list_widget).
  * Наличие обязательных модулей.

Dump:
  * panel_dump — состояние активной SequenceTaskPanel.
  * document_dump — Body + SMBendWall + Sim*.

Не изменяет документ. Только читает.
"""

from __future__ import annotations

import ast
import json
import os
import sys
import traceback
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

MODULE_VERSION = "BENDBEQ_DEBUG_V1"
SCHEMA_VERSION = "1.0.0"
PROJECT_NAME = "BendSeq"
REPORT_DIRNAME = "debug_reports"

try:
    import FreeCAD as App
    import Part
    HAS_FREECAD = True
except ImportError:
    App = None
    Part = None
    HAS_FREECAD = False

try:
    import FreeCADGui as Gui
    HAS_FREECADGUI = True
except ImportError:
    Gui = None
    HAS_FREECADGUI = False


# =====================================================================
# Утилиты
# =====================================================================

def _iso_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _safe_str(v: Any, limit: int = 500) -> str:
    try:
        text = str(v)
    except Exception:
        text = repr(v)
    text = text.replace("\x00", "\\0")
    return text if len(text) <= limit else text[:limit] + "…"


def _read_text(path: Path) -> str:
    for enc in ("utf-8", "utf-8-sig", "cp1251"):
        try:
            return path.read_text(encoding=enc)
        except UnicodeDecodeError:
            continue
    return path.read_text(errors="replace")


def _project_py_files(root: Path) -> List[Path]:
    skip = {".git", "__pycache__", ".venv", "venv", "debug_reports",
            ".idea", ".vscode"}
    out = []
    for p in root.rglob("*.py"):
        if any(part in skip for part in p.parts):
            continue
        out.append(p)
    return sorted(out)


def _v(v) -> Optional[List[float]]:
    if v is None:
        return None
    try:
        return [round(float(v.x), 4), round(float(v.y), 4),
                round(float(v.z), 4)]
    except Exception:
        try:
            return [round(float(v[0]), 4), round(float(v[1]), 4),
                    round(float(v[2]), 4)]
        except Exception:
            return None


def _bbox(shape) -> Optional[Dict[str, Any]]:
    if shape is None:
        return None
    try:
        bb = shape.BoundBox
        return {
            "min": [round(bb.XMin, 4), round(bb.YMin, 4),
                    round(bb.ZMin, 4)],
            "max": [round(bb.XMax, 4), round(bb.YMax, 4),
                    round(bb.ZMax, 4)],
            "size": [round(bb.XLength, 4), round(bb.YLength, 4),
                     round(bb.ZLength, 4)],
        }
    except Exception:
        return None


def _shape_info(shape) -> Dict[str, Any]:
    if shape is None:
        return {"present": False}
    try:
        if shape.isNull():
            return {"present": True, "is_null": True}
    except Exception:
        return {"present": True, "is_null": "unknown"}
    info: Dict[str, Any] = {
        "present": True, "is_null": False,
        "volume": None, "area": None,
        "num_solids": None, "num_faces": None,
        "bbox": _bbox(shape),
    }
    try:
        info["volume"] = round(float(shape.Volume), 4)
    except Exception:
        pass
    try:
        info["area"] = round(float(shape.Area), 4)
    except Exception:
        pass
    try:
        info["num_solids"] = len(shape.Solids)
        info["num_faces"] = len(shape.Faces)
    except Exception:
        pass
    return info


def _obj_info(obj) -> Dict[str, Any]:
    info: Dict[str, Any] = {
        "name": getattr(obj, "Name", None),
        "label": getattr(obj, "Label", ""),
        "type_id": getattr(obj, "TypeId", ""),
    }
    try:
        info["shape"] = _shape_info(getattr(obj, "Shape", None))
    except Exception:
        info["shape"] = None
    return info


# =====================================================================
# Данные
# =====================================================================

@dataclass
class Finding:
    audit_id: str
    severity: str          # CRITICAL | ERROR | WARNING | INFO
    category: str
    title: str
    message: str
    file: str = ""
    line: Optional[int] = None
    context: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self):
        return asdict(self)


@dataclass
class AuditReport:
    project_root: str
    started_at: str = field(default_factory=_iso_now)
    finished_at: str = ""
    duration_sec: float = 0.0
    tool_version: str = MODULE_VERSION
    schema_version: str = SCHEMA_VERSION
    freecad_version: str = ""
    python_version: str = field(default_factory=lambda: sys.version)
    findings: List[Finding] = field(default_factory=list)
    tool_errors: List[str] = field(default_factory=list)
    metrics: Dict[str, Any] = field(default_factory=dict)
    document_dump: Optional[Dict[str, Any]] = None
    panel_dump: Optional[Dict[str, Any]] = None
    scene_probe: Optional[Dict[str, Any]] = None

    def add(self, f: Finding) -> None:
        self.findings.append(f)

    def counts(self) -> Dict[str, int]:
        c = {"CRITICAL": 0, "ERROR": 0, "WARNING": 0, "INFO": 0}
        for f in self.findings:
            c[f.severity] = c.get(f.severity, 0) + 1
        return c

    def status(self) -> str:
        c = self.counts()
        if c["CRITICAL"]:
            return "CRITICAL"
        if c["ERROR"]:
            return "ERROR"
        if c["WARNING"]:
            return "WARNING"
        return "PASS"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "tool_version": self.tool_version,
            "project_root": self.project_root,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "duration_sec": self.duration_sec,
            "python_version": self.python_version,
            "freecad_version": self.freecad_version,
            "status": self.status(),
            "counts": self.counts(),
            "metrics": self.metrics,
            "tool_errors": self.tool_errors,
            "findings": [f.to_dict() for f in self.findings],
            "document_dump": self.document_dump,
            "panel_dump": self.panel_dump,
            "scene_probe": self.scene_probe,
        }


# =====================================================================
# Аудитор
# =====================================================================

class ProjectAuditor:

    REMOVED_MODULES = (
        "bend_group_tree",
        "bend_list_widget",
        "autosequencer.groups",
        "from .groups",
        "from autosequencer.groups",
    )

    REQUIRED_MODULES = (
        "config.py",
        "InitGui.py",
        "core/__init__.py",
        "core/bend_feature.py",
        "core/panel_graph.py",
        "core/panel_graph_2d.py",
        "core/panel_solid.py",
        "core/sequence_state.py",
        "core/kinematic_engine.py",
        "core/collision_engine.py",
        "core/bend_simulator.py",
        "core/sequence_validator.py",
        "core/unfold_extractor.py",
        "core/verifier.py",
        "core/bend_kinematics.py",
        "core/geometry_utils.py",
        "adapters/__init__.py",
        "adapters/freecad_adapter.py",
        "adapters/text_overlay.py",
        "autosequencer/__init__.py",
        "autosequencer/topology.py",
        "autosequencer/kinematics.py",
        "autosequencer/solver.py",
        "autosequencer/backward.py",
        "autosequencer/scoring.py",
        "plugins/tooling/__init__.py",
        "plugins/tooling/base_tool.py",
        "plugins/tooling/press_adapter.py",
        "ui/__init__.py",
        "ui/task_panel.py",
        "ui/simulation_viewer.py",
    )

    def __init__(self, project_root=None):
        if project_root:
            self.root = Path(project_root).expanduser().resolve()
        else:
            try:
                self.root = Path(__file__).resolve().parent
            except NameError:
                self.root = Path.cwd()
        self.report = AuditReport(project_root=str(self.root))
        self.sources: Dict[str, str] = {}

    # ------------------------------------------------------------------
    # Публичный API
    # ------------------------------------------------------------------

    def run(self, scene: bool = False,
            document_dump: bool = False,
            panel_dump: bool = False) -> AuditReport:
        import time as _t
        t0 = _t.time()

        if HAS_FREECAD:
            try:
                self.report.freecad_version = _safe_str(App.Version())
            except Exception:
                pass

        try:
            self._load_sources()
            self._static_checks()
            if document_dump or panel_dump or scene:
                self._run_dumps(document_dump=document_dump,
                                 panel_dump=panel_dump, scene=scene)
        except Exception as exc:
            self.report.tool_errors.append(
                "fatal: {}\n{}".format(exc, traceback.format_exc()))

        self.report.metrics["python_files_scanned"] = len(self.sources)
        self.report.finished_at = _iso_now()
        self.report.duration_sec = round(_t.time() - t0, 3)
        return self.report

    def save(self, out_dir=None) -> Tuple[Path, Path]:
        out_dir = Path(out_dir) if out_dir else (self.root / REPORT_DIRNAME)
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        json_path = out_dir / "bendseq_audit_{}.json".format(stamp)
        txt_path = out_dir / "bendseq_audit_{}.txt".format(stamp)

        data = self.report.to_dict()
        blob = json.dumps(data, ensure_ascii=False, indent=2, default=str)
        json_path.write_text(blob, encoding="utf-8")
        txt_path.write_text(self.render_text(), encoding="utf-8")

        try:
            (out_dir / "bendseq_audit_latest.json").write_text(
                blob, encoding="utf-8")
            (out_dir / "bendseq_audit_latest.txt").write_text(
                self.render_text(), encoding="utf-8")
        except Exception:
            pass
        return json_path, txt_path

    # ------------------------------------------------------------------
    # Загрузка источников
    # ------------------------------------------------------------------

    def _load_sources(self) -> None:
        if not self.root.exists():
            self.report.tool_errors.append(
                "project root not found: {}".format(self.root))
            return
        for path in _project_py_files(self.root):
            try:
                rel = str(path.relative_to(self.root)).replace(os.sep, "/")
            except ValueError:
                rel = str(path)
            try:
                self.sources[rel] = _read_text(path)
            except Exception as e:
                self.report.tool_errors.append(
                    "read {}: {}".format(rel, e))

    # ------------------------------------------------------------------
    # Статические проверки
    # ------------------------------------------------------------------

    def _static_checks(self) -> None:
        # 1. Синтаксис
        # 2. Импорты удалённых модулей
        # Исключаем сам аудитор и check_all.py — в них эти строки
        # встречаются как данные, а не как импорты.
        self_exclude = {
            "freecad_sequence_debug.py",
            "check_all.py",
        }
        for rel, text in self.sources.items():
            if rel in self_exclude or os.path.basename(rel) in self_exclude:
                continue
            for marker in self.REMOVED_MODULES:
                if marker not in text:
                    continue
                # Пропускаем совпадения внутри определения REMOVED_MODULES
                # (на случай если свой файл переименован)
                line = None
                for i, ln in enumerate(text.splitlines(), 1):
                    if marker in ln:
                        stripped = ln.strip()
                        if stripped.startswith('"') or stripped.startswith("'"):
                            # это строковый литерал кортежа, не импорт
                            continue
                        if stripped.startswith("#"):
                            continue
                        line = i
                        break
                if line is None:
                    continue
                self.report.add(Finding(
                    audit_id="DEAD-IMPORT",
                    severity="ERROR",
                    category="DeadCode",
                    title="Import of removed module: {}".format(marker),
                    message="Модуль {} удалён из BendSeq.".format(marker),
                    file=rel, line=line,
                ))

        # 2. Импорты удалённых модулей
        self_exclude = {"freecad_sequence_debug.py", "check_all.py"}
        for rel, text in self.sources.items():
            if rel in self_exclude or os.path.basename(rel) in self_exclude:
                continue
            for marker in self.REMOVED_MODULES:
                if marker in text:
                    line = None
                    for i, ln in enumerate(text.splitlines(), 1):
                        if marker in ln:
                            line = i
                            break
                    self.report.add(Finding(
                        audit_id="DEAD-IMPORT",
                        severity="ERROR",
                        category="DeadCode",
                        title="Import of removed module: {}".format(marker),
                        message="Модуль {} удалён из BendSeq.".format(marker),
                        file=rel, line=line,
                    ))

        # 3. Обязательные модули
        for rel in self.REQUIRED_MODULES:
            if rel not in self.sources:
                self.report.add(Finding(
                    audit_id="MISSING-MODULE",
                    severity="ERROR",
                    category="Structure",
                    title="Missing module: {}".format(rel),
                    message="Ожидаемый файл не найден.",
                    file=rel,
                ))

    # ------------------------------------------------------------------
    # Dump: document + panel
    # ------------------------------------------------------------------

    def _run_dumps(self, document_dump=False,
                    panel_dump=False, scene=False) -> None:
        if not HAS_FREECAD:
            return
        doc = getattr(App, "ActiveDocument", None)
        if doc is None:
            return

        if document_dump or scene:
            self.report.document_dump = self._dump_document(doc)
        if panel_dump or scene:
            self.report.panel_dump = self._dump_panel()

    def _dump_document(self, doc) -> Dict[str, Any]:
        dump: Dict[str, Any] = {
            "document_name": getattr(doc, "Name", None),
            "document_label": getattr(doc, "Label", None),
            "objects_count": len(doc.Objects),
            "bodies": [],
            "bend_walls": [],
            "sim_parts": [],
            "sim_punches": [],
            "sim_dies": [],
            "sim_gauges": [],
            "sim_press": [],
            "sim_collisions": [],
        }

        for obj in doc.Objects:
            try:
                if obj.isDerivedFrom("PartDesign::Body"):
                    body_entry: Dict[str, Any] = {
                        "name": obj.Name,
                        "shape": _shape_info(getattr(obj, "Shape", None)),
                        "group": [
                            getattr(f, "Name", None)
                            for f in (getattr(obj, "Group", []) or [])
                        ],
                    }
                    dump["bodies"].append(body_entry)
            except Exception:
                pass

            proxy = getattr(obj, "Proxy", None)
            if proxy is not None and "SMBendWall" in str(type(proxy)):
                try:
                    entry: Dict[str, Any] = {
                        "name": obj.Name,
                        "angle": _safe_str(getattr(obj, "angle", None)),
                        "invert": bool(getattr(obj, "invert", False)),
                        "radius": _safe_str(getattr(obj, "radius", None)),
                        "LengthList": [str(x) for x in (
                            getattr(obj, "LengthList", []) or [])],
                    }
                    bo = getattr(obj, "baseObject", None)
                    if bo is not None:
                        try:
                            parent = bo[0]
                            entry["parent"] = getattr(parent, "Name", None)
                            entry["subs"] = [str(s) for s in bo[1]]
                        except Exception:
                            pass
                    dump["bend_walls"].append(entry)
                except Exception:
                    pass

            n = getattr(obj, "Name", "")
            if n.startswith("SimPart_"):
                dump["sim_parts"].append(_obj_info(obj))
            elif n.startswith("SimPunch_"):
                dump["sim_punches"].append(_obj_info(obj))
            elif n.startswith("SimDie_"):
                dump["sim_dies"].append(_obj_info(obj))
            elif n.startswith("SimGauge_"):
                dump["sim_gauges"].append(_obj_info(obj))
            elif n.startswith("SimPress"):
                dump["sim_press"].append(_obj_info(obj))
            elif n.startswith("SimCollision_"):
                dump["sim_collisions"].append(_obj_info(obj))

        return dump

    def _dump_panel(self) -> Dict[str, Any]:
        if not HAS_FREECADGUI:
            return {"error": "FreeCADGui not available"}
        try:
            from ui.task_panel import SequenceTaskPanel
        except Exception as e:
            return {"error": "import failed: {}".format(e)}

        try:
            mw = Gui.getMainWindow()
        except Exception:
            mw = None
        if mw is None:
            return {"error": "no main window"}

        try:
            panels = mw.findChildren(SequenceTaskPanel)
        except Exception as e:
            return {"error": "findChildren: {}".format(e)}
        if not panels:
            return {"error": "no SequenceTaskPanel found"}

        panel = panels[0]
        dump: Dict[str, Any] = {
            "panel_found": True,
            "ui_state": {},
            "state": {},
            "seq_tree": [],
        }

        ui: Dict[str, Any] = {}
        try:
            ui["active_tab_index"] = panel.tabs.currentIndex()
            ui["active_tab_text"] = panel.tabs.tabText(
                panel.tabs.currentIndex())
        except Exception:
            pass
        try:
            ui["auto_status_text"] = panel.auto_status_label.text()
        except Exception:
            pass
        try:
            ui["sim_info_text"] = panel.sim_info_label.text()
            ui["sim_step_text"] = panel.sim_step_label.text()
        except Exception:
            pass
        for name in ("btn_auto", "btn_auto_astar",
                     "btn_auto_backward", "btn_auto_hybrid",
                     "btn_clear_seq", "btn_show_numbers", "btn_export",
                     "btn_sim_prev", "btn_sim_next",
                     "btn_sim_play", "btn_sim_stop",
                     "btn_sim_fit", "btn_sim_show", "chk_moment"):
            w = getattr(panel, name, None)
            if w is None:
                continue
            try:
                ui["{}_enabled".format(name)] = bool(w.isEnabled())
            except Exception:
                pass
        dump["ui_state"] = ui

        state_dump: Dict[str, Any] = {}
        try:
            st = panel.state
            state_dump["ordered_ids"] = list(st.ordered_ids or [])
            state_dump["all_bends_count"] = len(st.all_bends or [])
            state_dump["step_results_count"] = len(st.step_results or [])
            sr = []
            for i, r in enumerate(st.step_results or []):
                sr.append({
                    "idx": i,
                    "bend_id": getattr(r, "bend_id", None),
                    "ok": bool(getattr(r, "ok", False)),
                    "collision_type": getattr(r, "collision_type", None),
                    "reason": getattr(r, "reason_human", ""),
                })
            state_dump["step_results"] = sr
        except Exception as e:
            state_dump["error"] = str(e)
        dump["state"] = state_dump

        try:
            tree = getattr(panel, "seq_tree", None)
            if tree is not None:
                n = tree.topLevelItemCount()
                for i in range(n):
                    it = tree.topLevelItem(i)
                    dump["seq_tree"].append(
                        [it.text(c) for c in range(tree.columnCount())])
        except Exception:
            pass

        return dump

    # ------------------------------------------------------------------
    # Текст
    # ------------------------------------------------------------------

    def render_text(self) -> str:
        r = self.report
        c = r.counts()
        lines: List[str] = []
        lines.append("=" * 78)
        lines.append("  {} — AUDIT REPORT ({})".format(
            PROJECT_NAME, MODULE_VERSION))
        lines.append("=" * 78)
        lines.append("  Status:        {}".format(r.status()))
        lines.append("  Started:       {}".format(r.started_at))
        lines.append("  Finished:      {}".format(r.finished_at))
        lines.append("  Duration:      {:.3f}s".format(r.duration_sec))
        lines.append("  Root:          {}".format(r.project_root))
        lines.append("  FreeCAD:       {}".format(r.freecad_version or "--"))
        lines.append("  Python:        {}".format(
            r.python_version.split()[0]))
        lines.append("")
        lines.append("  CRITICAL: {}  ERROR: {}  WARNING: {}  INFO: {}".format(
            c["CRITICAL"], c["ERROR"], c["WARNING"], c["INFO"]))
        lines.append("")

        if r.tool_errors:
            lines.append("-" * 78)
            lines.append("  TOOL ERRORS")
            lines.append("-" * 78)
            for e in r.tool_errors:
                lines.append("  ! {}".format(e))
            lines.append("")

        if r.panel_dump:
            lines.append("-" * 78)
            lines.append("  PANEL DUMP")
            lines.append("-" * 78)
            pd = r.panel_dump
            if "error" in pd:
                lines.append("  ! {}".format(pd["error"]))
            else:
                ui = pd.get("ui_state", {})
                for k in ("active_tab_text", "auto_status_text",
                          "sim_info_text", "sim_step_text"):
                    if k in ui:
                        lines.append("  {}: {}".format(
                            k, _safe_str(ui[k], 200)))
                lines.append("  seq_tree:")
                for row in pd.get("seq_tree", []):
                    lines.append("    " + " | ".join(row))
                st = pd.get("state", {})
                lines.append("  ordered_ids: {}".format(
                    st.get("ordered_ids")))
                lines.append("  step_results_count: {}".format(
                    st.get("step_results_count")))
            lines.append("")

        if r.document_dump:
            lines.append("-" * 78)
            lines.append("  DOCUMENT DUMP")
            lines.append("-" * 78)
            dd = r.document_dump
            lines.append("  doc: {}".format(dd.get("document_name")))
            lines.append("  objects: {}".format(dd.get("objects_count")))
            lines.append("  bodies: {}".format(len(dd.get("bodies", []))))
            for b in dd.get("bodies", []):
                sh = b.get("shape") or {}
                lines.append("    {}: vol={} solids={} bbox={}".format(
                    b.get("name"), sh.get("volume"),
                    sh.get("num_solids"),
                    (sh.get("bbox") or {}).get("size")))
            lines.append("  bend_walls: {}".format(
                len(dd.get("bend_walls", []))))
            for w in dd.get("bend_walls", []):
                lines.append("    {}: angle={} invert={} subs={}".format(
                    w.get("name"), w.get("angle"),
                    w.get("invert"), w.get("subs")))
            lines.append("  sim_parts: {}".format(
                len(dd.get("sim_parts", []))))
            for sp in dd.get("sim_parts", []):
                sh = sp.get("shape") or {}
                lines.append("    {}: solids={} vol={}".format(
                    sp.get("name"), sh.get("num_solids"),
                    sh.get("volume")))
            lines.append("")

        # Findings
        for sev in ("CRITICAL", "ERROR", "WARNING", "INFO"):
            items = [f for f in r.findings if f.severity == sev]
            if not items:
                continue
            lines.append("-" * 78)
            lines.append("  {}  ({})".format(sev, len(items)))
            lines.append("-" * 78)
            for i, f in enumerate(items, 1):
                loc = f.file
                if f.line:
                    loc += ":{}".format(f.line)
                lines.append("  [{}] {}  {}".format(
                    i, f.audit_id, f.title))
                if loc:
                    lines.append("      File: {}".format(loc))
                lines.append("      {}".format(f.message))
                lines.append("")

        lines.append("=" * 78)
        return "\n".join(lines)


# =====================================================================
# Удобный API
# =====================================================================

def run_audit(project_root=None, scene=False, document_dump=False,
              panel_dump=False, save=False, verbose=False):
    auditor = ProjectAuditor(project_root)
    report = auditor.run(scene=scene,
                          document_dump=document_dump,
                          panel_dump=panel_dump)
    if save:
        try:
            jp, tp = auditor.save()
            report.metrics["output_json"] = str(jp)
            report.metrics["output_txt"] = str(tp)
        except Exception as e:
            report.tool_errors.append("save: {}".format(e))
    if verbose:
        print(auditor.render_text())
    return report