# -*- coding: utf-8 -*-
"""Библиотека инструментов: JSON-индекс + опциональная геометрия."""
from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Dict, List, Optional

try:
    import FreeCAD as App
    import Part
    _FC_OK = True
except ImportError:
    _FC_OK = False

from .tool_definition import PunchSpec, DieSpec, ToolSetSpec


LIBRARY_VERSION = 1


class ToolLibrary:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "punches").mkdir(exist_ok=True)
        (self.root / "dies").mkdir(exist_ok=True)

        self.punches: Dict[str, PunchSpec] = {}
        self.dies:    Dict[str, DieSpec]    = {}
        self.sets:    Dict[str, ToolSetSpec] = {}

        self._load()

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def _json_path(self) -> Path:
        return self.root / "library.json"

    def _load(self) -> None:
        p = self._json_path()
        if not p.exists():
            return
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            return
        for d in data.get("punches", []):
            try:
                s = PunchSpec.from_dict(d)
                self.punches[s.id] = s
            except Exception:
                pass
        for d in data.get("dies", []):
            try:
                s = DieSpec.from_dict(d)
                self.dies[s.id] = s
            except Exception:
                pass
        for d in data.get("sets", []):
            try:
                s = ToolSetSpec.from_dict(d)
                self.sets[s.id] = s
            except Exception:
                pass

    def save(self) -> None:
        data = {
            "version": LIBRARY_VERSION,
            "punches": [s.to_dict() for s in self.punches.values()],
            "dies":    [s.to_dict() for s in self.dies.values()],
            "sets":    [s.to_dict() for s in self.sets.values()],
        }
        self._json_path().write_text(
            json.dumps(data, indent=2, ensure_ascii=False),
            encoding="utf-8")

    # ------------------------------------------------------------------
    # CRUD — Punches
    # ------------------------------------------------------------------

    def add_punch(self, spec: PunchSpec) -> None:
        self.punches[spec.id] = spec
        self.save()

    def remove_punch(self, punch_id: str) -> None:
        self.punches.pop(punch_id, None)
        self.save()

    def list_punches(self) -> List[PunchSpec]:
        return sorted(self.punches.values(), key=lambda s: s.name)

    def get_punch(self, punch_id: str) -> Optional[PunchSpec]:
        return self.punches.get(punch_id)

    # ------------------------------------------------------------------
    # CRUD — Dies
    # ------------------------------------------------------------------

    def add_die(self, spec: DieSpec) -> None:
        self.dies[spec.id] = spec
        self.save()

    def remove_die(self, die_id: str) -> None:
        self.dies.pop(die_id, None)
        self.save()

    def list_dies(self) -> List[DieSpec]:
        return sorted(self.dies.values(), key=lambda s: s.name)

    def get_die(self, die_id: str) -> Optional[DieSpec]:
        return self.dies.get(die_id)

    # ------------------------------------------------------------------
    # CRUD — Sets (пресеты)
    # ------------------------------------------------------------------

    def add_set(self, spec: ToolSetSpec) -> None:
        self.sets[spec.id] = spec
        self.save()

    def remove_set(self, set_id: str) -> None:
        self.sets.pop(set_id, None)
        self.save()

    def list_sets(self) -> List[ToolSetSpec]:
        return sorted(self.sets.values(), key=lambda s: s.name)

    def get_set(self, set_id: str) -> Optional[ToolSetSpec]:
        return self.sets.get(set_id)

    # ------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------

    def get_punch_shape(self, punch_id: str, length: float = 500.0):
        spec = self.punches.get(punch_id)
        if spec is None:
            return None
        return _load_or_build(spec, self.root, is_punch=True, length=length)

    def get_die_shape(self, die_id: str, length: float = 500.0):
        spec = self.dies.get(die_id)
        if spec is None:
            return None
        return _load_or_build(spec, self.root, is_punch=False, length=length)

    # ------------------------------------------------------------------
    # Import from FreeCAD object
    # ------------------------------------------------------------------

    def import_from_freecad(self, obj, kind: str,
                            new_id: str, name: str) -> Optional[str]:
        """Сохранить shape объекта FreeCAD как .brep в библиотеке."""
        if not _FC_OK or obj is None:
            return None
        try:
            shape = obj.Shape
            if shape is None or shape.isNull():
                return None
        except Exception:
            return None

        sub = "punches" if kind == "punch" else "dies"
        fname = "{}.brep".format(new_id)
        rel = "{}/{}".format(sub, fname)
        target = self.root / rel
        try:
            shape.exportBrep(str(target))
        except Exception as e:
            App.Console.PrintError(
                "[Tooling] import export failed: {}\n".format(e))
            return None

        if kind == "punch":
            spec = PunchSpec(id=new_id, name=name,
                             geometry_mode="imported",
                             geometry_file=rel)
            self.punches[spec.id] = spec
        else:
            spec = DieSpec(id=new_id, name=name,
                           geometry_mode="imported",
                           geometry_file=rel)
            self.dies[spec.id] = spec
        self.save()
        return new_id

    def import_from_file(self, filepath: str, kind: str,
                         new_id: str, name: str) -> Optional[str]:
        """Скопировать STEP/FCStd/BREP в библиотеку и сделать спеку."""
        src = Path(filepath)
        if not src.exists():
            return None
        sub = "punches" if kind == "punch" else "dies"
        rel = "{}/{}".format(sub, src.name)
        target = self.root / rel
        try:
            shutil.copy(src, target)
        except Exception:
            return None
        if kind == "punch":
            spec = PunchSpec(id=new_id, name=name,
                             geometry_mode="file",
                             geometry_file=rel)
            self.punches[spec.id] = spec
        else:
            spec = DieSpec(id=new_id, name=name,
                           geometry_mode="file",
                           geometry_file=rel)
            self.dies[spec.id] = spec
        self.save()
        return new_id


# =====================================================================
# Load / build
# =====================================================================

def _load_or_build(spec, root: Path, is_punch: bool, length: float):
    if not _FC_OK:
        return None

    mode = getattr(spec, "geometry_mode", "parametric")
    if mode == "imported" or mode == "file":
        gf = getattr(spec, "geometry_file", None)
        if not gf:
            return None
        p = root / gf
        if not p.exists():
            App.Console.PrintWarning(
                "[Tooling] geometry file missing: {}\n".format(p))
            return None
        return _load_shape_from_file(p)

    # parametric
    from .parametric import build_punch, build_die
    if is_punch:
        return build_punch(spec, length=length)
    return build_die(spec, length=length)


def _load_shape_from_file(path: Path):
    suffix = path.suffix.lower()
    try:
        if suffix == ".brep":
            return Part.Shape.read(str(path))
        if suffix == ".step" or suffix == ".stp":
            sh = Part.Shape()
            sh.read(str(path))
            return sh
        if suffix in (".fcstd", ".FCStd"):
            doc = App.openDocument(str(path))
            # берём первый Part::Feature
            for o in doc.Objects:
                try:
                    if hasattr(o, "Shape") and not o.Shape.isNull():
                        return o.Shape.copy()
                except Exception:
                    continue
    except Exception as e:
        App.Console.PrintError(
            "[Tooling] load shape failed: {}\n".format(e))
    return None