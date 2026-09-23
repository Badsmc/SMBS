# -*- coding: utf-8 -*-
"""
autosequencer/kinematics.py
ReverseKinematicsEngine — обёртка над SequenceValidator с кэшем
по (already_ids_tuple, bend_id).

Версия: BENDBEQ_AUTOSEQ_KINEMATICS_V1
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from core.sequence_validator import StepValidationResult


class ReverseKinematicsEngine:
    """Кэширует StepValidationResult по префиксу уже выполненных гибов.

    При повторном запросе той же (already_done, bend) пары возвращает
    сохранённый результат — экономит дорогую OCC-симуляцию.
    """

    def __init__(self, validator, kinematics=None, logger=None):
        self.validator = validator
        self.kinematics = kinematics
        self._log = logger if callable(logger) else (lambda m: None)
        self._cache: Dict[Tuple, StepValidationResult] = {}

    def can_apply(self, current_shape, bend, step_idx: int,
                  already_done: list) -> StepValidationResult:
        """Валидировать bend при состоянии already_done.

        Args:
            current_shape: Part.Shape до гиба.
            bend:          BendSpec.
            step_idx:      0-based номер шага.
            already_done:  list[BendSpec].

        Returns:
            StepValidationResult.
        """
        already_ids = tuple(sorted(b.id for b in already_done))
        key = (already_ids, bend.id)

        cached = self._cache.get(key)
        if cached is not None:
            return cached

        try:
            res = self.validator.validate_step(
                current_shape=current_shape,
                bend=bend,
                step_idx=step_idx,
                already_done=already_done,
                kinematics=self.kinematics,
            )
        except Exception as e:
            self._log("[Kinematics] validate_step error: {}".format(e))
            res = StepValidationResult(
                bend_id=getattr(bend, "id", "?"),
                step_number=step_idx + 1,
                ok=False,
                shape_after=None,
                collision_type="EXCEPTION",
                reason_human="Kinematics error: {}".format(e),
            )

        self._cache[key] = res
        return res

    def clear_cache(self) -> None:
        self._cache.clear()

    def cache_size(self) -> int:
        return len(self._cache)

    def __repr__(self):
        return "ReverseKinematicsEngine(cache={})".format(len(self._cache))