# Миграция SheetMetal x BendSpec -> BendSeq

## Фазы

- Фаза 0 — config.py + копии «как есть»
- Фаза 1 — строгий state layer (FrozensetSequenceState, PanelGraph2D, panel_solid)
- Фаза 2 — backward.py, scoring.py, фасад autosequencer
- Фаза 3 — kinematic_engine (fractions), freecad_adapter (MatchingConfidence)
- Фаза 4 — UI: Backward, Hybrid, колонка Score
- Фаза 5 — аудит, тесты, документация

