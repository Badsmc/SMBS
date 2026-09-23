# BendSeq

Bend sequence planning for sheet metal (FreeCAD addon).

## Статус

Скелет проекта. Реализация — по фазам (см. docs/MIGRATION.md).

## Структура

- `core/` — модели, кинематика, симуляция, коллизии, верификация.
- `autosequencer/` — поиск последовательности (forward + backward + scoring).
- `adapters/` — мосты к FreeCAD (BendSpec, overlay).
- `plugins/tooling/` — адаптеры оснастки.
- `ui/` — task panel, viewer симуляции.
- `export/` — CSV/JSON/TXT отчёты.
- `docs/` — архитектура, миграция, модель данных.

