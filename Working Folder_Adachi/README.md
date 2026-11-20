# Working Folder — Adachi

This folder contains the independent G-code audit evidence, the revised full-cycle candidate program, and the staged single-motion commissioning test suite. Original files in `gcodes/` and `printer_data/` are untouched.

## Contents

- [`single_motion_tests/`](./single_motion_tests/README.md) — Staged single-motion commissioning G-code suite (**Phases 0–3**, 16 `.gcode` files), safe revisions of legacy single-motion tests (`PaintFile`, `cableChainTest`, `turntableBreakin`), physical pass/fail checklist, and offline validator (`validate_tests.py`). **Start here before running `testPaint.gcode` on hardware.**
- [`testPaint.gcode`](./testPaint.gcode) — Audit-revised full-cycle coating candidate (requires hardware validation of clearances, valve delays, and TMC diagnostics).
- [`testPaint_changes.md`](./testPaint_changes.md) — Summary of audit-driven edits applied to `testPaint.gcode` (Japanese).
- [`audit/`](./audit/README.txt) — Independent audit scripts (`verify.py`, `validate_followup.py`), pre-revision snapshots (`*.before`), and JSON evidence (`evidence.json`, `followup_validation.json`).
