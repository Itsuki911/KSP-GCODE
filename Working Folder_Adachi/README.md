# Working Folder — Adachi

This folder contains the independent G-code audit evidence, the revised full-cycle candidate program, and the staged single-motion commissioning test suite. Original files in `gcodes/` and `printer_data/` are untouched.

## Contents

- [`single_motion_tests/`](./single_motion_tests/README.md) — Staged single-motion commissioning G-code suite (**Phases 0–3**, 16 `.gcode` files), safe revisions of legacy single-motion tests (`PaintFile`, `cableChainTest`, `turntableBreakin`), physical pass/fail checklist, and offline validator (`validate_tests.py`). **Start here before running `testPaint.gcode` on hardware.**
- [`simulator/`](./simulator/README.md) — Local OSS simulator combining Klipper `GCodeMove`/`LookAheadQueue` motion planning, cylindrical/polar kinematics, a parametric 3D workpiece model (designed from `Worker-Image.jpeg`), MuJoCo 3D collision/clearance verification & viewer, and interactive Plotly 3D HTML / Matplotlib PNG visualization.
- [`Worker-Image.jpeg`](./Worker-Image.jpeg) — Reference photo of the coated workpiece mounted on the turntable used to design the parametric 3D workpiece model.
- [`testPaint.gcode`](./testPaint.gcode) — Audit-revised full-cycle coating candidate (requires hardware validation of clearances, valve delays, and TMC diagnostics).
- [`testPaint_changes.md`](./testPaint_changes.md) — Summary of audit-driven edits applied to `testPaint.gcode` (Japanese).
- [`audit/`](./audit/README.txt) — Independent audit scripts (`verify.py`, `validate_followup.py`), pre-revision snapshots (`*.before`), and JSON evidence (`evidence.json`, `followup_validation.json`).

- [`testPaint_finalReviced_simulation.gcode`](./testPaint_finalReviced_simulation.gcode) — 7度の仮想坂とinner Y131を用いたシミュレーション候補。
- [`testPaint_finalReviced_machine.gcode`](./testPaint_finalReviced_machine.gcode) — 同じ実行命令を持つマシン用候補。実機寸法・干渉・ローラー接触・エンドストップ・バルブ動作の確認前には実行しないでください。
