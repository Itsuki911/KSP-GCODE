Independent audit evidence, 2026-11-18

Reports:
  ../../gcodes/AnalysisResults.txt
  ../../gcodes/解析結果.txt

The *.before files preserve the previous reports verbatim.
gcode_hashes_before.json records all nine source G-codes and the candidate.
verify.py reads the supplied workspace and regenerates evidence.json.
It never connects to a printer and opens SQLite in mode=ro.

Run: python3 'Working Folder_Adachi/audit/verify.py'

Timing limitations:
- The supplied Klipper Move and LookAheadQueue classes are loaded through AST
  extraction, without starting Klippy or any MCU interface.
- Assumed base feed F1500, global speed125, accel1000, Z speed20, Z accel500,
  square_corner_velocity6, minimum_cruise_ratio0.5.
- G28 is assumed successful at configured endstops, with zero time cost.
- Queues are fully flushed at homing, M400, G4, and end-of-file; actual host
  streaming, lazy flushing and starvation are not emulated.
- Servo events are markers between moves. Servo/mechanical/fluid delays are
  not simulated, and timed intervals do not prove delivered liquid volume.
- Parser handles the limited motion commands in these inputs, not arbitrary
  Klipper macros. Unknown commands (including DE3 in a comparison file) appear
  in command_counts but are not an execution validation.
- No tool geometry, fixture envelope, actuator compliance, electrical faults,
  or physical collision model is included.
- Relative-mode and residual-offset examples demonstrate state dependency;
  they do not prove a specific historical machine state.

Log entries are occurrence-level evidence. Repeated diagnostic lines must not
be counted as independent shutdowns. DB job dates and log session dates differ;
do not correlate them solely by filename or similar symptoms.

Follow-up candidate revision (2026-11-18):
The user subsequently requested application of the audit findings to the candidate.
See ../testPaint_changes.md and followup_validation.json for the new revision.
The old candidate is preserved as testPaint.pre_followup.gcode.before.
The reports and evidence.json describe the pre-follow-up audit snapshot; they
were deliberately not overwritten during this candidate edit.
Run validate_followup.py to check the revised candidate without overwriting
that historical evidence. Its modal replay uses the local GCodeMove handlers.
