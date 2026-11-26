"""Offline modal-state and safety-invariant validator for single_motion_tests/*.gcode.

Uses the local Klipper GCodeMove implementation (no device I/O).
Verifies every test G-code across three hostile incoming modal states (G91,
non-100% M220, arbitrary F, non-zero XYZ G-code offsets) and checks that
original gcodes/*.gcode and Working Folder_Adachi/testPaint.gcode are untouched.
"""
import ast
import hashlib
import json
import logging
import math
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ADACHI_DIR = HERE.parent
ROOT = ADACHI_DIR.parent

source = ROOT / 'klipper/klippy/extras/gcode_move.py'
tree = ast.parse(source.read_text())
classes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GCodeMove']
assert len(classes) == 1
ns = {'logging': logging}
exec(compile(ast.Module(body=classes, type_ignores=[]), str(source), 'exec'), ns)

BOUNDS = (360.0, 180.0, 47.25)  # Strict Z <= upper endstop (47.250), never > 47.25
HOME_POS = (0.0, 0.0, 47.25)
SCENARIOS = [
    (False, 1.0, 1500.0, (0.0, 0.0, 0.0)),
    (True, 5.0, 500.0, (7.0, -2.0, 1.0)),
    (True, 0.1, 6000.0, (-4.0, 3.0, -2.0)),
]

# Expected valve-open counts per file (only Test 03 opens the valve, 3 times).
EXPECTED_VALVE_OPENS = {
    '03_valve_static.gcode': 3,
}
# Files expected to have zero axis moves (homing-only or valve-only).
ZERO_MOVE_FILES = {
    '01_home_z_only.gcode',
    '02_home_xy_only.gcode',
    '03_valve_static.gcode',
}


class Command:
    def __init__(self, text):
        self.text = text
        self.params = {}
        for word in text.split()[1:]:
            if '=' in word:
                key, value = word.split('=', 1)
            else:
                key, value = word[0], word[1:]
            self.params[key] = value

    def get_command_parameters(self):
        return self.params

    def get_commandline(self):
        return self.text

    def get_float(self, key, default=None, **limits):
        value = float(self.params[key]) if key in self.params else default
        if value is not None:
            if 'above' in limits:
                assert value > limits['above']
            if 'minval' in limits:
                assert value >= limits['minval']
        return value

    def get_int(self, key, default=None, **limits):
        return int(self.params[key]) if key in self.params else default

    def error(self, message):
        return ValueError(message)


def parse_commands(path: Path):
    return [
        (i, line.split(';', 1)[0].strip())
        for i, line in enumerate(path.read_text().splitlines(), 1)
        if line.split(';', 1)[0].strip()
    ]


def replay_file(path: Path, relative, factor, feed, offsets):
    commands = parse_commands(path)
    g = ns['GCodeMove'].__new__(ns['GCodeMove'])
    machine = [100.0, 50.0, 25.0, 0.0]
    g.is_printer_ready = True
    g.absolute_coord = not relative
    g.absolute_extrude = True
    g.axis_map = {'X': 0, 'Y': 1, 'Z': 2, 'E': 3}
    g.homing_position = list(offsets) + [0.0]
    g.base_position = list(offsets) + [0.0]
    g.last_position = machine.copy()
    g.speed_factor = factor / 60.0
    g.speed = feed * factor / 60.0
    g.extrude_factor = 1.0
    g.position_with_transform = lambda: machine.copy()

    moves = []
    valve = 'unknown'
    valve_opens = 0
    valve_closes = 0
    homed = set()
    dwells_ms = 0
    seen_preamble = set()

    def move(end, speed):
        nonlocal machine
        if machine[:3] != end[:3]:
            moved_axes = [i for i in range(3) if abs(machine[i] - end[i]) > 1e-9]
            for ax in moved_axes:
                assert ax in homed, f'{path.name}:{line} moved unhomed axis {ax}'
            assert valve != 'unknown', f'{path.name}:{line} moved before valve closed'
            for i in range(3):
                assert 0.0 <= end[i] <= BOUNDS[i] + 1e-9, (
                    f'{path.name}:{line} axis {i} out of safe bounds: {end[i]}'
                )
            if 2 in moved_axes and len(moved_axes) > 1:
                assert path.name == '14_purge_7deg_simulation_only.gcode', (
                    f'{path.name}:{line} unexpected simultaneous Z+XY move')
                assert moved_axes == [0, 2] and machine[1] == end[1] == 15.0
                points = {(round(machine[0], 4), round(machine[2], 4)),
                          (round(end[0], 4), round(end[2], 4))}
                assert points == {(277.0, 16.1712), (285.0, 19.0)}
                assert speed * abs(end[2] - machine[2]) / math.dist(end[:3], machine[:3]) <= 20.0

            if moved_axes == [2]:
                assert speed <= 20.0 + 1e-9, f'{path.name}:{line} Z speed {speed} > 20 mm/s'
            else:
                assert speed <= 125.0 + 1e-9, f'{path.name}:{line} XY speed {speed} > 125 units/s'
            moves.append({
                'line': line,
                'start': [round(v, 6) for v in machine[:3]],
                'end': [round(v, 6) for v in end[:3]],
                'requested_speed': round(speed, 6),
                'valve': valve,
            })
        machine = list(end)

    g.move_with_transform = move
    previous_op = None
    first_motion_or_home = False

    for line, text in commands:
        op = text.split()[0]
        command = Command(text)

        if op == 'G28':
            assert {'G21', 'G90', 'M220', 'SET_GCODE_OFFSET', 'BALL_VALVE_CLOSED'}.issubset(
                seen_preamble
            ), f'{path.name}:{line} G28 before complete safety preamble'
            first_motion_or_home = True
            axes = [i for i, a in enumerate('XYZ') if a in command.params] or [0, 1, 2]
            if any(a != 2 for a in axes):
                assert 2 in homed, f'{path.name}:{line} X/Y homed before Z'
            for i in axes:
                machine[i] = HOME_POS[i]
                homed.add(i)
            g._handle_home_rails_end(SimpleNamespace(get_axes=lambda: axes), [])
        elif op in ('BALL_VALVE_OPEN', 'BALL_VALVE_CLOSED'):
            assert previous_op == 'M400', f'{path.name}:{line} {op} without preceding M400'
            if op == 'BALL_VALVE_OPEN':
                valve = 'open'
                valve_opens += 1
            else:
                valve = 'closed'
                valve_closes += 1
                if not first_motion_or_home:
                    seen_preamble.add('BALL_VALVE_CLOSED')
        elif op == 'M400':
            pass
        elif op == 'G4':
            p_ms = command.get_int('P', default=0)
            assert p_ms > 0, f'{path.name}:{line} G4 requires positive P (ms)'
            dwells_ms += p_ms
        else:
            if op in ('G0', 'G1'):
                if any(a in command.params for a in 'XYZ'):
                    assert 'F' in command.params, f'{path.name}:{line} move missing explicit F'
                    assert {'G21', 'G90', 'M220', 'SET_GCODE_OFFSET', 'BALL_VALVE_CLOSED'}.issubset(
                        seen_preamble
                    ), f'{path.name}:{line} move before complete safety preamble'
                    first_motion_or_home = True
                op_exec = 'G1'
            else:
                op_exec = op
                if not first_motion_or_home:
                    seen_preamble.add(op)
            assert op_exec in ('G21', 'G90', 'M220', 'SET_GCODE_OFFSET', 'G1'), (
                f'{path.name}:{line} unexpected command {op}'
            )
            getattr(g, 'cmd_' + op_exec)(command)
        previous_op = op

    assert valve == 'closed', f'{path.name} did not end with valve closed'
    assert g.absolute_coord, f'{path.name} did not end in G90'
    assert g.homing_position[:3] == [0.0, 0.0, 0.0], f'{path.name} ended with non-zero offsets'
    assert abs(g.speed_factor - 1.0 / 60.0) < 1e-12, f'{path.name} did not end with M220 S100'
    assert abs(g.speed - 25.0) < 1e-9, f'{path.name} did not end with F1500 (25 units/s)'
    expected_opens = EXPECTED_VALVE_OPENS.get(path.name, 0)
    assert valve_opens == expected_opens, (
        f'{path.name} valve_opens={valve_opens}, expected {expected_opens}'
    )
    if path.name in ZERO_MOVE_FILES:
        assert len(moves) == 0, f'{path.name} expected 0 axis moves, got {len(moves)}'
    else:
        assert len(moves) > 0, f'{path.name} expected >0 axis moves'

    return {
        'moves': moves,
        'move_count': len(moves),
        'valve_opens': valve_opens,
        'valve_closes': valve_closes,
        'total_dwell_ms': dwells_ms,
        'homed_axes': sorted(homed),
    }


def main():
    sensor_check = HERE / '00_endstop_sensor_check.gcode'
    sensor_ops = [text.split()[0] for _, text in parse_commands(sensor_check)]
    assert sensor_ops == ['M400', 'M118', 'G4', 'M400', 'QUERY_ENDSTOPS'] + (['M118', 'G4', 'M400', 'QUERY_ENDSTOPS'] * 6) + ['M118']
    assert all(text == 'G4 P15000' for _, text in parse_commands(sensor_check) if text.startswith('G4 '))
    gcode_files = sorted(p for p in HERE.glob('*.gcode') if p != sensor_check)
    assert len(gcode_files) == 17, f'Expected 17 test gcode files, found {len(gcode_files)}'

    summary = {}
    for path in gcode_files:
        traces = [replay_file(path, *s) for s in SCENARIOS]
        assert traces[0] == traces[1] == traces[2], (
            f'{path.name}: incoming modal state changed trajectory'
        )
        t = traces[0]
        max_z = max((max(m['start'][2], m['end'][2]) for m in t['moves']), default=47.25 if 2 in t['homed_axes'] else None)
        min_z = min((min(m['start'][2], m['end'][2]) for m in t['moves']), default=47.25 if 2 in t['homed_axes'] else None)
        summary[path.name] = {
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'move_count': t['move_count'],
            'valve_opens': t['valve_opens'],
            'valve_closes': t['valve_closes'],
            'total_dwell_ms': t['total_dwell_ms'],
            'homed_axes': t['homed_axes'],
            'min_z': min_z,
            'max_z': max_z,
            'identical_across_3_modal_states': True,
        }

    # Verify original gcodes/*.gcode and Working Folder_Adachi/testPaint.gcode are untouched.
    orig_hashes = json.loads((ADACHI_DIR / 'audit/gcode_hashes_before.json').read_text())
    for rel, expected_sha in orig_hashes.items():
        if rel.startswith('gcodes/'):
            actual = hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()
            assert actual == expected_sha, f'Original file modified: {rel}'

    followup = json.loads((ADACHI_DIR / 'audit/followup_validation.json').read_text())
    candidate_sha = hashlib.sha256((ADACHI_DIR / 'testPaint.gcode').read_bytes()).hexdigest()
    assert candidate_sha == followup['candidate_sha256'], 'Working Folder_Adachi/testPaint.gcode changed'

    print('PASS: endstop check uses seven timed queries and commands no motion or valve.')
    print(f'PASS: {len(gcode_files)} single-motion test files validated across 3 hostile modal states.')
    print('  - All files enforce safety preamble/epilogue, M220 S100, explicit F, Z<=47.25, only approved test 14 X/Z slopes, M400 valve barriers.')
    print('  - All 9 original gcodes/*.gcode and Working Folder_Adachi/testPaint.gcode remain byte-identical.')
    for name, info in summary.items():
        print(
            f"  [OK] {name:38s} moves={info['move_count']:3d} "
            f"valve(O/C)={info['valve_opens']}/{info['valve_closes']} "
            f"dwell={info['total_dwell_ms']/1000.0:4.1f}s "
            f"Z=[{info['min_z']}, {info['max_z']}]"
        )


if __name__ == '__main__':
    main()
