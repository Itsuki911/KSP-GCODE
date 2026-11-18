"""Offline modal-state regression checks for the revised candidate. No device I/O."""
import ast
import hashlib
import json
import logging
import re
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
candidate = ROOT / 'Working Folder_Adachi/testPaint.gcode'
source = ROOT / 'klipper/klippy/extras/gcode_move.py'
tree = ast.parse(source.read_text())
classes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GCodeMove']
assert len(classes) == 1
ns = {'logging': logging}
exec(compile(ast.Module(body=classes, type_ignores=[]), str(source), 'exec'), ns)

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
    def get_command_parameters(self): return self.params
    def get_commandline(self): return self.text
    def get_float(self, key, default=None, **limits):
        value = float(self.params[key]) if key in self.params else default
        if value is not None:
            if 'above' in limits: assert value > limits['above']
            if 'minval' in limits: assert value >= limits['minval']
        return value
    def get_int(self, key, default=None, **limits):
        return int(self.params[key]) if key in self.params else default
    def error(self, message): return ValueError(message)

commands = [(i, line.split(';')[0].strip())
            for i, line in enumerate(candidate.read_text().splitlines(), 1)
            if line.split(';')[0].strip()]

def replay(relative, factor, feed, offsets):
    # Run real local GCodeMove handlers; emulate successful configured homing.
    g = ns['GCodeMove'].__new__(ns['GCodeMove'])
    machine = [100., 50., 25., 0.]
    g.is_printer_ready = True
    g.absolute_coord, g.absolute_extrude = not relative, True
    g.axis_map = {'X': 0, 'Y': 1, 'Z': 2, 'E': 3}
    g.homing_position = list(offsets) + [0.]
    g.base_position = list(offsets) + [0.]
    g.last_position = machine.copy()
    g.speed_factor, g.speed, g.extrude_factor = factor / 60., feed * factor / 60., 1.
    g.position_with_transform = lambda: machine.copy()
    moves, valve = [], 'unknown'
    homed = set()
    def move(end, speed):
        nonlocal machine
        if machine[:3] != end[:3]:
            assert homed == {0, 1, 2}
            assert valve != 'unknown'
            assert all(0 <= end[i] <= (360, 180, 60)[i] for i in range(3))
            assert end[2] <= 47.25
            axes = [i for i in range(3) if machine[i] != end[i]]
            assert not (2 in axes and len(axes) > 1), 'simultaneous Z/XY'
            moves.append({'line':line,'start':machine[:3], 'end':end[:3],
                          'requested_speed':speed, 'valve':valve})
        machine = list(end)
    g.move_with_transform = move
    previous_op = None
    for line, text in commands:
        op = text.split()[0]
        command = Command(text)
        if op == 'G28':
            axes = [i for i,a in enumerate('XYZ') if a in command.params] or [0,1,2]
            if any(a != 2 for a in axes): assert 2 in homed
            for i in axes:
                machine[i] = (0.,0.,47.25)[i]
                homed.add(i)
            g._handle_home_rails_end(SimpleNamespace(get_axes=lambda: axes), [])
        elif op.startswith('BALL_VALVE_'):
            assert previous_op == 'M400', 'valve transition without movement barrier'
            valve = 'open' if op == 'BALL_VALVE_OPEN' else 'closed'
        elif op == 'M400': pass  # No motion timing/device I/O in this harness.
        else:
            if op in ('G0','G1'):
                if any(a in command.params for a in 'XYZ'): assert 'F' in command.params
                op = 'G1'
            assert op in ('G21','G90','M220','SET_GCODE_OFFSET','G1'), op
            getattr(g, 'cmd_' + op)(command)
        previous_op = text.split()[0]
    assert valve == 'closed'
    assert g.absolute_coord and g.homing_position[:3] == [0.,0.,0.]
    assert abs(g.speed_factor - 1/60.) < 1e-12 and g.speed == 25.
    purge = [m for m in moves if m['valve'] == 'open']
    assert len(purge) == 17
    assert all(m['start'][1:] == [15.,19.] and m['end'][1:] == [15.,19.] for m in purge)
    assert sum(abs(m['end'][0]-m['start'][0]) for m in purge) == 340
    assert all(m['requested_speed'] == 93.75 for m in purge)
    return moves

scenarios = [(False,1.,1500.,(0.,0.,0.)), (True,5.,500.,(7.,-2.,1.)),
             (True,.1,6000.,(-4.,3.,-2.))]
traces = [replay(*s) for s in scenarios]
assert traces[0] == traces[1] == traces[2], 'incoming modal state affects trajectory'

def coating_strokes(moves):
    return [(m['start'],m['end']) for m in moves
            if m['start'][0] != m['end'][0]
            and m['start'][1:] == m['end'][1:]
            and tuple(m['start'][1:]) in ((45.,6.),(47.,6.),(77.,16.),(180.,8.))]
old = json.loads((OUT/'evidence.json').read_text())
original = old['files']['gcodes/testPaint.gcode']['analysis']['moves']
assert coating_strokes(traces[0]) == coating_strokes(original), 'working strokes changed'
hashes = json.loads((OUT/'gcode_hashes_before.json').read_text())
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h
           for p,h in hashes.items() if p.startswith('gcodes/'))
result = {'candidate_sha256':hashlib.sha256(candidate.read_bytes()).hexdigest(),
          'scenarios':scenarios,'identical_across_scenarios':True,
          'purge_strokes':17,'purge_degrees':340,'original_working_strokes_preserved':True,
          'original_gcodes_unchanged':True,'moves':traces[0],
          'limitations':'Offline GCodeMove semantics only; no homing physics, '
                        'servo settling, timing, liquid or collision validation.'}
(OUT/'followup_validation.json').write_text(json.dumps(result,indent=2)+'\n')
print('PASS: 3 initial modal states converge; 17 purge strokes / 340 degrees;')
print('working strokes preserved; no mixed Z/XY; bounds pass; valve barriers pass;')
print('normal completion closed, G90, zero XYZ offsets, M220 S100, F1500;')
print('all nine original G-codes unchanged. Physical behavior remains unverified.')
