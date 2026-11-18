"""Offline evidence collection; never connects to a printer or modifies inputs.

Run from any directory. Timings exclude homing, host/MCU latency, and mechanics.
Uses the supplied Klipper Move/LookAheadQueue classes with default runtime limits.
"""
from pathlib import Path
import ast
import collections
import datetime
import hashlib
import json
import math
import re
import sqlite3
import types
import zipfile

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent

def commands(path):
    return [(i, ' '.join(line.split(';')[0].strip().upper().split()))
            for i, line in enumerate(path.read_text().splitlines(), 1)
            if line.split(';')[0].strip()]

source = ROOT / 'klipper/klippy/toolhead.py'
tree = ast.parse(source.read_text())
classes = [n for n in tree.body if isinstance(n, ast.ClassDef)
           and n.name in ('Move', 'LookAheadQueue')]
ns = {'math': math, 'LOOKAHEAD_FLUSH_TIME': .150}
exec(compile(ast.Module(body=classes, type_ignores=[]), str(source), 'exec'), ns)
toolhead = types.SimpleNamespace(max_accel=1000., max_velocity=125.,
    junction_deviation=36.*(math.sqrt(2.)-1.)/1000.,
    mcr_pseudo_accel=500., extra_axes=[])

def analyze(path, relative=False, offset=(0., 0., 0.)):
    pos = [0., 0., 47.25, 0.]
    factor, feed, absolute = 1., 1500., not relative
    queue = ns['LookAheadQueue']()
    moves, faults, events = [], [], []
    elapsed, lower, dwell, homes = 0., 0., 0., 0
    def flush():
        nonlocal elapsed
        for m in queue.flush():
            m.audit['time'] = m.accel_t + m.cruise_t + m.decel_t
            m.audit['start_v'] = m.start_v
            m.audit['cruise_v'] = m.cruise_v
            m.audit['end_v'] = m.end_v
            elapsed += m.audit['time']
    for line, cmd in commands(path):
        words = cmd.split(); op = words[0]
        params = {m[0]:float(m[1]) for m in re.findall(r'([XYZFSP])(-?\d+(?:\.\d+)?)',cmd)}
        if op == 'G90': absolute = True
        elif op == 'G91': absolute = False
        elif op == 'M220': factor = params['S']/100.
        elif op == 'G28':
            flush(); homes += 1
            for i, a in enumerate('XYZ'):
                if len(words)==1 or a in words[1:]: pos[i] = (0.,0.,47.25)[i]
        elif op in ('M400','G4'):
            flush()
            if op == 'G4': dwell += params.get('P',0.)/1000.
        elif op.startswith('BALL_VALVE_'): events.append([line,op,len(moves)])
        elif op in ('G0','G1'):
            if 'F' in params: feed = params['F']
            end = pos.copy()
            for i,a in enumerate('XYZ'):
                if a in params: end[i] = params[a]+(offset[i] if absolute else pos[i])
            if any(not 0 <= end[i] <= (360,180,60)[i] for i in range(3) if end[i]!=pos[i]):
                faults.append({'line':line,'command':cmd,'target':end[:3]}); break
            m = ns['Move'](toolhead,pos,end,feed/60.*factor)
            if m.move_d:
                if m.axes_d[2]:
                    ratio = m.move_d/abs(m.axes_d[2]); m.limit_speed(20.*ratio,500.*ratio)
                data={'line':line,'command':cmd,'start':pos[:3], 'end':end[:3],
                      'distance':m.move_d, 'requested':feed/60.*factor,
                      'limited':math.sqrt(m.max_cruise_v2)}
                m.audit=data; moves.append(data); lower += m.min_move_t
                queue.add_move(m)
            pos=end
    flush()
    return {'g0_g1_time_excluding_homing':elapsed,'constant_speed_lower_bound':lower,
            'dwell':dwell,'g28_commands':homes,'faults':faults,'valve_events':events,'moves':moves}

result={'assumptions':['G90 unless explicitly testing G91','F1500 initial base feed',
    'zero G-code offsets unless explicitly tested','default runtime speed/acceleration limits',
    'homing succeeds at configured coordinate; homing time excluded',
    'offline planner timing is not an observed run or physical collision simulation']}
result['source_hashes']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in [source, ROOT/'printer_data/config/printer.cfg',
              ROOT/'mainsail-config/client.cfg', ROOT/'printer_data/database/moonraker-sql.db']}
paths=sorted((ROOT/'gcodes').glob('*.gcode'))+[ROOT/'Working Folder_Adachi/testPaint.gcode']
result['files']={}
for p in paths:
    rel=str(p.relative_to(ROOT)); cmd=commands(p)
    installed=ROOT/'printer_data/gcodes'/p.name
    result['files'][rel]={'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
      'lines':len(p.read_text().splitlines()),'commands':len(cmd),
      'command_counts':dict(collections.Counter(c.split()[0] for _,c in cmd)),
      'installed_identical':installed.exists() and p.read_bytes()==installed.read_bytes(),
      'analysis':analyze(p)}
dry=ROOT/'gcodes/ExternalPresentationDrytestPaintReverseOrder.gcode'
chunks=[]; chunk=[]
for _,c in commands(dry):
    chunk.append(c)
    if c=='G4 P4000': chunks.append(chunk);chunk=[]
result['dry_cycles']={'count':len(chunks),'unique':len({tuple(c) for c in chunks}),
                      'commands_per_cycle':[len(c) for c in chunks], 'trailing':chunk}
result['relative_mode']={p.name:analyze(p,relative=True)['faults'] for p in
  [ROOT/'gcodes/testPaint.gcode', ROOT/'gcodes/testPaintReverseOrder.gcode',dry]}
result['corrected_with_z_offset_1mm']=analyze(paths[-1],offset=(0,0,1))['moves'][:3]
result['archives']={}
for p in sorted((ROOT/'printer_data/config').glob('*.zip')):
    with zipfile.ZipFile(p) as z:
        result['archives'][p.name]={n:{'sha256':hashlib.sha256(z.read(n)).hexdigest(),
          'identical_to_current':(ROOT/n).exists() and (ROOT/n).read_bytes()==z.read(n)}
          for n in z.namelist() if n.lower().endswith('.gcode')}
c=sqlite3.connect('file:'+str(ROOT/'printer_data/database/moonraker-sql.db')+'?mode=ro',uri=True)
c.row_factory=sqlite3.Row
result['selected_job_dates_jst']=[]
for r in c.execute('SELECT job_id,start_time,end_time FROM job_history WHERE job_id IN (12,104,105,113,114)'):
    d=dict(r)
    for key in ('start_time','end_time'):
        d[key]=datetime.datetime.fromtimestamp(d[key],datetime.timezone(datetime.timedelta(hours=9))).isoformat()
    result['selected_job_dates_jst'].append(d)
result['job_counts']=[dict(r) for r in c.execute('SELECT filename,status,count(*) AS count,'
  'min(total_duration) AS min_total_duration,max(total_duration) AS max_total_duration '
  'FROM job_history GROUP BY filename,status')]
result['completed_dry_jobs']=[]
for r in c.execute("SELECT job_id,total_duration,print_duration,metadata FROM job_history WHERE filename LIKE 'External%' AND status='completed'"):
    d=dict(r);d['metadata']=json.loads(d['metadata']);result['completed_dry_jobs'].append(d)
c.close()
result['log_events']={}
result['log_session_dates']={}
pattern=re.compile(r'No trigger|Move out of range|reports error|Transition to shutdown|Unknown command|Lost communication|Timer too close')
for p in sorted((ROOT/'printer_data/logs').glob('klippy.log*')):
    result['log_session_dates'][p.name]=[l for l in p.read_text().splitlines() if 'Start printer at' in l]
    result['log_events'][p.name]=[{'line':i,'text':l} for i,l in enumerate(p.read_text(errors='replace').splitlines(),1)
        if pattern.search(l) and not l.startswith(('Stats ','Received ','Sent '))]
OUT.joinpath('evidence.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
for p,d in result['files'].items():
    a=d['analysis'];print(p,'commands',d['commands'],'move_time',round(a['g0_g1_time_excluding_homing'],3),
      'lower_bound',round(a['constant_speed_lower_bound'],3),'faults',a['faults'])
print('dry_cycles',result['dry_cycles']['count'],result['dry_cycles']['unique'],sorted(set(result['dry_cycles']['commands_per_cycle'])))
print('G91 failures',result['relative_mode'])
print('evidence:',OUT/'evidence.json')
