#!/usr/bin/env python3
"""
balance_analysis.py — Analyse a combined balance test log file.

Usage:
    python3 balance_analysis.py <logfile> [--stable-thresh 0.15] [--centered-hold 2.0]
"""

import re, sys, math, argparse
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

RE_CMD          = re.compile(r'\[(\d+\.\d+)\] \[ball_balance_node\]: CMD \| ball=\(([+-]?\d+\.\d+),([+-]?\d+\.\d+)\).*?stable=(True|False)')
RE_PID_ACTIVE   = re.compile(r'\[(\d+\.\d+)\] \[ball_balance_node\]: PID active')
RE_BALL_LOST    = re.compile(r'\[(\d+\.\d+)\] \[ball_balance_node\]: Ball lost for [\d.]+s')
RE_BALL_CENTERED= re.compile(r'\[(\d+\.\d+)\] \[ball_balance_node\]: Ball centered')
RE_POSE_SENT    = re.compile(r'\[(\d+\.\d+)\] \[pose_test_node\]: → Sent: (.+)')
RE_SETTLED      = re.compile(r'\[(\d+\.\d+)\] \[ball_balance_node\]: Arm state → SETTLED')
RE_MOVING       = re.compile(r'\[(\d+\.\d+)\] \[ball_balance_node\]: Arm state → MOVING')
RE_CORR         = re.compile(r'\[(\d+\.\d+)\] \[wrist_balance_controller\]: CORR \| flex ([\d.]+)→([\d.]+) \(Δ([+-][\d.]+)deg\)  roll ([\d.]+)→([\d.]+) \(Δ([+-][\d.]+)deg\)')
RE_DISABLED     = re.compile(r'\[(\d+\.\d+)\] \[wrist_balance_controller\]: Balance DISABLED')
RE_SUB_THRESH   = re.compile(r'\[(\d+\.\d+)\] \[ball_detector_nvidia\]: Ball detected sub-threshold: conf=([\d.]+)')
RE_CONTAINMENT  = re.compile(r'\[(\d+\.\d+)\] \[ball_detector_nvidia\]: Ball outside cup')
RE_ATTEMPT_TO   = re.compile(r'\[(\d+\.\d+)\] \[ball_balance_node\]: Attempt timeout')
RE_NO_BALL      = re.compile(r'\[(\d+\.\d+)\] \[ball_detector_nvidia\]: Cup.*?found but NO BALL')
RE_SUMMARY      = re.compile(
    r'\[(\d+\.\d+)\] \[ball_balance_node\]: PID_SUMMARY \| '
    r'frames=(\d+)/(\d+).*?'
    r'near\([^)]+\)=(\d+)\((\d+)%\).*?'
    r'tight\([^)]+\)=(\d+)\((\d+)%\).*?'
    r'min=([\d.]+)')

@dataclass
class CorrStep:
    t: float; flex_from: float; flex_to: float; roll_from: float; roll_to: float

@dataclass
class PIDSession:
    pose_name: str; start_t: float; end_t: Optional[float] = None
    cmds: List[Tuple] = field(default_factory=list)
    corrs: List[CorrStep] = field(default_factory=list)
    ball_lost_count: int = 0; centered_count: int = 0
    sub_thresh_count: int = 0; containment_rejected: int = 0
    ended_by: str = 'unknown'
    summaries: List[Tuple] = field(default_factory=list)

    @property
    def duration(self): return (self.end_t - self.start_t) if self.end_t else 0.0
    @property
    def min_magnitude(self): return min((c[3] for c in self.cmds), default=None)
    @property
    def final_magnitude(self): return self.cmds[-1][3] if self.cmds else None
    @property
    def best_quartile_dist(self):
        if not self.cmds: return None
        mags = sorted(c[3] for c in self.cmds)
        q = max(1, len(mags)//4)
        return sum(mags[:q])/q
    @property
    def stable_visits(self): return sum(1 for c in self.cmds if c[4])
    def near_visits(self, near_thresh):
        """Count CMD readings within near_thresh of center."""
        return sum(1 for c in self.cmds if c[3] < near_thresh)
    @property
    def summary_valid_frames(self):  return sum(s[0] for s in self.summaries)
    @property
    def summary_total_frames(self):  return sum(s[1] for s in self.summaries)
    @property
    def summary_near_frames(self):   return sum(s[2] for s in self.summaries)
    @property
    def summary_tight_frames(self):  return sum(s[3] for s in self.summaries)
    @property
    def summary_min_mag(self):
        mins=[s[4] for s in self.summaries if s[4]==s[4]]
        return min(mins) if mins else None
    @property
    def summary_detection_rate(self):
        t=self.summary_total_frames
        return self.summary_valid_frames/t if t>0 else None
    @property
    def convergence_slope(self):
        if len(self.cmds) < 3: return None
        ts = [c[0]-self.cmds[0][0] for c in self.cmds]
        ms = [c[3] for c in self.cmds]
        n = len(ts); st=sum(ts); sm=sum(ms); stt=sum(t*t for t in ts); stm=sum(t*m for t,m in zip(ts,ms))
        d = n*stt - st*st
        return (n*stm - st*sm)/d if abs(d)>1e-9 else None
    @property
    def monotone_progress_ratio(self):
        if len(self.cmds)<2: return None
        mags=[c[3] for c in self.cmds]
        return sum(1 for a,b in zip(mags,mags[1:]) if b<a)/(len(mags)-1)
    @property
    def servo_tracking_error(self):
        if len(self.corrs)<2: return None
        errs=[math.sqrt((a.flex_to-b.flex_from)**2+(a.roll_to-b.roll_from)**2)
              for a,b in zip(self.corrs,self.corrs[1:])]
        return sum(errs)/len(errs)
    @property
    def tracking_lag_ratio(self):
        if len(self.corrs)<2: return None
        ratios=[]
        for a,b in zip(self.corrs,self.corrs[1:]):
            cmd=math.sqrt((a.flex_to-a.flex_from)**2+(a.roll_to-a.roll_from)**2)
            act=math.sqrt((b.flex_from-a.flex_from)**2+(b.roll_from-a.roll_from)**2)
            if cmd>1e-6: ratios.append(min(act/cmd,2.0))
        return sum(ratios)/len(ratios) if ratios else None
    @property
    def oscillation_index(self):
        if len(self.corrs)<2: return None
        revs=sum(1 for a,b in zip(self.corrs,self.corrs[1:])
                 if abs(a.flex_to-a.flex_from)>1e-6 and abs(b.flex_to-b.flex_from)>1e-6
                 and (a.flex_to-a.flex_from)*(b.flex_to-b.flex_from)<0)
        return revs/(len(self.corrs)-1)
    @property
    def travel_limit_fraction(self):
        """Fraction of CORR steps where flex delta was zero due to travel limit.
        High value means the servo spent most of its time clamped — the ball
        needed more travel than max_total_rad allowed."""
        if not self.corrs: return None
        clamped = sum(1 for c in self.corrs
                     if abs(c.flex_to - c.flex_from) < 1e-4)
        return clamped / len(self.corrs)
    @property
    def mean_step_size_rad(self):
        """Mean absolute flex+roll displacement per CORR step in radians.
        Low value = small timid corrections; high = aggressive."""
        if not self.corrs: return None
        steps = [math.sqrt((c.flex_to-c.flex_from)**2 +
                           (c.roll_to-c.roll_from)**2)
                 for c in self.corrs]
        return sum(steps)/len(steps)
    @property
    def cumulative_flex_rad(self):
        """Total signed flex displacement across all CORR steps.
        Large positive/negative = controller drove hard in one direction."""
        if not self.corrs: return None
        return sum(c.flex_to - c.flex_from for c in self.corrs)
    @property
    def cumulative_roll_rad(self):
        """Total signed roll displacement across all CORR steps."""
        if not self.corrs: return None
        return sum(c.roll_to - c.roll_from for c in self.corrs)
    @property
    def roll_tracking_error(self):
        if len(self.corrs)<2: return None
        errs=[abs(a.roll_to-b.roll_from) for a,b in zip(self.corrs,self.corrs[1:])]
        return sum(errs)/len(errs)
    @property
    def roll_tracking_lag(self):
        if len(self.corrs)<2: return None
        ratios=[]
        for a,b in zip(self.corrs,self.corrs[1:]):
            cmd=abs(a.roll_to-a.roll_from); act=abs(b.roll_from-a.roll_from)
            if cmd>1e-6: ratios.append(min(act/cmd,2.0))
        return sum(ratios)/len(ratios) if ratios else None
    @property
    def roll_oscillation_index(self):
        if len(self.corrs)<2: return None
        revs=sum(1 for a,b in zip(self.corrs,self.corrs[1:])
                 if abs(a.roll_to-a.roll_from)>1e-6 and abs(b.roll_to-b.roll_from)>1e-6
                 and (a.roll_to-a.roll_from)*(b.roll_to-b.roll_from)<0)
        return revs/(len(self.corrs)-1)
    @property
    def roll_stall_fraction(self):
        if len(self.corrs)<2: return None
        active=[]
        for a,b in zip(self.corrs,self.corrs[1:]):
            cmd=abs(a.roll_to-a.roll_from)
            if cmd<0.003: continue
            act=abs(b.roll_from-a.roll_from)
            active.append(act/cmd if cmd>1e-6 else 1.0)
        return sum(1 for r in active if r<0.15)/len(active) if active else None
    @property
    def effective_correction_rate(self):
        """Net displacement / total attempted displacement.
        1.0 = all corrections in same direction (no oscillation waste).
        0.0 = corrections perfectly cancelled each other out.
        Low value = energy wasted on oscillation."""
        if not self.corrs: return None
        net = abs(self.cumulative_flex_rad or 0)
        total = sum(abs(c.flex_to - c.flex_from) for c in self.corrs)
        return net / total if total > 1e-6 else None

@dataclass
class PoseWindow:
    name: str; sent_t: float
    settled_t: Optional[float]=None; moving_t: Optional[float]=None
    sessions: List[PIDSession] = field(default_factory=list)
    @property
    def best_magnitude(self):
        """Single best (lowest) magnitude seen — used for 'global best' headline."""
        mags=[s.min_magnitude for s in self.sessions if s.min_magnitude is not None]
        return min(mags) if mags else None
    @property
    def best_sustained(self):
        """Best-quartile distance — resistant to single-frame flukes.
        Lower = ball stayed near center for multiple consecutive readings."""
        vals=[s.best_quartile_dist for s in self.sessions if s.best_quartile_dist is not None]
        return min(vals) if vals else None
    no_ball_count: int = 0
    @property
    def total_ball_lost(self): return sum(s.ball_lost_count for s in self.sessions) + self.no_ball_count
    @property
    def total_containment_rejected(self): return sum(s.containment_rejected for s in self.sessions)
    @property
    def worst_travel_limit(self):
        vals=[s.travel_limit_fraction for s in self.sessions if s.travel_limit_fraction is not None]
        return max(vals) if vals else None
    @property
    def servo_quality_score(self):
        """Composite servo score: lower = worse servo behavior.
        Combines tracking error, travel limit fraction, and effective correction rate.
        Used for servo ranking — independent of ball position quality."""
        scores = []
        for s in self.sessions:
            if s.tracking_lag_ratio is None: continue
            # Penalize travel limit clamping heavily
            tlf = s.travel_limit_fraction or 0
            # Penalize poor effective correction rate (oscillation waste)
            ecr = s.effective_correction_rate or 1.0
            # Penalize tracking error
            te  = min(s.servo_tracking_error or 0, 0.1) / 0.1
            # Score: 0=bad, 1=perfect
            score = (1 - tlf) * ecr * (1 - te)
            scores.append(score)
        return sum(scores)/len(scores) if scores else None

def parse_log(path):
    events=[]
    with open(path,'r',errors='replace') as f:
        for line in f:
            for pat,tag in [(RE_POSE_SENT,'pose_sent'),(RE_SETTLED,'settled'),
                            (RE_MOVING,'moving'),(RE_PID_ACTIVE,'pid_active'),
                            (RE_CMD,'cmd'),(RE_CORR,'corr'),(RE_BALL_LOST,'ball_lost'),
                            (RE_BALL_CENTERED,'centered'),(RE_DISABLED,'disabled'),
                            (RE_SUB_THRESH,'sub_thresh'),(RE_CONTAINMENT,'containment'),
                            (RE_ATTEMPT_TO,'attempt_timeout'),
                            (RE_NO_BALL,'no_ball'),
                            (RE_SUMMARY,'summary')]:
                m=pat.search(line)
                if m: events.append((float(m.group(1)),tag,m)); break
    events.sort(key=lambda e:e[0])
    return events

def build_sessions(events, stable_thresh):
    poses=[]; cur_pose=None; cur_sess=None
    for t,tag,m in events:
        if tag=='pose_sent':
            cur_pose=PoseWindow(name=m.group(2).strip(),sent_t=t)
            poses.append(cur_pose); cur_sess=None
        elif tag=='settled':
            if cur_pose: cur_pose.settled_t=t
        elif tag=='moving':
            if cur_pose: cur_pose.moving_t=t
            if cur_sess and cur_sess.end_t is None:
                cur_sess.end_t=t
                if cur_sess.ended_by=='unknown': cur_sess.ended_by='moving'
                cur_sess=None
        elif tag=='pid_active':
            if cur_pose:
                cur_sess=PIDSession(pose_name=cur_pose.name,start_t=t)
                cur_pose.sessions.append(cur_sess)
        elif tag=='cmd':
            if cur_sess:
                x,y=float(m.group(2)),float(m.group(3))
                mag=math.sqrt(x*x+y*y)
                cur_sess.cmds.append((t,x,y,mag,mag<stable_thresh))
        elif tag=='corr':
            if cur_sess:
                cur_sess.corrs.append(CorrStep(t=t,
                    flex_from=float(m.group(2)),flex_to=float(m.group(3)),
                    roll_from=float(m.group(5)),roll_to=float(m.group(6))))
        elif tag=='ball_lost':
            if cur_sess: cur_sess.ball_lost_count+=1
        elif tag=='centered':
            if cur_sess:
                cur_sess.centered_count+=1; cur_sess.end_t=t
                cur_sess.ended_by='centered'; cur_sess=None
        elif tag=='attempt_timeout':
            if cur_sess and cur_sess.end_t is None:
                cur_sess.end_t=t; cur_sess.ended_by='attempt_timeout'
        elif tag=='disabled':
            if cur_sess and cur_sess.end_t is None:
                cur_sess.end_t=t
                if cur_sess.ended_by=='unknown': cur_sess.ended_by='disabled'
        elif tag=='sub_thresh':
            if cur_sess: cur_sess.sub_thresh_count+=1
        elif tag=='no_ball':
            # Count at pose window level — fires outside PID sessions too
            if cur_pose: cur_pose.no_ball_count+=1
            if cur_sess: cur_sess.ball_lost_count+=1
        elif tag=='summary':
            if cur_sess:
                try:
                    cur_sess.summaries.append((
                        int(m.group(2)), int(m.group(3)),
                        int(m.group(4)), int(m.group(6)),
                        float(m.group(8))))
                except (ValueError, IndexError): pass
        elif tag=='containment':
            if cur_sess: cur_sess.containment_rejected+=1
    return poses

def bar(v,mx=1.0,w=20,f='█',e='░'):
    n=int(min(1.0,v/mx if mx>0 else 0)*w); return f*n+e*(w-n)
def stars(m):
    if m is None: return '  N/A '
    n=int((1.0-min(m,1.0))*5); return '★'*n+'☆'*(5-n)
def fmt(v,d=3,s=''):  return f'{v:.{d}f}{s}' if v is not None else '  --- '
def fmt_slope(v):
    if v is None: return '  ---     '
    arr='↓' if v<-0.005 else '↑' if v>0.005 else '→'
    return f'{v:+.4f}/s {arr}'
def fmt_pct(v):  return f'{100*v:.0f}%' if v is not None else ' --- '
def fmt_osc(v):
    if v is None: return '  ---  '
    lbl='HIGH' if v>0.6 else 'med' if v>0.3 else 'low'
    return f'{v:.2f}({lbl})'

def report(poses, stable_thresh, near_thresh, centered_hold, logfile):
    print()
    print('='*76)
    print(f'  BALANCE RUN ANALYSIS')
    print(f'  File   : {logfile}')
    print(f'  stable_thresh={stable_thresh:.2f} (tight center)  '
          f'near_thresh={near_thresh:.2f} (near center)  '
          f'centered_hold={centered_hold:.1f}s')
    print(f'  Note: visit counts based on ~1Hz throttled CMD log — actual time')
    print(f'  near center may be higher; use convergence_slope for trend analysis')
    print('='*76)

    all_sess=[s for pw in poses for s in pw.sessions]
    total_cmds       =sum(len(s.cmds) for s in all_sess)
    total_ball_lost  =sum(s.ball_lost_count for s in all_sess)
    total_centered   =sum(s.centered_count for s in all_sess)
    total_stable     =sum(s.stable_visits for s in all_sess)
    total_sub        =sum(s.sub_thresh_count for s in all_sess)
    total_cont       =sum(s.containment_rejected for s in all_sess)
    gbest            =min((s.min_magnitude for s in all_sess if s.min_magnitude is not None),default=None)
    gbest_pose       =next((pw.name for pw in poses if pw.best_magnitude==gbest),None) if gbest else None
    gbest_sust       =min((pw.best_sustained for pw in poses if pw.best_sustained is not None),default=None)
    gbest_sust_pose  =next((pw.name for pw in poses if pw.best_sustained==gbest_sust),None) if gbest_sust else None

    print()
    print('─'*76)
    print('  PER-POSE DETAIL')
    print('─'*76)

    for pw in poses:
        print(f'\n  ┌─ POSE: {pw.name}')
        if not pw.sessions:
            print(f'  └─ No PID sessions  ball_lost={pw.total_ball_lost}'); continue
        for i,s in enumerate(pw.sessions,1):
            n=len(s.cmds); dur=f'{s.duration:.1f}s' if s.end_t else '?'
            print(f'  │  Session {i}: {n:3d} CMDs  dur={dur:6s}  ended_by={s.ended_by}')
            if n==0:
                print(f'  │    Ball lost entire window — no position data')
            else:
                print(f'  │    Distance : min={fmt(s.min_magnitude)}  final={fmt(s.final_magnitude)}  best_Q={fmt(s.best_quartile_dist)}')
                nv = s.near_visits(near_thresh)
                print(f'  │    Progress : slope={fmt_slope(s.convergence_slope)}  '
                      f'mono={fmt_pct(s.monotone_progress_ratio)}  '
                      f'near_center={nv}/{n}  '
                      f'tight_center={s.stable_visits}/{n}  (CMD ~1Hz)')
            # High-frequency summary from PID_SUMMARY lines (15Hz)
            if s.summaries:
                sv  = s.summary_valid_frames
                st2 = s.summary_total_frames
                sn  = s.summary_near_frames
                stf = s.summary_tight_frames
                dr  = s.summary_detection_rate
                sm  = s.summary_min_mag
                np2 = 100*sn/sv  if sv>0 else 0
                tp2 = 100*stf/sv if sv>0 else 0
                print(f'  │    Hi-freq  : '
                      f'det={fmt_pct(dr)}  '
                      f'near({near_thresh:.0%})={sn}/{sv}({np2:.0f}%)  '
                      f'tight({stable_thresh:.0%})={stf}/{sv}({tp2:.0f}%)  '
                      f'min={fmt(sm)}  ({len(s.summaries)}s)')
            nc=len(s.corrs)
            if nc>=2:
                print(f'  │    Servo(flex): track_err={fmt(s.servo_tracking_error)} rad  '
                      f'lag={fmt_pct(s.tracking_lag_ratio)}  '
                      f'osc={fmt_osc(s.oscillation_index)}  ({nc} CORRs)')
                print(f'  │    Servo(roll): track_err={fmt(s.roll_tracking_error)} rad  '
                      f'lag={fmt_pct(s.roll_tracking_lag)}  '
                      f'osc={fmt_osc(s.roll_oscillation_index)}  '
                      f'stall={fmt_pct(s.roll_stall_fraction)}')
            else:
                print(f'  │    Servo    : insufficient CORR data ({nc} steps)')
            det=f'ball_lost={s.ball_lost_count}(pid)'
            if s.sub_thresh_count:   det+=f'  sub_thresh={s.sub_thresh_count}'
            if s.containment_rejected: det+=f'  containment_rejected={s.containment_rejected}  ← raise containment_margin?'
            if s.centered_count:     det+=f'  CENTERED={s.centered_count}'
            print(f'  │    Detection: {det}')
        nb = pw.no_ball_count
        nb_str = f'  no_ball_events(total)={nb}' if nb else ''
        print(f'  └─ Best dist: {fmt(pw.best_magnitude)}  {stars(pw.best_magnitude)}{nb_str}')

    print()
    print('─'*76)
    print('  OVERALL SUMMARY')
    print('─'*76)
    sp=100*total_stable/total_cmds if total_cmds else 0
    total_near=sum(s.near_visits(near_thresh) for s in all_sess)
    np_pct=100*total_near/total_cmds if total_cmds else 0
    print(f'  Total poses          : {len(poses)}')
    print(f'  Total PID sessions   : {len(all_sess)}')
    print(f'  Total CMD readings   : {total_cmds}')
    print(f'  Near center visits   : {total_near}/{total_cmds} ({np_pct:.0f}%)  '
          f'(within {near_thresh:.0%} cup radius, CMD ~1Hz)')
    print(f'  Tight center visits  : {total_stable}/{total_cmds} ({sp:.0f}%)  '
          f'(within {stable_thresh:.0%} cup radius, CMD ~1Hz)')
    hf_valid=sum(s.summary_valid_frames for s in all_sess)
    hf_total=sum(s.summary_total_frames for s in all_sess)
    hf_near =sum(s.summary_near_frames  for s in all_sess)
    hf_tight=sum(s.summary_tight_frames for s in all_sess)
    if hf_total>0:
        print(f'  ── PID_SUMMARY stats (15Hz) ──')
        print(f'  Detection rate (15Hz): {hf_valid}/{hf_total} ({100*hf_valid/hf_total:.0f}%)')
        print(f'  Near center  (15Hz)  : {hf_near}/{hf_valid} '
              f'({100*hf_near/hf_valid if hf_valid else 0:.0f}%)  within {near_thresh:.0%}')
        print(f'  Tight center (15Hz)  : {hf_tight}/{hf_valid} '
              f'({100*hf_tight/hf_valid if hf_valid else 0:.0f}%)  within {stable_thresh:.0%}')
    else:
        print(f'  (No PID_SUMMARY lines — upgrade ball_balance_node for 15Hz stats)')
    print(f'  Total centered events: {total_centered}')
    print(f'  Total ball-lost      : {total_ball_lost}')
    if total_sub:  print(f'  Sub-threshold dets   : {total_sub}  (ball visible but below conf_threshold)')
    if total_cont: print(f'  Containment rejected : {total_cont}  (consider raising containment_margin)')
    print(f'  Global best dist     : {fmt(gbest)}  {stars(gbest)}  '
          f'(single closest frame — may be a flyby)')
    if gbest_pose: print(f'  Best pose (flyby)    : {gbest_pose}')
    print(f'  Best sustained dist  : {fmt(gbest_sust)}  {stars(gbest_sust)}  '
          f'(best-quartile — ball held near center)')
    if gbest_sust_pose: print(f'  Best pose (sustained): {gbest_sust_pose}')

    all_te =[s.servo_tracking_error for s in all_sess if s.servo_tracking_error is not None]
    all_lag=[s.tracking_lag_ratio   for s in all_sess if s.tracking_lag_ratio   is not None]
    all_osc=[s.oscillation_index    for s in all_sess if s.oscillation_index    is not None]
    if all_te:
        print(f'\n  Servo quality (all sessions):')
        print(f'    Mean tracking error : {fmt(sum(all_te)/len(all_te))} rad per CORR step')
        print(f'    Mean lag ratio      : {fmt_pct(sum(all_lag)/len(all_lag))}  (of cmd displacement achieved before next correction)')
        print(f'    Mean oscillation    : {fmt_osc(sum(all_osc)/len(all_osc))}  (fraction of steps reversing direction)')

    print()
    print('─'*76)
    print('  POSE RANKING  (best closest approach — lower = better)')
    print('─'*76)
    # Rank by best_sustained (best-quartile) — flyby-resistant
    ranked=sorted([(pw.name, pw.best_sustained, pw.best_magnitude)
                   for pw in poses if pw.best_sustained is not None],
                  key=lambda x:x[1])
    no_data=[pw.name for pw in poses if pw.best_sustained is None]
    print(f'  {"":2s}  {"Pose":<34s}  {"sustained":>9s}  {"flyby":>7s}')
    for rank,(name,sust,flyby) in enumerate(ranked,1):
        print(f'  {rank:2d}. {name:<34s}  {fmt(sust)}  {stars(sust)}  '
              f'(best={fmt(flyby)})')
    if no_data: print(f'  (no data: {", ".join(no_data)})')

    # Servo quality ranking — independent of ball position
    print()
    print('─'*76)
    print('  SERVO QUALITY RANKING  (higher score = better servo behavior)')
    print('  Measures: travel limit clamping, oscillation waste, tracking error')
    print('  Independent of ball position — compare with pose ranking to find root cause')
    print('─'*76)
    servo_ranked = sorted(
        [(pw.name, pw.servo_quality_score,
          pw.worst_travel_limit,
          sum(s.effective_correction_rate or 0 for s in pw.sessions)/max(len(pw.sessions),1),
          sum(abs(s.cumulative_flex_rad or 0) for s in pw.sessions))
         for pw in poses if pw.servo_quality_score is not None],
        key=lambda x: -x[1])  # higher score = better
    print(f'  {"":2s}  {"Pose":<34s}  {"score":>5s}  {"travel_lim":>10s}  {"eff_rate":>8s}  {"cumul_flex":>10s}')
    for rank,(name,score,tlf,ecr,cf) in enumerate(servo_ranked,1):
        tlf_str = f'{100*tlf:.0f}% clamped' if tlf is not None else '  ---  '
        # eff_rate is unreliable when clamping is significant — clamped steps
        # contribute zero to both numerator and denominator, hiding the real picture
        if tlf is not None and tlf > 0.20:
            ecr_str = f'{100*ecr:.0f}% (unreliable—clamped)'
        else:
            ecr_str = f'{100*ecr:.0f}%' if ecr else ' --- '
        cf_str  = f'{math.degrees(cf):+.1f}deg'
        print(f'  {rank:2d}. {name:<34s}  {score:.3f}  {tlf_str:>10s}  {ecr_str:>28s}  {cf_str:>10s}')
    print()
    print('  Interpretation:')
    print('    travel_lim  = fraction of steps where servo hit max_total_rad and could not move')
    print('    eff_rate    = net displacement / total attempted displacement')
    print('                  low  = corrections cancelling each other (oscillation waste)')
    print('                  100% = all corrections in same direction (monotone)')
    print('                  NOTE: marked unreliable when travel_lim > 20% because')
    print('                        clamped steps (Δ=0) are excluded from both numerator')
    print('                        and denominator, making 100% look perfect when the')
    print('                        servo was actually stuck against its travel limit')
    print('    cumul_flex  = total signed flex travel (large = needed more range than available)')
    print('  If servo ranking matches ball ranking: gains/travel limits are the issue.')
    print('  If they diverge: servo is misbehaving independently of the command quality.')

    # Cross-ranking analysis and diagnostics
    print()
    print('─'*76)
    print('  CROSS-RANKING DIAGNOSIS')
    print('  Compares ball ranking vs servo ranking to identify root cause per pose')
    print('─'*76)

    # Build lookup dicts for ranks
    ball_rank  = {name: r+1 for r,(name,_,_) in enumerate(ranked)}
    servo_rank = {name: r+1 for r,(name,*_) in enumerate(servo_ranked)}
    all_names  = [pw.name for pw in poses]

    diag_lines = []
    for pw in poses:
        br = ball_rank.get(pw.name)
        sr = servo_rank.get(pw.name)
        if br is None and sr is None: continue

        # Gather signals
        sust     = pw.best_sustained
        tlf      = pw.worst_travel_limit or 0
        nb       = pw.no_ball_count
        n_poses  = len(poses)
        sq       = pw.servo_quality_score or 0
        ecr_avg  = (sum(s.effective_correction_rate or 0 for s in pw.sessions) /
                    max(len(pw.sessions),1))
        cf       = sum(abs(s.cumulative_flex_rad or 0) for s in pw.sessions)
        sub      = sum(s.sub_thresh_count for s in pw.sessions)
        cont     = sum(s.containment_rejected for s in pw.sessions)

        # Derive rank difference (positive = servo better than ball result)
        rank_gap = (br or n_poses) - (sr or n_poses)  # + means servo better than ball

        causes = []
        fixes  = []

        # Rule 1: travel limit hit hard
        if tlf > 0.20:
            causes.append(f'Servo hit travel limit {100*tlf:.0f}% of steps')
            fixes.append('increase max_total_rad or adjust starting pose')

        # Rule 2: high cumulative flex (even if not clamped — needed lots of travel)
        elif math.degrees(cf) > 20:
            causes.append(f'Large cumulative flex {math.degrees(cf):.1f}deg — ball far from center')
            fixes.append('pose needs more pre-correction or kp reduction')

        # Rule 3: ball lost dominates
        if nb > 3:
            causes.append(f'Ball lost {nb} times — detection unreliable at this pose')
            fixes.append('raise containment_margin or retrain model for this angle')

        # Rule 4: sub-threshold detections
        if sub > 2:
            causes.append(f'Ball below conf_threshold {sub} times')
            fixes.append('lower conf_threshold (currently 0.30) to ~0.20')

        # Rule 5: oscillation waste — only flag when NOT primarily travel-limited
        # If clamped >20%, eff_rate is unreliable (clamped steps hidden from calculation)
        if ecr_avg < 0.25 and (sr or n_poses) > n_poses//2 and tlf < 0.20:
            causes.append(f'Low correction efficiency {100*ecr_avg:.0f}% — corrections cancelling out')
            fixes.append('reduce kp to cut overshoot, or add D-term damping')

        # Rule 6: servo good but ball bad — only meaningful if not travel-limited
        # (when clamped, the servo isn't truly 'executing well', it's stuck)
        if sr is not None and br is not None and rank_gap > 3 and sq > 0.5 and tlf < 0.15:
            causes.append('Servo executing well but ball not converging')
            fixes.append('check coordinate sign mapping for this pose direction')

        # Rule 7: ball good despite poor servo (cup geometry helping)
        if sr is not None and br is not None and rank_gap < -3 and (sust or 1) < 0.3:
            causes.append('Ball converging despite servo inefficiency — cup geometry helping')
            fixes.append('no fix needed; improve servo tuning for robustness')

        # Overall verdict
        if not causes:
            if sust is not None and sust < stable_thresh * 2:
                verdict = '✓ GOOD — ball close to center, servo executing well'
            elif sust is not None and sust < 0.4:
                verdict = '~ OK — making progress, minor issues'
            else:
                verdict = '? UNCLEAR — insufficient data or mixed signals'
        else:
            verdict = '✗ ISSUE'

        br_str = f'ball={br:2d}' if br else 'ball= -'
        sr_str = f'servo={sr:2d}' if sr else 'servo= -'
        print(f'\n  {pw.name}')
        print(f'    Ranks: {br_str}  {sr_str}  │  {verdict}')
        for cause, fix in zip(causes, fixes):
            print(f'    CAUSE : {cause}')
            print(f'    FIX   : {fix}')
        if not causes:
            print(f'    No specific issues detected')

    print()
    print('─'*76)
    print('  BALL LOST BY POSE')
    print('─'*76)
    lost_data=[(pw.name,pw.total_ball_lost,pw.total_containment_rejected) for pw in poses]
    mx=max(v for _,v,_ in lost_data) if lost_data else 1
    for name,lost,rej in sorted(lost_data,key=lambda x:-x[1]):
        b=bar(lost,max(mx,1))
        extra=f'  containment_rejected={rej}' if rej else ''
        print(f'  {name:<34s}  lost={lost:3d}  {b}{extra}')

    print()
    print('='*76)
    print()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('logfile')
    ap.add_argument('--stable-thresh',type=float,default=0.15,
                    help='Tight center threshold (default 0.15 = 15%% of cup radius)')
    ap.add_argument('--near-thresh',  type=float,default=0.30,
                    help='Near-center threshold (default 0.30 = 30%% of cup radius)')
    ap.add_argument('--centered-hold',type=float,default=2.0)
    args=ap.parse_args()
    events=parse_log(args.logfile)
    poses=build_sessions(events,args.stable_thresh)
    if not poses:
        # No pose markers found — log was assembled without pose_test output.
        # Create a single synthetic pose window spanning the whole file so
        # servo quality, ball proximity, and PID_SUMMARY stats are still reported.
        print('NOTE: No pose_test_node markers found — running in single-session mode.')
        print('      Servo, ball, and summary stats will still be reported.\n')
        synthetic = PoseWindow(name='(full log — no pose markers)', sent_t=0.0)
        # Inject a single PID session spanning all events
        sess = PIDSession(pose_name=synthetic.name, start_t=0.0)
        for t, tag, m in events:
            if tag == 'cmd':
                x,y=float(m.group(2)),float(m.group(3))
                mag=math.sqrt(x*x+y*y)
                sess.cmds.append((t,x,y,mag,mag<args.stable_thresh))
            elif tag == 'corr':
                sess.corrs.append(CorrStep(t=t,
                    flex_from=float(m.group(2)),flex_to=float(m.group(3)),
                    roll_from=float(m.group(5)),roll_to=float(m.group(6))))
            elif tag == 'ball_lost':   sess.ball_lost_count += 1
            elif tag == 'centered':    sess.centered_count  += 1
            elif tag == 'sub_thresh':  sess.sub_thresh_count += 1
            elif tag == 'containment': sess.containment_rejected += 1
            elif tag == 'no_ball':     synthetic.no_ball_count += 1
            elif tag == 'summary':
                try:
                    sess.summaries.append((
                        int(m.group(2)), int(m.group(3)),
                        int(m.group(4)), int(m.group(6)),
                        float(m.group(8))))
                except (ValueError, IndexError): pass
        if sess.corrs: sess.end_t = sess.corrs[-1].t
        elif sess.cmds: sess.end_t = sess.cmds[-1][0]
        sess.ended_by = 'end_of_log'
        synthetic.sessions.append(sess)
        poses = [synthetic]
    report(poses,args.stable_thresh,args.near_thresh,args.centered_hold,args.logfile)

if __name__=='__main__': main()
