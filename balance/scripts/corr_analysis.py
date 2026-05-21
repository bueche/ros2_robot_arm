#!/usr/bin/env python3
"""
corr_analysis.py — Inspect wrist_balance_controller CORR step quality.

For each balance session finds:
  - Balance ENABLED tag (session start)
  - Start pose flex/roll values
  - Each CORR step: target change, achieved change, ratio, running cumulative

Usage:
    python3 corr_analysis.py <logfile> [--pose "pose 5"] [--max-steps 30]

Output columns:
    step  flex_from  flex_to  tgt_deg  act_deg  ratio%  cumul_deg  interval_ms
    tgt_deg     = commanded flex change (flex_to - flex_from) in degrees
    act_deg     = actually achieved (next flex_from - flex_from) in degrees
    ratio%      = act/tgt * 100  (>100% = overshot, <100% = undershot)
    cumul_deg   = running total of achieved flex from session start
    interval_ms = time between this CORR and next CORR (servo response window)
"""

import re
import sys
import argparse
from dataclasses import dataclass
from typing import Optional, List


RE_ENABLED = re.compile(
    r'\[(\d+\.\d+)\] \[wrist_balance_controller\]: Balance ENABLED')
RE_START   = re.compile(
    r'\[(\d+\.\d+)\] \[wrist_balance_controller\]: Start pose: '
    r'flex=([\d.]+)\s+roll=([\d.]+)')
RE_CORR    = re.compile(
    r'\[(\d+\.\d+)\] \[wrist_balance_controller\]: CORR \| '
    r'flex ([\d.]+)→([\d.]+) \(Δ([+-][\d.]+)deg\)  '
    r'roll ([\d.]+)→([\d.]+) \(Δ([+-][\d.]+)deg\)  '
    r'cumul: flex=([+-][\d.]+)deg roll=([+-][\d.]+)deg')
RE_DISABLED= re.compile(
    r'\[(\d+\.\d+)\] \[wrist_balance_controller\]: Balance DISABLED')
RE_POSE    = re.compile(
    r'\[(\d+\.\d+)\] \[pose_test_node\]: → Sent: (.+)')
RE_CMD     = re.compile(
    r'\[(\d+\.\d+)\] \[ball_balance_node\]: CMD \| '
    r'ball=\(([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)')


@dataclass
class CorrStep:
    t:          float
    flex_from:  float
    flex_to:    float
    roll_from:  float
    roll_to:    float
    tgt_flex_deg: float   # commanded delta in degrees
    tgt_roll_deg: float
    cumul_flex_deg: float  # cumulative from wrist controller
    cumul_roll_deg: float


@dataclass
class Session:
    pose_name:  str
    enabled_t:  float
    start_flex: Optional[float]
    start_roll: Optional[float]
    corrs:      List[CorrStep]
    disabled_t: Optional[float]


def parse(path):
    events = []
    with open(path, 'r', errors='replace') as f:
        for line in f:
            for pat, tag in [
                (RE_ENABLED,  'enabled'),
                (RE_START,    'start'),
                (RE_CORR,     'corr'),
                (RE_DISABLED, 'disabled'),
                (RE_POSE,     'pose'),
                (RE_CMD,      'cmd'),
            ]:
                m = pat.search(line)
                if m:
                    events.append((float(m.group(1)), tag, m))
                    break
    events.sort(key=lambda e: e[0])
    return events


def build_sessions(events):
    sessions = []
    cur_sess = None
    cur_pose = '(unknown)'

    for t, tag, m in events:
        if tag == 'pose':
            cur_pose = m.group(2).strip()

        elif tag == 'enabled':
            cur_sess = Session(
                pose_name=cur_pose,
                enabled_t=t,
                start_flex=None,
                start_roll=None,
                corrs=[],
                disabled_t=None)
            sessions.append(cur_sess)

        elif tag == 'start' and cur_sess:
            cur_sess.start_flex = float(m.group(2))
            cur_sess.start_roll = float(m.group(3))

        elif tag == 'corr' and cur_sess:
            cur_sess.corrs.append(CorrStep(
                t=t,
                flex_from=float(m.group(2)),
                flex_to=float(m.group(3)),
                roll_from=float(m.group(5)),
                roll_to=float(m.group(6)),
                tgt_flex_deg=float(m.group(4)),
                tgt_roll_deg=float(m.group(7)),
                cumul_flex_deg=float(m.group(8)),
                cumul_roll_deg=float(m.group(9))))

        elif tag == 'disabled' and cur_sess:
            cur_sess.disabled_t = t
            cur_sess = None

    return sessions


def analyse_session(sess, max_steps):
    corrs = sess.corrs
    n = len(corrs)

    print(f'\n{"="*80}')
    print(f'  POSE : {sess.pose_name}')
    print(f'  Start: flex={sess.start_flex:.4f} rad  roll={sess.start_roll:.4f} rad')
    dur = (sess.disabled_t - sess.enabled_t) if sess.disabled_t else None
    print(f'  Duration: {f"{dur:.1f}s" if dur else "?"}  '
          f'  CORR steps: {n}')
    print(f'{"="*80}')

    if n < 2:
        print('  (insufficient CORR data)')
        return

    print(f'  {"step":>4}  '
          f'{"── FLEX ──────────────────────────────────────":>44}  '
          f'{"── ROLL ──────────────────────────────────────":>44}  '
          f'{"cumul(f,r)":>14}  {"intv":>6}')
    print(f'  {"":>4}  '
          f'{"from":>8} {"to":>8}  {"tgt_deg":>8} {"act_deg":>8} {"ratio%":>7}  '
          f'{"from":>8} {"to":>8}  {"tgt_deg":>8} {"act_deg":>8} {"ratio%":>7}  '
          f'{"":>14}  {"":>6}')
    print('  ' + '-'*128)

    achieved_sum = 0.0
    limit = min(n, max_steps) if max_steps else n

    for i in range(limit):
        c = corrs[i]
        tgt_rad = c.flex_to - c.flex_from
        tgt_deg = tgt_rad * 57.296

        if i + 1 < n:
            act_rad = corrs[i+1].flex_from - c.flex_from
            interval_ms = (corrs[i+1].t - c.t) * 1000
        else:
            act_rad = None
            interval_ms = None

        act_deg = act_rad * 57.296 if act_rad is not None else None
        achieved_sum += act_rad if act_rad is not None else 0

        if act_rad is not None and abs(tgt_rad) > 1e-6:
            ratio = act_rad / tgt_rad * 100
            ratio_str = f'{ratio:+.0f}%'
        else:
            ratio_str = '  N/A'

        act_str      = f'{act_deg:+.3f}' if act_deg is not None else '   ---'
        interval_str = f'{interval_ms:.0f}ms' if interval_ms is not None else '  ---'
        cumul_str    = f'{c.cumul_flex_deg:+.2f}'


        # Flag definitions:
        # HOLD        : |tgt| < 0.003 rad (0.17 deg) — commanded tiny move, no judgment
        # STALL       : meaningful command (>=0.17 deg) but achieved <15% — servo failed to move
        # SM_OVERSHOOT: achieved 115-200% of command in correct direction — mild momentum carry
        # LG_OVERSHOOT: achieved >200% of command in correct direction — strong momentum/runaway
        # REVERSAL    : achieved motion in opposite direction to command — fighting prior step
        HOLD_THRESH = 0.003  # rad
        flag = ''
        if act_rad is not None:
            if abs(tgt_rad) < HOLD_THRESH:
                # HOLD command — servo should stay put
                if act_rad is not None and abs(act_rad) > HOLD_THRESH * 3:
                    flag = ' ← HOLD_DRIFT'  # commanded hold but drifted significantly
                else:
                    flag = ' ← HOLD'  # commanded hold, stayed put (ok)
            elif act_rad * tgt_rad < 0:
                flag = ' ← REVERSAL'
            else:
                r = abs(act_rad) / abs(tgt_rad) if abs(tgt_rad) > 1e-6 else 0
                if r < 0.15:   flag = ' ← STALL'
                elif r > 2.0:  flag = ' ← LG_OVERSHOOT'
                elif r > 1.15: flag = ' ← SM_OVERSHOOT'

        # Roll axis achieved
        if i + 1 < n:
            roll_act_rad = corrs[i+1].roll_from - c.roll_from
            roll_tgt_rad = c.roll_to - c.roll_from
            roll_act_deg = roll_act_rad * 57.296
            roll_tgt_deg = c.tgt_roll_deg
            if abs(roll_tgt_rad) > 1e-6:
                roll_ratio = roll_act_rad / roll_tgt_rad * 100
                roll_ratio_str = f'{roll_ratio:+.0f}%'
            else:
                roll_ratio_str = '  N/A'
            roll_act_str = f'{roll_act_deg:+.3f}'
            roll_tgt_str = f'{roll_tgt_deg:+.3f}'
        else:
            roll_act_str = roll_tgt_str = roll_ratio_str = '  ---'

        print(f'  {i+1:>4}  '
              f'flex: {c.flex_from:>8.4f}→{c.flex_to:>8.4f} '
              f'tgt={tgt_deg:>+7.3f} act={act_str:>7} {ratio_str:>7}  '
              f'roll: {c.roll_from:>8.4f}→{c.roll_to:>8.4f} '
              f'tgt={roll_tgt_str:>7} act={roll_act_str:>7} {roll_ratio_str:>7}  '
              f'cumul=({cumul_str},{c.cumul_roll_deg:+.2f})  '
              f'{interval_str:>7}{flag}')

    if limit < n:
        print(f'  ... ({n - limit} more steps not shown, use --max-steps 0 for all)')

    # Summary stats
    pairs_flex = []
    pairs_roll = []
    for i in range(min(n-1, limit)):
        c = corrs[i]
        flex_tgt = c.flex_to   - c.flex_from
        flex_act = corrs[i+1].flex_from - c.flex_from
        roll_tgt = c.roll_to   - c.roll_from
        roll_act = corrs[i+1].roll_from - c.roll_from
        pairs_flex.append((flex_tgt, flex_act))
        pairs_roll.append((roll_tgt, roll_act))
    pairs = pairs_flex  # keep for backward compat

    if pairs:
        # HOLD: |tgt| < 0.003 rad (~0.17 deg)  SMALL: 0.003-0.010  LARGE: >=0.010
        hold  = [(t,a) for t,a in pairs if abs(t) < 0.003]
        small = [(t,a) for t,a in pairs if 0.003 <= abs(t) < 0.010]
        large = [(t,a) for t,a in pairs if abs(t) >= 0.010]

        def stats(grp, label):
            if not grp: return
            ratios = [abs(a)/abs(t)*100 for t,a in grp if abs(t)>1e-6]
            stalls = sum(1 for t,a in grp if abs(t)>1e-6 and abs(a/t)<0.15)
            wrong  = sum(1 for t,a in grp if t*a<0 and abs(t)>1e-6)
            avg_tgt= sum(abs(t)*57.3 for t,a in grp)/len(grp)
            avg_act= sum(abs(a)*57.3 for t,a in grp)/len(grp)
            avg_rat= sum(ratios)/len(ratios) if ratios else 0
            print(f'\n  {label} steps (n={len(grp)}):')
            print(f'    avg commanded : {avg_tgt:.3f} deg')
            print(f'    avg achieved  : {avg_act:.3f} deg')
            print(f'    avg ratio     : {avg_rat:.0f}%')
            print(f'    stalls (<15%) : {stalls}  ({100*stalls/len(grp):.0f}%)')
            print(f'    wrong dir     : {wrong}  ({100*wrong/len(grp):.0f}%)')

        print(f'  Step breakdown:  hold={len(hold)}  small={len(small)}  large={len(large)}')
        print('\n  ── FLEX AXIS ──')
        stats(large, 'LARGE (>=0.57 deg, meaningful correction)')
        stats(small, 'SMALL (0.17-0.57 deg, fine correction)')

        # Roll axis stats
        roll_hold  = [(t,a) for t,a in pairs_roll if abs(t) < 0.003]
        roll_small = [(t,a) for t,a in pairs_roll if 0.003 <= abs(t) < 0.010]
        roll_large = [(t,a) for t,a in pairs_roll if abs(t) >= 0.010]
        print('\n  ── ROLL AXIS ──')
        stats(roll_large, 'LARGE (>=0.57 deg, meaningful correction)')
        stats(roll_small, 'SMALL (0.17-0.57 deg, fine correction)')
        if roll_hold:
            HOLD_THRESH = 0.003
            drifted_r = [(t,a) for t,a in roll_hold if abs(a) > HOLD_THRESH * 3]
            held_r    = [(t,a) for t,a in roll_hold if abs(a) <= HOLD_THRESH * 3]
            print(f'\n  ROLL HOLD (<0.17 deg) steps: {len(roll_hold)}  '
                  f'held={len(held_r)}  drift={len(drifted_r)}')
            if drifted_r:
                avg_dr = sum(abs(a)*57.3 for t,a in drifted_r)/len(drifted_r)
                print(f'    HOLD_DRIFT: {len(drifted_r)} steps moved avg {avg_dr:.3f} deg')
        if hold:
            HOLD_THRESH = 0.003
            drifted = [(t,a) for t,a in hold if abs(a) > HOLD_THRESH * 3]
            held    = [(t,a) for t,a in hold if abs(a) <= HOLD_THRESH * 3]
            avg_act = sum(abs(a)*57.3 for t,a in hold)/len(hold)
            print(f'\n  HOLD (<0.17 deg) steps: {len(hold)}  '
                  f'held={len(held)}  drift={len(drifted)}')
            if drifted:
                avg_drift = sum(abs(a)*57.3 for t,a in drifted)/len(drifted)
                print(f'    HOLD_DRIFT: {len(drifted)} steps moved avg {avg_drift:.3f} deg '
                      f'despite zero/tiny command — servo still in motion from prior step')

        # Interval analysis
        intervals = [(corrs[i+1].t - corrs[i].t)*1000
                     for i in range(min(n-1, limit))]
        if intervals:
            print(f'\n  Interval between CORRs:')
            print(f'    min={min(intervals):.0f}ms  '
                  f'max={max(intervals):.0f}ms  '
                  f'avg={sum(intervals)/len(intervals):.0f}ms')
            print(f'    Note: move_duration=100ms, correction_hz=5Hz (200ms interval)')
            print(f'    If avg interval << 200ms the wrist controller is running faster than expected.')


def main():
    ap = argparse.ArgumentParser(description='Analyse CORR step quality')
    ap.add_argument('logfile')
    ap.add_argument('--pose', default=None,
                    help='Filter to poses matching this string (e.g. "forward")')
    ap.add_argument('--max-steps', type=int, default=20,
                    help='Max CORR steps to print per session (0=all, default 20)')
    args = ap.parse_args()

    events   = parse(args.logfile)
    sessions = build_sessions(events)

    if not sessions:
        print('No Balance ENABLED events found.')
        sys.exit(1)

    filtered = sessions
    if args.pose:
        filtered = [s for s in sessions
                    if args.pose.lower() in s.pose_name.lower()]
        if not filtered:
            print(f'No sessions matching "{args.pose}"')
            print(f'Available poses: {sorted({s.pose_name for s in sessions})}')
            sys.exit(1)

    print(f'\nCORR STEP ANALYSIS')
    print(f'File : {args.logfile}')
    print(f'Filter: {args.pose or "(all poses)"}')
    print(f'Sessions found: {len(filtered)}')

    for sess in filtered:
        analyse_session(sess, args.max_steps)

    print()


if __name__ == '__main__':
    main()
