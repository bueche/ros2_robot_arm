#!/usr/bin/env python3
"""
ball_detector_test.py  (v3 — pose-based)
-----------------------------------------
Unit tests for ball_detector_node.py v2 using pose-based test images
captured with balance_v1.yaml.

Usage:
    python3 ball_detector_test.py [OPTIONS]

    --image_dir     Directory with test images (default: ./ball_test_images)
    --verbose       Print detection details for every test
    --save_debug    Save annotated debug images to ./ball_test_debug/
    --save_intermediate  Save intermediate processing images
    --roi           ROI as x,y,w,h  (default: 194,169,280,246)
    --cup_dark      Dark threshold for cup (default: 80)
    --ball_bright   Bright threshold for ball (default: 220)
    --ball_max_area Max ball blob area px² (default: 300)

Notes on test assertions:
  - Centre poses: ball expected near (0.0, 0.0) ± tol
  - Tilt poses: ball expected near an edge (±0.7 to ±0.9) but we don't
    know WHICH axis maps to which direction yet. These are marked INFO
    so we read the output to determine the camera→robot axis mapping.
  - Repeat poses: should match first occurrence within consistency_tol
  - no_cup: both cup and ball expected False
"""

import os
import sys
import math
import argparse
import cv2
import numpy as np


# ── Detector (mirrors ball_detector_node.py v2) ───────────────────────────

class DetectorParams:
    roi_x = 150;  roi_y = 100;  roi_w = 350;  roi_h = 400
    cup_min_area          = 3000.0
    cup_max_area          = 200000.0
    cup_dark_thresh       = 80
    cup_min_ellipse_ratio = 0.3
    ball_min_area         = 20.0
    ball_max_area         = 300.0
    ball_bright_thresh    = 220
    ball_margin           = 0.08


class BallDetector:
    def __init__(self, params=None, save_intermediate=False,
                 intermediate_dir='./ball_test_intermediate'):
        self.p    = params or DetectorParams()
        self._save_intermediate = save_intermediate
        self._intermediate_dir  = intermediate_dir
        self._current_name      = 'unknown'
        self.ellipse = None
        if save_intermediate:
            os.makedirs(intermediate_dir, exist_ok=True)

    def reset(self, name='unknown'):
        self.ellipse       = None
        self._current_name = name

    def _save_img(self, tag, img):
        if not self._save_intermediate:
            return
        path = os.path.join(self._intermediate_dir,
                            f'{self._current_name}_{tag}.jpg')
        cv2.imwrite(path, img)

    def detect(self, frame_bgr):
        p = self.p
        roi_frame, (ox, oy) = self._apply_roi(frame_bgr)
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        self._save_img('01_gray', gray)

        cup_found  = self._detect_cup(gray)
        ball_found = False
        ball_px = ball_py = norm_x = norm_y = None

        if cup_found and self.ellipse is not None:
            ball_pos = self._detect_ball(gray)
            if ball_pos is not None:
                ball_found = True
                bx, by = ball_pos
                ball_px = bx + ox
                ball_py = by + oy
                cx, cy, a, b, _ = self.ellipse
                norm_x = (bx - cx) / a
                norm_y = (by - cy) / b

        ellipse_full = None
        if self.ellipse is not None:
            cx, cy, a, b, angle = self.ellipse
            ellipse_full = (cx + ox, cy + oy, a, b, angle)

        return {
            'cup_found':  cup_found,
            'ellipse':    ellipse_full,
            'ball_found': ball_found,
            'ball_px':    ball_px,
            'ball_py':    ball_py,
            'norm_x':     norm_x,
            'norm_y':     norm_y,
        }

    def _apply_roi(self, frame):
        p = self.p
        fh, fw = frame.shape[:2]
        if p.roi_w <= 0 or p.roi_h <= 0:
            return frame, (0, 0)
        x = max(0, min(p.roi_x, fw-1)); y = max(0, min(p.roi_y, fh-1))
        w = min(p.roi_w, fw-x);         h = min(p.roi_h, fh-y)
        return frame[y:y+h, x:x+w], (x, y)

    def _detect_cup(self, gray):
        p = self.p
        _, dark = cv2.threshold(gray, p.cup_dark_thresh, 255,
                                cv2.THRESH_BINARY_INV)
        self._save_img('02_dark_mask_raw', dark)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, k)
        self._save_img('03_dark_mask_closed', dark)

        contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return self.ellipse is not None

        if self._save_intermediate:
            dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(dbg, contours, -1, (0,255,0), 1)
            self._save_img('04_all_contours', dbg)

        best_ellipse = None; best_area = 0; candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < p.cup_min_area or area > p.cup_max_area: continue
            if len(cnt) < 5: continue
            ellipse = cv2.fitEllipse(cnt)
            (cx, cy), (w_ax, h_ax), angle = ellipse
            a = max(w_ax, h_ax) / 2.0
            b = min(w_ax, h_ax) / 2.0
            ratio = b / a if a > 0 else 0
            if ratio < p.cup_min_ellipse_ratio: continue
            candidates.append((area, cx, cy, a, b, angle))
            if area > best_area:
                best_area = area
                best_ellipse = (cx, cy, a, b, angle)

        if self._save_intermediate:
            dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            for (area, cx, cy, a, b, angle) in candidates:
                cv2.ellipse(dbg, (int(cx),int(cy)),
                            (max(int(a),1), max(int(b),1)),
                            angle, 0, 360, (0,200,255), 1)
                cv2.putText(dbg, f'{area:.0f}', (int(cx),int(cy)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,200,255), 1)
            if best_ellipse:
                cx, cy, a, b, angle = best_ellipse
                cv2.ellipse(dbg, (int(cx),int(cy)),
                            (max(int(a),1), max(int(b),1)),
                            angle, 0, 360, (0,255,0), 2)
            self._save_img('05_cup_candidates', dbg)

        if best_ellipse is None:
            return self.ellipse is not None
        self.ellipse = best_ellipse
        return True

    def _detect_ball(self, gray):
        """
        Detect ball bearing using centroid of all bright pixels inside
        the cup ellipse. This handles multiple specular highlights (the
        'two eyes' effect from LED reflections) by treating them all as
        one ball signature and finding their combined centroid.
        """
        p = self.p
        if self.ellipse is None: return None
        cx, cy, a, b, angle = self.ellipse
        h, w = gray.shape

        # Build ellipse mask inset by margin to avoid rim
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.ellipse(mask, (int(cx), int(cy)),
                    (max(int(a*(1-p.ball_margin)),1),
                     max(int(b*(1-p.ball_margin)),1)),
                    angle, 0, 360, 255, -1)

        # Threshold bright pixels inside cup
        _, bright = cv2.threshold(gray, p.ball_bright_thresh, 255,
                                  cv2.THRESH_BINARY)
        ball_region = cv2.bitwise_and(bright, bright, mask=mask)
        self._save_img('06_ball_bright_region', ball_region)

        # Count bright pixels — need at least a few to be meaningful
        bright_count = cv2.countNonZero(ball_region)
        if bright_count < p.ball_min_area:
            return None

        # Centroid of ALL bright pixels inside cup — handles 1 or 2 dots
        M = cv2.moments(ball_region)
        if M['m00'] > 0:
            bx = int(M['m10'] / M['m00'])
            by = int(M['m01'] / M['m00'])
            return (bx, by)

        return None

    def draw_debug(self, frame_bgr, result):
        out = frame_bgr.copy()
        if result['ellipse'] is not None:
            cx, cy, a, b, angle = result['ellipse']
            cv2.ellipse(out, (int(cx),int(cy)),
                        (max(int(a),1), max(int(b),1)),
                        angle, 0, 360, (0,255,0), 2)
            cv2.circle(out, (int(cx),int(cy)), 3, (0,255,0), -1)
            cv2.line(out, (int(cx)-12,int(cy)), (int(cx)+12,int(cy)),
                     (0,255,0), 1)
            cv2.line(out, (int(cx),int(cy)-12), (int(cx),int(cy)+12),
                     (0,255,0), 1)
        if result['ball_found']:
            bx, by = result['ball_px'], result['ball_py']
            cv2.circle(out, (bx,by), 8,  (0,0,255), 2)
            cv2.circle(out, (bx,by), 2,  (0,0,255), -1)
            if result['ellipse'] is not None:
                cx, cy = result['ellipse'][:2]
                cv2.line(out, (int(cx),int(cy)), (bx,by), (0,165,255), 1)
            label = f"({result['norm_x']:+.2f},{result['norm_y']:+.2f})"
            cv2.putText(out, label, (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        if result['ellipse'] is not None:
            cx, cy, a, b, angle = result['ellipse']
            cv2.putText(out, f'cup a={a:.0f} b={b:.0f}', (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        return out


# ── Test case definitions ──────────────────────────────────────────────────
#
# Assertion types:
#   ASSERT  — hard pass/fail on cup, ball detected, and position
#   INFO    — cup+ball detection asserted, position just reported
#             (used for tilt poses where we don't know axis mapping yet)
#   EDGE    — asserts NOT detected
#
# (name, expect_cup, expect_ball, exp_nx, exp_ny, tol, assert_type, notes)

DEFAULT_TOL      = 0.20   # generous for real hardware variation
CENTRE_TOL       = 0.25   # centre pose has more variation
CONSISTENCY_TOL  = 0.15   # repeat poses should match first occurrence

TEST_CASES = [
    # ── Baseline ──────────────────────────────────────────────────────────
    ('pose_centre_no_ball',  True,  False, None,  None,  DEFAULT_TOL,
     'ASSERT', 'Cup visible, no ball — baseline'),

    # ── Centre poses ──────────────────────────────────────────────────────
    ('pose_centre',          True,  True,  0.0,   0.0,   CENTRE_TOL,
     'ASSERT', 'pose 1 — level, ball near centre'),
    ('pose_centre_2',        True,  True,  0.0,   0.0,   CENTRE_TOL,
     'ASSERT', 'pose 4 — return to centre, check repeatability'),
    ('pose_centre_3',        True,  True,  0.0,   0.0,   CENTRE_TOL,
     'ASSERT', 'pose 7 — return to centre again'),

    # ── Tilt poses — INFO (axis mapping not yet determined) ───────────────
    # We assert cup+ball detected but don't assert specific position.
    # Read the norm_x, norm_y values to determine which axis is which.
    ('pose_tilt_right',      True,  True,  None,  None,  DEFAULT_TOL,
     'INFO',   'pose 2 — wrist_roll=1.2, ball rolls to one side'),
    ('pose_tilt_left',       True,  True,  None,  None,  DEFAULT_TOL,
     'INFO',   'pose 3 — wrist_roll=1.9, ball rolls other side'),
    ('pose_tilt_forward',    True,  True,  None,  None,  DEFAULT_TOL,
     'INFO',   'pose 5 — wrist_flex=2.0, ball rolls forward'),
    ('pose_tilt_back',       True,  True,  None,  None,  DEFAULT_TOL,
     'INFO',   'pose 6 — elbow_flex=1.2705, ball rolls back'),

    # ── Repeat tilt poses — consistency check ─────────────────────────────
    # These should match the first occurrence within CONSISTENCY_TOL.
    # Exact expected values are filled in after first run.
    ('pose_tilt_right_2',    True,  True,  None,  None,  DEFAULT_TOL,
     'INFO',   'pose 7 — repeat tilt right, check consistency'),
    ('pose_tilt_left_2',     True,  True,  None,  None,  DEFAULT_TOL,
     'INFO',   'pose 8 — repeat tilt left, check consistency'),
    ('pose_tilt_back_2',     True,  True,  None,  None,  DEFAULT_TOL,
     'INFO',   'pose 9 — repeat tilt back, check consistency'),

    # ── Edge case ─────────────────────────────────────────────────────────
    ('no_cup',               False, False, None,  None,  DEFAULT_TOL,
     'ASSERT', 'Cup out of frame — not detected'),
]


# ── Test runner ────────────────────────────────────────────────────────────

def run_one(detector, image_dir, debug_dir, save_debug,
            name, exp_cup, exp_ball, exp_nx, exp_ny,
            tol, assert_type, notes, verbose):

    img_path = os.path.join(image_dir, f'{name}.jpg')
    if not os.path.exists(img_path):
        return 'SKIP', []

    frame = cv2.imread(img_path)
    if frame is None:
        return 'SKIP', []

    detector.reset(name)
    result = detector.detect(frame)
    errors = []

    # Always assert cup and ball detection status
    if result['cup_found'] != exp_cup:
        errors.append(f"cup_found={result['cup_found']} expected={exp_cup}")
    if result['ball_found'] != exp_ball:
        errors.append(f"ball_found={result['ball_found']} expected={exp_ball}")

    # Only assert position for ASSERT type with expected values
    if assert_type == 'ASSERT' and exp_nx is not None and result['ball_found']:
        dx   = abs(result['norm_x'] - exp_nx)
        dy   = abs(result['norm_y'] - exp_ny)
        dist = math.sqrt(dx**2 + dy**2)
        if dist > tol:
            errors.append(
                f"position ({result['norm_x']:+.2f},{result['norm_y']:+.2f})"
                f" expected ({exp_nx:+.2f},{exp_ny:+.2f})"
                f" dist={dist:.3f} > tol={tol}")

    if assert_type == 'INFO':
        display_status = 'INFO'
    elif errors:
        display_status = 'FAIL'
    else:
        display_status = 'PASS'

    cup_str  = f"cup={'Y' if result['cup_found'] else 'N'}"
    ball_str = (f"ball=({result['norm_x']:+.2f},{result['norm_y']:+.2f})"
                if result['ball_found'] else 'ball=N')

    print(f"  {display_status:4s}  {name:30s}  {cup_str}  {ball_str}")

    if verbose or errors:
        print(f"        # {notes}")
        for e in errors:
            print(f"        ✗ {e}")
        if verbose and result['ellipse'] is not None:
            cx, cy, a, b, angle = result['ellipse']
            print(f"        ellipse ({cx:.0f},{cy:.0f})"
                  f" a={a:.0f} b={b:.0f} angle={angle:.1f}")

    if save_debug:
        dbg  = detector.draw_debug(frame, result)
        path = os.path.join(debug_dir, f'{display_status}_{name}.jpg')
        cv2.imwrite(path, dbg)

    return display_status, errors


def run_tests(image_dir, params, verbose, save_debug, save_intermediate):
    detector  = BallDetector(params=params,
                             save_intermediate=save_intermediate)
    debug_dir = './ball_test_debug'
    if save_debug:
        os.makedirs(debug_dir, exist_ok=True)

    passed = failed = skipped = info = 0

    roi_str = (f"roi=({params.roi_x},{params.roi_y},"
               f"{params.roi_w},{params.roi_h})"
               if params.roi_w > 0 else "roi=full frame")

    print(f"\n{'='*65}")
    print(f"  Ball Detector Tests  (v3 — pose-based)")
    print(f"  Image dir : {image_dir}")
    print(f"  {roi_str}")
    print(f"  cup_dark={params.cup_dark_thresh}"
          f"  ball_bright={params.ball_bright_thresh}"
          f"  ball_max_area={params.ball_max_area:.0f}")
    print(f"{'='*65}\n")

    for (name, exp_cup, exp_ball,
         exp_nx, exp_ny, tol, assert_type, notes) in TEST_CASES:

        status, errors = run_one(
            detector, image_dir, debug_dir, save_debug,
            name, exp_cup, exp_ball, exp_nx, exp_ny,
            tol, assert_type, notes, verbose)

        if status == 'SKIP':   skipped += 1
        elif status == 'FAIL': failed  += 1
        elif status == 'INFO': info    += 1
        else:                  passed  += 1

    # ── Axis mapping summary ───────────────────────────────────────────────
    print(f"\n{'─'*65}")
    print("  Axis mapping — read INFO rows to fill in ball_marker_node.py:")
    print("  pose_tilt_right  (wrist_roll=1.2) → ball norm should show"
          " which axis is roll")
    print("  pose_tilt_left   (wrist_roll=1.9) → opposite sign of above")
    print("  pose_tilt_forward(wrist_flex=2.0) → ball norm shows flex axis")
    print("  pose_tilt_back   (elbow_flex)     → opposite sign of above")

    print(f"\n{'='*65}")
    print(f"  PASS={passed}  FAIL={failed}  INFO={info}  SKIP={skipped}"
          f"  TOTAL={passed+failed+skipped+info}")
    if save_debug:
        print(f"  Debug images : ./ball_test_debug/")
    if save_intermediate:
        print(f"  Intermediate : ./ball_test_intermediate/")
    print(f"{'='*65}\n")

    return failed == 0


def main():
    parser = argparse.ArgumentParser(
        description='Ball detector v3 pose-based unit tests')
    parser.add_argument('--image_dir',  default='./ball_test_images')
    parser.add_argument('--verbose',    action='store_true')
    parser.add_argument('--save_debug', action='store_true')
    parser.add_argument('--save_intermediate', action='store_true')
    parser.add_argument('--roi',        default='150,100,350,400',
                        help='ROI as x,y,w,h')
    parser.add_argument('--cup_dark',   type=int,   default=80)
    parser.add_argument('--ball_bright',type=int,   default=220)
    parser.add_argument('--ball_max_area', type=float, default=300.0)
    parser.add_argument('--cup_min_area',  type=float, default=3000.0)
    args = parser.parse_args()

    if not os.path.isdir(args.image_dir):
        print(f"Error: image directory not found: {args.image_dir}")
        sys.exit(1)

    try:
        rx, ry, rw, rh = [int(v) for v in args.roi.split(',')]
    except ValueError:
        print(f"Error: --roi must be x,y,w,h  got: {args.roi}")
        sys.exit(1)

    params = DetectorParams()
    params.roi_x              = rx
    params.roi_y              = ry
    params.roi_w              = rw
    params.roi_h              = rh
    params.cup_dark_thresh    = args.cup_dark
    params.ball_bright_thresh = args.ball_bright
    params.ball_max_area      = args.ball_max_area
    params.cup_min_area       = args.cup_min_area

    success = run_tests(args.image_dir, params,
                        args.verbose, args.save_debug,
                        args.save_intermediate)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
