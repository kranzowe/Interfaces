#!/usr/bin/env python3
"""
Obstacle-avoidance node for the wheeled rover using 2D planar LaserScan.

Extends LidarBugNode (gap-following course navigation) with a reactive
polar-sector gap-finder that detects obstacles in the forward FOV and
temporarily overrides steering to navigate around them.

Why this differs from the quadruped 3D-lidar version:
  - Input is a 2D LaserScan (ranges only), not a 3D elevation / height map,
    so height and slope detection are not available.  An obstacle is simply
    anything whose range is below a threshold.
  - The rover uses Ackermann steering — no holonomic strafe (vy).
  - Output is a PWM servo delta, not a yaw setpoint fed to a legged gait.
"""

import time
import math
import numpy as np

import rclpy
from geometry_msgs.msg import Twist

from lidar_bug import LidarBugNode          # reuse all scan pre-processing

# ── Tuning constants ─────────────────────────────────────────────────────────
FORWARD_FOV_DEG   = 60.0   # half-angle of the forward cone inspected (±60° = 120° total)
CENTRE_HALF_DEG   = 20.0   # inner corridor used for the danger / clear-exit checks
DANGER_DIST_M     = 0.8    # enter avoidance when closest reading in centre < this (m)
CLEAR_DIST_M      = 1.2    # exit avoidance once centre corridor clears beyond this (m)
OBS_DIST_M        = 1.0    # a sector is "blocked" when its minimum range < this (m)
POLAR_SECTORS     = 21     # number of angular slices across the ±FOV cone
MIN_GAP_DEG       = 12.0   # gap must span at least this many degrees to be viable
STEER_GAIN        = 12.0   # PWM units per degree of gap-centre angle
AVOID_SPEED_SCALE = 0.55   # fraction of normal ol_speed applied during avoidance
AVOID_TIMEOUT_S   = 5.0    # hard exit from avoidance even if obstacle still present
# ─────────────────────────────────────────────────────────────────────────────


class LidarObsAvoidNode(LidarBugNode):
    """
    Reactive obstacle avoidance layer on top of the gap-following controller.

    In normal mode the parent class (LidarBugNode) drives the rover exactly
    as before — integrating the scan, finding the widest open arc, and using
    a sliding-mode controller to steer toward it.

    When an obstacle is detected in the forward corridor this node takes over
    the steering output for as long as needed, then hands control back to the
    parent seamlessly.
    """

    def __init__(self):
        super().__init__()
        self._avoid_active  = False    # True while actively dodging an obstacle
        self._avoid_started = 0.0      # monotonic timestamp when avoidance began
        self._avoid_side    = 0        # +1 = gap is on the left, -1 = right
        self._avoid_steer   = 0.0      # PWM delta to add to neutral_steer

    # ── Scan callback override ────────────────────────────────────────────────
    def scan_cb(self, msg):
        # Always run the parent first so it computes optimal_angle and publishes
        # integral_scan / optimal_angle topics at the same cadence as before.
        super().scan_cb(msg)

        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return

        # Build an angle array centred on straight-ahead.
        # Convention (RPLIDAR default with angle_min = -π):
        #   index 0       → -180° (directly behind)
        #   index n // 2  →    0° (straight ahead)
        #   positive °    → CCW  → left of rover
        #   negative °    → CW   → right of rover
        angles_deg = (np.arange(n) - n // 2) * (360.0 / n)

        # ── Step 1: Danger check ───────────────────────────────────────────────
        # Sample only the narrow centre corridor (±CENTRE_HALF_DEG).
        # If the closest valid reading is inside DANGER_DIST_M we must avoid.
        centre_mask   = np.abs(angles_deg) <= CENTRE_HALF_DEG
        centre_r      = ranges[centre_mask]
        centre_finite = centre_r[np.isfinite(centre_r)]
        min_centre    = float(np.min(centre_finite)) if centre_finite.size > 0 else math.inf

        if not self._avoid_active and min_centre > DANGER_DIST_M:
            # Path ahead is clear — normal operation, reset steer delta.
            self._avoid_steer = 0.0
            return

        # ── Step 2: Sectorise the forward FOV ────────────────────────────────
        # Restrict to the wider ±FORWARD_FOV_DEG cone and divide it into
        # POLAR_SECTORS equal angular slices.  A sector is marked "blocked"
        # when its closest reading is below OBS_DIST_M (gives a margin beyond
        # DANGER_DIST_M so the path chosen is genuinely safe to enter).
        fov_mask     = np.abs(angles_deg) <= FORWARD_FOV_DEG
        fov_angles   = angles_deg[fov_mask]
        fov_ranges   = ranges[fov_mask]

        edges        = np.linspace(-FORWARD_FOV_DEG, FORWARD_FOV_DEG, POLAR_SECTORS + 1)
        centers      = 0.5 * (edges[:-1] + edges[1:])
        sector_width = edges[1] - edges[0]

        blocked = np.zeros(POLAR_SECTORS, dtype=bool)
        for s in range(POLAR_SECTORS):
            in_sector = (fov_angles >= edges[s]) & (fov_angles < edges[s + 1])
            sector_r  = fov_ranges[in_sector]
            finite_r  = sector_r[np.isfinite(sector_r)]
            if finite_r.size > 0 and float(np.min(finite_r)) < OBS_DIST_M:
                blocked[s] = True

        # ── Step 3: Find all gaps ─────────────────────────────────────────────
        # A gap is a maximal run of consecutive unblocked sectors.
        gaps = []
        i = 0
        while i < POLAR_SECTORS:
            if not blocked[i]:
                j = i
                while j < POLAR_SECTORS and not blocked[j]:
                    j += 1
                gaps.append((i, j - 1))   # inclusive start and end indices
                i = j
            else:
                i += 1

        # ── Step 4: Select the best gap ───────────────────────────────────────
        if not gaps:
            # The full forward FOV is blocked — hold the last chosen side and
            # apply a small fixed steer nudge (no strafe available on a rover).
            side = self._avoid_side if self._avoid_side != 0 else 1
            self._enter_avoid(side)
            self._avoid_steer = side * STEER_GAIN * 15.0   # gentle nudge (≈15°)
            return

        min_gap_sectors = max(1, round(MIN_GAP_DEG / sector_width))
        gap_center_deg  = self._pick_best_gap(gaps, centers, min_gap_sectors)

        # The side of the gap tells us which way to dodge.
        side = 1 if gap_center_deg >= 0 else -1
        self._enter_avoid(side)

        # Translate gap-centre angle to a PWM steering delta.
        # gap_center_deg > 0 (gap on left)  → _avoid_steer > 0 → angular.z > neutral → turn left
        # gap_center_deg < 0 (gap on right) → _avoid_steer < 0 → angular.z < neutral → turn right
        self._avoid_steer = STEER_GAIN * gap_center_deg

        # ── Step 5: Exit check ────────────────────────────────────────────────
        # Leave avoidance mode once the centre corridor is genuinely clear
        # (hysteresis: CLEAR_DIST_M > DANGER_DIST_M prevents rapid toggling)
        # or after a hard timeout so we never get stuck in avoidance forever.
        timed_out = (time.monotonic() - self._avoid_started) > AVOID_TIMEOUT_S
        if min_centre > CLEAR_DIST_M or timed_out:
            self._avoid_active = False
            self._avoid_side   = 0
            self._avoid_steer  = 0.0

    # ── Publish callback override ─────────────────────────────────────────────
    def pub_cb(self):
        if not self._avoid_active:
            # Normal mode: parent sliding-mode controller runs unchanged.
            super().pub_cb()
            return

        if self.previous_time == 0:
            return

        # Avoidance mode: bypass the sliding-mode course controller entirely.
        # Use the parent's speed parameters but scale them down so the rover
        # has more reaction time in tight spaces.
        msg = Twist()
        speed_term    = self.ol_speed if self.reverse_driving else -self.ol_speed
        msg.linear.x  = self.neutral_speed + speed_term * AVOID_SPEED_SCALE

        steer_cmd     = self.neutral_steer + self._avoid_steer
        msg.angular.z = float(np.clip(steer_cmd, 1010.0, 1990.0))

        self.init_vel_pub.publish(msg)

    # ── Internal helpers ──────────────────────────────────────────────────────
    def _pick_best_gap(self, gaps, centers, min_gap_sectors):
        """
        Return the centre angle (degrees) of the best available gap.

        Selection priority:
          1. Viable gaps (>= min_gap_sectors wide) that are nearest to
             straight-ahead; ties broken by preferring the wider gap.
          2. If no gap is wide enough, fall back to the single widest gap.
        """
        viable = []
        for i0, i1 in gaps:
            span = i1 - i0 + 1
            if span >= min_gap_sectors:
                ang = 0.5 * (centers[i0] + centers[i1])
                viable.append((abs(ang), ang, span))   # (dist-from-fwd, angle, width)

        if viable:
            viable.sort(key=lambda t: (t[0], -t[2]))   # nearest first, then widest
            return viable[0][1]

        # Fallback: absolute widest gap
        i0, i1 = max(gaps, key=lambda g: g[1] - g[0])
        return 0.5 * (centers[i0] + centers[i1])

    def _enter_avoid(self, side):
        """Latch into avoidance mode (idempotent — only stamps the clock once)."""
        if not self._avoid_active:
            self._avoid_active  = True
            self._avoid_started = time.monotonic()
            self._avoid_side    = int(math.copysign(1, side))


def main(args=None):
    rclpy.init(args=args)
    node = LidarObsAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
