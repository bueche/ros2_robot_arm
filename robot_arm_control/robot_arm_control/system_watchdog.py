#!/usr/bin/env python3
"""
system_watchdog.py — ROS2 system health monitor

Monitors key topics for gaps, stalls and jitter without requiring
any changes to the publishing nodes. Uses arrival timestamps only.

Publishes:
  /watchdog/status   std_msgs/String   — human readable health summary

Parameters:
  warn_gap_factor    float   Warn if gap > factor * expected_period  default: 3.0
  report_hz          float   How often to print summary              default: 0.2
  stall_sec          float   Topic considered stalled after this     default: 5.0
"""

import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Imu, JointState, Image
from std_msgs.msg import Bool, String
from std_msgs.msg import Header
from rclpy.time import Time


class TopicMonitor:
    """Tracks arrival statistics for one topic."""

    def __init__(self, name, expected_hz, warn_gap_factor, stall_sec):
        self.name          = name
        self.expected_hz   = expected_hz
        self.expected_gap  = 1.0 / expected_hz
        self.warn_gap      = self.expected_gap * warn_gap_factor
        self.stall_sec     = stall_sec

        self._lock         = threading.Lock()
        self._last_arrival = None
        self._msg_count    = 0
        self._gap_spikes   = 0   # gaps > warn_gap
        self._max_gap      = 0.0
        self._sum_gap      = 0.0
        self._gap_count    = 0

    def on_arrival(self):
        now = time.monotonic()
        with self._lock:
            self._msg_count += 1
            if self._last_arrival is not None:
                gap = now - self._last_arrival
                self._sum_gap   += gap
                self._gap_count += 1
                if gap > self._max_gap:
                    self._max_gap = gap
                if gap > self.warn_gap:
                    self._gap_spikes += 1
            self._last_arrival = now

    def status(self, now):
        """Return (health_str, is_healthy)."""
        with self._lock:
            if self._last_arrival is None:
                return f'{self.name}: NO DATA', False

            age = now - self._last_arrival
            if age > self.stall_sec:
                return (f'{self.name}: STALLED {age:.1f}s  '
                        f'(msgs={self._msg_count})', False)

            avg_gap = (self._sum_gap / self._gap_count
                       if self._gap_count > 0 else 0.0)
            avg_hz  = 1.0 / avg_gap if avg_gap > 0 else 0.0

            healthy = self._gap_spikes == 0 and age < self.warn_gap
            symbol  = '✅' if healthy else '⚠️ '

            return (f'{symbol} {self.name}: '
                    f'{avg_hz:.1f}Hz  '
                    f'max_gap={self._max_gap*1000:.0f}ms  '
                    f'spikes={self._gap_spikes}  '
                    f'msgs={self._msg_count}'), healthy

    def reset_stats(self):
        """Reset spike/gap counters for next reporting period."""
        with self._lock:
            self._gap_spikes = 0
            self._max_gap    = 0.0
            self._sum_gap    = 0.0
            self._gap_count  = 0


class SystemWatchdogNode(Node):

    def __init__(self):
        super().__init__('system_watchdog')

        self.declare_parameter('warn_gap_factor', 3.0)
        self.declare_parameter('report_hz',       0.2)   # report every 5s
        self.declare_parameter('stall_sec',       5.0)

        wgf      = self.get_parameter('warn_gap_factor').value
        stall    = self.get_parameter('stall_sec').value
        report   = self.get_parameter('report_hz').value

        # Topics to monitor: (topic, msg_type, expected_hz)
        watches = [
            ('/ball/position',      Point,      15.0),
            ('/ball/cup_detected',  Bool,       15.0),
            ('/imu/balance_error',  Vector3,    50.0),
            ('/imu/balance_cmd',    Vector3,    15.0),
            ('/imu/is_stable',      Bool,       15.0),
            ('/balance_enabled',    Bool,        2.0),
            ('/joint_states',       JointState, 50.0),
            ('/ball/image',         Image,       5.0),
        ]

        self._monitors = {}
        for topic, msg_type, hz in watches:
            mon = TopicMonitor(topic, hz, wgf, stall)
            self._monitors[topic] = mon
            # Closure to capture mon
            self.create_subscription(
                msg_type, topic,
                lambda msg, m=mon: m.on_arrival(), 10)

        # Pipeline latency tracking via /ball/image header stamp
        # ball_detector_oak stamps the image at publish time (producer clock)
        # We record arrival time (consumer clock) and compute the difference
        # This distinguishes producer slowness from consumer/network slowness
        self._pipeline_lock         = threading.Lock()
        self._pipeline_samples      = []   # list of latency_ms floats
        self._pipeline_max_ms       = 0.0
        self._pipeline_spikes       = 0    # samples > spike_thresh_ms
        self._pipeline_spike_thresh = 150  # ms — 2x expected at 15fps
        # Clock offset between producer (Orin) and consumer (Pi 5)
        # Estimated on first few samples, then held fixed
        self._clock_offset_ms       = None
        self._clock_offset_samples  = []

        self.create_subscription(
            Image, '/ball/image',
            self._image_cb, 10)

        self._pub_status = self.create_publisher(
            String, '/watchdog/status', 10)

        self.create_timer(1.0 / report, self._report)

        self.get_logger().info(
            f'System watchdog started — monitoring {len(watches)} topics, '
            f'reporting at {report}Hz')

    def _image_cb(self, msg: Image):
        """
        Compute pipeline latency using header stamp from ball_detector_oak.

        The image header.stamp is set by the producer (Orin/Humble) using its
        ROS clock. We compare against our local ROS clock to get pipeline latency.

        Because Orin and Pi 5 clocks are not perfectly synchronized, we
        estimate the clock offset from the first N samples (assuming network
        latency is small and roughly constant at startup) and then subtract
        it from subsequent measurements.

        This gives us:
          true_pipeline_ms = (arrival_ros_time - stamp_ros_time) - clock_offset
        which reflects only the DDS transport + GIL delay, not clock skew.
        """
        # Producer timestamp (when ball_detector_oak called get_clock().now())
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)

        # Consumer timestamp (now, on Pi 5)
        arrival_ns = self.get_clock().now().nanoseconds

        raw_latency_ms = (arrival_ns - stamp_ns) / 1_000_000.0

        with self._pipeline_lock:
            # Clock offset calibration — use first 20 samples
            # Assumes first samples have minimal contention (startup is quiet)
            if self._clock_offset_ms is None:
                self._clock_offset_samples.append(raw_latency_ms)
                if len(self._clock_offset_samples) >= 20:
                    # Use median of first 20 samples as clock offset estimate
                    sorted_s = sorted(self._clock_offset_samples)
                    self._clock_offset_ms = sorted_s[len(sorted_s)//2]
                    self.get_logger().info(
                        f'Pipeline clock offset calibrated: '
                        f'{self._clock_offset_ms:.1f}ms '
                        f'(median of first {len(sorted_s)} samples)')
                return  # don't record during calibration

            latency_ms = raw_latency_ms - self._clock_offset_ms

            self._pipeline_samples.append(latency_ms)
            if latency_ms > self._pipeline_max_ms:
                self._pipeline_max_ms = latency_ms
            if latency_ms > self._pipeline_spike_thresh:
                self._pipeline_spikes += 1
                self.get_logger().warn(
                    f'PIPELINE SPIKE: {latency_ms:.0f}ms '
                    f'(producer→consumer, thresh={self._pipeline_spike_thresh}ms)')

    def _report(self):
        now    = time.monotonic()
        lines  = ['=== System Watchdog ===']
        all_ok = True

        for topic, mon in self._monitors.items():
            status, healthy = mon.status(now)
            lines.append(f'  {status}')
            if not healthy:
                all_ok = False
            mon.reset_stats()

        # Pipeline latency summary
        with self._pipeline_lock:
            samples = self._pipeline_samples[:]
            max_ms  = self._pipeline_max_ms
            spikes  = self._pipeline_spikes
            offset  = self._clock_offset_ms
            self._pipeline_samples  = []
            self._pipeline_max_ms   = 0.0
            self._pipeline_spikes   = 0

        if offset is None:
            lines.append('  Pipeline latency: calibrating...')
        elif samples:
            avg_ms = sum(samples) / len(samples)
            p95_ms = sorted(samples)[int(len(samples) * 0.95)]
            healthy_pipe = spikes == 0
            if not healthy_pipe:
                all_ok = False
            sym = '✅' if healthy_pipe else '⚠️ '
            lines.append(
                f'  {sym} Pipeline (Orin→Pi5): '
                f'avg={avg_ms:.0f}ms  '
                f'p95={p95_ms:.0f}ms  '
                f'max={max_ms:.0f}ms  '
                f'spikes(>{self._pipeline_spike_thresh}ms)={spikes}')
        else:
            lines.append('  Pipeline latency: no /ball/image samples this period')

        lines.append(f'  Overall: {"✅ HEALTHY" if all_ok else "⚠️  ISSUES DETECTED"}')
        report = '\n'.join(lines)

        self.get_logger().info(report)

        msg = String()
        msg.data = report
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
