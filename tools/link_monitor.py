#!/usr/bin/env python3
"""Read-only ROS/serial continuity monitor for rm_vision."""

import argparse
import csv
import glob
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros

from auto_aim_interfaces.msg import Target, YoloBoxes
from std_msgs.msg import Float64, Float64MultiArray, String


QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def rpy(transform):
  x = transform.rotation.x
  y = transform.rotation.y
  z = transform.rotation.z
  w = transform.rotation.w
  roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
  pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
  yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
  return roll, pitch, yaw


def percentile(values, fraction):
  if not values:
    return float("nan")
  ordered = sorted(values)
  return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))]


def serial_io():
  for comm_path in glob.glob("/proc/[0-9]*/comm"):
    try:
      if Path(comm_path).read_text().strip() != "rm_serial_drive":
        continue
      pid = comm_path.split("/")[2]
      counters = {}
      for line in Path(f"/proc/{pid}/io").read_text().splitlines():
        key, value = line.split(":")
        counters[key] = int(value)
      return pid, counters["rchar"], counters["wchar"]
    except (OSError, ValueError, KeyError):
      continue
  return None, 0, 0


class LinkMonitor(Node):
  def __init__(self):
    super().__init__("link_monitor")
    self.buffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.buffer, self)
    self.target = None
    self.boxes = None
    self.latencies = []
    self.target_receipts = []
    self.tracking_changes = 0
    self.last_tracking = None
    self.yaw_samples = []
    self.pitch_samples = []
    self.tf_gaps = []
    self.trajectory_receipts = []
    self.commands = []
    self.planner_status = "not_received"
    self.planner_status_time = None
    self.last_sequence = -1
    self.last_command_age_ms = float("nan")
    self.serial_pid, self.serial_read, self.serial_write = serial_io()
    self.last_tf_time = None
    self.last_tf_stamp = None
    self.create_subscription(Target, "/tracker/target", self.on_target, QOS)
    self.create_subscription(YoloBoxes, "/detector/yolo_boxes", self.on_boxes, QOS)
    self.create_subscription(Float64, "/latency", self.on_latency, 10)
    self.create_subscription(
      Float64MultiArray, "/serial_driver/trajectory", self.on_trajectory, 10)
    self.create_subscription(String, "/serial_driver/planner_status", self.on_planner_status, 10)

  def on_planner_status(self, msg):
    self.planner_status = msg.data
    self.planner_status_time = time.monotonic()

  def on_target(self, msg):
    now = time.monotonic()
    self.target_receipts.append(now)
    self.target = msg
    if msg.tracking != self.last_tracking:
      self.tracking_changes += 1
      self.last_tracking = msg.tracking

  def on_boxes(self, msg):
    self.boxes = len(msg.boxes)

  def on_latency(self, msg):
    self.latencies.append(msg.data)

  def on_trajectory(self, msg):
    if len(msg.data) < 11:
      return
    self.trajectory_receipts.append(time.monotonic())
    self.commands.append((msg.data[0] > 0.5, msg.data[5], msg.data[6]))
    self.last_sequence = int(msg.data[1])
    self.last_command_age_ms = msg.data[3]

  def sample_tf(self):
    now = time.monotonic()
    try:
      transform = self.buffer.lookup_transform(
        "odom", "gimbal_link", Time(), timeout=rclpy.duration.Duration(seconds=0.005))
    except Exception:
      return
    stamp = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
    if stamp == self.last_tf_stamp:
      return
    self.last_tf_stamp = stamp
    _, pitch, yaw = rpy(transform.transform)
    if self.yaw_samples:
      self.yaw_samples.append(
        self.yaw_samples[-1] + math.remainder(yaw - self.yaw_samples[-1], 2.0 * math.pi))
      self.pitch_samples.append(pitch)
    else:
      self.yaw_samples.append(yaw)
      self.pitch_samples.append(pitch)
    if self.last_tf_time is not None:
      self.tf_gaps.append((now - self.last_tf_time) * 1000.0)
    self.last_tf_time = now

  def reset_window(self):
    self.latencies.clear()
    self.target_receipts.clear()
    self.tracking_changes = 0
    self.yaw_samples.clear()
    self.pitch_samples.clear()
    self.tf_gaps.clear()
    self.trajectory_receipts.clear()
    self.commands.clear()

  def report(self, window_start, writer):
    elapsed = max(time.monotonic() - window_start, 1e-6)
    pid, read_now, write_now = serial_io()
    if pid != self.serial_pid:
      old_read = old_write = 0
      self.serial_pid = pid
    else:
      old_read, old_write = self.serial_read, self.serial_write
    read_rate = (read_now - old_read) / elapsed
    write_rate = (write_now - old_write) / elapsed
    self.serial_read, self.serial_write = read_now, write_now

    yaw_steps = [
      (self.yaw_samples[index] - self.yaw_samples[index - 1]) * 180.0 / math.pi
      for index in range(1, len(self.yaw_samples))
    ]
    pitch_steps = [
      (self.pitch_samples[index] - self.pitch_samples[index - 1]) * 180.0 / math.pi
      for index in range(1, len(self.pitch_samples))
    ]
    target_intervals = [
      (self.target_receipts[index] - self.target_receipts[index - 1]) * 1000.0
      for index in range(1, len(self.target_receipts))
    ]
    target_aim_yaw = (
      math.atan2(self.target.position.y, self.target.position.x)
      if self.target is not None and self.target.tracking else float("nan"))
    trajectory_intervals = [
      (self.trajectory_receipts[index] - self.trajectory_receipts[index - 1]) * 1000.0
      for index in range(1, len(self.trajectory_receipts))
    ]
    command_valid = sum(1 for valid, _, _ in self.commands if valid)
    command_valid_changes = sum(
      1 for index in range(1, len(self.commands))
      if self.commands[index][0] != self.commands[index - 1][0])
    command_yaw_steps = [
      math.remainder(
        self.commands[index][1] - self.commands[index - 1][1], 2.0 * math.pi
      ) * 180.0 / math.pi
      for index in range(1, len(self.commands))
      if self.commands[index][0] and self.commands[index - 1][0]
    ]
    command_pitch_steps = [
      (self.commands[index][2] - self.commands[index - 1][2]) * 180.0 / math.pi
      for index in range(1, len(self.commands))
      if self.commands[index][0] and self.commands[index - 1][0]
    ]
    command_yaw_error = float("nan")
    command_pitch_error = float("nan")
    if self.commands and self.commands[-1][0] and self.yaw_samples:
      command_yaw_error = math.degrees(
        math.remainder(self.commands[-1][1] - self.yaw_samples[-1], 2.0 * math.pi))
      command_pitch_error = math.degrees(
        self.commands[-1][2] - self.pitch_samples[-1])
    row = {
      "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
      "serial_pid": self.serial_pid,
      "serial_read_Bps": round(read_rate),
      "serial_write_Bps": round(write_rate),
      "tf_hz": round((len(self.yaw_samples) - 1) / elapsed, 1),
      "tf_gap_p95_ms": round(percentile(self.tf_gaps, 0.95), 2),
      "tf_gap_max_ms": round(max(self.tf_gaps, default=float("nan")), 2),
      "yaw_step_p95_mdeg": round(percentile(yaw_steps, 0.95) * 1000.0, 2),
      "yaw_step_max_mdeg": round(max(yaw_steps, default=float("nan")) * 1000.0, 2),
      "pitch_step_p95_mdeg": round(percentile(pitch_steps, 0.95) * 1000.0, 2),
      "pitch_step_max_mdeg": round(max(pitch_steps, default=float("nan")) * 1000.0, 2),
      "target_msgs": len(self.target_receipts),
      "target_gap_p95_ms": round(percentile(target_intervals, 0.95), 2),
      "target_gap_max_ms": round(max(target_intervals, default=float("nan")), 2),
      "target_tracking": bool(self.target and self.target.tracking),
      "tracking_changes": self.tracking_changes,
      "boxes": self.boxes if self.boxes is not None else -1,
      "target_aim_yaw_deg": round(math.degrees(target_aim_yaw), 3),
      "a6_msgs": len(self.trajectory_receipts),
      "a6_valid": command_valid,
      "a6_valid_changes": command_valid_changes,
      "a6_last_sequence": self.last_sequence,
      "a6_last_age_ms": self.last_command_age_ms,
      "planner_status": self.planner_status,
      "planner_status_age_ms": round((time.monotonic() - self.planner_status_time) * 1000.0, 2)
        if self.planner_status_time is not None else float("nan"),
      "a6_gap_p95_ms": round(percentile(trajectory_intervals, 0.95), 2),
      "a6_gap_max_ms": round(max(trajectory_intervals, default=float("nan")), 2),
      "cmd_yaw_step_p95_deg": round(percentile(command_yaw_steps, 0.95), 3),
      "cmd_yaw_step_max_deg": round(max(command_yaw_steps, default=float("nan")), 3),
      "cmd_pitch_step_p95_deg": round(percentile(command_pitch_steps, 0.95), 3),
      "cmd_pitch_step_max_deg": round(max(command_pitch_steps, default=float("nan")), 3),
      "cmd_feedback_yaw_error_deg": round(command_yaw_error, 3),
      "cmd_feedback_pitch_error_deg": round(command_pitch_error, 3),
      "latency_p50_ms": round(percentile(self.latencies, 0.50), 2),
      "latency_p95_ms": round(percentile(self.latencies, 0.95), 2),
      "latency_max_ms": round(max(self.latencies, default=float("nan")), 2),
    }
    writer.writerow(row)
    print(
      f"tf={row['tf_hz']:5.1f}Hz gaps={row['tf_gap_p95_ms']:5.2f}/"
      f"{row['tf_gap_max_ms']:5.2f}ms "
      f"yaw_step={row['yaw_step_p95_mdeg']:7.2f}/{row['yaw_step_max_mdeg']:8.2f}mdeg "
      f"target={row['target_msgs']:2d} msg gaps={row['target_gap_p95_ms']:5.2f}/"
      f"{row['target_gap_max_ms']:5.2f}ms "
      f"track={row['target_tracking']} boxes={row['boxes']} "
      f"A6={row['a6_msgs']:2d}/{row['a6_valid']:2d} valid "
      f"valid_changes={row['a6_valid_changes']} "
      f"cmd_yaw_step={row['cmd_yaw_step_p95_deg']:6.3f}/"
      f"{row['cmd_yaw_step_max_deg']:7.3f}deg "
      f"tracking_error={row['cmd_feedback_yaw_error_deg']:6.3f}/"
      f"{row['cmd_feedback_pitch_error_deg']:6.3f}deg "
      f"latency={row['latency_p50_ms']:5.2f}/{row['latency_p95_ms']:5.2f}ms "
      f"serial={row['serial_read_Bps']}/{row['serial_write_Bps']}B/s "
      f"planner={row['planner_status']} age={row['a6_last_age_ms']}ms",
      flush=True,
    )
    self.reset_window()


def main():
  parser = argparse.ArgumentParser(description="rm_vision read-only link continuity monitor")
  parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl+C")
  parser.add_argument(
    "--output", default=str(Path(__file__).resolve().parents[1] / "logs" / "link_monitor.csv"))
  args = parser.parse_args()

  rclpy.init()
  node = LinkMonitor()
  output = Path(args.output)
  output.parent.mkdir(parents=True, exist_ok=True)
  started = time.monotonic()
  window_start = started
  with output.open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=[
      "timestamp", "serial_pid", "serial_read_Bps", "serial_write_Bps", "tf_hz",
      "tf_gap_p95_ms", "tf_gap_max_ms", "yaw_step_p95_mdeg", "yaw_step_max_mdeg",
      "pitch_step_p95_mdeg", "pitch_step_max_mdeg", "target_msgs",
      "target_gap_p95_ms", "target_gap_max_ms", "target_tracking",
      "tracking_changes", "boxes", "target_aim_yaw_deg", "a6_msgs", "a6_valid",
      "a6_valid_changes", "a6_gap_p95_ms", "a6_gap_max_ms", "cmd_yaw_step_p95_deg",
      "a6_last_sequence", "a6_last_age_ms", "planner_status", "planner_status_age_ms",
      "cmd_yaw_step_max_deg", "cmd_pitch_step_p95_deg", "cmd_pitch_step_max_deg",
      "cmd_feedback_yaw_error_deg", "cmd_feedback_pitch_error_deg",
      "latency_p50_ms", "latency_p95_ms", "latency_max_ms",
    ])
    writer.writeheader()
    try:
      while rclpy.ok() and (args.duration <= 0.0 or time.monotonic() - started < args.duration):
        rclpy.spin_once(node, timeout_sec=0.01)
        node.sample_tf()
        if time.monotonic() - window_start >= 1.0:
          node.report(window_start, writer)
          stream.flush()
          window_start = time.monotonic()
    except KeyboardInterrupt:
      pass
    finally:
      node.destroy_node()
      if rclpy.ok():
        rclpy.shutdown()
  print(f"CSV: {output}", flush=True)


if __name__ == "__main__":
  main()
