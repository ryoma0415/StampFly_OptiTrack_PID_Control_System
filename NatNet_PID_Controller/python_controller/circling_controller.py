#!/usr/bin/env python3
"""
Optitrack circling control program.
Generates a slow circular target around the configured center and reuses
the existing hover controller pipeline (NatNet -> filter -> PID -> relay).
"""

import time
from math import cos, sin, pi

from hovering_controller import HoveringController


class CirclingController(HoveringController):
    """Circle-tracking controller based on the hover controller."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        trajectory = self.config.get("trajectory", {}) if isinstance(self.config, dict) else {}
        self.circle_radius = self._read_float(trajectory, "radius", 0.3, min_value=0.0)
        self.circle_period_sec = self._read_float(trajectory, "period_sec", 30.0, min_value=0.1)
        self.circle_hold_sec = self._read_float(trajectory, "start_hold_sec", 5.0, min_value=0.0)
        self.circle_clockwise = self._read_bool(trajectory, "clockwise", True)
        self.trajectory_start_time = None

    @staticmethod
    def _read_float(config, key, default, min_value=None):
        try:
            value = float(config.get(key, default))
        except (TypeError, ValueError, AttributeError):
            value = default
        if min_value is not None:
            value = max(min_value, value)
        return value

    @staticmethod
    def _read_bool(config, key, default):
        value = config.get(key, default) if isinstance(config, dict) else default
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "y", "on")
        return bool(value)

    def on_control_start(self):
        self.trajectory_start_time = time.monotonic()

    def get_target_position(self, frame_time_monotonic=None):
        now = frame_time_monotonic if frame_time_monotonic is not None else time.monotonic()
        if self.trajectory_start_time is None:
            self.trajectory_start_time = now

        elapsed = max(0.0, now - self.trajectory_start_time)
        center_x, center_y = self.target_position

        if elapsed < self.circle_hold_sec:
            return (center_x + self.circle_radius, center_y)

        omega = 2.0 * pi / self.circle_period_sec
        theta = omega * (elapsed - self.circle_hold_sec)
        if self.circle_clockwise:
            theta = -theta

        target_x = center_x + self.circle_radius * cos(theta)
        target_y = center_y + self.circle_radius * sin(theta)
        return (target_x, target_y)

    def get_target_description(self):
        direction = "CW" if self.circle_clockwise else "CCW"
        center_x, center_y = self.target_position
        return (
            "Target: circle r={:.3f}m, period={:.1f}s, hold={:.1f}s, "
            "dir={}, center=({:.3f}, {:.3f}) m"
        ).format(
            self.circle_radius,
            self.circle_period_sec,
            self.circle_hold_sec,
            direction,
            center_x,
            center_y,
        )

    def get_active_mode_message(self):
        return "Circling control active... (stop to land)"

    def run(self):
        print("\n" + "=" * 50)
        print("Optitrack Circling Control System")
        print("=" * 50)
        print("Commands:")
        print("  start - takeoff and start circling control")
        print("  stop  - land")
        print("  exit  - exit program")
        print("=" * 50 + "\n")

        if not self.connect_serial():
            print("Failed to connect to ESP32")
            return

        try:
            while True:
                command = input("\nCommand > ").strip().lower()

                if command == "exit":
                    if self.is_flying:
                        print("Landing before exit...")
                        self.stop_hovering()
                    print("Exiting program")
                    break
                elif command == "start":
                    self.start_hovering()
                elif command == "stop":
                    self.stop_hovering()
                elif command == "":
                    continue
                else:
                    print(f"Unknown command: {command}")

        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt detected")

        finally:
            if self.is_flying:
                print("Landing for safety...")
                self.stop_hovering()
                time.sleep(2)

            self.disconnect_serial()
            print("\nProgram ended")


def main():
    controller = CirclingController()
    controller.run()


if __name__ == "__main__":
    main()
