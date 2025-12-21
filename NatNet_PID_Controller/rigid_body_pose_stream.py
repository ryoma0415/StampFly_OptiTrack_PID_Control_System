#!/usr/bin/env python3
"""
NatNet rigid body pose streaming test tool.

This experimental script establishes a NatNet connection to Motive and prints
rigid body translations and orientations. It is intended as a first step toward
leveraging Motive's rigid body pose for the hovering controller.
"""

import argparse
import os
import signal
import sys
import threading
import time
from math import asin, atan2, copysign, pi
from typing import Dict, Iterable, Optional

# Ensure the NatNet client can be imported when the script is executed directly.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from NatNetClient import NatNetClient  # noqa: E402

RAD_TO_DEG = 180.0 / pi


def quaternion_to_euler_xyz(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion (x, y, z, w) to roll/pitch/yaw in radians."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = copysign(pi / 2.0, sinp)
    else:
        pitch = asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class RigidBodyPosePrinter:
    """Handles NatNet connection and throttled console output."""

    def __init__(
        self,
        server_ip: str,
        local_ip: str,
        use_multicast: bool,
        body_filter: Optional[Iterable[int]] = None,
        body_labels: Optional[Dict[int, str]] = None,
        print_hz: float = 10.0,
    ):
        self.server_ip = server_ip
        self.local_ip = local_ip
        self.use_multicast = use_multicast
        self.body_filter = set(body_filter) if body_filter else None
        self.body_labels = body_labels or {}
        self.print_period = 1.0 / print_hz if print_hz > 0 else 0.0

        self.client: Optional[NatNetClient] = None
        self.stop_event = threading.Event()
        self.last_print_ts: Dict[int, float] = {}

    def start(self):
        """Connect to NatNet and begin printing pose updates."""
        if self.client is not None:
            raise RuntimeError("Client already started")

        self.client = NatNetClient()
        self.client.set_server_address(self.server_ip)
        self.client.set_client_address(self.local_ip)
        self.client.set_use_multicast(self.use_multicast)
        self.client.set_print_level(0)
        self.client.rigid_body_listener = self._on_rigid_body

        if not self.client.run('d'):
            print("[ERR] NatNet client failed to start. Check IP configuration.")
            self._shutdown_client()
            return

        # Give the command thread time to query server info.
        time.sleep(0.5)
        if not self.client.connected():
            print("[ERR] NatNet handshake incomplete. Verify Motive is streaming.")
            self._shutdown_client()
            return

        if self.client.command_socket:
            # Request model definitions so that Motive refreshes rigid body list.
            try:
                self.client.send_request(
                    self.client.command_socket,
                    self.client.NAT_REQUEST_MODELDEF,
                    "",
                    (self.client.server_ip_address, self.client.command_port),
                )
            except Exception:
                # Model definition is not critical for pose output, so continue.
                pass

        self._print_start_banner()

        try:
            while not self.stop_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()
        finally:
            self._shutdown_client()

    def stop(self):
        """Signal the printer to stop and shut down NatNet."""
        self.stop_event.set()
        self._shutdown_client()

    def _shutdown_client(self):
        if self.client is not None:
            try:
                self.client.shutdown()
            except Exception:
                pass
            self.client = None

    def _print_start_banner(self):
        heading = f"[OK] Connected to NatNet server {self.server_ip}"
        print(heading)
        if self.body_filter:
            combined = ", ".join(
                f"{body_id}{' (' + self.body_labels[body_id] + ')' if body_id in self.body_labels else ''}"
                for body_id in sorted(self.body_filter)
            )
            print(f"Monitoring rigid bodies: {combined}")
        else:
            print("Monitoring all rigid bodies.")
        print("Press Ctrl+C to stop.\n")

    def _on_rigid_body(self, rigid_body_id, position, rotation):
        if self.body_filter and rigid_body_id not in self.body_filter:
            return

        now = time.monotonic()
        last_ts = self.last_print_ts.get(rigid_body_id, 0.0)
        if self.print_period > 0.0 and (now - last_ts) < self.print_period:
            return
        self.last_print_ts[rigid_body_id] = now

        roll, pitch, yaw = quaternion_to_euler_xyz(
            rotation[0], rotation[1], rotation[2], rotation[3]
        )

        label = self.body_labels.get(rigid_body_id)
        header = f"[RB {rigid_body_id:3d}]"
        if label:
            header = f"{header} {label}"

        print(
            f"{header} "
            f"pos(m)=({position[0]:+7.4f}, {position[1]:+7.4f}, {position[2]:+7.4f}) "
            f"quat=({rotation[0]:+7.4f}, {rotation[1]:+7.4f}, {rotation[2]:+7.4f}, {rotation[3]:+7.4f}) "
            f"euler(deg)=({roll * RAD_TO_DEG:+6.2f}, {pitch * RAD_TO_DEG:+6.2f}, {yaw * RAD_TO_DEG:+6.2f})"
        )


def parse_body_filters(entries: Optional[Iterable[str]]):
    if not entries:
        return None, {}

    ids = []
    labels: Dict[int, str] = {}
    for item in entries:
        if ':' in item:
            id_part, label = item.split(':', 1)
        elif '=' in item:
            id_part, label = item.split('=', 1)
        else:
            id_part, label = item, ""

        try:
            body_id = int(id_part, 0)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid rigid body identifier: {item}"
            ) from exc

        ids.append(body_id)
        if label:
            labels[body_id] = label.strip()

    return ids, labels


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Stream Motive rigid body pose data via NatNet and print it for debugging."
        )
    )
    parser.add_argument(
        "--server",
        default="127.0.0.1",
        help="NatNet server IP address (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--client",
        default="127.0.0.1",
        help="Local interface IP address for NatNet (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--unicast",
        action="store_true",
        help="Use unicast instead of multicast (default: multicast).",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=10.0,
        help="Maximum print frequency per rigid body in Hz (default: 10.0).",
    )
    parser.add_argument(
        "--body",
        action="append",
        metavar="ID[:label]",
        help=(
            "Restrict output to a specific rigid body ID and optionally assign a label. "
            "May be provided multiple times (example: --body 1:StampFly)."
        ),
    )
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    body_ids, body_labels = parse_body_filters(args.body)
    printer = RigidBodyPosePrinter(
        server_ip=args.server,
        local_ip=args.client,
        use_multicast=not args.unicast,
        body_filter=body_ids,
        body_labels=body_labels,
        print_hz=args.print_hz,
    )

    def _handle_signal(signum, frame):
        _ = signum, frame  # Unused.
        printer.stop()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    printer.start()


if __name__ == "__main__":
    main()
