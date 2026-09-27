#!/usr/bin/env python3
"""
PX4 SITL + Gazebo x500 15-drone L-formation controller.

Mission:
  - Exactly 15 vehicles: UIDs 1..15 / PX4 instances 0..14.
  - Hard altitude cap: 2.0 m above each vehicle's home altitude.
  - Maximum commanded horizontal speed: 5 m/s.
  - Bottom UIDs 1..5 form the horizontal leg.
  - UIDs 6..15 form the vertical leg above UID 5, 100 m apart.
  - Vehicles are armed ONE AT A TIME immediately before their own takeoff.
  - A vehicle is sent to its formation slot immediately after becoming airborne;
    the controller does not wait for that vehicle to reach its slot before
    launching the next one.
  - After formation, the complete active formation can sweep horizontally.

Important fix:
  MAV_CMD_NAV_TAKEOFF parameter 7 is AMSL altitude, NOT relative altitude.
  This controller therefore commands HOME_AMSL + 2.0 m.  Sending "2.0" as
  an AMSL altitude can cause PX4 to reject/ignore takeoff in SITL.
"""

from __future__ import annotations

import argparse
import math
import signal
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

# ------------------------------ Fleet -------------------------------------

INSTANCES = list(range(15))
UIDS = list(range(1, 16))
BOTTOM_UIDS = list(range(1, 6))
VERTICAL_UIDS = list(range(6, 16))

GCS_LISTEN_PORT = 14550
GCS_LOCAL_PORT_BASE = 18570
CONTROLLER_SYS_ID = 250
CONTROLLER_COMP_ID = 1

FORMATION_SPACING_M = 100.0
FORMATION_SPEED_MPS = 5.0
HEIGHT_CAP_M = 2.0
SURVEY_WIDTH_M = 1000.0
MAX_RELAY_DISTANCE_M = 100.0
EARTH_RADIUS_M = 6378137.0

# MAVLink IDs / commands
CMD_ARM_DISARM = 400
CMD_NAV_TAKEOFF = 22
CMD_NAV_LAND = 21
CMD_DO_REPOSITION = 192
CMD_SET_MESSAGE_INTERVAL = 511
MAV_FRAME_GLOBAL = 0
MAV_MODE_FLAG_ARMED = 128

Point = Tuple[float, float]  # east, north metres


def sysid(uid: int) -> int:
    return uid


def local_port(uid: int) -> int:
    return GCS_LOCAL_PORT_BASE + uid - 1


def offset_to_latlon(lat0: float, lon0: float, east_m: float, north_m: float) -> Tuple[float, float]:
    dlat = north_m / EARTH_RADIUS_M * 180.0 / math.pi
    c = max(0.1, math.cos(math.radians(lat0)))
    dlon = east_m / (EARTH_RADIUS_M * c) * 180.0 / math.pi
    return lat0 + dlat, lon0 + dlon


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r1, r2 = math.radians(lat1), math.radians(lat2)
    dr = r2 - r1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dr / 2) ** 2 + math.cos(r1) * math.cos(r2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(max(0.0, min(1.0, a))))


def l_offsets(spacing: float) -> Dict[int, Point]:
    # UID 5 is the corner.  UID 1 is 400 m to the left of UID 5.
    result: Dict[int, Point] = {}
    for i, uid in enumerate(BOTTOM_UIDS):
        result[uid] = (-(4 - i) * spacing, 0.0)
    for i, uid in enumerate(VERTICAL_UIDS, start=1):
        result[uid] = (0.0, i * spacing)
    return result


@dataclass
class VehicleState:
    heartbeat: object = None
    position: object = None
    acks: Dict[int, object] = field(default_factory=dict)


class MavlinkBus:
    def __init__(self) -> None:
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise RuntimeError("pymavlink is not installed. Run: python3 -m pip install pymavlink") from exc

        self.mavutil = mavutil
        self.conn = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{GCS_LISTEN_PORT}",
            source_system=CONTROLLER_SYS_ID,
            source_component=CONTROLLER_COMP_ID,
            autoreconnect=True,
        )
        self.states: Dict[int, VehicleState] = {uid: VehicleState() for uid in UIDS}
        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.sockets = {uid: socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for uid in UIDS}

    def start(self) -> None:
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        try:
            self.conn.close()
        except Exception:
            pass
        for s in self.sockets.values():
            try:
                s.close()
            except Exception:
                pass
        if self.thread:
            self.thread.join(timeout=2)

    def _reader(self) -> None:
        while not self.stop_event.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1.0)
            except Exception:
                continue
            if msg is None:
                continue
            try:
                uid = int(msg.get_srcSystem())
            except Exception:
                continue
            if uid not in UIDS:
                continue
            typ = msg.get_type()
            with self.lock:
                st = self.states[uid]
                if typ == "HEARTBEAT":
                    st.heartbeat = msg
                elif typ == "GLOBAL_POSITION_INT":
                    st.position = msg
                elif typ == "COMMAND_ACK":
                    try:
                        st.acks[int(msg.command)] = msg
                    except Exception:
                        pass

    def state(self, uid: int) -> VehicleState:
        with self.lock:
            return self.states[uid]

    def send(self, uid: int, packed: bytes) -> None:
        with self.send_lock:
            self.sockets[uid].sendto(packed, ("127.0.0.1", local_port(uid)))

    def command_long(self, uid: int, command: int, params: Tuple[float, ...]) -> None:
        msg = self.conn.mav.command_long_encode(sysid(uid), 1, command, 0, *params)
        self.send(uid, msg.pack(self.conn.mav))

    def reposition(self, uid: int, lat: float, lon: float, amsl: float) -> None:
        # MAV_CMD_DO_REPOSITION:
        # p1=speed, p2=1(change mode), p3=0, p4=NaN(yaw), x/y=lat/lon*1e7, z=AMSL.
        msg = self.conn.mav.command_int_encode(
            sysid(uid),
            1,
            MAV_FRAME_GLOBAL,
            CMD_DO_REPOSITION,
            0,
            0,
            FORMATION_SPEED_MPS,
            1.0,
            0.0,
            math.nan,
            int(round(lat * 1e7)),
            int(round(lon * 1e7)),
            float(amsl),
        )
        self.send(uid, msg.pack(self.conn.mav))


class Controller:
    def __init__(self, args: argparse.Namespace) -> None:
        self.height = min(max(0.5, args.altitude), HEIGHT_CAP_M)
        self.spacing = max(1.0, args.spacing)
        self.connect_timeout = args.connect_timeout
        self.arm_timeout = args.arm_timeout
        self.takeoff_timeout = args.takeoff_timeout
        self.formation_timeout = args.formation_timeout
        self.tol = args.position_tolerance
        self.alt_tol = args.altitude_tolerance
        self.survey_width = min(max(1.0, args.survey_width), 1000.0)
        self.survey_passes = min(max(1, args.survey_passes), 2)
        self.hold_s = args.hold
        self.offsets = l_offsets(self.spacing)
        self.bus = MavlinkBus()
        self.origin: Optional[Tuple[float, float]] = None
        self.active: List[int] = []
        self.home_amsl: Dict[int, float] = {}

    def start(self) -> None:
        self.bus.start()

    def stop(self) -> None:
        self.bus.stop()

    def armed(self, uid: int) -> bool:
        hb = self.bus.state(uid).heartbeat
        return bool(hb is not None and (int(hb.base_mode) & MAV_MODE_FLAG_ARMED))

    def position(self, uid: int) -> Optional[Tuple[float, float, float, float]]:
        p = self.bus.state(uid).position
        if p is None:
            return None
        try:
            return float(p.lat) / 1e7, float(p.lon) / 1e7, float(p.relative_alt) / 1000.0, float(p.alt) / 1000.0
        except Exception:
            return None

    def wait_for_telemetry(self) -> None:
        print("Controller listening on UDP 14550.")
        print("Waiting for all 15 PX4 heartbeats...")
        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline:
            ready = [uid for uid in UIDS if self.bus.state(uid).heartbeat is not None]
            print(f"  Heartbeats: {len(ready)}/15", end="\r", flush=True)
            if len(ready) == 15:
                print("\nAll 15 PX4 vehicles connected.")
                break
            time.sleep(0.5)
        else:
            missing = [uid for uid in UIDS if self.bus.state(uid).heartbeat is None]
            raise RuntimeError(f"Missing PX4 heartbeats: {missing}")

        # We need valid GPS/global position before sending global commands.
        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline:
            ready = [uid for uid in UIDS if self.position(uid) is not None]
            print(f"  Global positions: {len(ready)}/15", end="\r", flush=True)
            if len(ready) == 15:
                print("\nGlobal position telemetry ready for all 15.")
                return
            time.sleep(0.5)
        missing = [uid for uid in UIDS if self.position(uid) is None]
        raise RuntimeError(f"Missing GLOBAL_POSITION_INT telemetry: {missing}")

    def set_origin(self) -> None:
        # UID 5 is the formation corner. Its current GPS position is the origin.
        p = self.position(5)
        if p is None:
            raise RuntimeError("UID 5 has no position telemetry.")
        self.origin = (p[0], p[1])
        for uid in UIDS:
            q = self.position(uid)
            if q is None:
                raise RuntimeError(f"UID {uid} lost position telemetry before takeoff.")
            self.home_amsl[uid] = q[3] - q[2]
        print(f"Formation corner (UID 5): {p[0]:.7f}, {p[1]:.7f}")
        print(f"Hard altitude cap: {HEIGHT_CAP_M:.1f} m above home")

    def targets(self) -> Dict[int, Tuple[float, float]]:
        if self.origin is None:
            raise RuntimeError("Origin not initialized.")
        lat0, lon0 = self.origin
        return {uid: offset_to_latlon(lat0, lon0, self.offsets[uid][0], self.offsets[uid][1]) for uid in UIDS}

    def send_arm(self, uid: int) -> None:
        self.bus.command_long(uid, CMD_ARM_DISARM, (1.0, 0, 0, 0, 0, 0, 0))

    def arm_one(self, uid: int) -> bool:
        if self.armed(uid):
            return True
        for attempt in range(1, 5):
            self.send_arm(uid)
            deadline = time.monotonic() + self.arm_timeout / 4.0
            while time.monotonic() < deadline:
                if self.armed(uid):
                    print(f"  UID {uid}: ARMED")
                    return True
                time.sleep(0.1)
            print(f"  UID {uid}: arm retry {attempt}/4")
        print(f"  UID {uid}: ARM FAILED")
        return False

    def takeoff_one(self, uid: int) -> bool:
        p = self.position(uid)
        if p is None:
            print(f"  UID {uid}: no position; SKIP")
            return False

        # CRITICAL: TAKEOFF z is AMSL, so use home AMSL + relative height.
        home = self.home_amsl.get(uid, p[3] - p[2])
        target_amsl = home + self.height
        print(f"  UID {uid}: TAKEOFF target = home AMSL {home:.2f} + {self.height:.2f} = {target_amsl:.2f} m")

        for retry in range(1, 5):
            try:
                self.bus.command_long(
                    uid,
                    CMD_NAV_TAKEOFF,
                    (0.0, 0.0, 0.0, math.nan, p[0], p[1], target_amsl),
                )
            except Exception as exc:
                print(f"  UID {uid}: takeoff send error: {exc}")

            deadline = time.monotonic() + self.takeoff_timeout / 4.0
            while time.monotonic() < deadline:
                q = self.position(uid)
                if q is not None and q[2] >= 0.45:
                    print(f"  UID {uid}: AIRBORNE at {q[2]:.2f} m")
                    return True
                if not self.armed(uid):
                    # Re-arm only this drone; never pre-arm the next one.
                    print(f"  UID {uid}: disarmed before airborne; re-arming")
                    if not self.arm_one(uid):
                        break
                time.sleep(0.1)
            print(f"  UID {uid}: takeoff retry {retry}/4")

        print(f"  UID {uid}: TAKEOFF FAILED; continuing without it")
        return False

    def send_formation(self, uid: int, target: Tuple[float, float]) -> bool:
        try:
            amsl = self.home_amsl[uid] + self.height
            self.bus.reposition(uid, target[0], target[1], amsl)
            return True
        except Exception as exc:
            print(f"  UID {uid}: formation command error: {exc}")
            return False

    def launch_and_form(self, uid: int, target: Tuple[float, float]) -> bool:
        print(f"\n--- UID {uid}: ARM -> TAKEOFF -> FORMATION ---")
        if not self.arm_one(uid):
            return False
        if not self.takeoff_one(uid):
            return False
        for attempt in range(1, 4):
            if self.send_formation(uid, target):
                print(f"  UID {uid}: formation target sent; moving to next drone")
                return True
            time.sleep(0.3)
        print(f"  UID {uid}: formation command failed")
        return False

    def build_formation(self, targets: Dict[int, Tuple[float, float]]) -> None:
        print("\n=== BUILDING L FORMATION ===")
        print("Only the current drone is armed at any moment.")
        print("5 bottom drones first; then 10 vertical drones.")

        for uid in BOTTOM_UIDS:
            if self.launch_and_form(uid, targets[uid]):
                self.active.append(uid)

        print(f"Bottom leg commands sent: {self.active}")

        for uid in VERTICAL_UIDS:
            if self.launch_and_form(uid, targets[uid]):
                self.active.append(uid)

        print(f"Active formation UIDs: {self.active}")
        if not self.active:
            raise RuntimeError("No drone successfully took off.")

    def slot_error(self, uid: int, target: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        p = self.position(uid)
        if p is None:
            return None
        return haversine_m(p[0], p[1], target[0], target[1]), abs(p[2] - self.height)

    def verify_formation(self, targets: Dict[int, Tuple[float, float]], timeout: float = 90.0) -> None:
        print("\n=== VERIFYING L FORMATION ===")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            bad = []
            for uid in list(self.active):
                err = self.slot_error(uid, targets[uid])
                if err is None or err[0] > self.tol or err[1] > self.alt_tol:
                    bad.append(uid)
            if not bad:
                print("Formation verified for active drones:", self.active)
                return
            for uid in bad:
                self.send_formation(uid, targets[uid])
            time.sleep(1.0)
        print("Formation verification timeout; continuing with active drones:", self.active)

    def neighbor_distances(self) -> Dict[int, float]:
        out = {}
        for uid in self.active:
            p = self.position(uid)
            if p is None:
                continue
            nearest = float("inf")
            for other in self.active:
                if other == uid:
                    continue
                q = self.position(other)
                if q is None:
                    continue
                nearest = min(nearest, haversine_m(p[0], p[1], q[0], q[1]))
            if nearest < float("inf"):
                out[uid] = nearest
        return out

    def verify_relay(self) -> None:
        distances = self.neighbor_distances()
        violations = {uid: d for uid, d in distances.items() if d > MAX_RELAY_DISTANCE_M + 2.0}
        if violations:
            print("WARNING: relay-distance violations:", violations)
        else:
            print("Relay check: every active drone has a neighbour within 100 m.")

    def survey_targets(self, corner_east: float) -> Dict[int, Tuple[float, float]]:
        if self.origin is None:
            raise RuntimeError("Origin missing")
        lat0, lon0 = self.origin
        return {
            uid: offset_to_latlon(lat0, lon0, corner_east + self.offsets[uid][0], self.offsets[uid][1])
            for uid in self.active
        }

    def move_survey(self, corner_east: float) -> None:
        targets = self.survey_targets(corner_east)
        print(f"Moving complete active formation to survey x={corner_east:.0f} m")
        for uid, target in targets.items():
            self.send_formation(uid, target)

        deadline = time.monotonic() + self.formation_timeout
        while time.monotonic() < deadline:
            remaining = []
            for uid, target in targets.items():
                err = self.slot_error(uid, target)
                if err is None or err[0] > self.tol or err[1] > self.alt_tol:
                    remaining.append(uid)
            if not remaining:
                print("Survey endpoint reached.")
                self.verify_relay()
                return
            for uid in remaining:
                self.send_formation(uid, targets[uid])
            time.sleep(1.0)
        raise RuntimeError(f"Survey movement timeout; remaining UIDs: {remaining}")

    def survey(self) -> None:
        if len(self.active) < 2:
            print("Not enough active drones for a relay survey; skipping survey.")
            return
        print("\n=== SURVEY MODE ===")
        self.verify_relay()
        self.move_survey(self.survey_width)
        if self.survey_passes >= 2:
            self.move_survey(0.0)
        print("Survey complete.")

    def land_all(self) -> None:
        print("\nLanding all 15 vehicles...")
        for uid in UIDS:
            p = self.position(uid)
            if p is None:
                continue
            try:
                self.bus.command_long(uid, CMD_NAV_LAND, (0, 0, 0, math.nan, p[0], p[1], 0))
            except Exception as exc:
                print(f"  UID {uid}: land command error: {exc}")

    def hold(self) -> None:
        if self.hold_s <= 0:
            return
        print(f"Holding formation for {self.hold_s:.0f} s. Ctrl+C to land.")
        end = time.monotonic() + self.hold_s
        while time.monotonic() < end:
            time.sleep(1.0)

    def run(self) -> None:
        self.start()
        try:
            self.wait_for_telemetry()
            self.set_origin()
            targets = self.targets()
            self.build_formation(targets)
            self.verify_formation(targets)
            self.survey()
            self.hold()
        finally:
            self.land_all()
            time.sleep(3.0)
            self.stop()


def main() -> int:
    p = argparse.ArgumentParser(description="PX4/Gazebo 15-drone x500 L formation and survey")
    p.add_argument("--altitude", type=float, default=2.0)
    p.add_argument("--spacing", type=float, default=100.0)
    p.add_argument("--survey-width", type=float, default=1000.0)
    p.add_argument("--survey-passes", type=int, default=2)
    p.add_argument("--connect-timeout", type=float, default=90.0)
    p.add_argument("--arm-timeout", type=float, default=12.0)
    p.add_argument("--takeoff-timeout", type=float, default=40.0)
    p.add_argument("--formation-timeout", type=float, default=300.0)
    p.add_argument("--position-tolerance", type=float, default=8.0)
    p.add_argument("--altitude-tolerance", type=float, default=0.5)
    p.add_argument("--hold", type=float, default=120.0)
    args = p.parse_args()

    try:
        import pymavlink  # noqa: F401
    except ImportError:
        print("ERROR: pymavlink is not installed. Run: python3 -m pip install pymavlink")
        return 1

    c = Controller(args)
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        c.run()
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C received; landing command sent.")
        return 130
    except Exception as exc:
        print(f"\nCONTROLLER ERROR: {exc}")
        try:
            c.land_all()
        except Exception:
            pass
        try:
            c.stop()
        except Exception:
            pass
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
