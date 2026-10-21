#!/usr/bin/env python3
"""READ-ONLY state probe for the SO-ARM101 Feetech buses.

Covers the pre-flight checks (T1/T2/T3/T16) and the post-calibration checks
(T8/T9/T10) of the lerobot 0.6.1 upgrade plan. It never writes a register --
it talks to `scservo_sdk` directly rather than going through a lerobot
`MotorsBus`, so no configure/calibrate write path is reachable from here.

What it reports, per motor:

  Phase (0x12)         bit 0x10 set => STS3215 is in MULTI-TURN mode, which is
                       what lets Present_Position escape [0, 4095] and produces
                       the 4095/8190 wrist_roll drift. lerobot >=0.6.0 clears
                       this bit in `configure_motors()` (PR #3373).
  Present_Position     raw, sign-magnitude decoded (sign bit 15). Anything
                       outside [0, 4095] is a live overflow.
  Homing_Offset        sign-magnitude, sign bit 11 => valid magnitude <= 2047.
                       `set_half_turn_homings()` computes `pos - 2047` and
                       `encode_sign_magnitude()` RAISES above 2047, so a motor
                       reading > 4094 will crash calibration mid-run.
  Min/Max_Position_Limit
                       the EEPROM copy of the calibrated range. Compared
                       against the on-disk calibration JSON to identify which
                       arm is on which port without unplugging anything.

Usage:

    python probe_feetech_state.py
    python probe_feetech_state.py --ports COM3 --watch --id 5
    python probe_feetech_state.py --json out.json
"""

import argparse
import json
import sys
import time
from pathlib import Path

BAUDRATE = 1_000_000
PROTOCOL = 0
RESOLUTION = 4096

# STS3215 control table, mirrored from
# lerobot/motors/feetech/tables.py (STS_SMS_SERIES_CONTROL_TABLE).
ADDR = {
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Phase": (18, 1),
    "Homing_Offset": (31, 2),
    "Torque_Enable": (40, 1),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
}

# Sign-magnitude bit index per register (STS_SMS_SERIES_ENCODINGS_TABLE).
SIGN_BIT = {
    "Homing_Offset": 11,
    "Present_Position": 15,
}

MOTOR_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

PHASE_MULTITURN_BIT = 0x10


def decode_sign_magnitude(encoded: int, sign_bit_index: int) -> int:
    """Mirror of lerobot.motors.encoding_utils.decode_sign_magnitude."""
    direction_bit = (encoded >> sign_bit_index) & 1
    magnitude = encoded & ((1 << sign_bit_index) - 1)
    return -magnitude if direction_bit else magnitude


class Bus:
    """Minimal read-only Feetech bus."""

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        import scservo_sdk as scs

        self.scs = scs
        self.port = port
        self.ph = scs.PortHandler(port)
        if not self.ph.openPort():
            raise RuntimeError(f"Could not open port {port}")
        if not self.ph.setBaudRate(baudrate):
            raise RuntimeError(f"Could not set baudrate {baudrate} on {port}")
        self.pkt = scs.PacketHandler(PROTOCOL)

    def ping(self, motor_id: int):
        model, comm, _err = self.pkt.ping(self.ph, motor_id)
        return model if comm == self.scs.COMM_SUCCESS else None

    def read(self, name: str, motor_id: int):
        addr, size = ADDR[name]
        if size == 1:
            val, comm, _err = self.pkt.read1ByteTxRx(self.ph, motor_id, addr)
        else:
            val, comm, _err = self.pkt.read2ByteTxRx(self.ph, motor_id, addr)
        if comm != self.scs.COMM_SUCCESS:
            return None
        if name in SIGN_BIT:
            return decode_sign_magnitude(val, SIGN_BIT[name])
        return val

    def close(self):
        self.ph.closePort()


def probe_motor(bus: Bus, motor_id: int) -> dict | None:
    model = bus.ping(motor_id)
    if model is None:
        return None
    row = {"id": motor_id, "name": MOTOR_NAMES.get(motor_id, f"id{motor_id}"), "model": model}
    for reg in (
        "Phase",
        "Lock",
        "Present_Position",
        "Homing_Offset",
        "Min_Position_Limit",
        "Max_Position_Limit",
        "Present_Voltage",
        "Present_Temperature",
        "Status",
    ):
        row[reg] = bus.read(reg, motor_id)

    pos = row["Present_Position"]
    off = row["Homing_Offset"]
    row["multiturn"] = bool(row["Phase"] is not None and row["Phase"] & PHASE_MULTITURN_BIT)
    row["pos_out_of_range"] = bool(pos is not None and not (0 <= pos <= RESOLUTION - 1))
    row["offset_unencodable"] = bool(off is not None and abs(off) > 2047)

    # Raw encoder value, independent of the stored homing offset:
    #   Present_Position = Actual_Position - Homing_Offset
    row["actual_position"] = None if (pos is None or off is None) else pos + off

    # Would set_half_turn_homings() raise if calibration ran right now?
    #
    # No -- it calls reset_calibration() FIRST, which zeroes Homing_Offset, so the
    # subsequent read returns `actual_position`, not `Present_Position`. Only a raw
    # encoder value outside [0, 4095] (i.e. genuine multi-turn) can push the computed
    # offset `actual - 2047` past the +/-2047 that encode_sign_magnitude() accepts.
    actual = row["actual_position"]
    row["would_crash_calibration"] = bool(
        actual is not None and abs(actual - (RESOLUTION // 2 - 1)) > 2047
    )

    # A stored offset that puts the resting pose outside its own stored range is
    # self-inconsistent calibration -- recalibration is the fix.
    lo, hi = row["Min_Position_Limit"], row["Max_Position_Limit"]
    row["calibration_inconsistent"] = bool(
        pos is not None and lo is not None and hi is not None and not (lo <= pos <= hi)
    )
    return row


def identify_arm(rows: list[dict], calib_root: Path) -> str:
    """Match EEPROM min/max limits against the on-disk calibration JSONs.

    Avoids the unplug-and-replug dance of `lerobot-find-port`: each arm's
    stored range is distinctive enough to name the port.
    """
    candidates = {
        "follower (so_follower)": calib_root / "robots" / "so_follower" / "my_so_arm.json",
        "leader (so_leader)": calib_root / "teleoperators" / "so_leader" / "my_so_arm.json",
    }
    eeprom = {
        r["name"]: (r["Min_Position_Limit"], r["Max_Position_Limit"])
        for r in rows
        if r["Min_Position_Limit"] is not None
    }
    best, best_score = "unknown", 0
    for label, path in candidates.items():
        if not path.is_file():
            continue
        want = json.loads(path.read_text())
        score = sum(
            1
            for name, (lo, hi) in eeprom.items()
            if name in want and want[name]["range_min"] == lo and want[name]["range_max"] == hi
        )
        if score > best_score:
            best, best_score = label, score
    if best_score == 0:
        return "unknown (EEPROM matches neither calibration file)"
    return f"{best}  [{best_score}/{len(eeprom)} joints match its JSON]"


def print_port_report(port: str, rows: list[dict], calib_root: Path) -> None:
    print(f"\n{'=' * 78}\nPORT {port}   ->  {identify_arm(rows, calib_root)}\n{'=' * 78}")
    hdr = f"{'id':>2} {'name':<14} {'phase':>5} {'lock':>4} {'pos':>7} {'homing':>7} {'min':>6} {'max':>6} {'V':>5} {'C':>3}"
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        volt = f"{r['Present_Voltage'] / 10:.1f}" if r["Present_Voltage"] is not None else "  ?"
        print(
            f"{r['id']:>2} {r['name']:<14} "
            f"{r['Phase'] if r['Phase'] is not None else '?':>5} "
            f"{r['Lock'] if r['Lock'] is not None else '?':>4} "
            f"{r['Present_Position'] if r['Present_Position'] is not None else '?':>7} "
            f"{r['Homing_Offset'] if r['Homing_Offset'] is not None else '?':>7} "
            f"{r['Min_Position_Limit'] if r['Min_Position_Limit'] is not None else '?':>6} "
            f"{r['Max_Position_Limit'] if r['Max_Position_Limit'] is not None else '?':>6} "
            f"{volt:>5} {r['Present_Temperature'] if r['Present_Temperature'] is not None else '?':>3}"
        )

    flags = []
    for r in rows:
        if r["multiturn"]:
            flags.append(f"  [T1] id{r['id']} {r['name']}: MULTI-TURN (Phase={r['Phase']}, bit 0x10 set)")
        if r["pos_out_of_range"]:
            flags.append(
                f"  [T2] id{r['id']} {r['name']}: position {r['Present_Position']} OUTSIDE [0,4095]"
            )
        if r["offset_unencodable"]:
            flags.append(
                f"  [!!] id{r['id']} {r['name']}: homing offset {r['Homing_Offset']} exceeds +/-2047"
            )
        if r["would_crash_calibration"]:
            flags.append(
                f"  [!!] id{r['id']} {r['name']}: raw encoder {r['actual_position']} is outside "
                f"[0,4095] -- set_half_turn_homings() would raise ValueError "
                f"(needs offset {r['actual_position'] - 2047})"
            )
        if r["calibration_inconsistent"]:
            flags.append(
                f"  [T3] id{r['id']} {r['name']}: resting pos {r['Present_Position']} outside its own "
                f"stored range [{r['Min_Position_Limit']},{r['Max_Position_Limit']}] -- "
                f"stored calibration is self-inconsistent (raw encoder {r['actual_position']} is fine)"
            )
        if r["Status"]:
            flags.append(f"  [hw] id{r['id']} {r['name']}: Status register = {r['Status']} (nonzero)")

    print("\nFINDINGS:")
    print("\n".join(flags) if flags else "  none - all motors single-turn and in range")


def watch(bus: Bus, motor_id: int, seconds: float, hz: float) -> None:
    """T2/T10: stream raw position while the joint is rotated by hand."""
    print(f"\nWatching id{motor_id} ({MOTOR_NAMES.get(motor_id, '?')}) on {bus.port} "
          f"for {seconds:.0f}s -- rotate it a full turn each way.")
    print(f"{'t(s)':>6} {'pos':>7} {'min_seen':>9} {'max_seen':>9}  flag")
    lo, hi = None, None
    t0 = time.time()
    period = 1.0 / hz
    while (t := time.time() - t0) < seconds:
        pos = bus.read("Present_Position", motor_id)
        if pos is not None:
            lo = pos if lo is None else min(lo, pos)
            hi = pos if hi is None else max(hi, pos)
            flag = "OUT OF RANGE" if not (0 <= pos <= RESOLUTION - 1) else ""
            print(f"{t:>6.1f} {pos:>7} {lo:>9} {hi:>9}  {flag}")
        time.sleep(period)
    print(f"\nrange seen: [{lo}, {hi}]"
          f"{'  -- OVERFLOW CONFIRMED' if hi is not None and hi > RESOLUTION - 1 else '  -- stayed in range'}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ports", default="COM3,COM4", help="comma-separated serial ports (default: COM3,COM4)")
    p.add_argument("--ids", default="1,2,3,4,5,6", help="motor ids to probe (default: 1-6)")
    p.add_argument("--baud", type=int, default=BAUDRATE)
    p.add_argument(
        "--calibration-dir",
        default=str(Path(__file__).resolve().parents[3] / "calibration"),
        help="calibration root used to identify which arm is on which port",
    )
    p.add_argument("--json", help="also write the full probe result to this JSON file")
    p.add_argument("--watch", action="store_true", help="stream one motor's position (T2/T10)")
    p.add_argument("--id", type=int, default=5, help="motor id for --watch (default 5 = wrist_roll)")
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--hz", type=float, default=5.0)
    args = p.parse_args()

    ports = [s.strip() for s in args.ports.split(",") if s.strip()]
    ids = [int(s) for s in args.ids.split(",") if s.strip()]
    calib_root = Path(args.calibration_dir)

    result: dict[str, list[dict]] = {}
    exit_code = 0

    for port in ports:
        try:
            bus = Bus(port, args.baud)
        except RuntimeError as exc:
            print(f"\n[ERROR] {exc}", file=sys.stderr)
            print("        Is the arm powered? Is another process holding the port?", file=sys.stderr)
            exit_code = 1
            continue

        try:
            rows = []
            for motor_id in ids:
                row = probe_motor(bus, motor_id)
                if row is None:
                    print(f"[WARN] {port}: id{motor_id} did not respond", file=sys.stderr)
                    continue
                rows.append(row)

            if not rows:
                print(f"[ERROR] {port}: no motors responded", file=sys.stderr)
                exit_code = 1
                continue

            # T16: port-identity guard.
            if len(rows) != 6:
                print(f"[WARN] {port}: {len(rows)}/6 motors responded -- do NOT write "
                      f"calibration to this port until resolved", file=sys.stderr)
                exit_code = 1

            result[port] = rows
            print_port_report(port, rows, calib_root)

            if args.watch and args.id in ids:
                watch(bus, args.id, args.seconds, args.hz)
        finally:
            bus.close()

    if args.json:
        Path(args.json).write_text(json.dumps(result, indent=2))
        print(f"\nwrote {args.json}")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
