#!/usr/bin/env python3
"""Collect near-upright open-loop data for neural system identification.

Modes:
  passive      - zero speed, free release near upright
  constant     - constant cart speed
  pulse        - zero / speed pulse / zero
  random_steps - piecewise-constant random speed commands

Every START OFF->ON edge starts one trial and creates one CSV file.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
import datetime as dt
import math
import random
import struct
import sys
import time
from pathlib import Path

import serial


SOF = 0xAA
CMD_PING = 0x01
CMD_SET_ENABLE = 0x02
CMD_SET_SPEED_HZ = 0x03
CMD_SET_ACC = 0x04
CMD_RESET_FAULT = 0x05
CMD_SET_ZERO = 0x07
CMD_SET_OUTPUT_MODE = 0x08
MSG_TELEMETRY = 0x84
TLM_FMT = "<QiifIBBBBB"
TLM_SIZE = struct.calcsize(TLM_FMT)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(msg_id: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_id, len(payload)]) + payload
    return bytes([SOF]) + body + struct.pack("<H", crc16(body))


class Parser:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.telemetry_queue: deque[dict] = deque()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buf.extend(data)
        frames = []
        while True:
            while self.buf and self.buf[0] != SOF:
                self.buf.pop(0)
            if len(self.buf) < 5:
                break
            length = self.buf[2]
            total = length + 5
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            del self.buf[:total]
            payload = frame[3:3 + length]
            rx_crc = struct.unpack("<H", frame[3 + length:5 + length])[0]
            if crc16(frame[1:3 + length]) == rx_crc:
                frames.append((frame[1], payload))
        return frames


def decode_telemetry(payload: bytes) -> dict | None:
    if len(payload) != TLM_SIZE:
        return None
    ts, enc, pos, speed, acc, limit, fault, enabled, soft, start = struct.unpack(
        TLM_FMT, payload
    )
    return {
        "esp_ts_us": ts,
        "encoder_count": enc,
        "position_steps": pos,
        "applied_speed_hz": speed,
        "applied_acc": acc,
        "limit": limit,
        "fault": fault,
        "enabled": enabled,
        "soft_limit": soft,
        "start": start,
    }


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.02
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def send_speed(ser: serial.Serial, speed_hz: float) -> None:
    ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", float(speed_hz))))


def shutdown_drive(ser: serial.Serial) -> None:
    try:
        send_speed(ser, 0.0)
        ser.write(build(CMD_SET_ENABLE, bytes([0])))
    except serial.SerialException:
        pass


def read_telemetry(
    ser: serial.Serial, parser: Parser, timeout_s: float = 0.5
) -> dict | None:
    if parser.telemetry_queue:
        return parser.telemetry_queue.popleft()

    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        data = ser.read(512)
        if not data:
            continue
        for msg_id, payload in parser.feed(data):
            if msg_id == MSG_TELEMETRY:
                telemetry = decode_telemetry(payload)
                if telemetry is not None:
                    parser.telemetry_queue.append(telemetry)
        if parser.telemetry_queue:
            return parser.telemetry_queue.popleft()
    return None


def wait_for_start_edge(ser: serial.Serial, parser: Parser) -> bool:
    saw_off = False
    warned = False
    print("Waiting for START OFF -> ON...")
    while True:
        telemetry = read_telemetry(ser, parser)
        if telemetry is None:
            print("Telemetry timeout while waiting for START.")
            return False
        ser.write(build(CMD_PING))
        if not telemetry["start"]:
            saw_off = True
        elif saw_off:
            return True
        elif not warned:
            print("START is ON. Switch it OFF, prepare the pendulum, then switch it ON.")
            warned = True


def wrap_counts(count: int, center: int, cpr: int) -> int:
    error = count - center
    return ((error + cpr // 2) % cpr) - cpr // 2


def parse_levels(text: str) -> list[float]:
    levels = [float(value.strip()) for value in text.split(",") if value.strip()]
    if not levels:
        raise argparse.ArgumentTypeError("at least one speed level is required")
    return levels


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect open-loop near-upright identification data."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument(
        "--mode",
        choices=["passive", "constant", "pulse", "random_steps"],
        required=True,
    )
    parser.add_argument("--experiments", type=int, default=1)
    parser.add_argument("--repeat", action="store_true")
    parser.add_argument(
        "--outdir",
        default=str(Path(__file__).resolve().parent / "logs_identification"),
    )
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--speed-hz", type=float, default=1000.0)
    parser.add_argument(
        "--return-pause-s",
        type=float,
        default=1.0,
        help="Pause after a constant-speed trial before returning the cart.",
    )
    parser.add_argument(
        "--return-speed-hz",
        type=float,
        default=2000.0,
        help="Absolute cart speed used for the unlogged return movement.",
    )
    parser.add_argument(
        "--return-tolerance-steps",
        type=int,
        default=50,
        help="Stop return movement this close to the trial start position.",
    )
    parser.add_argument(
        "--return-timeout-s",
        type=float,
        default=10.0,
        help="Maximum duration of the unlogged return movement.",
    )
    parser.add_argument(
        "--no-return",
        action="store_true",
        help="Do not return the cart after constant-speed trials.",
    )
    parser.add_argument("--pre-s", type=float, default=0.10)
    parser.add_argument("--pulse-s", type=float, default=0.10)
    parser.add_argument("--post-s", type=float, default=0.40)
    parser.add_argument("--step-min-s", type=float, default=0.08)
    parser.add_argument("--step-max-s", type=float, default=0.30)
    parser.add_argument(
        "--levels-hz",
        type=parse_levels,
        default=parse_levels("-2000,-1000,-500,0,500,1000,2000"),
        help="Comma-separated random-step speed levels.",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--acc", type=int, default=190000)
    parser.add_argument("--max-angle-rad", type=float, default=0.30)
    parser.add_argument("--max-position-steps", type=int, default=8000)
    parser.add_argument("--theta-center-counts", type=int, default=4001)
    parser.add_argument("--encoder-ppr", type=int, default=2000)
    parser.add_argument("--decode", choices=["x2", "x4"], default="x4")
    parser.add_argument("--zero-cart", action="store_true")
    parser.add_argument("--reset-fault", action="store_true")
    return parser.parse_args()


def command_for_trial(
    args: argparse.Namespace,
    t_s: float,
    rng: random.Random,
    random_state: dict,
) -> tuple[float, str]:
    if args.mode == "passive":
        return 0.0, "passive"
    if args.mode == "constant":
        return args.speed_hz, "constant"
    if args.mode == "pulse":
        if t_s < args.pre_s:
            return 0.0, "pre"
        if t_s < args.pre_s + args.pulse_s:
            return args.speed_hz, "pulse"
        return 0.0, "post"

    if t_s >= random_state["next_change"]:
        previous = random_state["command"]
        choices = [level for level in args.levels_hz if level != previous]
        random_state["command"] = rng.choice(choices or args.levels_hz)
        random_state["next_change"] = t_s + rng.uniform(
            args.step_min_s, args.step_max_s
        )
        random_state["segment"] += 1
    return random_state["command"], f"step_{random_state['segment']:03d}"


def trial_duration(args: argparse.Namespace) -> float:
    if args.mode == "pulse":
        return args.pre_s + args.pulse_s + args.post_s
    return args.duration_s


def output_path(args: argparse.Namespace, trial: int) -> Path:
    directory = Path(args.outdir) / args.mode
    directory.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return directory / f"{args.mode}_{stamp}_trial{trial:03d}.csv"


def return_cart_to_start(
    ser: serial.Serial,
    parser: Parser,
    args: argparse.Namespace,
    start_position: int,
) -> str:
    if args.return_pause_s > 0.0:
        print(
            f"Data logging finished. Return starts in "
            f"{args.return_pause_s:.1f}s."
        )
        deadline = time.perf_counter() + args.return_pause_s
        last_ping = 0.0
        while time.perf_counter() < deadline:
            now = time.perf_counter()
            if now - last_ping >= 0.05:
                ser.write(build(CMD_PING))
                last_ping = now
            read_telemetry(ser, parser, timeout_s=0.05)

    telemetry = read_telemetry(ser, parser)
    if telemetry is None:
        return "return_telemetry_timeout"
    if telemetry["fault"] or telemetry["limit"] or telemetry["soft_limit"]:
        return "return_blocked_by_safety"
    if not telemetry["start"]:
        return "return_start_off"

    current_position = telemetry["position_steps"]
    error = start_position - current_position
    if abs(error) <= args.return_tolerance_steps:
        return "return_not_needed"

    direction = 1.0 if error > 0 else -1.0
    return_speed = direction * abs(args.return_speed_hz)
    ser.write(build(CMD_SET_ENABLE, bytes([1])))
    send_speed(ser, return_speed)
    print(
        f"Returning cart: {current_position:+d} -> {start_position:+d} steps "
        f"at {return_speed:+.0f} Hz (not logged)."
    )

    deadline = time.perf_counter() + args.return_timeout_s
    last_ping = 0.0
    result = "return_timeout"
    try:
        while time.perf_counter() < deadline:
            now = time.perf_counter()
            if now - last_ping >= 0.05:
                ser.write(build(CMD_PING))
                last_ping = now
            telemetry = read_telemetry(ser, parser, timeout_s=0.2)
            if telemetry is None:
                result = "return_telemetry_timeout"
                break
            if telemetry["fault"] or telemetry["limit"] or telemetry["soft_limit"]:
                result = "return_blocked_by_safety"
                break
            if not telemetry["start"]:
                result = "return_start_off"
                break

            current_position = telemetry["position_steps"]
            error = start_position - current_position
            reached = (
                abs(error) <= args.return_tolerance_steps
                or (direction > 0.0 and error < 0)
                or (direction < 0.0 and error > 0)
            )
            if reached:
                result = "return_completed"
                break
    finally:
        shutdown_drive(ser)

    return result


def run_trial(
    ser: serial.Serial,
    parser: Parser,
    args: argparse.Namespace,
    trial: int,
    encoder_cpr: int,
) -> tuple[str, int, Path]:
    path = output_path(args, trial)
    rng = random.Random(args.seed + trial - 1)
    random_state = {"command": 0.0, "next_change": 0.0, "segment": 0}
    counts_to_rad = 2.0 * math.pi / encoder_cpr
    duration_s = trial_duration(args)
    stop_reason = "completed"
    rows = 0
    last_command: float | None = None
    start_position: int | None = None

    if args.zero_cart:
        ser.write(build(CMD_SET_ZERO))
    ser.write(build(CMD_SET_ACC, struct.pack("<I", args.acc)))
    ser.write(build(CMD_SET_ENABLE, bytes([1])))
    send_speed(ser, 0.0)

    fields = [
        "pc_t_s", "esp_ts_us", "trial_idx", "mode", "phase", "seed",
        "encoder_count", "theta_counts", "theta_rad", "position_steps",
        "command_speed_hz", "applied_speed_hz", "applied_acc",
        "limit", "fault", "enabled", "soft_limit", "start", "stop_reason",
    ]
    t0 = time.perf_counter()
    last_keepalive = t0
    print(f"Trial {trial}: mode={args.mode}, output={path}")

    try:
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            while True:
                telemetry = read_telemetry(ser, parser)
                if telemetry is None:
                    stop_reason = "telemetry_timeout"
                    break

                now = time.perf_counter()
                t_s = now - t0
                if start_position is None:
                    start_position = telemetry["position_steps"]
                theta_counts = wrap_counts(
                    telemetry["encoder_count"], args.theta_center_counts, encoder_cpr
                )
                theta_rad = theta_counts * counts_to_rad
                command, phase = command_for_trial(args, t_s, rng, random_state)

                if telemetry["fault"]:
                    stop_reason = "fault"
                elif telemetry["limit"]:
                    stop_reason = "limit"
                elif telemetry["soft_limit"]:
                    stop_reason = "soft_limit"
                elif abs(theta_rad) > args.max_angle_rad:
                    stop_reason = "angle_limit"
                elif abs(telemetry["position_steps"]) > args.max_position_steps:
                    stop_reason = "position_limit"
                elif not telemetry["start"]:
                    stop_reason = "start_off"
                elif t_s >= duration_s:
                    stop_reason = "completed"

                if stop_reason != "completed" or t_s >= duration_s:
                    command = 0.0
                if last_command is None or abs(command - last_command) >= 1.0:
                    send_speed(ser, command)
                    last_command = command

                writer.writerow({
                    "pc_t_s": f"{t_s:.6f}",
                    "esp_ts_us": telemetry["esp_ts_us"],
                    "trial_idx": trial,
                    "mode": args.mode,
                    "phase": phase,
                    "seed": args.seed + trial - 1,
                    "encoder_count": telemetry["encoder_count"],
                    "theta_counts": theta_counts,
                    "theta_rad": f"{theta_rad:.8f}",
                    "position_steps": telemetry["position_steps"],
                    "command_speed_hz": f"{command:.3f}",
                    "applied_speed_hz": f"{telemetry['applied_speed_hz']:.3f}",
                    "applied_acc": telemetry["applied_acc"],
                    "limit": telemetry["limit"],
                    "fault": telemetry["fault"],
                    "enabled": telemetry["enabled"],
                    "soft_limit": telemetry["soft_limit"],
                    "start": telemetry["start"],
                    "stop_reason": stop_reason if stop_reason != "completed" or t_s >= duration_s else "",
                })
                rows += 1

                if now - last_keepalive >= 0.05:
                    ser.write(build(CMD_PING))
                    last_keepalive = now
                if stop_reason != "completed" or t_s >= duration_s:
                    break
    finally:
        shutdown_drive(ser)

    if (
        args.mode == "constant"
        and not args.no_return
        and start_position is not None
        and stop_reason not in {
            "fault",
            "limit",
            "soft_limit",
            "telemetry_timeout",
            "start_off",
        }
    ):
        return_result = return_cart_to_start(
            ser, parser, args, start_position
        )
        print(f"Cart return: {return_result}")

    return stop_reason, rows, path


def validate_args(args: argparse.Namespace) -> str | None:
    if args.experiments < 1:
        return "--experiments must be positive"
    if args.duration_s <= 0.0:
        return "--duration-s must be positive"
    if args.max_angle_rad <= 0.0 or args.max_position_steps <= 0:
        return "safety limits must be positive"
    if (
        args.return_pause_s < 0.0
        or args.return_speed_hz <= 0.0
        or args.return_tolerance_steps < 0
        or args.return_timeout_s <= 0.0
    ):
        return "return parameters must be non-negative and speeds/timeouts positive"
    if args.mode == "pulse" and (
        args.pre_s < 0.0 or args.pulse_s <= 0.0 or args.post_s < 0.0
    ):
        return "pulse timing must satisfy pre>=0, pulse>0, post>=0"
    if args.mode == "random_steps" and (
        args.step_min_s <= 0.0 or args.step_max_s < args.step_min_s
    ):
        return "random-step timing must satisfy 0 < min <= max"
    return None


def main() -> int:
    args = parse_args()
    error = validate_args(args)
    if error:
        print(error)
        return 2

    encoder_cpr = args.encoder_ppr * (2 if args.decode == "x2" else 4)
    ser = open_serial(args.port, args.baud)
    parser = Parser()
    try:
        ser.write(build(CMD_SET_OUTPUT_MODE, bytes([0])))
        time.sleep(0.05)
        ser.reset_input_buffer()
        if args.reset_fault:
            ser.write(build(CMD_RESET_FAULT))

        trial = 1
        while args.repeat or trial <= args.experiments:
            if not wait_for_start_edge(ser, parser):
                return 1
            reason, rows, path = run_trial(
                ser, parser, args, trial, encoder_cpr
            )
            print(f"Stopped: {reason}. Rows: {rows}. CSV: {path}")
            trial += 1
    except KeyboardInterrupt:
        print()
    finally:
        shutdown_drive(ser)
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
