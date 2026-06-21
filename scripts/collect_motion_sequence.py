#!/usr/bin/env python3
"""Open-loop motion sequence collector for pendulum experiments.

Procedure:
  1. Start this script with START=OFF.
  2. Put the pendulum in the desired initial state, e.g. hanging down.
  3. Press START. The script enables the drive, runs one cyclic sequence,
     logs telemetry to a CSV file, then disables the drive.
  4. Return START to OFF, set the next initial state, press START again.

Each START runs the same parameterized sequence and writes a separate CSV file.
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

import serial  # pyserial


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
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def build(msg_id: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_id, len(payload)]) + payload
    return bytes([SOF]) + body + struct.pack("<H", crc16(body))


class Parser:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.telemetry_queue: deque[dict] = deque()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buf.extend(data)
        out = []
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
            msg_id = frame[1]
            payload = frame[3:3 + length]
            rx_crc = struct.unpack("<H", frame[3 + length:5 + length])[0]
            if crc16(frame[1:3 + length]) == rx_crc:
                out.append((msg_id, payload))
        return out


def decode_telemetry(payload: bytes) -> dict | None:
    if len(payload) != TLM_SIZE:
        return None
    ts, enc, pos, spd, acc, limit, fault, en, soft, start = struct.unpack(
        TLM_FMT, payload
    )
    return {
        "esp_ts_us": ts,
        "encoder_count": enc,
        "position_steps": pos,
        "applied_speed_hz": spd,
        "applied_acc": acc,
        "limit": limit,
        "fault": fault,
        "enabled": en,
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


def send_speed(ser: serial.Serial, hz: float) -> None:
    ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", float(hz))))


def shutdown_drive(ser: serial.Serial) -> None:
    try:
        send_speed(ser, 0.0)
        ser.write(build(CMD_SET_ENABLE, bytes([0])))
    except serial.SerialException:
        pass


def wrap_counts(count: int, center: int, cpr: int) -> int:
    err = count - center
    return ((err + cpr // 2) % cpr) - cpr // 2


def build_cycle_sequence(
    speed_hz: float,
    move_s: float,
    stop_s: float,
    cycles: int,
    first_direction: str,
) -> list[dict]:
    if speed_hz <= 0.0:
        raise ValueError("--speed-hz must be positive")
    if move_s <= 0.0:
        raise ValueError("--move-s must be positive")
    if stop_s < 0.0:
        raise ValueError("--stop-s cannot be negative")
    if cycles <= 0:
        raise ValueError("--cycles must be positive")

    sign = 1.0 if first_direction == "positive" else -1.0
    seq = []
    for cycle_idx in range(1, cycles + 1):
        seq.append({
            "cycle_idx": cycle_idx,
            "phase": "move_first",
            "speed_hz": sign * speed_hz,
            "duration_s": move_s,
        })
        if stop_s > 0.0:
            seq.append({
                "cycle_idx": cycle_idx,
                "phase": "stop_first",
                "speed_hz": 0.0,
                "duration_s": stop_s,
            })
        seq.append({
            "cycle_idx": cycle_idx,
            "phase": "move_second",
            "speed_hz": -sign * speed_hz,
            "duration_s": move_s,
        })
        if stop_s > 0.0:
            seq.append({
                "cycle_idx": cycle_idx,
                "phase": "stop_second",
                "speed_hz": 0.0,
                "duration_s": stop_s,
            })
    return seq


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Run open-loop cart motion sequences and log pendulum telemetry."
    )
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--outdir", default="scripts/logs_sequence")
    ap.add_argument("--speed-hz", type=float, default=1500.0,
                    help="Absolute speed for both directions.")
    ap.add_argument("--move-s", type=float, default=0.25,
                    help="Time spent moving in each direction.")
    ap.add_argument("--stop-s", type=float, default=0.40,
                    help="Stop time after each move.")
    ap.add_argument("--cycles", type=int, default=3,
                    help="Number of left/right cycles per START.")
    ap.add_argument("--first-direction", choices=["positive", "negative"],
                    default="positive",
                    help="Sign of the first speed command. Swap if left/right is reversed.")
    ap.add_argument("--experiments", type=int, default=1,
                    help="How many START-triggered experiment files to collect.")
    ap.add_argument("--repeat", action="store_true",
                    help="Keep waiting for more START edges after --experiments.")
    ap.add_argument("--acc", type=int, default=190000)
    ap.add_argument("--max-speed-hz", type=float, default=25000.0)
    ap.add_argument("--max-position-steps", type=int, default=11000)
    ap.add_argument("--noise-hz", type=float, default=0.0,
                    help="Random additive speed noise amplitude in Hz.")
    ap.add_argument("--noise-period-s", type=float, default=0.10,
                    help="How often to draw a new noise target.")
    ap.add_argument("--noise-alpha", type=float, default=0.15,
                    help="Smoothing factor toward each noise target, 1=no smoothing.")
    ap.add_argument("--noise-on-stop", action="store_true",
                    help="Also apply speed noise during stop phases.")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--reset-fault", action="store_true")
    ap.add_argument("--zero-cart", action="store_true",
                    help="Send SET_ZERO before every trial. Use only from a known cart zero.")
    ap.add_argument("--theta-center-counts", type=int, default=4001)
    ap.add_argument("--encoder-ppr", type=int, default=2000)
    ap.add_argument("--decode", choices=["x2", "x4"], default="x4")
    return ap.parse_args()


def make_output_path(outdir: str, trial_idx: int) -> Path:
    path = Path(outdir)
    path.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return path / f"motion_sequence_{stamp}_trial{trial_idx:02d}.csv"


def write_sample(
    writer: csv.DictWriter,
    pc_t_s: float,
    tlm: dict,
    theta_counts: int,
    theta_rad: float,
    trial_idx: int,
    segment_idx: int,
    segment_t_s: float,
    cycle_idx: int,
    phase: str,
    base_hz: float,
    noise_hz: float,
    command_hz: float,
    stop_reason: str = "",
) -> None:
    writer.writerow({
        "pc_t_s": f"{pc_t_s:.6f}",
        "esp_ts_us": tlm["esp_ts_us"],
        "trial_idx": trial_idx,
        "segment_idx": segment_idx,
        "segment_t_s": f"{segment_t_s:.6f}",
        "cycle_idx": cycle_idx,
        "phase": phase,
        "encoder_count": tlm["encoder_count"],
        "theta_counts": theta_counts,
        "theta_rad": f"{theta_rad:.8f}",
        "position_steps": tlm["position_steps"],
        "base_speed_hz": f"{base_hz:.3f}",
        "noise_speed_hz": f"{noise_hz:.3f}",
        "command_speed_hz": f"{command_hz:.3f}",
        "applied_speed_hz": f"{tlm['applied_speed_hz']:.3f}",
        "applied_acc": tlm["applied_acc"],
        "limit": tlm["limit"],
        "fault": tlm["fault"],
        "enabled": tlm["enabled"],
        "soft_limit": tlm["soft_limit"],
        "start": tlm["start"],
        "stop_reason": stop_reason,
    })


def read_telemetry(ser: serial.Serial, parser: Parser, timeout_s: float = 0.5) -> dict | None:
    if parser.telemetry_queue:
        return parser.telemetry_queue.popleft()

    end = time.perf_counter() + timeout_s
    while time.perf_counter() < end:
        data = ser.read(512)
        if not data:
            continue
        for msg_id, payload in parser.feed(data):
            if msg_id != MSG_TELEMETRY:
                continue
            tlm = decode_telemetry(payload)
            if tlm is not None:
                parser.telemetry_queue.append(tlm)
        if parser.telemetry_queue:
            return parser.telemetry_queue.popleft()
    return None


def wait_for_start_edge(ser: serial.Serial, parser: Parser) -> bool:
    saw_off = False
    warned_on = False
    print("Waiting for START OFF -> ON...")
    while True:
        tlm = read_telemetry(ser, parser)
        if tlm is None:
            print("Telemetry timeout while waiting for START.")
            return False
        if not tlm["start"]:
            saw_off = True
            ser.write(build(CMD_PING))
            continue
        if not saw_off:
            if not warned_on:
                print("START is already ON. Switch it OFF, then ON for the next trial.")
                warned_on = True
            ser.write(build(CMD_PING))
            continue
        return True


def run_trial(
    ser: serial.Serial,
    parser: Parser,
    args: argparse.Namespace,
    trial_idx: int,
    seq: list[dict],
    encoder_cpr: int,
) -> tuple[str, int, Path]:
    out_path = make_output_path(args.outdir, trial_idx)
    counts_to_rad = 2.0 * math.pi / float(encoder_cpr)
    rows = 0
    stop_reason = "completed"
    rng = random.Random(args.seed + trial_idx - 1)
    noise_target = 0.0
    noise_value = 0.0
    next_noise_update = 0.0
    noise_alpha = max(0.0, min(1.0, args.noise_alpha))

    fieldnames = [
        "pc_t_s", "esp_ts_us", "trial_idx", "segment_idx", "segment_t_s",
        "cycle_idx", "phase",
        "encoder_count", "theta_counts", "theta_rad", "position_steps",
        "base_speed_hz", "noise_speed_hz", "command_speed_hz",
        "applied_speed_hz", "applied_acc",
        "limit", "fault", "enabled", "soft_limit", "start", "stop_reason",
    ]

    if args.zero_cart:
        ser.write(build(CMD_SET_ZERO))

    ser.write(build(CMD_SET_ACC, struct.pack("<I", args.acc)))
    ser.write(build(CMD_SET_ENABLE, bytes([1])))

    t0 = time.perf_counter()
    segment_idx = 0
    segment_start = t0
    last_keepalive = t0
    current_cmd = max(-args.max_speed_hz, min(args.max_speed_hz, seq[0]["speed_hz"]))
    send_speed(ser, current_cmd)
    print(
        f"Trial {trial_idx}: speed={args.speed_hz:.1f}Hz "
        f"limit={args.max_speed_hz:.1f}Hz "
        f"noise={args.noise_hz:.1f}Hz "
        f"move={args.move_s:.3f}s stop={args.stop_s:.3f}s "
        f"cycles={args.cycles} -> {out_path}"
    )

    try:
        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            while True:
                tlm = read_telemetry(ser, parser)
                if tlm is None:
                    stop_reason = "telemetry_timeout"
                    break

                now = time.perf_counter()
                pc_t_s = now - t0
                segment_t_s = now - segment_start
                segment = seq[segment_idx]
                duration_s = segment["duration_s"]
                base_cmd = max(
                    -args.max_speed_hz,
                    min(args.max_speed_hz, segment["speed_hz"]),
                )

                if args.noise_hz > 0.0 and pc_t_s >= next_noise_update:
                    noise_target = rng.uniform(-args.noise_hz, args.noise_hz)
                    next_noise_update = pc_t_s + max(0.001, args.noise_period_s)
                noise_value += noise_alpha * (noise_target - noise_value)
                apply_noise = args.noise_on_stop or base_cmd != 0.0
                noise_cmd = noise_value if apply_noise else 0.0
                desired_cmd = max(
                    -args.max_speed_hz,
                    min(args.max_speed_hz, base_cmd + noise_cmd),
                )

                if now - last_keepalive >= 0.05:
                    ser.write(build(CMD_PING))
                    last_keepalive = now

                if segment_t_s >= duration_s:
                    segment_idx += 1
                    if segment_idx >= len(seq):
                        stop_reason = "completed"
                        current_cmd = 0.0
                    else:
                        segment_start = now
                        segment_t_s = 0.0
                        last_keepalive = now
                        segment = seq[segment_idx]
                        base_cmd = max(
                            -args.max_speed_hz,
                            min(args.max_speed_hz, segment["speed_hz"]),
                        )
                        apply_noise = args.noise_on_stop or base_cmd != 0.0
                        noise_cmd = noise_value if apply_noise else 0.0
                        desired_cmd = max(
                            -args.max_speed_hz,
                            min(args.max_speed_hz, base_cmd + noise_cmd),
                        )

                if tlm["fault"]:
                    stop_reason = "fault"
                elif tlm["limit"]:
                    stop_reason = "limit"
                elif tlm["soft_limit"]:
                    stop_reason = "soft_limit"
                elif abs(tlm["position_steps"]) > args.max_position_steps:
                    stop_reason = "position_limit"
                elif not tlm["start"]:
                    stop_reason = "start_off"

                theta_counts = wrap_counts(
                    tlm["encoder_count"], args.theta_center_counts, encoder_cpr
                )
                theta_rad = theta_counts * counts_to_rad

                write_sample(
                    writer, pc_t_s, tlm, theta_counts, theta_rad, trial_idx,
                    min(segment_idx, len(seq) - 1), segment_t_s,
                    segment["cycle_idx"], segment["phase"], base_cmd, noise_cmd,
                    desired_cmd,
                    "" if stop_reason == "completed" and segment_idx < len(seq) else stop_reason,
                )
                rows += 1

                if stop_reason == "completed" and segment_idx < len(seq):
                    if abs(desired_cmd - current_cmd) >= 1.0:
                        current_cmd = desired_cmd
                        send_speed(ser, current_cmd)

                if stop_reason != "completed" or segment_idx >= len(seq):
                    break
    finally:
        shutdown_drive(ser)

    return stop_reason, rows, out_path


def main() -> int:
    args = parse_args()
    encoder_cpr = args.encoder_ppr * (2 if args.decode == "x2" else 4)
    try:
        seq = build_cycle_sequence(
            args.speed_hz, args.move_s, args.stop_s, args.cycles,
            args.first_direction,
        )
    except ValueError as exc:
        print(f"Bad sequence parameters: {exc}")
        return 2

    ser = open_serial(args.port, args.baud)
    parser = Parser()

    try:
        ser.write(build(CMD_SET_OUTPUT_MODE, bytes([0])))
        time.sleep(0.05)
        ser.reset_input_buffer()
        if args.reset_fault:
            ser.write(build(CMD_RESET_FAULT))

        trial_idx = 1
        while True:
            if trial_idx > args.experiments and not args.repeat:
                break

            if not wait_for_start_edge(ser, parser):
                break

            stop_reason, rows, out_path = run_trial(
                ser, parser, args, trial_idx, seq, encoder_cpr
            )
            print(f"Stopped: {stop_reason}. Rows: {rows}. CSV: {out_path}")
            trial_idx += 1

    except KeyboardInterrupt:
        print()
    finally:
        shutdown_drive(ser)
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
