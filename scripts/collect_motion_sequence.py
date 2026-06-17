#!/usr/bin/env python3
"""Open-loop motion sequence collector for pendulum experiments.

Procedure:
  1. Start this script with START=OFF.
  2. Put the pendulum in the desired initial state, e.g. hanging down.
  3. Press START. The script enables the drive, runs one speed sequence,
     logs telemetry to a CSV file, then disables the drive.
  4. Return START to OFF, set the next initial state, press START again.

Each START runs the next --trial sequence and writes a separate CSV file.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
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


def parse_sequence(text: str) -> list[tuple[float, float]]:
    seq = []
    for raw_part in text.split(","):
        part = raw_part.strip()
        if not part:
            continue
        if ":" not in part:
            raise ValueError(f"bad segment '{part}', expected speed_hz:seconds")
        speed_s, dur_s = part.split(":", 1)
        speed_hz = float(speed_s)
        duration_s = float(dur_s)
        if duration_s <= 0.0:
            raise ValueError(f"bad duration in '{part}'")
        seq.append((speed_hz, duration_s))
    if not seq:
        raise ValueError("empty sequence")
    return seq


def default_trials() -> list[list[tuple[float, float]]]:
    return [
        parse_sequence("1200:0.25,-1200:0.25,0:0.40"),
        parse_sequence("1800:0.25,-1800:0.25,0:0.40"),
        parse_sequence("2500:0.20,-2500:0.20,0:0.50"),
    ]


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Run open-loop cart motion sequences and log pendulum telemetry."
    )
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--outdir", default="scripts/logs_sequence")
    ap.add_argument("--trial", action="append",
                    help="Sequence as speed_hz:seconds,... Use multiple --trial values.")
    ap.add_argument("--repeat-last", action="store_true",
                    help="After the last --trial, keep repeating it on each START.")
    ap.add_argument("--acc", type=int, default=190000)
    ap.add_argument("--max-speed-hz", type=float, default=6000.0)
    ap.add_argument("--max-position-steps", type=int, default=11000)
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
    command_hz: float,
    stop_reason: str = "",
) -> None:
    writer.writerow({
        "pc_t_s": f"{pc_t_s:.6f}",
        "esp_ts_us": tlm["esp_ts_us"],
        "trial_idx": trial_idx,
        "segment_idx": segment_idx,
        "segment_t_s": f"{segment_t_s:.6f}",
        "encoder_count": tlm["encoder_count"],
        "theta_counts": theta_counts,
        "theta_rad": f"{theta_rad:.8f}",
        "position_steps": tlm["position_steps"],
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
                return tlm
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
    seq: list[tuple[float, float]],
    encoder_cpr: int,
) -> tuple[str, int, Path]:
    out_path = make_output_path(args.outdir, trial_idx)
    counts_to_rad = 2.0 * math.pi / float(encoder_cpr)
    rows = 0
    stop_reason = "completed"

    fieldnames = [
        "pc_t_s", "esp_ts_us", "trial_idx", "segment_idx", "segment_t_s",
        "encoder_count", "theta_counts", "theta_rad", "position_steps",
        "command_speed_hz", "applied_speed_hz", "applied_acc",
        "limit", "fault", "enabled", "soft_limit", "start", "stop_reason",
    ]

    if args.zero_cart:
        ser.write(build(CMD_SET_ZERO))

    ser.write(build(CMD_SET_ACC, struct.pack("<I", args.acc)))
    ser.write(build(CMD_SET_ENABLE, bytes([1])))

    t0 = time.perf_counter()
    segment_idx = 0
    segment_start = t0
    current_cmd = max(-args.max_speed_hz, min(args.max_speed_hz, seq[0][0]))
    send_speed(ser, current_cmd)
    print(f"Trial {trial_idx}: {seq} -> {out_path}")

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
                speed_hz, duration_s = seq[segment_idx]

                if segment_t_s >= duration_s:
                    segment_idx += 1
                    if segment_idx >= len(seq):
                        stop_reason = "completed"
                        current_cmd = 0.0
                    else:
                        segment_start = now
                        segment_t_s = 0.0
                        current_cmd = max(
                            -args.max_speed_hz,
                            min(args.max_speed_hz, seq[segment_idx][0]),
                        )
                        send_speed(ser, current_cmd)

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
                    min(segment_idx, len(seq) - 1), segment_t_s, current_cmd,
                    "" if stop_reason == "completed" and segment_idx < len(seq) else stop_reason,
                )
                rows += 1

                if stop_reason != "completed" or segment_idx >= len(seq):
                    break
    finally:
        shutdown_drive(ser)

    return stop_reason, rows, out_path


def main() -> int:
    args = parse_args()
    encoder_cpr = args.encoder_ppr * (2 if args.decode == "x2" else 4)
    if args.trial:
        try:
            trials = [parse_sequence(t) for t in args.trial]
        except ValueError as exc:
            print(f"Bad --trial: {exc}")
            return 2
    else:
        trials = default_trials()

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
            if trial_idx <= len(trials):
                seq = trials[trial_idx - 1]
            elif args.repeat_last:
                seq = trials[-1]
            else:
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
