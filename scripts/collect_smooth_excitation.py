#!/usr/bin/env python3
"""Collect identification data using smooth, repeatable multisine excitation.

The command sent to the cart is a sum of sinusoids with randomized phases and
slightly randomized frequencies. It is deterministic for a given --seed,
band-limited, smooth and richer than a fixed left/right motion sequence.
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
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
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
    ts, enc, pos, speed, acc, limit, fault, enabled, soft, start = (
        struct.unpack(TLM_FMT, payload)
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


def wrap_counts(count: int, center: int, cpr: int) -> int:
    error = count - center
    return ((error + cpr // 2) % cpr) - cpr // 2


def smoothstep01(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


class Multisine:
    def __init__(
        self,
        seed: int,
        components: int,
        min_freq_hz: float,
        max_freq_hz: float,
        peak_hz: float,
        duration_s: float,
    ) -> None:
        if components < 1:
            raise ValueError("--components must be positive")
        if min_freq_hz <= 0.0 or max_freq_hz <= min_freq_hz:
            raise ValueError("frequency range must satisfy 0 < min < max")

        rng = random.Random(seed)
        if components == 1:
            base_freqs = [(min_freq_hz + max_freq_hz) * 0.5]
        else:
            ratio = max_freq_hz / min_freq_hz
            base_freqs = [
                min_freq_hz * ratio ** (i / (components - 1))
                for i in range(components)
            ]

        self.freqs = [
            max(min_freq_hz, min(max_freq_hz, f * rng.uniform(0.93, 1.07)))
            for f in base_freqs
        ]
        self.phases = [rng.uniform(0.0, 2.0 * math.pi) for _ in self.freqs]
        self.weights = [1.0 / math.sqrt(f) for f in self.freqs]

        # Normalize against the actual peak over the requested experiment.
        samples = max(2000, int(duration_s * max_freq_hz * 100.0))
        raw_peak = 0.0
        for i in range(samples + 1):
            t_s = duration_s * i / samples
            raw_peak = max(raw_peak, abs(self.raw(t_s)))
        self.scale = peak_hz / raw_peak if raw_peak > 0.0 else 0.0

    def raw(self, t_s: float) -> float:
        return sum(
            weight * math.sin(2.0 * math.pi * freq * t_s + phase)
            for freq, phase, weight in zip(
                self.freqs, self.phases, self.weights
            )
        )

    def value(self, t_s: float) -> float:
        return self.scale * self.raw(t_s)


class StateEstimator:
    def __init__(self, theta_center: int, encoder_cpr: int, alpha: float) -> None:
        self.theta_center = theta_center
        self.encoder_cpr = encoder_cpr
        self.counts_to_rad = 2.0 * math.pi / encoder_cpr
        self.alpha = max(0.0, min(1.0, alpha))
        self.prev_ts_us: int | None = None
        self.prev_theta: float | None = None
        self.prev_position: int | None = None
        self.theta_dot = 0.0
        self.cart_velocity = 0.0

    def update(self, telemetry: dict) -> dict:
        theta_counts = wrap_counts(
            telemetry["encoder_count"], self.theta_center, self.encoder_cpr
        )
        theta = theta_counts * self.counts_to_rad
        dt_s = 0.0
        if self.prev_ts_us is not None:
            dt_s = (telemetry["esp_ts_us"] - self.prev_ts_us) * 1e-6

        if dt_s > 0.0 and self.prev_theta is not None:
            # Wrapped difference avoids a false velocity spike at +/-pi.
            delta = (theta - self.prev_theta + math.pi) % (2.0 * math.pi) - math.pi
            raw_omega = delta / dt_s
            self.theta_dot = (
                self.alpha * self.theta_dot + (1.0 - self.alpha) * raw_omega
            )
        if dt_s > 0.0 and self.prev_position is not None:
            raw_velocity = (
                telemetry["position_steps"] - self.prev_position
            ) / dt_s
            self.cart_velocity = (
                self.alpha * self.cart_velocity
                + (1.0 - self.alpha) * raw_velocity
            )

        self.prev_ts_us = telemetry["esp_ts_us"]
        self.prev_theta = theta
        self.prev_position = telemetry["position_steps"]
        return {
            "dt_s": dt_s,
            "theta_counts": theta_counts,
            "theta_rad": theta,
            "theta_dot_rad_s": self.theta_dot,
            "cart_velocity_steps_s": self.cart_velocity,
        }


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Collect pendulum data with smooth multisine excitation."
    )
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--duration-s", type=float, default=20.0)
    ap.add_argument("--experiments", type=int, default=1)
    ap.add_argument("--repeat", action="store_true")
    ap.add_argument(
        "--outdir",
        default=str(Path(__file__).resolve().parent / "logs_excitation"),
    )

    ap.add_argument("--peak-hz", type=float, default=3500.0)
    ap.add_argument("--min-freq-hz", type=float, default=0.12)
    ap.add_argument("--max-freq-hz", type=float, default=2.2)
    ap.add_argument("--components", type=int, default=9)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--ramp-s", type=float, default=1.5)
    ap.add_argument("--acc", type=int, default=190000)
    ap.add_argument("--max-speed-hz", type=float, default=6000.0)

    ap.add_argument(
        "--position-gain",
        type=float,
        default=0.12,
        help="Centering correction in Hz per cart step.",
    )
    ap.add_argument(
        "--velocity-gain",
        type=float,
        default=0.01,
        help="Cart damping in Hz per (step/s).",
    )
    ap.add_argument("--max-position-steps", type=int, default=10500)
    ap.add_argument("--zero-cart", action="store_true")
    ap.add_argument("--reset-fault", action="store_true")

    ap.add_argument("--theta-center-counts", type=int, default=4001)
    ap.add_argument("--encoder-ppr", type=int, default=2000)
    ap.add_argument("--decode", choices=["x2", "x4"], default="x4")
    ap.add_argument("--filter-alpha", type=float, default=0.85)
    return ap.parse_args()


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
            print("START is ON. Switch it OFF, then ON.")
            warned = True


def output_path(outdir: str, trial: int) -> Path:
    directory = Path(outdir)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return directory / f"smooth_excitation_{stamp}_trial{trial:02d}.csv"


def run_trial(
    ser: serial.Serial,
    parser: Parser,
    args: argparse.Namespace,
    trial: int,
    encoder_cpr: int,
) -> tuple[str, int, Path]:
    signal = Multisine(
        seed=args.seed + trial - 1,
        components=args.components,
        min_freq_hz=args.min_freq_hz,
        max_freq_hz=args.max_freq_hz,
        peak_hz=args.peak_hz,
        duration_s=args.duration_s,
    )
    estimator = StateEstimator(
        args.theta_center_counts, encoder_cpr, args.filter_alpha
    )
    path = output_path(args.outdir, trial)
    rows = 0
    stop_reason = "completed"

    if args.zero_cart:
        ser.write(build(CMD_SET_ZERO))
    ser.write(build(CMD_SET_ACC, struct.pack("<I", args.acc)))
    ser.write(build(CMD_SET_ENABLE, bytes([1])))

    print(
        f"Trial {trial}: seed={args.seed + trial - 1}, "
        f"frequencies={', '.join(f'{f:.3f}' for f in signal.freqs)} Hz"
    )
    print(f"Writing {path}")

    fields = [
        "pc_t_s", "esp_ts_us", "dt_s", "trial_idx", "seed",
        "encoder_count", "theta_counts", "theta_rad", "theta_dot_rad_s",
        "position_steps", "cart_velocity_steps_s",
        "envelope", "excitation_speed_hz", "centering_speed_hz",
        "damping_speed_hz", "command_speed_hz", "applied_speed_hz",
        "applied_acc", "limit", "fault", "enabled", "soft_limit", "start",
        "stop_reason",
    ]

    t0 = time.perf_counter()
    last_keepalive = t0
    last_print = t0

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
                state = estimator.update(telemetry)

                ramp_in = smoothstep01(t_s / max(0.001, args.ramp_s))
                ramp_out = smoothstep01(
                    (args.duration_s - t_s) / max(0.001, args.ramp_s)
                )
                envelope = min(ramp_in, ramp_out)
                excitation = envelope * signal.value(t_s)
                centering = -args.position_gain * telemetry["position_steps"]
                damping = -args.velocity_gain * state["cart_velocity_steps_s"]
                command = max(
                    -args.max_speed_hz,
                    min(
                        args.max_speed_hz,
                        excitation + centering + damping,
                    ),
                )

                if telemetry["fault"]:
                    stop_reason = "fault"
                elif telemetry["limit"]:
                    stop_reason = "limit"
                elif telemetry["soft_limit"]:
                    stop_reason = "soft_limit"
                elif abs(telemetry["position_steps"]) > args.max_position_steps:
                    stop_reason = "position_limit"
                elif not telemetry["start"]:
                    stop_reason = "start_off"
                elif t_s >= args.duration_s:
                    stop_reason = "completed"

                if stop_reason != "completed" or t_s >= args.duration_s:
                    command = 0.0
                send_speed(ser, command)

                writer.writerow({
                    "pc_t_s": f"{t_s:.6f}",
                    "esp_ts_us": telemetry["esp_ts_us"],
                    "dt_s": f"{state['dt_s']:.6f}",
                    "trial_idx": trial,
                    "seed": args.seed + trial - 1,
                    "encoder_count": telemetry["encoder_count"],
                    "theta_counts": state["theta_counts"],
                    "theta_rad": f"{state['theta_rad']:.8f}",
                    "theta_dot_rad_s": f"{state['theta_dot_rad_s']:.8f}",
                    "position_steps": telemetry["position_steps"],
                    "cart_velocity_steps_s": (
                        f"{state['cart_velocity_steps_s']:.3f}"
                    ),
                    "envelope": f"{envelope:.6f}",
                    "excitation_speed_hz": f"{excitation:.3f}",
                    "centering_speed_hz": f"{centering:.3f}",
                    "damping_speed_hz": f"{damping:.3f}",
                    "command_speed_hz": f"{command:.3f}",
                    "applied_speed_hz": (
                        f"{telemetry['applied_speed_hz']:.3f}"
                    ),
                    "applied_acc": telemetry["applied_acc"],
                    "limit": telemetry["limit"],
                    "fault": telemetry["fault"],
                    "enabled": telemetry["enabled"],
                    "soft_limit": telemetry["soft_limit"],
                    "start": telemetry["start"],
                    "stop_reason": (
                        stop_reason
                        if stop_reason != "completed" or t_s >= args.duration_s
                        else ""
                    ),
                })
                rows += 1

                if now - last_keepalive >= 0.05:
                    ser.write(build(CMD_PING))
                    last_keepalive = now
                if now - last_print >= 0.5:
                    last_print = now
                    print(
                        f"t={t_s:5.1f}s theta={state['theta_rad']:+.3f} "
                        f"pos={telemetry['position_steps']:+6d} "
                        f"exc={excitation:+7.1f} cmd={command:+7.1f}Hz"
                    )

                if stop_reason != "completed" or t_s >= args.duration_s:
                    break
    finally:
        shutdown_drive(ser)

    return stop_reason, rows, path


def main() -> int:
    args = parse_args()
    if args.duration_s <= 0.0 or args.peak_hz <= 0.0:
        print("--duration-s and --peak-hz must be positive")
        return 2
    if args.peak_hz > args.max_speed_hz:
        print("--peak-hz cannot exceed --max-speed-hz")
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
