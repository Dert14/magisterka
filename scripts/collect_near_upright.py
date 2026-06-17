#!/usr/bin/env python3
"""PC-side data collector and simple near-upright controller.

The ESP32 firmware remains an I/O layer. This script:
  - switches UART output to binary frames,
  - receives telemetry at the firmware rate,
  - estimates theta/theta_dot and cart velocity,
  - optionally sends a simple stabilizing speed command,
  - logs state/action samples for later modelling.

Example:
    python scripts/collect_near_upright.py --port COM5 --seconds 20
"""
from __future__ import annotations

import argparse
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
CMD_GET_STATUS = 0x06
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


def wrap_counts(count: int, center: int, cpr: int) -> int:
    err = count - center
    return ((err + cpr // 2) % cpr) - cpr // 2


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class StateEstimator:
    def __init__(self, theta_center: int, encoder_cpr: int, omega_alpha: float) -> None:
        self.theta_center = theta_center
        self.encoder_cpr = encoder_cpr
        self.counts_to_rad = 2.0 * math.pi / float(encoder_cpr)
        self.omega_alpha = omega_alpha
        self.prev_theta: float | None = None
        self.prev_pos: int | None = None
        self.prev_ts_us: int | None = None
        self.theta_dot = 0.0
        self.cart_vel_steps_s = 0.0

    def update(self, tlm: dict) -> dict:
        theta_counts = wrap_counts(
            tlm["encoder_count"], self.theta_center, self.encoder_cpr
        )
        theta_rad = theta_counts * self.counts_to_rad

        dt_s = 0.0
        if self.prev_ts_us is not None:
            dt_s = (tlm["esp_ts_us"] - self.prev_ts_us) * 1e-6

        if dt_s > 0.0 and self.prev_theta is not None:
            raw_theta_dot = (theta_rad - self.prev_theta) / dt_s
            self.theta_dot = (
                self.omega_alpha * self.theta_dot
                + (1.0 - self.omega_alpha) * raw_theta_dot
            )

        if dt_s > 0.0 and self.prev_pos is not None:
            self.cart_vel_steps_s = (
                tlm["position_steps"] - self.prev_pos
            ) / dt_s

        self.prev_theta = theta_rad
        self.prev_pos = tlm["position_steps"]
        self.prev_ts_us = tlm["esp_ts_us"]

        return {
            "dt_s": dt_s,
            "theta_counts": theta_counts,
            "theta_rad": theta_rad,
            "theta_dot_rad_s": self.theta_dot,
            "cart_vel_steps_s": self.cart_vel_steps_s,
        }


class Controller:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.next_excitation_change = 0.0
        self.excitation = 0.0
        self.rng = random.Random(args.seed)

    def command(self, t_s: float, state: dict, tlm: dict) -> float:
        if self.args.mode == "passive":
            base = 0.0
        else:
            x = float(tlm["position_steps"])
            base = (
                self.args.k_theta * state["theta_rad"]
                + self.args.k_omega * state["theta_dot_rad_s"]
                - self.args.k_x * x
                - self.args.k_v * state["cart_vel_steps_s"]
            )

        if self.args.excite_hz > 0.0 and t_s >= self.next_excitation_change:
            self.excitation = self.rng.uniform(-self.args.excite_hz, self.args.excite_hz)
            self.next_excitation_change = t_s + self.args.excite_period_s

        return clamp(base + self.excitation, -self.args.max_speed_hz, self.args.max_speed_hz)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Collect near-upright pendulum state/action data from ESP32 I/O firmware."
    )
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--outdir", default="scripts/logs_pc")
    ap.add_argument("--mode", choices=["balance", "passive"], default="balance")
    ap.add_argument("--arm", choices=["immediate", "start"], default="start",
                    help="Start immediately or wait for telemetry start_state=1.")
    ap.add_argument("--ignore-start-off", action="store_true",
                    help="Do not stop when start_state returns to 0 after arming.")
    ap.add_argument("--allow-start-already-on", action="store_true",
                    help="Allow arming if start_state is already 1 when the script starts.")
    ap.add_argument("--zero-cart", action="store_true",
                    help="Send SET_ZERO before arming. Use only after homing/centering cart.")
    ap.add_argument("--reset-fault", action="store_true")

    ap.add_argument("--theta-center-counts", type=int, default=4001)
    ap.add_argument("--encoder-ppr", type=int, default=2000)
    ap.add_argument("--decode", choices=["x2", "x4"], default="x4")
    ap.add_argument("--omega-alpha", type=float, default=0.85)

    ap.add_argument("--k-theta", type=float, default=-160000.0,
                    help="Hz/rad. Sign is matched to the current ESP32 wiring.")
    ap.add_argument("--k-omega", type=float, default=-12000.0,
                    help="Hz/(rad/s).")
    ap.add_argument("--k-x", type=float, default=0.0,
                    help="Hz/step cart centering gain.")
    ap.add_argument("--k-v", type=float, default=0.0,
                    help="Hz/(step/s) cart velocity damping gain.")
    ap.add_argument("--acc", type=int, default=190000)
    ap.add_argument("--max-speed-hz", type=float, default=10000.0)
    ap.add_argument("--max-angle-rad", type=float, default=0.35)
    ap.add_argument("--max-position-steps", type=int, default=10000)
    ap.add_argument("--excite-hz", type=float, default=0.0,
                    help="Uniform random additive excitation amplitude in Hz.")
    ap.add_argument("--excite-period-s", type=float, default=0.25)
    ap.add_argument("--seed", type=int, default=1)
    return ap.parse_args()


def send_speed(ser: serial.Serial, hz: float) -> None:
    ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", float(hz))))


def shutdown_drive(ser: serial.Serial) -> None:
    try:
        send_speed(ser, 0.0)
        ser.write(build(CMD_SET_ENABLE, bytes([0])))
    except serial.SerialException:
        pass


def make_output_path(outdir: str) -> Path:
    path = Path(outdir)
    path.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return path / f"near_upright_{stamp}.csv"


def write_sample(
    writer: csv.DictWriter,
    t_s: float,
    tlm: dict,
    state: dict,
    cmd_hz: float,
    mode: str,
    stop_reason: str = "",
) -> None:
    writer.writerow({
        "pc_t_s": f"{t_s:.6f}",
        "esp_ts_us": tlm["esp_ts_us"],
        "dt_s": f"{state['dt_s']:.6f}",
        "encoder_count": tlm["encoder_count"],
        "theta_counts": state["theta_counts"],
        "theta_rad": f"{state['theta_rad']:.8f}",
        "theta_dot_rad_s": f"{state['theta_dot_rad_s']:.8f}",
        "position_steps": tlm["position_steps"],
        "cart_vel_steps_s": f"{state['cart_vel_steps_s']:.3f}",
        "command_speed_hz": f"{cmd_hz:.3f}",
        "applied_speed_hz": f"{tlm['applied_speed_hz']:.3f}",
        "applied_acc": tlm["applied_acc"],
        "limit": tlm["limit"],
        "fault": tlm["fault"],
        "enabled": tlm["enabled"],
        "soft_limit": tlm["soft_limit"],
        "start": tlm["start"],
        "mode": mode,
        "stop_reason": stop_reason,
    })


def main() -> int:
    args = parse_args()
    encoder_cpr = args.encoder_ppr * (2 if args.decode == "x2" else 4)
    out_path = make_output_path(args.outdir)

    ser = open_serial(args.port, args.baud)
    parser = Parser()
    estimator = StateEstimator(args.theta_center_counts, encoder_cpr, args.omega_alpha)
    controller = Controller(args)

    rows_written = 0
    stop_reason = "completed"
    armed = False
    t0 = time.perf_counter()
    last_rx = t0
    last_print = t0
    saw_start_off = False

    fieldnames = [
        "pc_t_s", "esp_ts_us", "dt_s",
        "encoder_count", "theta_counts", "theta_rad", "theta_dot_rad_s",
        "position_steps", "cart_vel_steps_s",
        "command_speed_hz", "applied_speed_hz", "applied_acc",
        "limit", "fault", "enabled", "soft_limit", "start",
        "mode", "stop_reason",
    ]

    try:
        ser.write(build(CMD_SET_OUTPUT_MODE, bytes([0])))
        time.sleep(0.05)
        ser.reset_input_buffer()
        if args.reset_fault:
            ser.write(build(CMD_RESET_FAULT))
        if args.zero_cart:
            ser.write(build(CMD_SET_ZERO))
        ser.write(build(CMD_SET_ACC, struct.pack("<I", args.acc)))

        print(f"Logging to {out_path}")
        if args.arm == "start":
            print("Waiting for START telemetry state...")
        else:
            print("Arming immediately.")

        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            while True:
                now = time.perf_counter()
                if now - last_rx > 0.5:
                    stop_reason = "telemetry_timeout"
                    break
                if armed and now - t0 >= args.seconds:
                    break

                data = ser.read(512)
                if not data:
                    continue

                for msg_id, payload in parser.feed(data):
                    if msg_id != MSG_TELEMETRY:
                        continue
                    tlm = decode_telemetry(payload)
                    if tlm is None:
                        continue

                    last_rx = time.perf_counter()

                    if not armed:
                        if args.arm == "start":
                            if not tlm["start"]:
                                saw_start_off = True
                                ser.write(build(CMD_PING))
                                continue
                            if not saw_start_off and not args.allow_start_already_on:
                                stop_reason = "start_already_on"
                                break
                        armed = True
                        t0 = time.perf_counter()
                        if args.mode != "passive":
                            ser.write(build(CMD_SET_ENABLE, bytes([1])))

                    state = estimator.update(tlm)
                    t_s = time.perf_counter() - t0
                    cmd_hz = controller.command(t_s, state, tlm)

                    if (
                        args.arm == "start"
                        and not args.ignore_start_off
                        and not tlm["start"]
                    ):
                        stop_reason = "start_off"
                    elif tlm["fault"]:
                        stop_reason = "fault"
                    elif tlm["limit"]:
                        stop_reason = "limit"
                    elif tlm["soft_limit"]:
                        stop_reason = "soft_limit"
                    elif abs(state["theta_rad"]) > args.max_angle_rad:
                        stop_reason = "angle_limit"
                    elif abs(tlm["position_steps"]) > args.max_position_steps:
                        stop_reason = "position_limit"

                    if stop_reason != "completed":
                        write_sample(
                            writer, t_s, tlm, state, 0.0, args.mode, stop_reason
                        )
                        rows_written += 1
                        break

                    send_speed(ser, cmd_hz)

                    write_sample(writer, t_s, tlm, state, cmd_hz, args.mode)
                    rows_written += 1

                    if time.perf_counter() - last_print > 0.5:
                        last_print = time.perf_counter()
                        print(
                            f"t={t_s:6.2f}s theta={state['theta_rad']:+.4f} "
                            f"omega={state['theta_dot_rad_s']:+.2f} "
                            f"x={tlm['position_steps']:+6d} cmd={cmd_hz:+7.1f}Hz"
                        )

                if stop_reason != "completed":
                    break

    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
        print()
    finally:
        shutdown_drive(ser)
        ser.close()

    print(f"Stopped: {stop_reason}. Rows: {rows_written}. CSV: {out_path}")
    return 0 if rows_written > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
