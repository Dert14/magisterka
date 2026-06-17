#!/usr/bin/env python3
"""Konsola PC do stanowiska wahadla (warstwa I/O na ESP32).

Implementuje ten sam protokol ramkowy co firmware:
    [SOF=0xAA][ID][LEN][PAYLOAD...][CRC16_LO][CRC16_HI]
CRC16-CCITT (poly 0x1021, init 0xFFFF) po [ID, LEN, PAYLOAD].

Wymaga: pyserial  (pip install pyserial)

Komunikacja idzie po tym samym porcie USB co programowanie (UART0, 460800).
ESP ma dwa tryby wyjscia, by logi nie smiecily w binarnej telemetrii:
  - BINARY - czyste ramki (ten skrypt wymusza go automatycznie),
  - DEBUG  - czytelny tekst + logi ESP_LOG (np. `pio device monitor`).

Przyklady:
    python pc_console.py --port COM5 ping
    python pc_console.py --port COM5 enable 1
    python pc_console.py --port COM5 speed 4000
    python pc_console.py --port COM5 acc 50000
    python pc_console.py --port COM5 reset
    python pc_console.py --port COM5 zero
    python pc_console.py --port COM5 status
    python pc_console.py --port COM5 monitor
    python pc_console.py --port COM5 heartbeat 8000
    python pc_console.py --port COM5 mode dbg
    python pc_console.py --port COM5 steptest 4000
    python pc_console.py --port COM5 steptest 4000 8000 5 5000
"""
import argparse
import struct
import sys
import time

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

MSG_PONG = 0x81
MSG_ACK = 0x82
MSG_NACK = 0x83
MSG_TELEMETRY = 0x84

MSG_NAME = {
    MSG_PONG: "PONG",
    MSG_ACK: "ACK",
    MSG_NACK: "NACK",
    MSG_TELEMETRY: "TLM",
}

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
    crc = crc16(body)
    return bytes([SOF]) + body + struct.pack("<H", crc)


class Parser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            while self.buf and self.buf[0] != SOF:
                self.buf.pop(0)
            if len(self.buf) < 5:
                break
            length = self.buf[2]
            total = 5 + length
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


def decode_telemetry(payload: bytes):
    if len(payload) != TLM_SIZE:
        return None
    ts, enc, pos, spd, acc, limit, fault, en, soft, start = struct.unpack(
        TLM_FMT, payload)
    return {
        "ts_us": ts,
        "enc": enc,
        "pos": pos,
        "speed_hz": spd,
        "acc": acc,
        "limit": limit,
        "fault": fault,
        "enabled": en,
        "soft": soft,
        "start": start,
    }


def pretty(msg_id: int, payload: bytes) -> str:
    name = MSG_NAME.get(msg_id, f"0x{msg_id:02X}")
    if msg_id == MSG_TELEMETRY:
        t = decode_telemetry(payload)
        if t:
            return ("TLM ts={ts_us} enc={enc} pos={pos} spd={speed_hz:.1f}Hz "
                    "acc={acc} limit={limit} fault={fault} en={enabled} "
                    "soft={soft} start={start}".format(**t))
    if payload:
        return f"{name} {payload.hex()}"
    return name


def open_serial(port: str, baud: int):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def wait_settle(ser, parser, seconds: float):
    last = None
    end = time.time() + seconds
    last_ping = 0.0
    while time.time() < end:
        now = time.time()
        if now - last_ping >= 0.05:
            ser.write(build(CMD_PING))
            last_ping = now
        data = ser.read(256)
        if data:
            for msg_id, payload in parser.feed(data):
                if msg_id == MSG_TELEMETRY:
                    t = decode_telemetry(payload)
                    if t:
                        last = t
    return last


def get_pos(ser, parser, timeout: float = 0.4):
    ser.write(build(CMD_GET_STATUS))
    end = time.time() + timeout
    last = None
    while time.time() < end:
        data = ser.read(256)
        if data:
            for msg_id, payload in parser.feed(data):
                if msg_id == MSG_TELEMETRY:
                    t = decode_telemetry(payload)
                    if t:
                        last = t
        if last is not None:
            break
    return last["pos"] if last else None


def move_to(ser, parser, target: int, hz: float, tol: int = 50,
            timeout_s: float = 20.0):
    cur = get_pos(ser, parser)
    if cur is None:
        cur = 0
    direction = 1.0 if target >= cur else -1.0
    ser.write(build(CMD_SET_ENABLE, bytes([1])))
    ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", direction * abs(hz))))

    reached = False
    last_ping = time.time()
    end = time.time() + timeout_s
    while time.time() < end and not reached:
        now = time.time()
        if now - last_ping >= 0.05:
            ser.write(build(CMD_PING))
            last_ping = now
        data = ser.read(256)
        if not data:
            continue
        for msg_id, payload in parser.feed(data):
            if msg_id != MSG_TELEMETRY:
                continue
            t = decode_telemetry(payload)
            if not t:
                continue
            pos = t["pos"]
            if (direction > 0 and pos >= target - tol) or \
               (direction < 0 and pos <= target + tol):
                reached = True
                break

    ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", 0.0)))
    t = wait_settle(ser, parser, 0.4)
    return t["pos"] if t else None


def read_for(ser, parser, seconds: float):
    end = time.time() + seconds
    while time.time() < end:
        data = ser.read(256)
        if data:
            for msg_id, payload in parser.feed(data):
                print("  <-", pretty(msg_id, payload))
        else:
            time.sleep(0.005)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="np. COM5 lub /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=460800)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")
    p = sub.add_parser("enable"); p.add_argument("val", type=int)
    p = sub.add_parser("speed"); p.add_argument("hz", type=float)
    p = sub.add_parser("acc"); p.add_argument("val", type=int)
    sub.add_parser("reset")
    sub.add_parser("zero")
    sub.add_parser("status")
    p = sub.add_parser("mode"); p.add_argument("which", choices=["bin", "dbg"])
    p = sub.add_parser("monitor"); p.add_argument("secs", type=float,
                                                   nargs="?", default=5.0)
    p = sub.add_parser("heartbeat"); p.add_argument("hz", type=float)
    p.add_argument("secs", type=float, nargs="?", default=10.0)
    p = sub.add_parser("steptest")
    p.add_argument("hz", type=float)
    p.add_argument("amp", type=int, nargs="?", default=8000)
    p.add_argument("reps", type=int, nargs="?", default=5)
    p.add_argument("small", type=int, nargs="?", default=5000)

    args = ap.parse_args()
    ser = open_serial(args.port, args.baud)
    parser = Parser()

    if args.cmd == "mode":
        ser.write(build(CMD_SET_OUTPUT_MODE,
                        bytes([1 if args.which == "dbg" else 0])))
        read_for(ser, parser, 0.3)
        return

    ser.write(build(CMD_SET_OUTPUT_MODE, bytes([0])))
    time.sleep(0.05)
    ser.reset_input_buffer()

    if args.cmd == "ping":
        ser.write(build(CMD_PING))
    elif args.cmd == "enable":
        ser.write(build(CMD_SET_ENABLE, bytes([1 if args.val else 0])))
    elif args.cmd == "speed":
        ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", args.hz)))
    elif args.cmd == "acc":
        ser.write(build(CMD_SET_ACC, struct.pack("<I", args.val)))
    elif args.cmd == "reset":
        ser.write(build(CMD_RESET_FAULT))
    elif args.cmd == "zero":
        ser.write(build(CMD_SET_ZERO))
    elif args.cmd == "status":
        ser.write(build(CMD_GET_STATUS))
    elif args.cmd == "monitor":
        read_for(ser, parser, args.secs)
        return
    elif args.cmd == "heartbeat":
        ser.write(build(CMD_SET_ENABLE, bytes([1])))
        ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", args.hz)))
        end = time.time() + args.secs
        last_ping = 0.0
        while time.time() < end:
            now = time.time()
            if now - last_ping >= 0.05:
                ser.write(build(CMD_PING))
                last_ping = now
            data = ser.read(256)
            if data:
                for msg_id, payload in parser.feed(data):
                    if msg_id == MSG_TELEMETRY:
                        print("  <-", pretty(msg_id, payload))
        ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", 0.0)))
        return
    elif args.cmd == "steptest":
        hz, amp, reps, small = args.hz, args.amp, args.reps, args.small
        ser.write(build(CMD_SET_ZERO))
        wait_settle(ser, parser, 0.3)
        print(f"steptest: {reps}x (+{amp} -> 0), potem (+{small} -> 0) @ {hz:.0f} Hz")
        print("  ZAZNACZ fizycznie pozycje startowa wozka.")

        pause_s = 5.0

        for i in range(reps):
            move_to(ser, parser, amp, hz)
            print(f"  pauza {pause_s:.0f}s...")
            wait_settle(ser, parser, pause_s)
            move_to(ser, parser, 0, hz)
            print(f"  cykl {i + 1}/{reps}: +{amp} -> 0  (zadane)")
            print(f"  pauza {pause_s:.0f}s...")
            wait_settle(ser, parser, pause_s)

        move_to(ser, parser, small, hz)
        print(f"  pauza {pause_s:.0f}s...")
        wait_settle(ser, parser, pause_s)
        move_to(ser, parser, 0, hz)
        print(f"  cykl dodatkowy: +{small} -> 0  (zadane)")

        ser.write(build(CMD_SET_SPEED_HZ, struct.pack("<f", 0.0)))
        print("--- koniec ---")
        print("  Sprawdz FIZYCZNIE, czy wozek wrocil na zaznaczona kreske startowa.")
        return

    read_for(ser, parser, 0.5)


if __name__ == "__main__":
    sys.exit(main())
