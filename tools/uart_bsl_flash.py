"""Flash MSPM0G3507 MAIN through the ROM UART BSL and a CH340 bridge."""
import argparse
import struct
import time
from pathlib import Path

import serial
from serial.tools import list_ports

HEADER = b"\x80"
CMD_CONNECTION = 0x12
CMD_GET_ID = 0x19
CMD_PASSWORD = 0x21
CMD_MASS_ERASE = 0x15
CMD_PROGRAM = 0x20
CMD_START_APP = 0x40
MAIN_END = 0x00020000
PROGRAM_CHUNK = 1024


def crc32(data):
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc


def packet(command, payload=b""):
    body = bytes([command]) + payload
    return HEADER + struct.pack("<H", len(body)) + body + struct.pack("<I", crc32(body))


def parse_ti_txt(path):
    chunks = []
    address = None
    data = bytearray()
    for raw in Path(path).read_text(encoding="ascii").splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.lower() == "q":
            break
        if line.startswith("@"):
            if address is not None and data:
                chunks.append((address, bytes(data)))
            address = int(line[1:], 16)
            data = bytearray()
        else:
            if address is None:
                raise ValueError("TI-TXT data appears before an address")
            data.extend(bytes.fromhex(line))
    if address is not None and data:
        chunks.append((address, bytes(data)))
    if not chunks:
        raise ValueError("TI-TXT contains no data")
    for start, block in chunks:
        if start < 0 or start + len(block) > MAIN_END:
            raise ValueError(f"refusing non-MAIN range 0x{start:08X}..0x{start + len(block):08X}")
    frames = []
    for start, block in chunks:
        for offset in range(0, len(block), PROGRAM_CHUNK):
            frames.append((start + offset, block[offset:offset + PROGRAM_CHUNK]))
    return frames


def detect_ch340():
    matches = []
    for port in list_ports.comports():
        text = f"{port.description} {port.manufacturer or ''} {port.hwid}".lower()
        if "ch340" in text or "ch341" in text or "1a86" in text:
            matches.append(port)
    if len(matches) != 1:
        listing = ", ".join(f"{p.device}:{p.description}" for p in list_ports.comports())
        raise RuntimeError(f"expected one CH340, found {len(matches)}; ports=[{listing}]")
    return matches[0].device


def read_exact(ser, count):
    data = ser.read(count)
    if len(data) != count:
        raise RuntimeError(f"timeout: wanted {count} bytes, got {data.hex() or '<none>'}")
    return data


def ack(ser, operation):
    value = read_exact(ser, 1)[0]
    if value != 0:
        raise RuntimeError(f"{operation} packet ACK failed: 0x{value:02X}")


def status(ser, operation):
    response = read_exact(ser, 9)
    code = response[4]
    if code != 0:
        raise RuntimeError(f"{operation} status failed: 0x{code:02X}")


def transact_status(ser, command, payload, operation, retries=0):
    # The ROM may still be completing the preceding flash/authentication
    # operation even though its status packet is already on the UART.
    frame = packet(command, payload)
    for attempt in range(retries + 1):
        time.sleep(0.050)
        ser.write(frame)
        try:
            ack(ser, operation)
            status(ser, operation)
            return
        except RuntimeError:
            if attempt >= retries:
                raise
            print(f"RETRY {operation} {attempt + 1}/{retries}", flush=True)
            ser.reset_input_buffer()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--listen-seconds", type=float, default=60.0)
    parser.add_argument("--connect-delay", type=float, default=0.0)
    args = parser.parse_args()
    frames = parse_ti_txt(args.image)
    port = detect_ch340()
    print(f"CH340_PORT={port}", flush=True)
    print("BSL_LISTENING_9600_8N1", flush=True)
    print("PRESS_BSL_THEN_RST_RELEASE_RST_RELEASE_BSL", flush=True)

    with serial.Serial(port, 9600, timeout=0.35, write_timeout=5) as ser:
        if args.connect_delay > 0:
            print(f"ONE_SHOT_CONNECTION_IN={args.connect_delay:.1f}s", flush=True)
            time.sleep(args.connect_delay)
            ser.reset_input_buffer()
            ser.write(packet(CMD_CONNECTION))
            if ser.read(1) != b"\x00":
                raise RuntimeError("one-shot BSL connection failed")
            ser.write(b"\xBB")
            if ser.read(1) != b"\x51":
                raise RuntimeError("one-shot BSL sync/header probe failed")
        else:
            deadline = time.monotonic() + args.listen_seconds
            while time.monotonic() < deadline:
                ser.reset_input_buffer()
                ser.write(packet(CMD_CONNECTION))
                if ser.read(1) == b"\x00":
                    ser.write(b"\xBB")
                    if ser.read(1) == b"\x51":
                        break
                time.sleep(0.05)
            else:
                raise RuntimeError("BSL connection window expired")

        print("BSL_CONNECTED", flush=True)
        ser.timeout = 5
        ser.write(packet(CMD_GET_ID))
        device_id = read_exact(ser, 33)
        print(f"DEVICE_ID_RESPONSE={device_id.hex()}", flush=True)

        ser.write(packet(CMD_PASSWORD, b"\xFF" * 32))
        ack(ser, "password")
        status(ser, "password")
        print("BSL_UNLOCKED", flush=True)

        transact_status(ser, CMD_MASS_ERASE, b"", "mass erase MAIN")
        print("MAIN_FLASH_ERASED", flush=True)
        # Allow the flash controller and ROM parser to settle after a full
        # MAIN erase before presenting the first Program Data header.
        time.sleep(1.0)

        total = len(frames)
        for number, (address, data) in enumerate(frames, 1):
            transact_status(ser, CMD_PROGRAM, struct.pack("<I", address) + data,
                            f"program packet {number}", retries=3)
            print(f"PROGRAMMED {number}/{total}", flush=True)

        ser.write(packet(CMD_START_APP))
        ack(ser, "start application")
        print("FLASH_COMPLETE_AND_APP_STARTED", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"BSL_FLASH_ABORTED: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
