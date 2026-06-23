import argparse
import socket
import struct
import time

import numpy as np


IMAGE_W = 240
IMAGE_H = 240
CHUNK_DATA_SIZE = 1400
UDPPACKETHEADER_SIZE = 9
MINI_HDR_SIZE = 8
CMD_IMAGE_CHUNK = 19


def make_frame(width, height, frame_id):
    x = np.arange(width, dtype=np.uint8)
    y = np.arange(height, dtype=np.uint8)[:, None]
    return ((x + y + frame_id * 3) & 0xFF).astype(np.uint8).tobytes()


def send_frame(sock, host, port, frame_id, raw):
    total_chunks = (len(raw) + CHUNK_DATA_SIZE - 1) // CHUNK_DATA_SIZE
    for idx in range(total_chunks):
        chunk = raw[idx * CHUNK_DATA_SIZE:(idx + 1) * CHUNK_DATA_SIZE]
        payload_sz = MINI_HDR_SIZE + len(chunk)
        header = (
            b"RCSA"
            + struct.pack("<H", frame_id & 0xFFFF)
            + bytes([CMD_IMAGE_CHUNK])
            + struct.pack("<H", payload_sz)
        )
        mini_header = (
            bytes([0x01])
            + struct.pack("<H", idx)
            + struct.pack("<H", total_chunks)
            + bytes([0x00, 0x00, 0x00])
        )
        sock.sendto(header + mini_header + chunk, (host, port))


def main():
    parser = argparse.ArgumentParser(description="Send synthetic UDP image frames to receive_stream.py.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5009)
    parser.add_argument("--width", type=int, default=IMAGE_W)
    parser.add_argument("--height", type=int, default=IMAGE_H)
    parser.add_argument("--fps", type=float, default=10.0)
    args = parser.parse_args()

    delay = 1.0 / max(args.fps, 0.1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame_id = 0
    print(f"[test_sender] sending to {args.host}:{args.port} image={args.width}x{args.height}")
    while True:
        raw = make_frame(args.width, args.height, frame_id)
        send_frame(sock, args.host, args.port, frame_id, raw)
        frame_id += 1
        time.sleep(delay)


if __name__ == "__main__":
    main()
