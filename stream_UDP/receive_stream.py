import argparse
import os
import socket
import struct
import time
from collections import defaultdict

import cv2
import numpy as np


# Keep these values in sync with optical_flow_refactor_UDP/src/config/param_conf.h
LISTEN_PORT = 5009
IMAGE_W = 120
IMAGE_H = 120
CHUNK_DATA_SIZE = 1400

UDPPACKETHEADER_SIZE = 9
MINI_HDR_SIZE = 8
SIGNATURE = b"RCSA"
CMD_IMAGE_CHUNK = 19
FRAME_TIMEOUT_S = 5.0
STATS_INTERVAL_S = 1.0


def local_ipv4_addrs():
    addrs = set()
    try:
        hostname = socket.gethostname()
        for item in socket.getaddrinfo(hostname, None, socket.AF_INET):
            addrs.add(item[4][0])
    except OSError:
        pass

    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        addrs.add(probe.getsockname()[0])
        probe.close()
    except OSError:
        pass

    return sorted(addr for addr in addrs if not addr.startswith("127."))


def parse_packet(data: bytes):
    if len(data) < UDPPACKETHEADER_SIZE + MINI_HDR_SIZE:
        return None, "short"
    if data[:4] != SIGNATURE:
        return None, "bad_sig"

    frame_id = struct.unpack_from("<H", data, 4)[0]
    cmd = data[6]
    payload_sz = struct.unpack_from("<H", data, 7)[0]
    if cmd != CMD_IMAGE_CHUNK:
        return None, "bad_cmd"
    if payload_sz != len(data) - UDPPACKETHEADER_SIZE:
        return None, "bad_payload_size"

    mode = data[9]
    if mode != 0x01:
        return None, "bad_mode"

    chunk_idx = struct.unpack_from("<H", data, 10)[0]
    total_chunks = struct.unpack_from("<H", data, 12)[0]
    if total_chunks == 0 or chunk_idx >= total_chunks:
        return None, "bad_chunk_index"

    img_data = data[UDPPACKETHEADER_SIZE + MINI_HDR_SIZE:]
    return (frame_id, chunk_idx, total_chunks, img_data), None


def print_stats(rx_packets, rx_bytes, complete_frames, pending, dropped, fps_display=None, last_assembly_ms=None):
    drop_text = " ".join(f"{k}={v}" for k, v in sorted(dropped.items())) or "none"
    fps_text = "" if fps_display is None else f" fps={fps_display:.1f}"
    assembly_text = "" if last_assembly_ms is None else f" last_assembly_ms={last_assembly_ms:.1f}"
    print(
        f"[receiver] stats packets={rx_packets} bytes={rx_bytes} "
        f"frames={complete_frames} pending={len(pending)}{fps_text}{assembly_text} drop={drop_text}"
    )


def dump_raw_packet(prefix, data, max_bytes=64):
    shown = data[:max_bytes]
    hex_text = " ".join(f"{b:02X}" for b in shown)
    ascii_text = "".join(chr(b) if 32 <= b <= 126 else "." for b in shown)
    suffix = "" if len(data) <= max_bytes else f" ... +{len(data) - max_bytes} bytes"
    print(f"{prefix} len={len(data)} hex=[{hex_text}]{suffix} ascii=[{ascii_text}]")


def main():
    parser = argparse.ArgumentParser(description="Receive raw grayscale image frames over UDP.")
    parser.add_argument("--port", type=int, default=LISTEN_PORT)
    parser.add_argument("--bind", default="", help="Local address to bind. Empty means all interfaces.")
    parser.add_argument("--width", type=int, default=IMAGE_W)
    parser.add_argument("--height", type=int, default=IMAGE_H)
    parser.add_argument("--no-window", action="store_true", help="Print stats without opening an OpenCV window.")
    parser.add_argument("--scale", type=float, default=4.0, help="Display scale factor (default 4 → 120x120 shows as 480x480).")
    parser.add_argument("--reuse-address", action="store_true", help="Allow address reuse instead of exclusive bind.")
    parser.add_argument("--raw", action="store_true", help="Only print raw packets; do not assemble image frames.")
    parser.add_argument("--save-dir", default="", help="Directory for completed frame PNGs.")
    parser.add_argument("--save-every", type=int, default=30, help="Save every N completed frames when --save-dir is set.")
    parser.add_argument("--frame-timeout", type=float, default=FRAME_TIMEOUT_S, help="Seconds to wait for missing chunks before dropping a frame.")
    parser.add_argument("--recv-buf", type=int, default=4 * 1024 * 1024, help="Requested UDP receive buffer size in bytes.")
    parser.add_argument("--keep-old-frames", action="store_true", help="Keep older incomplete frames instead of favoring the newest frame.")
    args = parser.parse_args()

    image_size = args.width * args.height

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.recv_buf > 0:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.recv_buf)
    if args.reuse_address:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    elif hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)

    try:
        sock.bind((args.bind, args.port))
    except OSError as exc:
        bind_addr = args.bind or "0.0.0.0"
        print(f"[receiver] bind failed on {bind_addr}:{args.port}: {exc}")
        print(f"[receiver] check owner: netstat -ano -p udp | findstr :{args.port}")
        raise SystemExit(1)
    sock.settimeout(0.05)

    bind_addr = args.bind or "0.0.0.0"
    print(f"[receiver] listening on {bind_addr}:{args.port}  image={args.width}x{args.height}")
    ips = local_ipv4_addrs()
    if ips:
        print(f"[receiver] local IPv4: {', '.join(ips)}")
        print("[receiver] set PYTHON_PC_IP_* on ESP32 to the WiFi IPv4 shown above")

    pending = defaultdict(lambda: {"chunks": {}, "total": None, "first_ts": time.monotonic(), "ts": time.monotonic()})

    fps_counter = 0
    fps_ts = time.monotonic()
    fps_display = 0.0
    stats_ts = time.monotonic()
    rx_packets = 0
    rx_bytes = 0
    complete_frames = 0
    dropped = defaultdict(int)
    first_packet = True
    last_assembly_ms = None

    if not args.no_window:
        cv2.namedWindow("UDP Stream", cv2.WINDOW_NORMAL)
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    while True:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            now = time.monotonic()
            stale = [fid for fid, f in pending.items() if now - f["ts"] > args.frame_timeout]
            for fid in stale:
                dropped["stale_frame"] += 1
                del pending[fid]

            if now - stats_ts >= STATS_INTERVAL_S:
                print_stats(rx_packets, rx_bytes, complete_frames, pending, dropped, last_assembly_ms=last_assembly_ms)
                stats_ts = now

            if not args.no_window and cv2.waitKey(1) & 0xFF == ord("q"):
                break
            continue

        rx_packets += 1
        rx_bytes += len(data)
        if first_packet:
            print(f"[receiver] first packet from {addr[0]}:{addr[1]} len={len(data)}")
            dump_raw_packet("[receiver] raw first", data)
            first_packet = False

        if args.raw:
            dump_raw_packet(f"[receiver] raw from {addr[0]}:{addr[1]}", data)
            now = time.monotonic()
            if now - stats_ts >= STATS_INTERVAL_S:
                print_stats(rx_packets, rx_bytes, complete_frames, pending, dropped, last_assembly_ms=last_assembly_ms)
                stats_ts = now
            continue

        result, reason = parse_packet(data)
        if result is None:
            dropped[reason] += 1
            if rx_packets <= 20 or (rx_packets % 50) == 0:
                dump_raw_packet(f"[receiver] dropped reason={reason}", data)
            continue

        frame_id, chunk_idx, total_chunks, img_data = result
        if not args.keep_old_frames and pending:
            stale_ids = [fid for fid in pending if ((frame_id - fid) & 0xFFFF) < 0x8000 and fid != frame_id]
            for fid in stale_ids:
                dropped["superseded_frame"] += 1
                del pending[fid]

        frame = pending[frame_id]
        if not frame["chunks"]:
            frame["first_ts"] = time.monotonic()
        frame["chunks"][chunk_idx] = img_data
        frame["total"] = total_chunks
        frame["ts"] = time.monotonic()

        displayed_frame = False
        if len(frame["chunks"]) == total_chunks:
            raw = b"".join(frame["chunks"][i] for i in range(total_chunks))
            del pending[frame_id]

            if len(raw) < image_size:
                dropped["short_frame"] += 1
                continue

            gray = np.frombuffer(raw[:image_size], dtype=np.uint8).reshape((args.height, args.width))
            complete_frames += 1
            last_assembly_ms = (time.monotonic() - frame["first_ts"]) * 1000.0

            if args.save_dir and (complete_frames == 1 or complete_frames % max(args.save_every, 1) == 0):
                out_path = os.path.join(args.save_dir, f"frame_{complete_frames:06d}_{frame_id:05d}.png")
                cv2.imwrite(out_path, gray)
                print(f"[receiver] saved {out_path}")

            fps_counter += 1
            now = time.monotonic()
            if now - fps_ts >= 1.0:
                fps_display = fps_counter / (now - fps_ts)
                fps_counter = 0
                fps_ts = now

            if not args.no_window:
                disp = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                if args.scale != 1.0:
                    w = int(args.width * args.scale)
                    h = int(args.height * args.scale)
                    disp = cv2.resize(disp, (w, h), interpolation=cv2.INTER_NEAREST)
                cv2.putText(
                    disp,
                    f"FPS: {fps_display:.1f}  frame#{frame_id}",
                    (4, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )
                cv2.imshow("UDP Stream", disp)
                displayed_frame = True

        now = time.monotonic()
        if now - stats_ts >= STATS_INTERVAL_S:
            print_stats(rx_packets, rx_bytes, complete_frames, pending, dropped, fps_display, last_assembly_ms)
            stats_ts = now

        if displayed_frame and not args.no_window and cv2.waitKey(1) & 0xFF == ord("q"):
            break

    sock.close()
    if not args.no_window:
        cv2.destroyAllWindows()
    print("[receiver] stopped")


if __name__ == "__main__":
    main()
