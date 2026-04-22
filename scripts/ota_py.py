#!/usr/bin/env python3
"""
scripts/ota_py.py — OTA upload that sends firmware in small TLS records.

Why: the STM32's mbedTLS has SSL_MAX_CONTENT_LEN=4096. curl/OpenSSL sends
TLS records up to 16 KB by default, exceeding the firmware's receive buffer.
Python's ssl.SSLSocket.write() sends exactly one TLS record per call, so
writing 4096 bytes at a time keeps every record within the firmware's limit.

Usage:
    python3 scripts/ota_py.py [path/to/zephyr.signed.bin]

Defaults:
    BIN  — /home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.bin
    HOST — 192.168.1.80
    TOKEN — read from zephyr/src/http/http_auth.hpp
"""

import ssl
import socket
import sys
import os
import re
import time

# ── Configuration ──────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
DEFAULT_BIN = "/home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.bin"
AUTH_HPP    = os.path.join(REPO_ROOT, "zephyr/src/http/http_auth.hpp")
HOST        = os.environ.get("HDV_HOST", "192.168.1.80")
PORT        = 443
# Write in chunks <= SSL_MAX_CONTENT_LEN on the STM32 so each Python write()
# produces exactly one TLS record that mbedTLS can receive.
CHUNK       = 4096

# ── Helpers ───────────────────────────────────────────────────────────────────
def get_token():
    env = os.environ.get("HDV_TOKEN", "")
    if env:
        return env
    if not os.path.exists(AUTH_HPP):
        sys.exit(f"[error] {AUTH_HPP} not found. Set HDV_TOKEN=<hex> or run gen_http_token.sh")
    with open(AUTH_HPP) as f:
        m = re.search(r'HDV_API_TOKEN\[\] = "([^"]+)"', f.read())
    if not m:
        sys.exit("[error] Token not found in http_auth.hpp")
    return m.group(1)


def make_ssl_context():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    ctx.set_ciphers("ECDHE-RSA-AES128-GCM-SHA256")
    return ctx


def connect(ctx):
    raw = socket.create_connection((HOST, PORT), timeout=30)
    return ctx.wrap_socket(raw, server_hostname=HOST)


def send_chunked(sock, data):
    """Send bytes in CHUNK-sized writes so each SSL record <= CHUNK."""
    sent = 0
    while sent < len(data):
        n = sock.write(data[sent: sent + CHUNK])
        sent += n
    return sent


def read_response(sock):
    """Read HTTP response until connection closes or we see a blank line + body."""
    buf = b""
    sock.settimeout(30)
    try:
        while True:
            chunk = sock.read(4096)
            if not chunk:
                break
            buf += chunk
    except (ssl.SSLError, socket.timeout, OSError):
        pass
    return buf.decode(errors="replace")


def http_post(ctx, path, body: bytes, token: str, content_type: str = "application/octet-stream"):
    """
    Send an HTTP/1.0 POST with the given body split across small TLS records.
    Returns (status_code, response_body_str).
    """
    header = (
        f"POST {path} HTTP/1.0\r\n"
        f"Host: {HOST}\r\n"
        f"Authorization: Bearer {token}\r\n"
        f"Content-Type: {content_type}\r\n"
        f"Content-Length: {len(body)}\r\n"
        f"Connection: close\r\n"
        f"\r\n"
    ).encode()

    sock = connect(ctx)
    try:
        send_chunked(sock, header)
        # Send firmware body in 4096-byte chunks — one TLS record each.
        total = 0
        pct_prev = -1
        for offset in range(0, len(body), CHUNK):
            slice_ = body[offset: offset + CHUNK]
            n = sock.write(slice_)
            total += n
            pct = total * 100 // len(body)
            if pct != pct_prev:
                bar = "#" * (pct // 5) + "-" * (20 - pct // 5)
                print(f"\r  [{bar}] {pct:3d}%  {total}/{len(body)} B ", end="", flush=True)
                pct_prev = pct
        print()  # newline after progress
        resp = read_response(sock)
    finally:
        sock.close()

    # Parse status line
    lines = resp.split("\r\n")
    status = 0
    if lines:
        parts = lines[0].split()
        if len(parts) >= 2:
            try:
                status = int(parts[1])
            except ValueError:
                pass
    body_resp = resp.split("\r\n\r\n", 1)[1] if "\r\n\r\n" in resp else resp
    return status, body_resp


def http_post_empty(ctx, path, token: str):
    """POST with no body (reboot)."""
    header = (
        f"POST {path} HTTP/1.0\r\n"
        f"Host: {HOST}\r\n"
        f"Authorization: Bearer {token}\r\n"
        f"Content-Length: 0\r\n"
        f"Connection: close\r\n"
        f"\r\n"
    ).encode()
    try:
        sock = connect(ctx)
        send_chunked(sock, header)
        sock.settimeout(5)
        try:
            sock.read(4096)
        except Exception:
            pass
        sock.close()
    except Exception:
        pass  # board resets — connection drop expected


def check_alive(ctx, token: str, timeout: int = 15):
    header = (
        f"GET / HTTP/1.0\r\n"
        f"Host: {HOST}\r\n"
        f"Authorization: Bearer {token}\r\n"
        f"Connection: close\r\n"
        f"\r\n"
    ).encode()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            sock = connect(ctx)
            send_chunked(sock, header)
            resp = read_response(sock)
            sock.close()
            lines = resp.split("\r\n")
            if lines:
                parts = lines[0].split()
                if len(parts) >= 2:
                    code = int(parts[1])
                    if code in (200, 302):
                        return True
        except Exception:
            time.sleep(1)
    return False


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    bin_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BIN
    token = get_token()

    if not os.path.exists(bin_path):
        sys.exit(f"[error] Binary not found: {bin_path}")

    with open(bin_path, "rb") as f:
        firmware = f.read()

    bin_kb = (len(firmware) + 1023) // 1024

    print("=====================================================")
    print("  HDV-Sim OTA Firmware Update (Python / small records)")
    print("=====================================================")
    print(f"  Target : https://{HOST}")
    print(f"  Binary : {bin_path}")
    print(f"  Size   : {bin_kb} KB ({len(firmware)} bytes)")
    print(f"  Token  : {token[:8]}…")
    print(f"  Chunk  : {CHUNK} B per TLS record")
    print("=====================================================")

    ctx = make_ssl_context()

    # Step 1: board reachable?
    print("\n[1/3] Checking board is reachable…")
    if not check_alive(ctx, token, timeout=15):
        sys.exit(f"[error] Board not responding at https://{HOST}:443\n"
                 "        Check cable, IP, and that the board has booted.")
    print("      OK — board is up.")

    # Step 2: upload
    print("\n[2/3] Uploading firmware…")
    print("      (Sending in 4096-byte TLS records — one per ssl.write())")
    status, body = http_post(ctx, "/api/firmware", firmware, token)

    if status == 200:
        m = re.search(r'"bytes":(\d+)', body)
        written = m.group(1) if m else "?"
        print(f"      Upload OK — {written} bytes written to slot1.")
    elif status == 401:
        sys.exit("[error] Authentication failed (HTTP 401). Token mismatch?")
    elif status == 503:
        sys.exit("[error] Another upload already in progress (HTTP 503).")
    else:
        sys.exit(f"[error] Upload failed (HTTP {status}): {body}")

    # Step 3: reboot
    print("\n[3/3] Triggering reboot…")
    http_post_empty(ctx, "/api/reboot", token)
    print("      Reboot command sent.")
    print("\n=====================================================")
    print("  MCUboot will swap slot1 → slot0 on next boot.")
    print("  Board will be back in ~10 s.")
    print("=====================================================\n")

    print("  Waiting for board to come back online…")
    for i in range(1, 31):
        time.sleep(1)
        print(f"  [{i:2d}s] ", end="", flush=True)
        try:
            sock = connect(ctx)
            header = (
                f"GET / HTTP/1.0\r\nHost: {HOST}\r\n"
                f"Authorization: Bearer {token}\r\nConnection: close\r\n\r\n"
            ).encode()
            send_chunked(sock, header)
            resp = read_response(sock)
            sock.close()
            lines = resp.split("\r\n")
            if lines:
                parts = lines[0].split()
                if len(parts) >= 2 and int(parts[1]) in (200, 302):
                    print("board is back up!")
                    print("\n  OTA complete.")
                    sys.exit(0)
        except Exception:
            print("waiting…", end="\r", flush=True)

    print("\n  Board did not respond within 30 s.")
    print("  Check UART: picocom -b 115200 /dev/ttyACM0")
    sys.exit(1)


if __name__ == "__main__":
    main()
