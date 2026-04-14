#!/usr/bin/env python3
"""Send a Python script to Isaac Sim's VS Code extension executor via TCP socket.

The isaacsim.code_editor.vscode extension listens on port 8226 (default).
Protocol: connect -> send script bytes -> receive JSON response -> connection closed.

Usage:
    ./run_in_isaac.py examples/hand_on_1_amr.py
    ./run_in_isaac.py examples/hand_on_2_franka.py --wait   # retry until Isaac Sim is ready
"""
import argparse
import json
import socket
import sys
import time


def run_in_isaac(script_path: str, host: str, port: int) -> dict:
    with open(script_path) as f:
        code = f.read()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(600)
    try:
        sock.connect((host, port))
        sock.sendall(code.encode())
        # Half-close write side so server knows we're done sending
        sock.shutdown(socket.SHUT_WR)
        # Read full response (server closes connection when done)
        chunks = []
        while True:
            try:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                chunks.append(chunk)
            except socket.timeout:
                break
    finally:
        sock.close()

    raw = b"".join(chunks).decode()
    if not raw:
        return {"status": "ok", "output": "(no response)"}
    return json.loads(raw)


def main():
    parser = argparse.ArgumentParser(description="Run a Python script inside Isaac Sim")
    parser.add_argument("script", help="Path to the .py file to execute")
    parser.add_argument("--host", default="127.0.0.1", help="Isaac Sim host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8226, help="Executor port (default: 8226)")
    parser.add_argument("--wait", action="store_true",
                        help="Retry connection until Isaac Sim is ready (useful on first boot)")
    args = parser.parse_args()

    if args.wait:
        print(f"Waiting for Isaac Sim at {args.host}:{args.port}...", end="", flush=True)
        for attempt in range(60):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2)
                s.connect((args.host, args.port))
                s.close()
                print(" ready!")
                break
            except (ConnectionRefusedError, socket.timeout, OSError):
                print(".", end="", flush=True)
                time.sleep(5)
        else:
            print("\nERROR: Isaac Sim did not become ready after 5 minutes.")
            print("Check: docker compose logs isaac-sim")
            sys.exit(1)

    print(f"Sending {args.script} to Isaac Sim at {args.host}:{args.port}...")
    try:
        reply = run_in_isaac(args.script, args.host, args.port)
    except ConnectionRefusedError:
        print("ERROR: Connection refused. Is Isaac Sim running?")
        print("  Start it with:  docker compose up -d")
        print("  Or wait for it: ./run_in_isaac.py script.py --wait")
        sys.exit(1)

    if reply.get("output"):
        print(reply["output"])

    if reply["status"] == "error":
        print(f"\n{'=' * 50}", file=sys.stderr)
        print(f"ERROR: {reply.get('ename', '?')}: {reply.get('evalue', '?')}", file=sys.stderr)
        for tb in reply.get("traceback", []):
            print(tb, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
