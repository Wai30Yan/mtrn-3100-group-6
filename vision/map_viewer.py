#!/usr/bin/env python3
# =============================================================================
#  MTRN3100 Micromouse - 4.3 Autonomous Mapping: live map visualisation.
#
#  The spec requires "a visualisation of the map ... shown on the screen with a
#  % completion score" while the robot explores. The robot (Rust firmware)
#  prints one line per update over USB CDC:
#
#      MAP,<row>,<col>,<wall_bitmask NESW>,<visited_pct>
#
#  e.g.  MAP,3,4,5,42   (cell 3,4 has N+S walls, 42% of cells visited)
#
#  Usage:
#      python map_viewer.py --port /dev/tty.usbmodem1101
#      python map_viewer.py --demo            # replay a built-in sample
#
#  AI ASSISTANCE (assignment 5.1): written with the assistance of a generative
#  AI (Anthropic Claude).
# =============================================================================
import argparse
import sys
import time

import cv2
import numpy as np

import mazelib as ml

K = 60  # px per cell for the display


def draw(grid_walls, visited, pct, n):
    S = n * K
    img = np.full((S + 60, S, 3), 30, dtype=np.uint8)
    for r in range(n):
        for c in range(n):
            if visited[r, c]:
                cv2.rectangle(img, (c * K + 1, r * K + 1),
                              ((c + 1) * K - 1, (r + 1) * K - 1), (60, 90, 60), -1)
    for r in range(n):
        for c in range(n):
            m = grid_walls[r, c]
            x, y = c * K, r * K
            if m >> ml.N & 1:
                cv2.line(img, (x, y), (x + K, y), (255, 255, 255), 2)
            if m >> ml.S & 1:
                cv2.line(img, (x, y + K), (x + K, y + K), (255, 255, 255), 2)
            if m >> ml.W & 1:
                cv2.line(img, (x, y), (x, y + K), (255, 255, 255), 2)
            if m >> ml.E & 1:
                cv2.line(img, (x + K, y), (x + K, y + K), (255, 255, 255), 2)
    cv2.putText(img, f"mapping: {pct}% complete", (10, S + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    return img


def lines_from_serial(port, baud):
    import serial  # pyserial
    with serial.Serial(port, baud, timeout=1) as s:
        while True:
            line = s.readline().decode(errors="replace").strip()
            if line:
                yield line


def lines_demo():
    import random
    random.seed(7)
    n = 9
    cells = [(r, c) for r in range(n) for c in range(n)]
    random.shuffle(cells)
    for i, (r, c) in enumerate(cells):
        yield f"MAP,{r},{c},{random.randint(0, 15)},{int(100 * (i + 1) / len(cells))}"
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description="live mapping visualisation (4.3)")
    ap.add_argument("--port", help="robot serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--n", type=int, default=9)
    ap.add_argument("--demo", action="store_true", help="replay a fake run")
    args = ap.parse_args()

    if not args.demo and not args.port:
        ap.error("--port or --demo required")

    n = args.n
    walls = np.zeros((n, n), dtype=np.uint8)
    visited = np.zeros((n, n), dtype=bool)
    pct = 0
    source = lines_demo() if args.demo else lines_from_serial(args.port, args.baud)

    cv2.imshow("mapping", draw(walls, visited, pct, n))
    for line in source:
        if not line.startswith("MAP,"):
            continue  # ignore the robot's other debug output
        try:
            _, r, c, m, p = line.split(",")
            r, c, m, pct = int(r), int(c), int(m), int(p)
        except ValueError:
            print(f"bad line: {line!r}", file=sys.stderr)
            continue
        if 0 <= r < n and 0 <= c < n:
            walls[r, c] = m & 0xF
            visited[r, c] = True
        cv2.imshow("mapping", draw(walls, visited, pct, n))
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
