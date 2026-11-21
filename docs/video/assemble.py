#!/usr/bin/env python3
"""Cut the recorded frames into one side-by-side comparison.

Alignment: Gazebo numbers saved frames from the moment the sensor starts, on
the simulation clock, so frame N of a run is at RIG_SPAWNED + N/30 simulated
seconds. The disturbance start is read back out of the recorded base trace
rather than assumed, and both sides are cut to the same window around it. That
is what makes the two halves of the screen show the same instant of the same
waveform.

The labels carry each run's own measured RMS, computed here from that run's
world-frame tool trace - not copied from the report. Label text is kept as a
list of lines and joined when the file is written, so no escape sequence ever
has to survive a shell, a Python literal and ffmpeg's own parser.
"""

import csv
import glob
import os
from pathlib import Path
import re
import subprocess
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
FPS = 30
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
LEAD = 0.7          # seconds of frame shown before the shake starts
LENGTH = 8.0        # seconds of each comparison


def trace(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([int(r["%time"]) for r in rows], float) / 1e9
    xyz = np.array([[float(r[f"field.pose.pose.position.{a}"]) for a in "xyz"]
                    for r in rows])
    return t, xyz


def spawn_time(run):
    text = (OUT / run / "stage.log").read_text()
    return float(re.search(r"RIG_SPAWNED sim_time=([\d.]+)", text).group(1))


def measure(run, axis):
    """Shake start in simulated time, and the tool RMS over the steady window."""
    tb, base = trace(OUT / run / "base.csv")
    te, tool = trace(OUT / run / "ee.csv")
    swing = base[:, axis] - base[0, axis]
    moving = np.abs(swing) > 0.002
    start, end = tb[moving][0], tb[moving][-1]
    reference = tool[te < start - 0.2].mean(axis=0)
    window = (te >= start + 1.5) & (te <= end - 1.0)
    distance = np.linalg.norm(tool[window] - reference, axis=1)
    return start, 1000 * np.sqrt((distance ** 2).mean())


def pattern(run, camera):
    first = sorted(glob.glob(str(OUT / run / "frames" / camera / "*.jpg")))[0]
    stem = re.sub(r"\d{4}\.jpg$", "", os.path.basename(first))
    return str(OUT / run / "frames" / camera / f"{stem}%04d.jpg")


def label_file(name, lines):
    folder = OUT / "labels"
    folder.mkdir(parents=True, exist_ok=True)
    target = folder / f"{name}.txt"
    target.write_text("\n".join(lines), encoding="utf-8")
    return str(target)


def side(run, camera, axis):
    start, rms = measure(run, axis)
    first = round((start - LEAD - spawn_time(run)) * FPS)
    return dict(run=run, pattern=pattern(run, camera), start=max(int(first), 0),
                rms=rms)


def build(part, left, right, header, left_lines, right_lines, camera, axis):
    a, b = side(left, camera, axis), side(right, camera, axis)
    labels = {
        "head": label_file(f"{part}_head", header),
        "left": label_file(f"{part}_left",
                           [line.format(rms=a["rms"]) for line in left_lines]),
        "right": label_file(f"{part}_right",
                            [line.format(rms=b["rms"]) for line in right_lines]),
    }
    # expansion=none: drawtext otherwise reads '%' as the start of a strftime
    # escape and silently drops the whole line, which is what swallowed a
    # header carrying per-cent figures.
    text = (f"fontfile={FONT}:fontcolor=white:fontsize=25:line_spacing=8:"
            "expansion=none:x=(w-text_w)/2:y=14")
    graph = (
        f"[0:v]scale=960:540,pad=960:620:0:80:color=0x111111,"
        f"drawtext={text}:textfile={labels['left']}[l];"
        f"[1:v]scale=960:540,pad=960:620:0:80:color=0x111111,"
        f"drawtext={text}:textfile={labels['right']}[r];"
        f"[l][r]hstack=inputs=2,pad=1920:724:0:104:color=0x111111,"
        f"drawtext=fontfile={FONT}:fontcolor=0xffd080:fontsize=23:"
        f"expansion=none:x=(w-text_w)/2:y=16:line_spacing=6:"
        f"textfile={labels['head']}[v]"
    )
    output = OUT / f"{part}.mp4"
    command = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-framerate", str(FPS), "-start_number", str(a["start"]), "-i", a["pattern"],
        "-framerate", str(FPS), "-start_number", str(b["start"]), "-i", b["pattern"],
        "-filter_complex", graph, "-map", "[v]",
        "-frames:v", str(int(LENGTH * FPS)), "-c:v", "libx264", "-preset", "slow",
        "-crf", "20", "-pix_fmt", "yuv420p", str(output),
    ]
    subprocess.run(command, check=True)
    print(f"{part}: {left} RMS {a['rms']:.3f} mm from frame {a['start']}, "
          f"{right} RMS {b['rms']:.3f} mm from frame {b['start']} -> {output.name}")
    return output


def main():
    parts = [
        build(
            "part1_old_vs_new", "seg1_old", "seg1_new",
            ["第一段　基座 Y 向 20 mm、1 Hz 平移扰动，两侧同一条波形",
             "广角 0.78 mm/像素　红杆固定于世界系，是不动的基准"],
            ["四年前的代码（基线 3888830）", "末端三维位移 RMS {rms:.2f} mm"],
            ["现在的代码", "末端三维位移 RMS {rms:.2f} mm"],
            "wide", 1),
        build(
            "part2_ff_off_vs_on", "seg2_off_a", "seg2_on_a",
            ["第二段　基座 Z 向 30 mm、3 Hz 扰动，两侧都是现在的代码，只切换动力学前馈",
             "特写 0.195 mm/像素　红杆水平、固定于世界系，垂直运动贴着它读",
             "三对重复的改善为 22.8% / 39.9% / 46.5%，此处取居中的一对"],
            ["动力学前馈　关", "末端三维位移 RMS {rms:.2f} mm"],
            ["动力学前馈　开", "末端三维位移 RMS {rms:.2f} mm"],
            "macro", 2),
    ]
    listing = OUT / "concat.txt"
    listing.write_text("".join(f"file '{p}'\n" for p in parts))
    final = OUT / "comparison.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat",
                    "-safe", "0", "-i", str(listing), "-c", "copy", str(final)],
                   check=True)
    print(f"\nfinal: {final}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
