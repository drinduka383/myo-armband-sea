#!/usr/bin/env python3
import argparse
import csv
import os
import queue
import signal
import tempfile
from collections import deque
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", tempfile.gettempdir())

STATE_COLOR = {"REST": "#9ca3af", "GESTURE_REJECT": "#f59e0b", "MOTION_REJECT": "#ef4444", "TARGET": "#22c55e"}


def pyplot():
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise SystemExit("matplotlib is missing; run: pip install -r requirements.txt") from error
    return plt


def live_plot_error():
    backend = pyplot().get_backend().lower()
    if backend == "agg":
        return "Matplotlib has no interactive backend. On Fedora, install it manually with: sudo dnf install python3-tkinter"
    return None


def run_live_plot(messages, window_seconds=10.0):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    plt = pyplot()
    from matplotlib.patches import Patch
    colors = plt.cm.tab10.colors[:8]
    figure, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8), gridspec_kw={"height_ratios": (3, 2, 1)})
    raw_time, mav_time, state_time = deque(), deque(), deque()
    raw_channels = [deque() for _ in range(8)]
    mav_channels = [deque() for _ in range(8)]
    states = deque()
    raw_lines = [axes[0].plot([], [], color=colors[i], linewidth=0.8)[0] for i in range(8)]
    mav_lines = [axes[1].plot([], [], color=colors[i], label=f"CH{i + 1}")[0] for i in range(8)]
    state_points = axes[2].scatter([], [], marker="s", s=70)
    axes[0].set_ylabel("Raw EMG\n(offset by channel)")
    axes[0].set_yticks([i * 260 for i in range(8)], [f"CH{i + 1}" for i in range(8)])
    axes[0].set_ylim(-150, 7 * 260 + 150)
    axes[1].set_ylabel("MAV (200 ms)")
    axes[1].legend(ncol=8, fontsize=8, loc="upper left")
    axes[2].set_ylabel("Detector state")
    axes[2].set_yticks([])
    axes[2].set_ylim(-0.5, 0.5)
    axes[2].set_xlabel("Rolling time (s)")
    axes[2].legend(
        handles=[Patch(color=color, label=state.replace("_", " ")) for state, color in STATE_COLOR.items()],
        ncol=4,
        loc="upper left",
    )
    for axis in axes:
        axis.grid(True, alpha=0.25)

    origin = None
    while plt.fignum_exists(figure.number):
        try:
            message = messages.get(timeout=0.05)
        except queue.Empty:
            plt.pause(0.01)
            continue
        if message is None:
            break
        if origin is None:
            origin = message["time"]
        now = message["time"] - origin
        for timestamp, sample in message["raw"]:
            raw_time.append(timestamp - origin)
            for index, value in enumerate(sample):
                raw_channels[index].append(value + index * 260)
        mav_time.append(now)
        state_time.append(now)
        for index, value in enumerate(message["mav"]):
            mav_channels[index].append(value)
        states.append(message["detection"]["state"])
        cutoff = now - window_seconds
        while raw_time and raw_time[0] < cutoff:
            raw_time.popleft()
            for channel in raw_channels:
                channel.popleft()
        while mav_time and mav_time[0] < cutoff:
            mav_time.popleft()
            state_time.popleft()
            states.popleft()
            for channel in mav_channels:
                channel.popleft()
        for line, channel in zip(raw_lines, raw_channels):
            line.set_data(raw_time, channel)
        for line, channel in zip(mav_lines, mav_channels):
            line.set_data(mav_time, channel)
        state_points.set_offsets([(timestamp, 0) for timestamp in state_time])
        state_points.set_color([STATE_COLOR.get(state, "#9ca3af") for state in states])
        axes[0].set_xlim(max(0, now - window_seconds), now + 0.1)
        mav_max = max((max(channel, default=0) for channel in mav_channels), default=20)
        axes[1].set_ylim(0, max(20, mav_max * 1.15))
        detection = message["detection"]
        figure.suptitle(
            f"target={detection['chosen_target']}  orientation={detection['orientation']}  activity={detection['total_activity']:.1f}  "
            f"EMG effort={detection['emg_effort'] * 100:.0f}%  target_score={detection['target_similarity']:.2f}  "
            f"reject={detection['best_rejection_label']}:{detection['best_rejection_similarity']:.2f}  "
            f"margin={detection['score_margin']:.2f}  gyro={message['gyro']:.1f}  state={detection['state']}  command={message['command']}"
        )
        figure.canvas.draw_idle()
        plt.pause(0.01)
    plt.close(figure)


def plot_csv(csv_file, output=None):
    plt = pyplot()
    from matplotlib.patches import Patch
    with open(csv_file, newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise SystemExit("CSV has no samples")
    required = {"timestamp", "target_similarity", "best_rejection_similarity", "state", "command", *[f"mav_{i}" for i in range(1, 9)]}
    missing = required - rows[0].keys()
    if missing:
        raise SystemExit(f"CSV is missing fields: {', '.join(sorted(missing))}")
    t0 = float(rows[0]["timestamp"])
    t = [float(row["timestamp"]) - t0 for row in rows]
    colors = plt.cm.tab10.colors[:8]
    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 9))
    for index in range(8):
        axes[0].plot(t, [float(row[f"mav_{index + 1}"]) for row in rows], color=colors[index], label=f"CH{index + 1}")
    axes[0].set_ylabel("MAV")
    axes[0].legend(ncol=8, fontsize=8)
    axes[1].plot(t, [float(row["target_similarity"]) for row in rows], label="Target similarity")
    axes[1].plot(t, [float(row["best_rejection_similarity"]) for row in rows], label="Best rejection")
    axes[1].set_ylabel("Cosine similarity")
    axes[1].legend()
    axes[2].scatter(t, [0] * len(t), c=[STATE_COLOR.get(row["state"], "#9ca3af") for row in rows], marker="s", s=18)
    axes[2].set_yticks([])
    axes[2].set_ylim(-0.5, 0.5)
    axes[2].set_ylabel("Detector state")
    axes[2].legend(handles=[Patch(color=color, label=state.replace("_", " ")) for state, color in STATE_COLOR.items()], ncol=4)
    axes[3].plot(t, [float(row["force_N"]) for row in rows], label="Virtual force (N)")
    axes[3].plot(t, [float(row["theta_deg"]) for row in rows], label="Virtual angle (deg)")
    axes[3].set_ylabel("SEA model")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
    figure.tight_layout()
    output = Path(output) if output else Path(csv_file).with_suffix(".png")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    print(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    parser.add_argument("--output")
    args = parser.parse_args()
    plot_csv(args.csv_file, args.output)


if __name__ == "__main__":
    main()
