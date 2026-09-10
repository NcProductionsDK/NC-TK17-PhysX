"""Summarize predicted versus next-sampled body-chain motion; no file writes."""
from pathlib import Path
import argparse
import re
import statistics


def summarize(path):
    pattern = re.compile(
        r'body-contact pose-audit target="([^"]+)" person=(\d+).*?'
        r'camera_hold=(\d+).*?error=([\d.eE+-]+) '
        r'predicted_move=([\d.eE+-]+) observed_move=([\d.eE+-]+)'
    )
    groups = {}
    for line in path.read_text(errors="replace").splitlines():
        match = pattern.search(line)
        if not match:
            continue
        target, person, held, error, predicted, observed = match.groups()
        groups.setdefault((target, person, held), []).append(
            tuple(map(float, (error, predicted, observed)))
        )
    if not groups:
        print("No body-contact pose-audit records. Reproduce contact with the diagnostic DLL and debug enabled.")
        return
    for (target, person, held), rows in sorted(groups.items()):
        print(f"{target} person={person} camera_hold={held} samples={len(rows)}")
        print(f"  prediction error: median={statistics.median(r[0] for r in rows):.7f}, max={max(r[0] for r in rows):.7f}")
        moving = [r for r in rows if r[1] > .0001]
        if moving:
            print(f"  predicted movement >0.0001: {len(moving)}; observed movement <10% of prediction: {sum(r[2] < .1*r[1] for r in moving)}")
    print("Sampled errors do not by themselves distinguish axis mismatch, latency, body motion, or held geometry. Inspect matching pose-joint records.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path,
                        default=Path(__file__).resolve().parents[2] / "The Klub 17/Logs/NC-TK17-PhysX.log")
    summarize(parser.parse_args().log)
