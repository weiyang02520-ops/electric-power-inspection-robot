"""Generate conservative static PNG curves from recorded evaluator CSV data."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any


PLOT_LABEL = "SYNTHETIC SOFTWARE VALIDATION — NOT REAL ROBOT DATA"


def _float(row: dict[str, str], key: str) -> float | None:
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def generate_plots(run_dir: Path) -> dict[str, Any]:
    """Write plots from existing samples only; never synthesize missing values."""
    samples_path = run_dir / "samples.csv"
    plots_dir = run_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    if not samples_path.exists():
        marker = plots_dir / "NOT_ENOUGH_DATA_FOR_PLOTS"
        marker.write_text("samples.csv is absent; no plot data were available.\n", encoding="utf-8")
        return {"files": [], "warning": "NOT_ENOUGH_DATA_FOR_PLOTS"}
    rows = _read(samples_path)
    if len(rows) < 2:
        marker = plots_dir / "NOT_ENOUGH_DATA_FOR_PLOTS"
        marker.write_text("fewer than two recorded samples; no curve was invented.\n", encoding="utf-8")
        return {"files": [], "warning": "NOT_ENOUGH_DATA_FOR_PLOTS"}
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        marker = plots_dir / "NOT_ENOUGH_DATA_FOR_PLOTS"
        marker.write_text(f"matplotlib unavailable ({type(exc).__name__}); no plot was invented.\n", encoding="utf-8")
        return {"files": [], "warning": "NOT_ENOUGH_DATA_FOR_PLOTS"}

    x = [_float(row, "elapsed_time") for row in rows]
    phase = [row.get("phase", "") for row in rows]
    files: list[str] = []

    def save(name: str, series: list[tuple[str, str]]) -> None:
        fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        for label, key in series:
            ys = [_float(row, key) for row in rows]
            if not any(value is not None for value in ys):
                continue
            ax.plot(x, ys, label=label, linewidth=1.8)
        ax.set_xlabel("elapsed time (s)")
        ax.set_title(f"{name.replace('_', ' ').title()}\n{PLOT_LABEL}", fontsize=10)
        ax.grid(True, alpha=0.3)
        if ax.lines:
            ax.legend(loc="best")
        fig.savefig(plots_dir / f"{name}.png", dpi=140)
        plt.close(fig)
        files.append(str(plots_dir / f"{name}.png"))

    save("gnss_quality", [("HDOP", "gnss_hdop"), ("satellites", "gnss_satellites"), ("differential_age", "gnss_differential_age")])
    save("lidar_quality", [("geometry score", "geometry_score"), ("temporal match", "temporal_match_ratio"), ("angular coverage", "angular_coverage")])
    save("fusion_confidence", [("measurement confidence", "measurement_confidence"), ("position uncertainty", "position_uncertainty"), ("adaptive position uncertainty", "adaptive_position_uncertainty")])

    fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
    state_rows = [("gnss_state", "GNSS"), ("lidar_state", "LiDAR"), ("navigation_state", "Navigation"), ("relocalization_state", "Relocalization")]
    states = sorted({row.get(key, "") for key, _ in state_rows for row in rows if row.get(key, "")})
    ymap = {state: idx for idx, state in enumerate(states)}
    for key, label in state_rows:
        xx, yy = [], []
        for t, row in zip(x, rows):
            state = row.get(key, "")
            if t is not None and state in ymap:
                xx.append(t); yy.append(ymap[state])
        if xx:
            ax.step(xx, yy, where="post", label=label, linewidth=1.6)
    ax.set_yticks(list(ymap.values()), list(ymap.keys()))
    ax.set_xlabel("elapsed time (s)")
    ax.set_title(f"State timeline\n{PLOT_LABEL}", fontsize=10)
    ax.grid(True, alpha=0.3)
    if ax.lines:
        ax.legend(loc="best")
    fig.savefig(plots_dir / "state_timeline.png", dpi=140)
    plt.close(fig)
    files.append(str(plots_dir / "state_timeline.png"))
    (plots_dir / "README.txt").write_text("Plots use only evaluator samples.csv; missing values are omitted, never fabricated.\n", encoding="utf-8")
    return {"files": files, "warning": None}

