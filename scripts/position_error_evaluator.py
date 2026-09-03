#!/usr/bin/env python3
"""
Position error evaluator for DG202611 experiments.

Computes localization accuracy metrics from estimated vs ground truth positions.
"""

import sys
import csv
import math
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class PositionSample:
    """Single position measurement."""
    timestamp: float
    estimated_x: float
    estimated_y: float
    ground_truth_x: float
    ground_truth_y: float


@dataclass
class PositionErrorMetrics:
    """Position error statistics."""
    count: int
    mean: float
    median: float
    rmse: float
    p95: float
    max_error: float
    min_error: float


def compute_error(sample: PositionSample) -> float:
    """Compute Euclidean distance error for one sample."""
    dx = sample.estimated_x - sample.ground_truth_x
    dy = sample.estimated_y - sample.ground_truth_y
    return math.sqrt(dx * dx + dy * dy)


def compute_metrics(samples: List[PositionSample]) -> Optional[PositionErrorMetrics]:
    """Compute position error metrics from samples."""
    if not samples:
        return None

    errors = [compute_error(s) for s in samples]
    errors_sorted = sorted(errors)

    count = len(errors)
    mean_error = sum(errors) / count
    median_error = errors_sorted[count // 2]
    rmse = math.sqrt(sum(e * e for e in errors) / count)

    p95_index = int(count * 0.95)
    if p95_index >= count:
        p95_index = count - 1
    p95_error = errors_sorted[p95_index]

    max_error = max(errors)
    min_error = min(errors)

    return PositionErrorMetrics(
        count=count,
        mean=mean_error,
        median=median_error,
        rmse=rmse,
        p95=p95_error,
        max_error=max_error,
        min_error=min_error
    )


def read_csv(filepath: str) -> List[PositionSample]:
    """Read position samples from CSV file.

    Expected columns: timestamp, estimated_x, estimated_y, ground_truth_x, ground_truth_y
    """
    samples = []

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                sample = PositionSample(
                    timestamp=float(row['timestamp']),
                    estimated_x=float(row['estimated_x']),
                    estimated_y=float(row['estimated_y']),
                    ground_truth_x=float(row['ground_truth_x']),
                    ground_truth_y=float(row['ground_truth_y'])
                )
                samples.append(sample)
            except (KeyError, ValueError) as e:
                print(f"Warning: skipping invalid row: {e}", file=sys.stderr)
                continue

    return samples


def print_metrics(metrics: PositionErrorMetrics) -> None:
    """Print metrics in human-readable format."""
    print("Position Error Metrics:")
    print(f"  Sample Count: {metrics.count}")
    print(f"  Mean Error:   {metrics.mean:.4f} m")
    print(f"  Median Error: {metrics.median:.4f} m")
    print(f"  RMSE:         {metrics.rmse:.4f} m")
    print(f"  P95 Error:    {metrics.p95:.4f} m")
    print(f"  Max Error:    {metrics.max_error:.4f} m")
    print(f"  Min Error:    {metrics.min_error:.4f} m")


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: position_error_evaluator.py <csv_file>", file=sys.stderr)
        print("", file=sys.stderr)
        print("CSV format: timestamp,estimated_x,estimated_y,ground_truth_x,ground_truth_y", file=sys.stderr)
        return 1

    filepath = sys.argv[1]

    try:
        samples = read_csv(filepath)
    except FileNotFoundError:
        print(f"Error: file not found: {filepath}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error reading file: {e}", file=sys.stderr)
        return 1

    if not samples:
        print("Error: no valid samples found", file=sys.stderr)
        return 1

    metrics = compute_metrics(samples)
    if not metrics:
        print("Error: failed to compute metrics", file=sys.stderr)
        return 1

    print_metrics(metrics)
    return 0


if __name__ == "__main__":
    sys.exit(main())
