#!/usr/bin/env python3
"""
Feature repeatability evaluator for DG202611 experiments.

Computes repeatability metrics for LiDAR features across consecutive frames.
"""

import sys
import csv
from dataclasses import dataclass
from typing import List, Dict, Optional


@dataclass
class FrameFeatures:
    """Features detected in one frame."""
    frame_id: int
    timestamp: float
    feature_ids: List[int]
    valid_feature_count: int


@dataclass
class RepeatabilityMetrics:
    """Feature repeatability statistics."""
    total_frames: int
    total_pairs: int
    total_features: int
    repeated_features: int
    overall_repeatability: float
    per_frame_repeatability: List[float]
    mean_per_frame: float


def compute_frame_pair_repeatability(frame1: FrameFeatures, frame2: FrameFeatures) -> Optional[float]:
    """Compute repeatability between two consecutive frames.

    Repeatability = (# repeated features) / (# features in earlier frame)

    Returns None if earlier frame has no features.
    """
    if frame1.valid_feature_count == 0:
        return None

    # Find features present in both frames
    repeated = len(set(frame1.feature_ids) & set(frame2.feature_ids))

    return repeated / frame1.valid_feature_count


def compute_metrics(frames: List[FrameFeatures]) -> Optional[RepeatabilityMetrics]:
    """Compute overall repeatability metrics from frame sequence."""
    if len(frames) < 2:
        return None

    per_frame_repeatability = []
    total_features = 0
    repeated_features = 0
    valid_pairs = 0

    for i in range(len(frames) - 1):
        frame1 = frames[i]
        frame2 = frames[i + 1]

        rep = compute_frame_pair_repeatability(frame1, frame2)
        if rep is not None:
            per_frame_repeatability.append(rep)

            # Accumulate for overall metric
            repeated = len(set(frame1.feature_ids) & set(frame2.feature_ids))
            repeated_features += repeated
            total_features += frame1.valid_feature_count
            valid_pairs += 1

    if valid_pairs == 0:
        return None

    overall_repeatability = repeated_features / total_features if total_features > 0 else 0.0
    mean_per_frame = sum(per_frame_repeatability) / len(per_frame_repeatability)

    return RepeatabilityMetrics(
        total_frames=len(frames),
        total_pairs=valid_pairs,
        total_features=total_features,
        repeated_features=repeated_features,
        overall_repeatability=overall_repeatability,
        per_frame_repeatability=per_frame_repeatability,
        mean_per_frame=mean_per_frame
    )


def read_csv(filepath: str) -> List[FrameFeatures]:
    """Read frame features from CSV file.

    Expected columns: frame_id, timestamp, feature_ids (comma-separated), valid_feature_count
    """
    frames = []

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                # Parse comma-separated feature IDs
                feature_ids_str = row['feature_ids'].strip()
                if feature_ids_str:
                    feature_ids = [int(x) for x in feature_ids_str.split(',')]
                else:
                    feature_ids = []

                frame = FrameFeatures(
                    frame_id=int(row['frame_id']),
                    timestamp=float(row['timestamp']),
                    feature_ids=feature_ids,
                    valid_feature_count=int(row['valid_feature_count'])
                )
                frames.append(frame)
            except (KeyError, ValueError) as e:
                print(f"Warning: skipping invalid row: {e}", file=sys.stderr)
                continue

    # Sort by frame_id to ensure consecutive pairs
    frames.sort(key=lambda f: f.frame_id)

    return frames


def print_metrics(metrics: RepeatabilityMetrics) -> None:
    """Print metrics in human-readable format."""
    print("Feature Repeatability Metrics:")
    print(f"  Total Frames:         {metrics.total_frames}")
    print(f"  Valid Pairs:          {metrics.total_pairs}")
    print(f"  Total Features:       {metrics.total_features}")
    print(f"  Repeated Features:    {metrics.repeated_features}")
    print(f"  Overall Repeatability: {metrics.overall_repeatability:.4f}")
    print(f"  Mean Per-Frame:       {metrics.mean_per_frame:.4f}")

    if len(metrics.per_frame_repeatability) <= 10:
        print(f"  Per-Frame Values:     {[f'{x:.3f}' for x in metrics.per_frame_repeatability]}")


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: feature_repeatability_evaluator.py <csv_file>", file=sys.stderr)
        print("", file=sys.stderr)
        print("CSV format: frame_id,timestamp,feature_ids,valid_feature_count", file=sys.stderr)
        print("feature_ids: comma-separated integers (e.g., \"0,1,2,3\")", file=sys.stderr)
        return 1

    filepath = sys.argv[1]

    try:
        frames = read_csv(filepath)
    except FileNotFoundError:
        print(f"Error: file not found: {filepath}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error reading file: {e}", file=sys.stderr)
        return 1

    if len(frames) < 2:
        print("Error: need at least 2 frames for repeatability", file=sys.stderr)
        return 1

    metrics = compute_metrics(frames)
    if not metrics:
        print("Error: failed to compute metrics", file=sys.stderr)
        return 1

    print_metrics(metrics)
    return 0


if __name__ == "__main__":
    sys.exit(main())
