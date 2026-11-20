#!/usr/bin/env python3
"""Train a lightweight baseline classifier on pose-sequence features."""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import GroupShuffleSplit
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler


def featurize(X: np.ndarray) -> np.ndarray:
    # X: samples, frames, keypoints, channels(x,y,conf)
    xy = X[..., :2]
    conf = X[..., 2:3]
    velocity = np.diff(xy, axis=1, prepend=xy[:, :1])
    features = [
        xy.mean(axis=1).reshape(len(X), -1),
        xy.std(axis=1).reshape(len(X), -1),
        np.abs(velocity).mean(axis=1).reshape(len(X), -1),
        np.abs(velocity).max(axis=1).reshape(len(X), -1),
        conf.mean(axis=1).reshape(len(X), -1),
    ]
    return np.concatenate(features, axis=1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-npz", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model-out", type=Path, required=True)
    parser.add_argument("--report-out", type=Path, required=True)
    args = parser.parse_args()

    data = np.load(args.pose_npz, allow_pickle=True)
    X = data["X"]
    y = data["y"].astype(str)
    segment_ids = data["segment_id"].astype(str)
    groups = np.array([sid.split("_")[0] for sid in segment_ids])
    Xf = featurize(X)

    splitter = GroupShuffleSplit(n_splits=1, test_size=0.25, random_state=42)
    train_idx, test_idx = next(splitter.split(Xf, y, groups=groups))

    clf = make_pipeline(
        StandardScaler(),
        RandomForestClassifier(
            n_estimators=400,
            class_weight="balanced",
            random_state=42,
            min_samples_leaf=2,
        ),
    )
    clf.fit(Xf[train_idx], y[train_idx])
    pred = clf.predict(Xf[test_idx])
    labels = sorted(set(y))
    report = {
        "num_samples": int(len(y)),
        "train_samples": int(len(train_idx)),
        "test_samples": int(len(test_idx)),
        "classes": labels,
        "classification_report": classification_report(y[test_idx], pred, labels=labels, output_dict=True, zero_division=0),
        "confusion_matrix": confusion_matrix(y[test_idx], pred, labels=labels).tolist(),
    }

    args.model_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    final_clf = make_pipeline(
        StandardScaler(),
        RandomForestClassifier(
            n_estimators=500,
            class_weight="balanced",
            random_state=42,
            min_samples_leaf=2,
        ),
    )
    final_clf.fit(Xf, y)
    with args.model_out.open("wb") as handle:
        pickle.dump({"pipeline": final_clf, "classes": labels}, handle)
    with args.report_out.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
