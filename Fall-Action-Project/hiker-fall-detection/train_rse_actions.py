#!/usr/bin/env python3
"""Train/evaluate the five-class pose action model and save the live bundle."""
from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from sklearn.model_selection import GroupShuffleSplit
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

import action_core

MAP = {"falling_down": "falling_down", "lying_down": "lying_down", "walking": "walking",
       "standing_still": "standing", "standing": "standing",
       "sitting_kneeling": "sitting", "sitting": "sitting"}
CLASSES = ["falling_down", "lying_down", "walking", "standing", "sitting"]


def resize12(X: np.ndarray) -> np.ndarray:
    if X.shape[1] == action_core.WINDOW:
        return X.astype(np.float32)
    idx = np.linspace(0, X.shape[1] - 1, action_core.WINDOW)
    lo, hi = np.floor(idx).astype(int), np.ceil(idx).astype(int)
    frac = (idx - lo).reshape(1, -1, 1, 1)
    return X[:, lo] * (1 - frac) + X[:, hi] * frac


def load_gmdcsa(npz_path: Path, manifest: Path):
    data = np.load(npz_path, allow_pickle=True)
    rows = list(__import__("csv").DictReader(manifest.open(newline="", encoding="utf-8")))
    by_id = {r["segment_id"]: r for r in rows}
    ids = data["segment_id"].astype(str)
    groups = np.array([by_id.get(s, {}).get("subject", s) + ":" + by_id.get(s, {}).get("file_name", s) for s in ids])
    return resize12(data["X"]), np.array([MAP[str(v)] for v in data["y"]]), ids, groups, np.array(["GMDCSA24"] * len(ids))


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--gmdcsa-npz", type=Path, default=Path("/home/robotx/fall_action_project/outputs/pose_sequences_gmdcsa24.npz"))
    p.add_argument("--gmdcsa-manifest", type=Path, default=Path("/home/robotx/fall_action_project/outputs/gmdcsa24_segments.csv"))
    p.add_argument("--user-npz", type=Path, default=Path("data/user_pose_sequences.npz"))
    p.add_argument("--model-out", type=Path, default=Path("models/actions.pkl"))
    p.add_argument("--report-out", type=Path, default=Path("reports/training_report.json"))
    p.add_argument("--falling-aug-npz", type=Path, help="Optional falling_down-only pose samples added to final fitting")
    args = p.parse_args()
    gx, gy, gi, gg, gs = load_gmdcsa(args.gmdcsa_npz, args.gmdcsa_manifest)
    ud = np.load(args.user_npz, allow_pickle=True)
    X = np.concatenate([gx, ud["X"]], axis=0)
    y = np.concatenate([gy, ud["y"].astype(str)], axis=0)
    ids = np.concatenate([gi, ud["segment_id"].astype(str)], axis=0)
    groups = np.concatenate([gg, ud["group"].astype(str)], axis=0)
    sources = np.concatenate([gs, ud["source"].astype(str)], axis=0)
    keep = np.isin(y, CLASSES)
    X, y, ids, groups, sources = X[keep], y[keep], ids[keep], groups[keep], sources[keep]
    features = action_core.featurize(X)
    split = GroupShuffleSplit(n_splits=1, test_size=0.25, random_state=42)
    train_idx, test_idx = next(split.split(features, y, groups))
    candidates = {
        "logistic_regression": make_pipeline(StandardScaler(), LogisticRegression(max_iter=2000, class_weight="balanced")),
        "random_forest": make_pipeline(StandardScaler(), RandomForestClassifier(n_estimators=500, min_samples_leaf=2, class_weight="balanced", random_state=42, n_jobs=-1)),
    }
    comparison = {}
    for name, clf in candidates.items():
        clf.fit(features[train_idx], y[train_idx])
        pred = clf.predict(features[test_idx])
        comparison[name] = {"accuracy": float(accuracy_score(y[test_idx], pred)),
            "classification_report": classification_report(y[test_idx], pred, labels=CLASSES, output_dict=True, zero_division=0),
            "confusion_matrix": confusion_matrix(y[test_idx], pred, labels=CLASSES).tolist()}
    fit_X, fit_y = features, y
    fit_ids, fit_groups, fit_sources = ids, groups, sources
    augmented = 0
    if args.falling_aug_npz:
        ad = np.load(args.falling_aug_npz, allow_pickle=True)
        ax = action_core.featurize(resize12(ad["X"]))
        ay = ad["y"].astype(str)
        if not np.all(ay == "falling_down"):
            raise ValueError("--falling-aug-npz may contain only falling_down samples")
        fit_X = np.concatenate([fit_X, ax], axis=0)
        fit_y = np.concatenate([fit_y, ay], axis=0)
        fit_ids = np.concatenate([fit_ids, ad["segment_id"].astype(str)], axis=0)
        fit_groups = np.concatenate([fit_groups, ad["group"].astype(str)], axis=0)
        fit_sources = np.concatenate([fit_sources, ad["source"].astype(str)], axis=0)
        augmented = int(len(ay))
    final = candidates["random_forest"]
    final.fit(fit_X, fit_y)
    args.model_out.parent.mkdir(parents=True, exist_ok=True)
    with args.model_out.open("wb") as handle:
        pickle.dump({"pipeline": final, "classes": np.array(CLASSES), "feature_version": action_core.FEATURE_VERSION}, handle)
    report = {"classes": CLASSES, "num_sequences": int(len(fit_y)), "base_sequences": int(len(y)),
              "falling_photo_augmented": augmented, "train_sequences": int(len(train_idx)),
              "test_sequences": int(len(test_idx)), "num_groups": int(len(set(groups))),
              "groups_by_source": {s: int(len(set(fit_groups[fit_sources == s]))) for s in sorted(set(fit_sources))},
              "class_counts": {c: int(np.sum(fit_y == c)) for c in CLASSES}, "comparison": comparison,
              "split": "grouped by source video/subject; random_state=42", "selected_model": "random_forest",
              "feature_version": action_core.FEATURE_VERSION, "external_dataset": "GMDCSA24 local copy",
              "user_segments": "data/user_segments.csv; approximate temporal annotations require review"}
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    np.savez_compressed(args.report_out.with_suffix(".dataset.npz"), X=fit_X, y=fit_y,
                        segment_id=fit_ids, group=fit_groups, source=fit_sources)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
