#!/usr/bin/env python3
"""Generate evidence-based figures for the fall-action training report."""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import PercentFormatter
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, f1_score
from sklearn.model_selection import GroupShuffleSplit, train_test_split
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "figures"
OUT.mkdir(parents=True, exist_ok=True)
COLORS = {
    "blue": "#2563EB",
    "cyan": "#0891B2",
    "green": "#16A34A",
    "orange": "#EA580C",
    "red": "#DC2626",
    "purple": "#7C3AED",
    "grey": "#64748B",
}


def save(name: str) -> None:
    plt.tight_layout()
    plt.savefig(OUT / name, dpi=190, bbox_inches="tight", facecolor="white")
    plt.close()


def featurize(X: np.ndarray) -> np.ndarray:
    xy = X[..., :2]
    velocity = np.diff(xy, axis=1, prepend=xy[:, :1])
    return np.concatenate(
        [
            xy.mean(axis=1).reshape(len(X), -1),
            xy.std(axis=1).reshape(len(X), -1),
            np.abs(velocity).mean(axis=1).reshape(len(X), -1),
            np.abs(velocity).max(axis=1).reshape(len(X), -1),
        ],
        axis=1,
    )


def pretty(label: str) -> str:
    return label.replace("_", " ").title()


combined = json.loads((ROOT / "combined_pose_rf_report.json").read_text(encoding="utf-8"))
baseline = json.loads((ROOT / "gmdcsa24_pose_rf_report.json").read_text(encoding="utf-8"))
data = np.load(ROOT / "pose_sequences_combined_gmdcsa24_dji_aug20.npz", allow_pickle=True)
X = data["X"]
y = data["y"].astype(str)
segment_ids = data["segment_id"].astype(str)
groups = np.array([segment_id.split("_")[0] for segment_id in segment_ids])
Xf = featurize(X)

splitter = GroupShuffleSplit(n_splits=1, test_size=0.25, random_state=42)
train_idx, test_idx = next(splitter.split(Xf, y, groups=groups))
classes = combined["classes"]

# 1. Dataset class distribution, split into train and held-out test.
train_counts = Counter(y[train_idx])
test_counts = Counter(y[test_idx])
xpos = np.arange(len(classes))
train_values = np.array([train_counts.get(label, 0) for label in classes])
test_values = np.array([test_counts.get(label, 0) for label in classes])
plt.figure(figsize=(11, 5.4))
plt.bar(xpos, train_values, label="Training groups", color=COLORS["blue"])
plt.bar(xpos, test_values, bottom=train_values, label="Held-out groups", color=COLORS["orange"])
for x, train, test in zip(xpos, train_values, test_values):
    plt.text(x, train / 2, str(train), ha="center", va="center", color="white", fontweight="bold")
    if test:
        plt.text(x, train + test / 2, str(test), ha="center", va="center", color="white", fontweight="bold")
plt.xticks(xpos, [pretty(label) for label in classes], rotation=20, ha="right")
plt.ylabel("Pose sequences")
plt.title("Combined dataset distribution: 268 training and 138 held-out sequences")
plt.legend(frameon=False)
save("01_dataset_class_distribution.png")

# 2. Group split.
group_counts = Counter(groups)
group_names = sorted(group_counts)
train_groups = set(groups[train_idx])
group_colors = [COLORS["blue"] if group in train_groups else COLORS["orange"] for group in group_names]
plt.figure(figsize=(9, 5.2))
bars = plt.bar(group_names, [group_counts[group] for group in group_names], color=group_colors)
for bar, group in zip(bars, group_names):
    split = "train" if group in train_groups else "test"
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 3, f"{int(bar.get_height())}\n{split}", ha="center")
plt.ylabel("Pose sequences")
plt.title("Group-independent evaluation split")
plt.ylim(0, max(group_counts.values()) + 25)
save("02_group_split.png")

# 3. Overall baseline-versus-combined evaluation.
metric_names = ["Accuracy", "Weighted precision", "Weighted recall", "Weighted F1"]
def overall(report: dict) -> list[float]:
    cr = report["classification_report"]
    weighted = cr["weighted avg"]
    return [cr["accuracy"], weighted["precision"], weighted["recall"], weighted["f1-score"]]

baseline_values = overall(baseline)
combined_values = overall(combined)
width = 0.36
plt.figure(figsize=(10, 5.2))
plt.bar(xpos[:4] - width / 2, baseline_values, width, label="GMDCSA24 baseline", color=COLORS["grey"])
plt.bar(xpos[:4] + width / 2, combined_values, width, label="Combined current model", color=COLORS["blue"])
for x, value in zip(xpos[:4] - width / 2, baseline_values):
    plt.text(x, value + 0.018, f"{value:.3f}", ha="center", fontsize=9)
for x, value in zip(xpos[:4] + width / 2, combined_values):
    plt.text(x, value + 0.018, f"{value:.3f}", ha="center", fontsize=9)
plt.xticks(xpos[:4], metric_names)
plt.ylim(0, 1.08)
plt.gca().yaxis.set_major_formatter(PercentFormatter(1.0))
plt.title("Held-out evaluation: baseline versus combined model")
plt.legend(frameon=False, loc="lower right")
save("03_overall_metric_comparison.png")

# 4. Per-class precision, recall and F1.
precision = [combined["classification_report"][label]["precision"] for label in classes]
recall = [combined["classification_report"][label]["recall"] for label in classes]
f1 = [combined["classification_report"][label]["f1-score"] for label in classes]
width = 0.25
plt.figure(figsize=(12, 5.7))
plt.bar(xpos - width, precision, width, label="Precision", color=COLORS["cyan"])
plt.bar(xpos, recall, width, label="Recall", color=COLORS["orange"])
plt.bar(xpos + width, f1, width, label="F1", color=COLORS["green"])
plt.xticks(xpos, [pretty(label) for label in classes], rotation=18, ha="right")
plt.ylim(0, 1.08)
plt.gca().yaxis.set_major_formatter(PercentFormatter(1.0))
plt.title("Combined model performance by action class")
plt.legend(frameon=False, ncol=3)
plt.text(2, 0.06, "Running has zero held-out samples", ha="center", color=COLORS["red"], fontsize=9)
save("04_combined_per_class_metrics.png")


def plot_confusion(report: dict, filename: str, title: str) -> None:
    labels = report["classes"]
    matrix = np.asarray(report["confusion_matrix"], dtype=int)
    plt.figure(figsize=(8.4, 7.2))
    plt.imshow(matrix, cmap="Blues")
    plt.colorbar(label="Number of sequences")
    for row in range(matrix.shape[0]):
        for col in range(matrix.shape[1]):
            value = matrix[row, col]
            color = "white" if value > matrix.max() * 0.48 else "black"
            plt.text(col, row, str(value), ha="center", va="center", color=color, fontweight="bold")
    plt.xticks(range(len(labels)), [pretty(label) for label in labels], rotation=35, ha="right")
    plt.yticks(range(len(labels)), [pretty(label) for label in labels])
    plt.xlabel("Predicted class")
    plt.ylabel("True class")
    plt.title(title)
    save(filename)


# 5 and 6. Confusion matrices.
plot_confusion(combined, "05_combined_confusion_matrix.png", "Combined model confusion matrix (138 held-out sequences)")
plot_confusion(baseline, "06_baseline_confusion_matrix.png", "GMDCSA24 baseline confusion matrix (110 held-out sequences)")

# 7. Random Forest learning curve on the same fixed held-out groups.
fractions = np.array([0.25, 0.50, 0.75, 1.00])
train_accuracy = []
test_accuracy = []
test_f1 = []
for fraction in fractions:
    if fraction < 1.0:
        selected, _ = train_test_split(
            train_idx,
            train_size=fraction,
            random_state=42,
            stratify=y[train_idx],
        )
    else:
        selected = train_idx
    model = make_pipeline(
        StandardScaler(),
        RandomForestClassifier(
            n_estimators=400,
            class_weight="balanced",
            random_state=42,
            min_samples_leaf=2,
        ),
    )
    model.fit(Xf[selected], y[selected])
    train_pred = model.predict(Xf[selected])
    test_pred = model.predict(Xf[test_idx])
    train_accuracy.append(accuracy_score(y[selected], train_pred))
    test_accuracy.append(accuracy_score(y[test_idx], test_pred))
    test_f1.append(f1_score(y[test_idx], test_pred, average="weighted", zero_division=0))

plt.figure(figsize=(9.5, 5.4))
plt.plot(fractions * 100, train_accuracy, "o-", label="Training accuracy", color=COLORS["grey"], linewidth=2)
plt.plot(fractions * 100, test_accuracy, "o-", label="Held-out accuracy", color=COLORS["blue"], linewidth=2)
plt.plot(fractions * 100, test_f1, "o-", label="Held-out weighted F1", color=COLORS["green"], linewidth=2)
plt.xticks(fractions * 100, [f"{int(value)}%" for value in fractions * 100])
plt.ylim(0.45, 1.04)
plt.gca().yaxis.set_major_formatter(PercentFormatter(1.0))
plt.xlabel("Fraction of the 268-sequence training split used")
plt.ylabel("Score")
plt.title("Random Forest learning curve on fixed held-out groups")
plt.legend(frameon=False)
plt.grid(axis="y", alpha=0.22)
save("07_random_forest_learning_curve.png")

# 8. Aggregate feature importance by body keypoint from a final 500-tree fit.
final_model = make_pipeline(
    StandardScaler(),
    RandomForestClassifier(
        n_estimators=500,
        class_weight="balanced",
        random_state=42,
        min_samples_leaf=2,
    ),
)
final_model.fit(Xf, y)
importances = final_model[-1].feature_importances_.reshape(4, 17, 2).sum(axis=(0, 2))
keypoint_names = [
    "Nose", "Left eye", "Right eye", "Left ear", "Right ear", "Left shoulder",
    "Right shoulder", "Left elbow", "Right elbow", "Left wrist", "Right wrist",
    "Left hip", "Right hip", "Left knee", "Right knee", "Left ankle", "Right ankle",
]
order = np.argsort(importances)[::-1]
plt.figure(figsize=(10.2, 5.8))
bars = plt.bar(np.arange(17), importances[order], color=COLORS["purple"])
plt.xticks(np.arange(17), [keypoint_names[index] for index in order], rotation=40, ha="right")
plt.ylabel("Aggregated feature importance")
plt.title("Body keypoints used most by the final Random Forest")
for bar, value in zip(bars[:6], importances[order][:6]):
    plt.text(bar.get_x() + bar.get_width() / 2, value + 0.002, f"{value:.3f}", ha="center", fontsize=8)
save("08_keypoint_feature_importance.png")

print(f"Generated {len(list(OUT.glob('*.png')))} figures in {OUT}")
