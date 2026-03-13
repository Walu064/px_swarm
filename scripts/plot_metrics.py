#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import math
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SPEED_BUCKETS = [3, 6, 12]


@dataclass
class RunRecord:
    run: str
    mission_kind: str
    drones: int
    speed_nom: int
    leader_cmd_p95: float
    leader_speed_p95: float
    leader_delay_mean: float
    followers_speed_p95: float
    followers_delay_mean: float
    followers_below_ratio: float
    followers_above_ratio: float
    followers_min_dist_mean: float
    followers_min_dist_worst: float
    followers_below_count_mean: float
    followers_pso_fit_mean: float
    followers_pso_fit_max: float
    followers_pso_desired_err_mean: float
    followers_pso_desired_err_last: float
    followers_pso_current_err_mean: float
    followers_pso_current_err_last: float
    quality_score: float


def quantile_safe(series: pd.Series, q: float) -> float:
    s = pd.to_numeric(series, errors="coerce").dropna()
    if s.empty:
        return float("nan")
    return float(s.quantile(q))


def mean_safe(series: pd.Series) -> float:
    s = pd.to_numeric(series, errors="coerce").dropna()
    if s.empty:
        return float("nan")
    return float(s.mean())


def compute_speed_mag(df: pd.DataFrame) -> pd.Series:
    vn = pd.to_numeric(df.get("vel_n_m_s"), errors="coerce")
    ve = pd.to_numeric(df.get("vel_e_m_s"), errors="coerce")
    vd = pd.to_numeric(df.get("vel_d_m_s"), errors="coerce")
    return np.sqrt(vn * vn + ve * ve + vd * vd)


def compute_cmd_norm(df: pd.DataFrame) -> pd.Series:
    a = pd.to_numeric(df.get("cmd_a"), errors="coerce")
    b = pd.to_numeric(df.get("cmd_b"), errors="coerce")
    c = pd.to_numeric(df.get("cmd_c"), errors="coerce")
    return np.sqrt(a * a + b * b + c * c)


def infer_speed_nom(cmd_p95: float, speed_p95: float) -> int:
    source = cmd_p95 if math.isfinite(cmd_p95) and cmd_p95 > 0.2 else speed_p95
    if not math.isfinite(source):
        return -1
    return min(SPEED_BUCKETS, key=lambda x: abs(source - x))


def compute_violation_ratio(df: pd.DataFrame, col_name: str) -> float:
    cnt = pd.to_numeric(df.get(col_name), errors="coerce")
    neigh = pd.to_numeric(df.get("neighbor_count"), errors="coerce")
    valid = neigh > 0
    if not valid.any():
        return float("nan")
    ratio = (cnt[valid] / neigh[valid]).clip(lower=0.0)
    return mean_safe(ratio)


def parse_mission_kind(run_name: str) -> str:
    if run_name.startswith("mission_simple_"):
        return "simple"
    if run_name.startswith("mission_aggressive_"):
        return "aggressive"
    return "other"


def load_runs(log_dir: Path) -> tuple[list[RunRecord], dict[str, list[float]], dict[str, list[float]]]:
    run_records: list[RunRecord] = []
    follower_delay_samples: dict[str, list[float]] = {}
    leader_delay_samples: dict[str, list[float]] = {}

    for run_path in sorted(p for p in log_dir.glob("*") if p.is_dir()):
        node_files = sorted(run_path.glob("px_swarm_node_*.csv"))
        if not node_files:
            node_files = sorted(run_path.glob("swarm_node_*.csv"))
        if not node_files:
            continue

        ids: list[int] = []
        for p in node_files:
            m = re.search(r"(?:px_swarm_node|swarm_node)_(\d+)\.csv$", p.name)
            if m:
                ids.append(int(m.group(1)))
        if 0 not in ids:
            continue

        leader_csv = run_path / "px_swarm_node_0.csv"
        if not leader_csv.exists():
            leader_csv = run_path / "swarm_node_0.csv"
        leader_df = pd.read_csv(leader_csv)
        if leader_df.empty:
            continue

        leader_speed = compute_speed_mag(leader_df)
        leader_cmd_norm = compute_cmd_norm(leader_df)
        cmd_vel_ned = leader_cmd_norm[(leader_df.get("cmd_mode") == "vel_ned") & (leader_cmd_norm > 0.2)]

        leader_cmd_p95 = quantile_safe(cmd_vel_ned, 0.95)
        leader_speed_p95 = quantile_safe(leader_speed, 0.95)
        leader_delay = pd.to_numeric(leader_df.get("avg_rx_delay_ms"), errors="coerce").dropna()
        leader_delay_mean = mean_safe(leader_delay)

        followers = [i for i in ids if i != 0]
        if not followers:
            continue

        f_speed_p95: list[float] = []
        f_delay_mean: list[float] = []
        f_below_ratio: list[float] = []
        f_above_ratio: list[float] = []
        f_delay_samples: list[float] = []
        f_min_dist_mean: list[float] = []
        f_min_dist_worst: list[float] = []
        f_below_count_mean: list[float] = []
        f_pso_fit_mean: list[float] = []
        f_pso_fit_max: list[float] = []
        f_pso_desired_err_mean: list[float] = []
        f_pso_desired_err_last: list[float] = []
        f_pso_current_err_mean: list[float] = []
        f_pso_current_err_last: list[float] = []

        for fid in followers:
            fpath = run_path / f"px_swarm_node_{fid}.csv"
            if not fpath.exists():
                fpath = run_path / f"swarm_node_{fid}.csv"
            if not fpath.exists():
                continue
            fdf = pd.read_csv(fpath)
            if fdf.empty:
                continue

            f_speed_p95.append(quantile_safe(compute_speed_mag(fdf), 0.95))
            delay_series = pd.to_numeric(fdf.get("avg_rx_delay_ms"), errors="coerce").dropna()
            f_delay_mean.append(mean_safe(delay_series))
            f_delay_samples.extend(delay_series.tolist())
            f_below_ratio.append(compute_violation_ratio(fdf, "below_min_count"))
            f_above_ratio.append(compute_violation_ratio(fdf, "above_max_count"))
            min_dist_series = pd.to_numeric(fdf.get("min_dist_m"), errors="coerce").dropna()
            below_count_series = pd.to_numeric(fdf.get("below_min_count"), errors="coerce").dropna()
            pso_fit_series = pd.to_numeric(fdf.get("pso_best_fitness"), errors="coerce").dropna()
            pso_desired_err_series = pd.to_numeric(fdf.get("pso_desired_target_err_m"), errors="coerce").dropna()
            pso_current_err_series = pd.to_numeric(fdf.get("pso_current_target_err_m"), errors="coerce").dropna()

            f_min_dist_mean.append(mean_safe(min_dist_series))
            if not min_dist_series.empty:
                f_min_dist_worst.append(float(min_dist_series.min()))
            f_below_count_mean.append(mean_safe(below_count_series))
            f_pso_fit_mean.append(mean_safe(pso_fit_series))
            if not pso_fit_series.empty:
                f_pso_fit_max.append(float(pso_fit_series.max()))
            f_pso_desired_err_mean.append(mean_safe(pso_desired_err_series))
            if not pso_desired_err_series.empty:
                f_pso_desired_err_last.append(float(pso_desired_err_series.iloc[-1]))
            f_pso_current_err_mean.append(mean_safe(pso_current_err_series))
            if not pso_current_err_series.empty:
                f_pso_current_err_last.append(float(pso_current_err_series.iloc[-1]))

        if not f_speed_p95:
            continue

        speed_nom = infer_speed_nom(leader_cmd_p95, leader_speed_p95)
        if speed_nom < 0:
            continue

        scenario = f"{len(ids)}D_{speed_nom}mps"
        follower_delay_samples.setdefault(scenario, []).extend(f_delay_samples)
        leader_delay_samples.setdefault(scenario, []).extend(leader_delay.tolist())

        # Simple quality score: lower is better.
        quality_score = (
            0.5 * np.nanmean(f_below_ratio)
            + 0.5 * np.nanmean(f_above_ratio)
            + 0.02 * np.nanmean(f_delay_mean)
        )

        run_records.append(
            RunRecord(
                run=run_path.name,
                mission_kind=parse_mission_kind(run_path.name),
                drones=len(ids),
                speed_nom=speed_nom,
                leader_cmd_p95=leader_cmd_p95,
                leader_speed_p95=leader_speed_p95,
                leader_delay_mean=leader_delay_mean,
                followers_speed_p95=float(np.nanmean(f_speed_p95)),
                followers_delay_mean=float(np.nanmean(f_delay_mean)),
                followers_below_ratio=float(np.nanmean(f_below_ratio)),
                followers_above_ratio=float(np.nanmean(f_above_ratio)),
                followers_min_dist_mean=float(np.nanmean(f_min_dist_mean)),
                followers_min_dist_worst=float(np.nanmin(f_min_dist_worst)) if f_min_dist_worst else float("nan"),
                followers_below_count_mean=float(np.nanmean(f_below_count_mean)),
                followers_pso_fit_mean=float(np.nanmean(f_pso_fit_mean)),
                followers_pso_fit_max=float(np.nanmax(f_pso_fit_max)) if f_pso_fit_max else float("nan"),
                followers_pso_desired_err_mean=float(np.nanmean(f_pso_desired_err_mean)),
                followers_pso_desired_err_last=float(np.nanmean(f_pso_desired_err_last)),
                followers_pso_current_err_mean=float(np.nanmean(f_pso_current_err_mean)),
                followers_pso_current_err_last=float(np.nanmean(f_pso_current_err_last)),
                quality_score=float(quality_score),
            )
        )

    return run_records, follower_delay_samples, leader_delay_samples


def heatmap(df: pd.DataFrame, value_col: str, title: str, out_path: Path) -> None:
    pivot = df.pivot_table(index="drones", columns="speed_nom", values=value_col, aggfunc="mean")
    pivot = pivot.sort_index().reindex(columns=SPEED_BUCKETS)
    fig, ax = plt.subplots(figsize=(7, 4.8))
    im = ax.imshow(pivot.values, aspect="auto", cmap="YlOrRd")
    ax.set_title(title)
    ax.set_xlabel("Leader speed [m/s]")
    ax.set_ylabel("Number of drones")
    ax.set_xticks(np.arange(len(pivot.columns)))
    ax.set_xticklabels([str(c) for c in pivot.columns])
    ax.set_yticks(np.arange(len(pivot.index)))
    ax.set_yticklabels([str(i) for i in pivot.index])
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.iloc[i, j]
            txt = "NA" if pd.isna(v) else f"{100.0 * v:.1f}%"
            ax.text(j, i, txt, ha="center", va="center", color="black", fontsize=9)
    fig.colorbar(im, ax=ax, shrink=0.9, label="ratio")
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def delay_boxplot(follower_delay_samples: dict[str, list[float]], out_path: Path) -> None:
    labels = sorted(follower_delay_samples.keys(), key=lambda x: (int(x.split("D_")[0]), int(x.split("_")[1].replace("mps", ""))))
    data = [follower_delay_samples[k] for k in labels if follower_delay_samples[k]]
    labels = [k for k in labels if follower_delay_samples[k]]
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.boxplot(data, tick_labels=labels, showfliers=False)
    ax.set_title("Follower communication delay distribution")
    ax.set_ylabel("avg_rx_delay_ms")
    ax.set_xlabel("Scenario")
    ax.tick_params(axis="x", rotation=35)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def speed_compare(df: pd.DataFrame, out_path: Path) -> None:
    g = (
        df.groupby(["drones", "speed_nom"], as_index=False)[
            ["leader_cmd_p95", "leader_speed_p95", "followers_speed_p95"]
        ]
        .mean()
        .sort_values(["drones", "speed_nom"])
    )
    labels = [f"{int(r.drones)}D/{int(r.speed_nom)}" for r in g.itertuples()]
    x = np.arange(len(labels))
    w = 0.25
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.bar(x - w, g["leader_cmd_p95"], width=w, label="leader cmd p95")
    ax.bar(x, g["leader_speed_p95"], width=w, label="leader speed p95")
    ax.bar(x + w, g["followers_speed_p95"], width=w, label="followers speed p95")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=35)
    ax.set_ylabel("m/s")
    ax.set_title("Speed tracking by scenario")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def quality_bar(df: pd.DataFrame, out_path: Path) -> None:
    g = (
        df.groupby(["drones", "speed_nom"], as_index=False)["quality_score"]
        .mean()
        .sort_values(["drones", "speed_nom"])
    )
    labels = [f"{int(r.drones)}D/{int(r.speed_nom)}" for r in g.itertuples()]
    fig, ax = plt.subplots(figsize=(10, 4.8))
    bars = ax.bar(labels, g["quality_score"])
    for b in bars:
        ax.text(
            b.get_x() + b.get_width() / 2.0,
            b.get_height(),
            f"{b.get_height():.3f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    ax.set_ylabel("quality score (lower is better)")
    ax.set_title("Scenario quality score (followers)")
    ax.tick_params(axis="x", rotation=35)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def scenario_compare_ml_focus(df: pd.DataFrame, out_path: Path) -> None:
    focus = df[df["mission_kind"].isin(["simple", "aggressive"])].copy()
    if focus.empty:
        return

    grouped = (
        focus.groupby(["mission_kind", "drones"], as_index=False)[
            [
                "followers_below_count_mean",
                "followers_min_dist_mean",
                "followers_pso_desired_err_mean",
                "followers_pso_fit_mean",
            ]
        ]
        .mean()
        .sort_values(["mission_kind", "drones"])
    )

    metrics = [
        ("followers_below_count_mean", "Mean Too-Close Count", "count"),
        ("followers_min_dist_mean", "Mean Min Distance", "m"),
        ("followers_pso_desired_err_mean", "Mean Desired Target Error", "m"),
        ("followers_pso_fit_mean", "Mean PSO Fitness", "cost"),
    ]
    colors = {"simple": "#355C7D", "aggressive": "#C06C84"}
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
    axes = axes.flatten()

    for ax, (metric, title, ylabel) in zip(axes, metrics):
        for mission_kind in ["simple", "aggressive"]:
            sub = grouped[grouped["mission_kind"] == mission_kind]
            if sub.empty:
                continue
            ax.plot(
                sub["drones"],
                sub[metric],
                marker="o",
                linewidth=2,
                label=mission_kind,
                color=colors[mission_kind],
            )
            for row in sub.itertuples():
                ax.text(
                    row.drones,
                    getattr(row, metric),
                    f"{getattr(row, metric):.2f}",
                    fontsize=8,
                    ha="center",
                    va="bottom",
                )
        ax.set_title(title)
        ax.set_xlabel("Number of drones")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)

    axes[0].legend()
    fig.suptitle("Simple vs Aggressive: Metrics Most Relevant for ML Improvements", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def scenario_compare_summary(df: pd.DataFrame, out_path: Path) -> None:
    focus = df[df["mission_kind"].isin(["simple", "aggressive"])].copy()
    if focus.empty:
        return

    grouped = (
        focus.groupby(["mission_kind", "drones"], as_index=False)[
            [
                "followers_pso_desired_err_last",
                "followers_pso_current_err_last",
                "followers_min_dist_worst",
                "followers_pso_fit_max",
            ]
        ]
        .mean()
        .sort_values(["mission_kind", "drones"])
    )

    metrics = [
        ("followers_pso_desired_err_last", "Final Desired Error", "m"),
        ("followers_pso_current_err_last", "Final Tracking Error", "m"),
        ("followers_min_dist_worst", "Worst Min Distance", "m"),
        ("followers_pso_fit_max", "Worst PSO Fitness", "cost"),
    ]
    colors = {"simple": "#355C7D", "aggressive": "#C06C84"}
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
    axes = axes.flatten()

    for ax, (metric, title, ylabel) in zip(axes, metrics):
        for mission_kind in ["simple", "aggressive"]:
            sub = grouped[grouped["mission_kind"] == mission_kind]
            if sub.empty:
                continue
            ax.plot(
                sub["drones"],
                sub[metric],
                marker="o",
                linewidth=2,
                label=mission_kind,
                color=colors[mission_kind],
            )
        ax.set_title(title)
        ax.set_xlabel("Number of drones")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)

    axes[0].legend()
    fig.suptitle("Simple vs Aggressive: End-State and Worst-Case Comparison", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate experiment plots from swarm logs.")
    parser.add_argument("--log-dir", default="logs", help="Path to logs directory")
    parser.add_argument("--out-dir", default="logs/plots", help="Output directory for plots")
    args = parser.parse_args()

    log_dir = Path(args.log_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    runs, follower_delay_samples, _leader_delay_samples = load_runs(log_dir)
    if not runs:
        raise SystemExit("No valid runs found in logs directory.")

    df = pd.DataFrame([r.__dict__ for r in runs])
    df.to_csv(out_dir / "summary_runs.csv", index=False)

    heatmap(
        df,
        "followers_above_ratio",
        "Followers too-far ratio (> max spacing)",
        out_dir / "heatmap_followers_above.png",
    )
    heatmap(
        df,
        "followers_below_ratio",
        "Followers too-close ratio (< min spacing)",
        out_dir / "heatmap_followers_below.png",
    )
    delay_boxplot(follower_delay_samples, out_dir / "boxplot_followers_delay.png")
    speed_compare(df, out_dir / "speed_tracking_compare.png")
    quality_bar(df, out_dir / "quality_score.png")
    scenario_compare_ml_focus(df, out_dir / "scenario_compare_ml_focus.png")
    scenario_compare_summary(df, out_dir / "scenario_compare_summary.png")

    print(f"Saved summary: {out_dir / 'summary_runs.csv'}")
    print(f"Saved plots in: {out_dir}")
    for p in sorted(glob.glob(str(out_dir / "*.png"))):
        print(f" - {p}")


if __name__ == "__main__":
    main()
