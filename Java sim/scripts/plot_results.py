"""Generate exactly eight figures from a single complete experiment batch."""
from pathlib import Path
import argparse
import sys
import warnings

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SYSTEMS = ("TURTLEBOT", "DRONE", "HYBRID")
TYPES = ("PATROL_DENSITY", "ATTACK_DENSITY", "TCO")
ROOT = Path(__file__).resolve().parents[1]
NUMERIC = (
    "schema_version", "runs", "valid_capture_coefficient_runs",
    "mean_capture_coefficient", "std_capture_coefficient",
    "patrol_density_robots_per_m", "attack_probability_per_tick",
    "duration_min", "perimeter_length_m",
)
REQUIRED = set(NUMERIC) | {
    "experiment_id", "design_id", "point_id", "experiment_type", "system_type",
    "resource_scenario", "patrol_strategy", "attack_model", "attack_duration_min",
    "perimeter_type", "cost_model", "capture_definition", "mean_activity_coefficient",
    "mean_tco_eur", "std_tco_eur", "energy_price_eur_per_kwh",
}
COMPARISON_CONTROLS = (
    "patrol_deviation_probability", "patrol_max_deviation_duration_min",
    "encounter_rearm_policy", "charge_threshold_wh",
)

def validate(data):
    missing = REQUIRED - set(data.columns)
    if missing:
        raise ValueError("Missing required columns: " + ", ".join(sorted(missing)))
    data = data.copy()
    if data.empty:
        raise ValueError("The summary contains no experiment results")
    for col in NUMERIC:
        data[col] = pd.to_numeric(data[col], errors="raise")
        if not np.isfinite(data[col]).all():
            raise ValueError(f"{col} contains missing/undefined or non-finite values; check resolved attack counts")
    for col in ("runs", "valid_capture_coefficient_runs", "schema_version"):
        if (data[col] % 1 != 0).any():
            raise ValueError(f"{col} must contain integers")
    if not (data.schema_version == 1).all():
        raise ValueError("Unsupported CSV schema version")
    if data.experiment_id.nunique() != 1 or data.experiment_id.isna().any():
        raise ValueError("Select exactly one experiment batch")
    if set(data.experiment_type) != set(TYPES) or set(data.system_type) != set(SYSTEMS):
        raise ValueError("All three experiment types and all three system types are required")
    if ((data.runs < 1) | (data.valid_capture_coefficient_runs < 1)
            | (data.valid_capture_coefficient_runs > data.runs)).any():
        raise ValueError("Invalid total/defined capture run counts")
    if not data.mean_capture_coefficient.between(0, 1).all() or (data.std_capture_coefficient < 0).any():
        raise ValueError("Invalid capture coefficient or standard deviation")
    if (data.patrol_density_robots_per_m <= 0).any() or not data.attack_probability_per_tick.between(0, 1).all():
        raise ValueError("Invalid density or attack probability")
    if (data.duration_min <= 0).any() or (data.perimeter_length_m <= 0).any():
        raise ValueError("Invalid duration or perimeter length")
    if data.duplicated(["design_id", "point_id"]).any():
        raise ValueError("Duplicate configuration summaries; do not concatenate batches")
    for col in ("resource_scenario", "design_id", "point_id"):
        if data[col].isna().any() or data[col].astype(str).str.strip().eq("").any():
            raise ValueError(f"Missing {col}")
    for col in ("mean_activity_coefficient", "mean_tco_eur", "std_tco_eur", "energy_price_eur_per_kwh"):
        data[col] = pd.to_numeric(data[col], errors="raise")
    activity = data.mean_activity_coefficient
    if np.isinf(activity).any() or (activity.dropna() < 0).any():
        raise ValueError("Invalid measured activity coefficient")
    missing_controls = [col for col in COMPARISON_CONTROLS if col not in data.columns]
    if missing_controls:
        warnings.warn(
            "Missing comparison controls: " + ", ".join(missing_controls)
            + "; configurations have not been fully validated as comparable.",
            UserWarning, stacklevel=2,
        )
    for col in COMPARISON_CONTROLS:
        if col not in data.columns:
            continue  # Older completed summaries remain usable with the explicit warning above.
        if data[col].isna().any() or data[col].astype(str).str.strip().eq("").any():
            raise ValueError(f"Missing controlled values: {col}")
        if col != "encounter_rearm_policy":
            data[col] = pd.to_numeric(data[col], errors="raise")
            if not np.isfinite(data[col]).all():
                raise ValueError(f"Non-finite controlled values: {col}")
        if data[col].nunique(dropna=False) != 1:
            raise ValueError(f"Comparisons contain differing controlled values: {col}")
    for col in ("duration_min", "perimeter_length_m", "perimeter_type", "patrol_strategy",
                "attack_model", "attack_duration_min", "capture_definition"):
        if data[col].nunique(dropna=False) != 1:
            raise ValueError(f"Comparisons contain differing controlled values: {col}")
    if not (data.attack_model == "PoissonAttack").all():
        raise ValueError("Attack probability labels require PoissonAttack")
    if not (data.capture_definition == "intercepted_over_resolved").all():
        raise ValueError("Unexpected capture metric definition")
    for kind in TYPES:
        for system in SYSTEMS:
            group = data[(data.experiment_type == kind) & (data.system_type == system)]
            if group.empty or group.design_id.nunique() != 1:
                raise ValueError(f"Expected one design for {kind}/{system}")
            if kind == "PATROL_DENSITY":
                grids = []
                for scenario, curve in group.groupby("resource_scenario"):
                    x = curve.patrol_density_robots_per_m
                    if len(x) < 2 or x.duplicated().any():
                        raise ValueError(f"Need distinct density points for {system}/{scenario}")
                    grids.append(set(x))
                if len(grids) < 2 or any(grid != grids[0] for grid in grids):
                    raise ValueError(f"Need at least two resource scenarios on the same density grid for {system}")
                if group.attack_probability_per_tick.nunique() != 1:
                    raise ValueError("Patrol-density figures require a fixed attack probability")
            elif kind == "ATTACK_DENSITY":
                if (group.resource_scenario.nunique() != 1 or group.patrol_density_robots_per_m.nunique() != 1
                        or len(group) < 2 or group.attack_probability_per_tick.duplicated().any()):
                    raise ValueError(f"Attack sweep must vary only probability for {system}")
            else:
                if len(group) < 2:
                    raise ValueError(f"Need at least two TCO configurations for {system}")
    tco = data[data.experiment_type == "TCO"]
    for col in ("mean_tco_eur", "std_tco_eur", "energy_price_eur_per_kwh"):
        if not np.isfinite(tco[col]).all() or (tco[col] < 0).any():
            raise ValueError(f"TCO requires finite non-negative {col}")
    if tco.cost_model.nunique() != 1 or tco.energy_price_eur_per_kwh.nunique() != 1:
        raise ValueError("TCO comparisons must share cost model and energy tariff")
    if tco.attack_probability_per_tick.nunique() != 1:
        raise ValueError("TCO comparisons require a fixed attack probability")
    return data

def generate(data, output=ROOT / "outputs" / "plots"):
    data = validate(data)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 10, "figure.figsize": (7, 4.5), "savefig.dpi": 200})
    paths = []
    def save(fig, ax, filename):
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        path = output / filename
        fig.savefig(path)
        plt.close(fig)
        paths.append(path)
    for i, system in enumerate(SYSTEMS, 1):
        fig, ax = plt.subplots()
        group = data[(data.experiment_type == "PATROL_DENSITY") & (data.system_type == system)]
        for scenario, curve in group.groupby("resource_scenario", sort=True):
            curve = curve.sort_values("patrol_density_robots_per_m")
            ax.errorbar(curve.patrol_density_robots_per_m, curve.mean_capture_coefficient,
                        yerr=curve.std_capture_coefficient, marker="o", capsize=3, label=scenario)
            for row in curve.itertuples():
                label = "activity undefined" if pd.isna(row.mean_activity_coefficient) else f"activity={row.mean_activity_coefficient:.2f}"
                ax.annotate(label, (row.patrol_density_robots_per_m, row.mean_capture_coefficient),
                            xytext=(3, 7), textcoords="offset points", fontsize=7)
        ax.set(xlabel="Patrol density (robots/m)", ylabel="Mean capture coefficient",
               title=f"{system.title()}: patrol density (error bars: SD)")
        save(fig, ax, f"figure_{i}_patrol_density_{system.lower()}.png")
    for i, system in enumerate(SYSTEMS, 4):
        fig, ax = plt.subplots()
        curve = data[(data.experiment_type == "ATTACK_DENSITY") & (data.system_type == system)].sort_values("attack_probability_per_tick")
        ax.errorbar(curve.attack_probability_per_tick, curve.mean_capture_coefficient,
                    yerr=curve.std_capture_coefficient, marker="o", capsize=3, label=system.title())
        ax.set(xlabel="Attack probability per one-minute tick", ylabel="Mean capture coefficient",
               title=f"{system.title()}: attack intensity (error bars: SD)")
        save(fig, ax, f"figure_{i}_attack_density_{system.lower()}.png")
    # Both orientations reuse these exact rows, ordering, means, and SDs.
    tco = data[data.experiment_type == "TCO"]
    for swapped, number in ((False, 7), (True, 8)):
        fig, ax = plt.subplots()
        for system in SYSTEMS:
            curve = tco[tco.system_type == system].sort_values("mean_tco_eur")
            cost, capture = curve.mean_tco_eur, curve.mean_capture_coefficient
            cost_sd, capture_sd = curve.std_tco_eur, curve.std_capture_coefficient
            ax.errorbar(capture if swapped else cost, cost if swapped else capture,
                        xerr=capture_sd if swapped else cost_sd, yerr=cost_sd if swapped else capture_sd,
                        marker="o", capsize=3, label=system.title())
        ax.set(xlabel="Mean capture coefficient" if swapped else "Mean TCO (EUR)",
               ylabel="Mean TCO (EUR)" if swapped else "Mean capture coefficient",
               title="TCO and capture coefficient (error bars: SD)")
        save(fig, ax, f"figure_{number}_{'tco_vs_capture' if swapped else 'capture_vs_tco'}.png")
    return paths

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary_csv", type=Path, help="experiment_summary.csv from one complete batch")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "plots")
    args = parser.parse_args(argv)
    try:
        if not args.summary_csv.is_file():
            raise ValueError(f"Experiment summary not found: {args.summary_csv}; run an experiment batch first")
        status = args.summary_csv.parent / "status.txt"
        if not status.is_file():
            raise ValueError("Missing batch status.txt; require a COMPLETE experiment batch")
        if status.read_text().strip() != "COMPLETE":
            raise ValueError("Experiment batch is incomplete or failed")
        paths = generate(pd.read_csv(args.summary_csv), args.output)
    except (ValueError, OSError, KeyError) as error:
        parser.exit(1, f"Cannot generate figures: {error}\n")
    print(f"Generated {len(paths)} figures in {args.output}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
