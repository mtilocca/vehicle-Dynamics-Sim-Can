#!/usr/bin/env python3
"""
plotter_analysis.py — Unified simulation analytics entry point.

Loads a simulation CSV and presents an interactive menu to choose which
analysis to run:
  1) Vehicle Dynamics  (Figures 1–5)
  2) Sensor Analysis   (Figures 6–13)
  3) Both

All generated figures are saved as PNG files in the output directory
(default: tmp/) and then displayed interactively.

Usage:
    python3 plotter_analysis.py [sim_out.csv] [--out-dir DIR]
"""
import argparse
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd

# Make tools.analytics importable when run from the repo root
_REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools.analytics import (
    DataUtils,
    VehicleDynamicsPlot,
    TireDynamicsPlot,
    LateralDynamicsPlot,
    WheelTorquesPlot,
    CombinedSlipPlot,
    SensorOverviewPlot,
    BatteryDetailPlot,
    WheelSensorsPlot,
    NoiseSpectrumPlot,
    BiasDriftPlot,
    ImuValidationPlot,
    GnssValidationPlot,
    RadarValidationPlot,
)

# ── Ordered plot-class registries ──────────────────────────────────────────────

_VEHICLE_DYNAMICS_CLASSES = [
    VehicleDynamicsPlot,
    TireDynamicsPlot,
    LateralDynamicsPlot,
    WheelTorquesPlot,
    CombinedSlipPlot,
]

_SENSOR_ANALYSIS_CLASSES = [
    SensorOverviewPlot,
    BatteryDetailPlot,
    WheelSensorsPlot,
    NoiseSpectrumPlot,
    BiasDriftPlot,
    ImuValidationPlot,
    GnssValidationPlot,
    RadarValidationPlot,
]

# ── Interactive menu ───────────────────────────────────────────────────────────

def _show_menu() -> str:
    print("\n" + "=" * 52)
    print("   Vehicle Dynamics Simulation Analytics")
    print("=" * 52)
    print("  1)  Vehicle Dynamics only  (Figs 1–5)")
    print("  2)  Sensor Analysis only   (Figs 6–13)")
    print("  3)  Both")
    print("=" * 52)
    while True:
        choice = input("Select [1/2/3]: ").strip()
        if choice in ("1", "2", "3"):
            return choice
        print("  Please enter 1, 2, or 3.")

# ── Figure runner ──────────────────────────────────────────────────────────────

def _collect_figures(df: pd.DataFrame,
                     class_list: list,
                     out_dir: str) -> list:
    """Instantiate each class, run it, and collect non-None Figures."""
    figures = []
    for cls in class_list:
        fig = cls().run(df, out_dir)
        if fig is not None:
            figures.append(fig)
    return figures

# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Vehicle Dynamics Simulation Analytics",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "csv", nargs="?", default="sim_out.csv",
        help="Path to simulation CSV (default: sim_out.csv)",
    )
    parser.add_argument(
        "--out-dir", default="tmp",
        help="Output directory for saved PNGs (default: tmp/)",
    )
    args = parser.parse_args()

    # Load & validate
    print(f"[INFO] Loading: {args.csv}")
    df = pd.read_csv(args.csv)
    print(f"[INFO] Loaded {len(df)} rows, {len(df.columns)} columns")
    DataUtils.validate_dataframe(df, args.csv)
    DataUtils.print_summary(df)

    # Menu
    choice = _show_menu()

    # Build selected class list
    to_run = []
    if choice in ("1", "3"):
        to_run.extend(_VEHICLE_DYNAMICS_CLASSES)
    if choice in ("2", "3"):
        to_run.extend(_SENSOR_ANALYSIS_CLASSES)

    # Run, save, collect
    all_figures = _collect_figures(df, to_run, args.out_dir)

    print(f"\n[INFO] Generated {len(all_figures)} figure(s) → '{args.out_dir}/'")
    print("[INFO] Displaying figures ...")
    plt.show()


if __name__ == "__main__":
    main()
