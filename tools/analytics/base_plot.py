"""Base class for all analytics plot classes."""
import os
from abc import ABC, abstractmethod

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.figure import Figure


class BasePlot(ABC):
    """
    Abstract base class for every analytics plot.

    Subclasses must implement:
        _filename (property) — output stem, e.g. "fig01_vehicle_dynamics"
        plot(df)             — build and return a Figure, or None if data absent

    Typical usage:
        fig = MyPlot().run(df, out_dir="tmp")   # generate + save + print
        plt.show()                               # called once by the entry point
    """

    # ── subclass contract ──────────────────────────────────────────────────

    @property
    @abstractmethod
    def _filename(self) -> str:
        """Output filename stem (no path, no extension)."""

    @abstractmethod
    def plot(self, df: pd.DataFrame) -> "Figure | None":
        """
        Build and return the matplotlib Figure from *df*.

        Return None (and print a [SKIP] message) when required columns are
        absent — BasePlot.run() will handle the skip gracefully.
        """

    # ── concrete shared helpers ────────────────────────────────────────────

    def save(self, fig: Figure, out_dir: str = "tmp") -> str:
        """Save *fig* as PNG inside *out_dir*.  Returns the full path."""
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f"{self._filename}.png")
        fig.savefig(path, dpi=150, bbox_inches="tight")
        return path

    def run(self, df: pd.DataFrame, out_dir: str = "tmp") -> "Figure | None":
        """Orchestrate plot → save → print.  Returns the Figure (or None)."""
        print(f"[INFO] Generating {self._filename} ...")
        fig = self.plot(df)
        if fig is None:
            print(f"[SKIP] {self._filename} — required data not present")
            return None
        path = self.save(fig, out_dir)
        print(f"[INFO] Saved {path}")
        return fig
