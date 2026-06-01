from __future__ import annotations

from pathlib import Path

# =============================================================================
# Project paths
# =============================================================================
# Default assumption:
#   WOC/
#   ├─ app.py
#   ├─ core/
#   ├─ data/metric_run.xlsx
#   └─ script/simulation_queue.xlsx
PROJECT_ROOT = Path(__file__).resolve().parent.parent

DEFAULT_QUEUE_PATH = PROJECT_ROOT / "script" / "simulation_queue.xlsx"
DEFAULT_METRIC_PATH = PROJECT_ROOT / "data" / "metric_run.xlsx"


# =============================================================================
# Metric defaults
# =============================================================================
# The app will use the first existing metric in each list.
# If none exists, it falls back to the first/second metric column found.
DEFAULT_X_METRIC_CANDIDATES = [
    "Froude_number",
    "FroudeNumber",
    "froude_number",
]

DEFAULT_Y_METRIC_CANDIDATES = [
    "CMA_4set_PD",
    "CMAPD",
    "CMA_PD",
]


# =============================================================================
# Plot grouping candidates
# =============================================================================
# Only columns that actually exist in the loaded master table are shown in the UI.
COLOR_BY_CANDIDATES = [
    "run_prefix",
    "health_label",
    "symmetry_label",
    "assist_label",
    "opt_mode_label",
    "model",
    "gaitMode",
    "Gaitmode",
    "gait_mode",
    "optMode_type",
]

MARKER_BY_CANDIDATES = [
    "symmetry_label",
    "assist_label",
    "health_label",
    "opt_mode_label",
    "gaitMode",
    "Gaitmode",
    "gait_mode",
    "optMode_type",
]

DEFAULT_COLOR_BY = "health_label"
DEFAULT_MARKER_BY = "symmetry_label"


# =============================================================================
# Hover / table display
# =============================================================================
HOVER_COLUMNS = [
    "ID",
    "run_prefix",
    "run_number",
    "metric_iter",
    "queue_iter",
    "model",
    "health_label",
    "symmetry_label",
    "assist_label",
    "opt_mode_label",
    "gaitMode",
    "Gaitmode",
    "optMode_type",
    "mocoFinalTime",
    "trigger",
    "maxVal",
]

DEFAULT_VISIBLE_COLUMNS = [
    "ID",
    "run_prefix",
    "run_number",
    "metric_iter",
    "model",
    "health_label",
    "symmetry_label",
    "assist_label",
    "opt_mode_label",
    "Froude_number",
    "CMA_4set_PD",
    "propulsion",
    "apGRF_max",
    "velocity_average",
    "stride_length",
    "mocoFinalTime",
]
