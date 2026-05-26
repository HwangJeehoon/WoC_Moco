from __future__ import annotations

import os
import sys
from pathlib import Path

import pandas as pd
import streamlit as st

from core.table_builder import build_master_table, get_categorical_columns, get_metric_columns
from core.filter_utils import apply_basic_filters
from core.plot_utils import scatter_metric_plot

st.set_page_config(page_title="Simulation Result Explorer", layout="wide")

# -----------------------------------------------------------------------------
# Default local file paths
# -----------------------------------------------------------------------------
# Edit these two paths to your normal working files.
# Relative paths are interpreted from the folder where you run `streamlit run app.py`.


def get_distribution_root() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent.parent

    bundle_env = os.environ.get("WOC_BUNDLE_DIR")
    if bundle_env:
        return Path(bundle_env).resolve().parent.parent

    return Path(__file__).resolve().parent


DIST_ROOT = get_distribution_root()

DEFAULT_QUEUE_PATH = DIST_ROOT / "script" / "simulation_queue.xlsx"
DEFAULT_METRIC_PATH = DIST_ROOT / "data" / "metric_run.xlsx"

st.title("Simulation Result Explorer")
st.caption("Filter simulation runs, group conditions, and explore metric relationships.")


@st.cache_data(show_spinner=False)
def load_master_table_from_paths(queue_path_str: str, metric_path_str: str) -> pd.DataFrame:
    return build_master_table(Path(queue_path_str), Path(metric_path_str))


@st.cache_data(show_spinner=False)
def load_master_table_from_uploads(queue_bytes: bytes, metric_bytes: bytes) -> pd.DataFrame:
    tmp_dir = Path(".streamlit_cache_files")
    tmp_dir.mkdir(exist_ok=True)
    queue_path = tmp_dir / "simulation_queue.xlsx"
    metric_path = tmp_dir / "metric_run.xlsx"
    queue_path.write_bytes(queue_bytes)
    metric_path.write_bytes(metric_bytes)
    return build_master_table(queue_path, metric_path)


def checkbox_group(label: str, options: list[str], default: list[str] | None = None, columns: int = 3) -> list[str]:
    """Render a compact checkbox group and return selected values."""
    if default is None:
        default = options

    st.markdown(f"**{label}**")

    if not options:
        st.caption("No options found.")
        return []

    selected: list[str] = []
    cols = st.columns(min(columns, max(1, len(options))))
    for i, option in enumerate(options):
        with cols[i % len(cols)]:
            checked = st.checkbox(
                str(option),
                value=option in default,
                key=f"{label}_{option}",
            )
        if checked:
            selected.append(option)

    return selected


with st.sidebar:
    st.header("1. Load files")

    use_default_files = st.checkbox("Use default local files", value=True)

    if use_default_files:
        queue_path_text = st.text_input("Default queue path", value=str(DEFAULT_QUEUE_PATH))
        metric_path_text = st.text_input("Default metric path", value=str(DEFAULT_METRIC_PATH))
        queue_file = None
        metric_file = None
    else:
        queue_file = st.file_uploader("simulation_queue.xlsx", type=["xlsx"])
        metric_file = st.file_uploader("metric_run.xlsx", type=["xlsx"])
        queue_path_text = ""
        metric_path_text = ""

    st.divider()
    st.header("2. Filters")

try:
    if use_default_files:
        queue_path = Path(queue_path_text)
        metric_path = Path(metric_path_text)

        if not queue_path.exists() or not metric_path.exists():
            st.info(
                "Default files were not found. Either edit the paths in the sidebar "
                "or uncheck 'Use default local files' and upload files manually."
            )
            st.write("Queue path:", queue_path.resolve())
            st.write("Metric path:", metric_path.resolve())
            st.stop()

        master = load_master_table_from_paths(str(queue_path), str(metric_path))
    else:
        if queue_file is None or metric_file is None:
            st.info("Upload `simulation_queue.xlsx` and `metric_run.xlsx` to start.")
            st.stop()
        master = load_master_table_from_uploads(queue_file.getvalue(), metric_file.getvalue())
except Exception as e:
    st.error("Failed to load files.")
    st.exception(e)
    st.stop()

with st.sidebar:
    if "assist_on" in master.columns:
        assist_values = checkbox_group("Assist", ["Assist On", "Assist Off"], default=["Assist On", "Assist Off"], columns=2)
    else:
        assist_values = None

    health_options = sorted(master["health_label"].dropna().unique().tolist()) if "health_label" in master.columns else []
    health_values = checkbox_group("Health / Pathology", health_options, default=health_options, columns=2)

    symmetry_options = [x for x in ["Symmetric", "Asymmetric"] if "symmetry_label" in master.columns and x in set(master["symmetry_label"].dropna().unique())]
    if not symmetry_options and "symmetry_label" in master.columns:
        symmetry_options = sorted(master["symmetry_label"].dropna().unique().tolist())
    symmetry_values = checkbox_group("Symmetricity", symmetry_options, default=symmetry_options, columns=2)

    opt_options = sorted(master["assist_label"].dropna().unique().tolist()) if "assist_label" in master.columns else []
    opt_values = checkbox_group("Assist / Opt group", opt_options, default=opt_options, columns=3)

    st.markdown("**Model**")
    model_options = sorted(master["model"].dropna().unique().tolist()) if "model" in master.columns else []
    model_values = st.multiselect(
        "Model filter. Empty means all models.",
        model_options,
        default=[],
        label_visibility="collapsed",
    )
    model_values_for_filter = model_values if model_values else None

filtered = apply_basic_filters(
    master,
    assist_values=assist_values,
    health_values=health_values,
    symmetry_values=symmetry_values,
    opt_values=opt_values,
    model_values=model_values_for_filter,
)

metric_cols = get_metric_columns(master)
metric_cols = [c for c in metric_cols if c in filtered.columns]
cat_cols = get_categorical_columns(filtered)

preferred_x = "Froude_number" if "Froude_number" in metric_cols else (metric_cols[0] if metric_cols else None)
preferred_y = "CMA_4set_PD" if "CMA_4set_PD" in metric_cols else (metric_cols[1] if len(metric_cols) > 1 else preferred_x)

with st.sidebar:
    st.divider()
    st.header("3. Plot settings")

    if not metric_cols:
        st.error("No numeric metric columns were found in metric_run.xlsx.")
        st.stop()

    x_metric = st.selectbox("X metric", metric_cols, index=metric_cols.index(preferred_x) if preferred_x in metric_cols else 0)
    y_metric = st.selectbox("Y metric", metric_cols, index=metric_cols.index(preferred_y) if preferred_y in metric_cols else 0)

    group_default = "health_label" if "health_label" in cat_cols else (cat_cols[0] if cat_cols else None)
    group_by = st.selectbox("Color by", [None] + cat_cols, index=([None] + cat_cols).index(group_default) if group_default in cat_cols else 0)

    symbol_default = "symmetry_label" if "symmetry_label" in cat_cols else None
    symbol_by = st.selectbox("Symbol by", [None] + cat_cols, index=([None] + cat_cols).index(symbol_default) if symbol_default in cat_cols else 0)

    st.subheader("Axis scale")
    log_x = st.checkbox("Log X axis", value=False)
    log_y = st.checkbox("Log Y axis", value=False)

    with st.expander("Figure style", expanded=False):
        marker_size = st.slider("Marker size", min_value=3, max_value=14, value=6, step=1)
        title_font_size = st.slider("Title font size", min_value=16, max_value=40, value=26, step=1)
        axis_title_font_size = st.slider("Axis title font size", min_value=14, max_value=34, value=22, step=1)
        axis_tick_font_size = st.slider("Axis tick font size", min_value=10, max_value=28, value=17, step=1)
        legend_font_size = st.slider("Legend font size", min_value=10, max_value=28, value=17, step=1)

left, right = st.columns([1, 1])
with left:
    st.metric("Total runs", len(master))
with right:
    st.metric("Filtered runs", len(filtered))

if filtered.empty:
    st.warning("No runs match the current filters.")
    st.stop()

plot_df = filtered.copy()

if log_x:
    before = len(plot_df)
    plot_df = plot_df[pd.to_numeric(plot_df[x_metric], errors="coerce") > 0]
    removed = before - len(plot_df)
    if removed > 0:
        st.warning(f"Log X axis: removed {removed} runs because {x_metric} <= 0 or NaN.")

if log_y:
    before = len(plot_df)
    plot_df = plot_df[pd.to_numeric(plot_df[y_metric], errors="coerce") > 0]
    removed = before - len(plot_df)
    if removed > 0:
        st.warning(f"Log Y axis: removed {removed} runs because {y_metric} <= 0 or NaN.")

if plot_df.empty:
    st.warning("No runs remain after applying log-scale constraints.")
    st.stop()

fig = scatter_metric_plot(
    plot_df,
    x_metric,
    y_metric,
    group_by=group_by,
    symbol_by=symbol_by,
    marker_size=marker_size,
    title_font_size=title_font_size,
    axis_title_font_size=axis_title_font_size,
    axis_tick_font_size=axis_tick_font_size,
    legend_font_size=legend_font_size,
    log_x=log_x,
    log_y=log_y,
)
st.plotly_chart(fig, use_container_width=True)

st.subheader("Save plot")
html_bytes = fig.to_html(include_plotlyjs="cdn", full_html=True).encode("utf-8")
st.download_button(
    "Download plot as HTML",
    data=html_bytes,
    file_name=f"plot_{y_metric}_vs_{x_metric}.html",
    mime="text/html",
)

try:
    png_bytes = fig.to_image(format="png", scale=3)
    st.download_button(
        "Download plot as PNG",
        data=png_bytes,
        file_name=f"plot_{y_metric}_vs_{x_metric}.png",
        mime="image/png",
    )
except Exception:
    st.caption("PNG export needs kaleido. Install it with: pip install kaleido")

st.subheader("Filtered run table")
visible_default = [c for c in [
    "ID", "metric_iter", "model", "health_label", "symmetry_label", "assist_label",
    "Froude_number", "CMA_4set_PD", "propulsion", "apGRF_max", "velocity_average",
    "stride_length", "mocoFinalTime"
] if c in filtered.columns]

show_cols = st.multiselect("Columns to show", filtered.columns.tolist(), default=visible_default)
st.dataframe(filtered[show_cols] if show_cols else filtered, use_container_width=True, height=420)

csv = filtered.to_csv(index=False).encode("utf-8-sig")
st.download_button(
    "Download filtered CSV",
    data=csv,
    file_name="filtered_simulation_runs.csv",
    mime="text/csv",
)
