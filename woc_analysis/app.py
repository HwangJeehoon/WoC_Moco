from __future__ import annotations

from pathlib import Path
import re

import pandas as pd
import streamlit as st

from config import (
    COLOR_BY_CANDIDATES,
    DEFAULT_COLOR_BY,
    DEFAULT_METRIC_PATH,
    DEFAULT_QUEUE_PATH,
    DEFAULT_VISIBLE_COLUMNS,
    DEFAULT_X_METRIC_CANDIDATES,
    DEFAULT_Y_METRIC_CANDIDATES,
    HOVER_COLUMNS,
    MARKER_BY_CANDIDATES,
    DEFAULT_MARKER_BY,
)
from core.table_builder import build_master_table, get_metric_columns
from core.filter_utils import apply_basic_filters
from core.plot_utils import scatter_metric_plot

st.set_page_config(page_title="Simulation Result Explorer", layout="wide")

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


def get_id_prefix_options(df: pd.DataFrame, id_col: str = "ID") -> list[str]:
    """Return detected alphabetic prefixes from run IDs, preserving lowercase style."""
    if id_col not in df.columns:
        return []

    prefixes = (
        df[id_col]
        .dropna()
        .astype(str)
        .str.extract(r"^\s*([A-Za-z]+)", expand=False)
        .dropna()
        .str.lower()
        .unique()
        .tolist()
    )
    return sorted(prefixes)


def parse_custom_prefixes(text: str) -> list[str]:
    """Parse custom prefix input such as 'sp, af' or 'sp af'."""
    if not text:
        return []
    out: list[str] = []
    for token in re.split(r"[,;\s]+", text.strip()):
        token = token.strip().lower()
        if token and token not in out:
            out.append(token)
    return out


def existing_columns(df: pd.DataFrame, candidates: list[str]) -> list[str]:
    """Return configured columns that exist in df, preserving order and removing duplicates."""
    result: list[str] = []
    for col in candidates:
        if col in df.columns and col not in result:
            result.append(col)
    return result


def pick_first_existing(candidates: list[str], available: list[str], fallback: str | None = None) -> str | None:
    """Pick the first configured candidate that exists in available."""
    available_set = set(available)
    for col in candidates:
        if col in available_set:
            return col
    return fallback


def option_index(options: list[str | None], default_value: str | None) -> int:
    """Safe selectbox default index."""
    if default_value in options:
        return options.index(default_value)
    return 0


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

    st.markdown("**Run ID prefix / number range**")
    use_id_range_filter = st.checkbox(
        "Use prefix/range filter",
        value=False,
        help="Example: select sp and enter 001-025, select af and enter 006-012.",
    )

    id_range_rules = None
    if use_id_range_filter:
        detected_prefixes = get_id_prefix_options(master, id_col="ID")
        selected_prefixes = st.multiselect(
            "Detected prefixes",
            detected_prefixes,
            default=[],
            help="Prefixes are detected from the beginning of the ID column.",
        )
        custom_prefixes = parse_custom_prefixes(
            st.text_input(
                "Custom prefixes",
                value="",
                placeholder="e.g. sp, af",
                help="Use this if the prefix is not listed or if you want to type it directly.",
            )
        )

        prefix_list = []
        for p in [*selected_prefixes, *custom_prefixes]:
            p = str(p).strip().lower()
            if p and p not in prefix_list:
                prefix_list.append(p)

        if prefix_list:
            st.caption("Range format: 001-025, 006~012, 010, or 001-005, 010, 020-025")
            id_range_rules = {}
            for prefix in prefix_list:
                default_value = ""
                if prefix == "sp":
                    default_value = "001-025"
                elif prefix == "af":
                    default_value = "006-012"

                id_range_rules[prefix] = st.text_input(
                    f"{prefix} number range",
                    value=default_value,
                    key=f"id_range_{prefix}",
                    placeholder="001-025",
                    help="Leave empty to include all run numbers for this prefix.",
                )
        else:
            st.caption("Select or type at least one prefix to enable this filter.")
            id_range_rules = {}

filtered = apply_basic_filters(
    master,
    assist_values=assist_values,
    health_values=health_values,
    symmetry_values=symmetry_values,
    opt_values=opt_values,
    model_values=model_values_for_filter,
    id_range_rules=id_range_rules,
)

metric_cols = get_metric_columns(master)
metric_cols = [c for c in metric_cols if c in filtered.columns]
color_candidates = existing_columns(filtered, COLOR_BY_CANDIDATES)
marker_candidates = existing_columns(filtered, MARKER_BY_CANDIDATES)

preferred_x = pick_first_existing(
    DEFAULT_X_METRIC_CANDIDATES,
    metric_cols,
    fallback=metric_cols[0] if metric_cols else None,
)
preferred_y = pick_first_existing(
    DEFAULT_Y_METRIC_CANDIDATES,
    metric_cols,
    fallback=metric_cols[1] if len(metric_cols) > 1 else preferred_x,
)

with st.sidebar:
    st.divider()
    st.header("3. Plot settings")

    if not metric_cols:
        st.error("No numeric metric columns were found in metric_run.xlsx.")
        st.stop()

    x_metric = st.selectbox("X metric", metric_cols, index=metric_cols.index(preferred_x) if preferred_x in metric_cols else 0)
    y_metric = st.selectbox("Y metric", metric_cols, index=metric_cols.index(preferred_y) if preferred_y in metric_cols else 0)

    st.caption("Color/marker options are controlled in config.py.")

    color_options = [None] + color_candidates
    marker_options = [None] + marker_candidates

    group_default = DEFAULT_COLOR_BY if DEFAULT_COLOR_BY in color_candidates else (color_candidates[0] if color_candidates else None)
    symbol_default = DEFAULT_MARKER_BY if DEFAULT_MARKER_BY in marker_candidates else None

    group_by = st.selectbox(
        "Color by",
        color_options,
        index=option_index(color_options, group_default),
        format_func=lambda x: "None" if x is None else str(x),
    )

    symbol_by = st.selectbox(
        "Marker by",
        marker_options,
        index=option_index(marker_options, symbol_default),
        format_func=lambda x: "None" if x is None else str(x),
    )

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

hover_cols = existing_columns(plot_df, HOVER_COLUMNS)

fig = scatter_metric_plot(
    plot_df,
    x_metric,
    y_metric,
    group_by=group_by,
    symbol_by=symbol_by,
    hover_cols=hover_cols,
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
visible_default = existing_columns(filtered, DEFAULT_VISIBLE_COLUMNS)

show_cols = st.multiselect("Columns to show", filtered.columns.tolist(), default=visible_default)
st.dataframe(filtered[show_cols] if show_cols else filtered, use_container_width=True, height=420)

csv = filtered.to_csv(index=False).encode("utf-8-sig")
st.download_button(
    "Download filtered CSV",
    data=csv,
    file_name="filtered_simulation_runs.csv",
    mime="text/csv",
)
