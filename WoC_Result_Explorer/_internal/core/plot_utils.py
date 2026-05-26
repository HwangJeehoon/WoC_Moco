from __future__ import annotations

import pandas as pd
import plotly.express as px


QUALITATIVE_COLORMAP = (
    px.colors.qualitative.Bold
    + px.colors.qualitative.Set2
    + px.colors.qualitative.Dark24
    + px.colors.qualitative.Plotly
)


def scatter_metric_plot(
    df: pd.DataFrame,
    x_metric: str,
    y_metric: str,
    group_by: str | None = None,
    symbol_by: str | None = None,
    *,
    marker_size: int = 6,
    title_font_size: int = 26,
    axis_title_font_size: int = 22,
    axis_tick_font_size: int = 17,
    legend_font_size: int = 17,
    log_x: bool = False,
    log_y: bool = False,
):
    """Create the main interactive scatter plot."""
    hover_cols = [
        c
        for c in [
            "ID",
            "metric_iter",
            "queue_iter",
            "model",
            "health_label",
            "symmetry_label",
            "assist_label",
            "mocoFinalTime",
            "trigger",
            "maxVal",
        ]
        if c in df.columns
    ]

    fig = px.scatter(
        df,
        x=x_metric,
        y=y_metric,
        color=group_by if group_by else None,
        symbol=symbol_by if symbol_by else None,
        hover_data=hover_cols,
        title=f"{y_metric} vs {x_metric}",
        color_discrete_sequence=QUALITATIVE_COLORMAP,
    )

    fig.update_traces(marker={"size": marker_size, "opacity": 0.82})

    fig.update_layout(
        height=720,
        title={
            "font": {"size": title_font_size},
            "x": 0.02,
            "xanchor": "left",
        },
        font={"size": axis_tick_font_size},
        legend_title_text=group_by if group_by else "Group",
        legend={
            "font": {"size": legend_font_size},
            "title": {"font": {"size": legend_font_size + 1}},
            "itemsizing": "constant",
            "bgcolor": "rgba(255,255,255,0.65)",
        },
        margin={"l": 90, "r": 40, "t": 90, "b": 80},
    )

    fig.update_xaxes(
        title_font={"size": axis_title_font_size},
        tickfont={"size": axis_tick_font_size},
        showgrid=True,
        zeroline=False,
    )
    fig.update_yaxes(
        title_font={"size": axis_title_font_size},
        tickfont={"size": axis_tick_font_size},
        showgrid=True,
        zeroline=False,
    )

    if log_x:
        fig.update_xaxes(type="log")

    if log_y:
        fig.update_yaxes(type="log")

    return fig
