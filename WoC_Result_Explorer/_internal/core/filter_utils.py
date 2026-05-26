from __future__ import annotations

import pandas as pd


def apply_basic_filters(
    df: pd.DataFrame,
    assist_filter: str = "All",
    assist_values: list[str] | None = None,
    health_values: list[str] | None = None,
    symmetry_values: list[str] | None = None,
    opt_values: list[str] | None = None,
    model_values: list[str] | None = None,
) -> pd.DataFrame:
    out = df.copy()

    if assist_values is not None and "assist_on" in out.columns:
        include_on = "Assist On" in assist_values
        include_off = "Assist Off" in assist_values
        if include_on and not include_off:
            out = out[out["assist_on"] == True]
        elif include_off and not include_on:
            out = out[out["assist_on"] == False]
        elif not include_on and not include_off:
            out = out.iloc[0:0]
    elif assist_filter == "Assist only" and "assist_on" in out.columns:
        out = out[out["assist_on"] == True]
    elif assist_filter == "No assist only" and "assist_on" in out.columns:
        out = out[out["assist_on"] == False]

    if health_values is not None and "health_label" in out.columns:
        out = out[out["health_label"].isin(health_values)]

    if symmetry_values is not None and "symmetry_label" in out.columns:
        out = out[out["symmetry_label"].isin(symmetry_values)]

    if opt_values and "assist_label" in out.columns:
        out = out[out["assist_label"].isin(opt_values)]

    if model_values and "model" in out.columns:
        out = out[out["model"].isin(model_values)]

    return out
