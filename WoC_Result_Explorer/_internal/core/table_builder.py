from __future__ import annotations

from pathlib import Path
import pandas as pd

from .data_loader import load_queue_tables, read_metric_data


def _is_empty_value(value: object) -> bool:
    if value is None or pd.isna(value):
        return True
    return str(value).strip() == ""


def _find_column_case_insensitive(df: pd.DataFrame, candidates: list[str]) -> str | None:
    lower_to_original = {str(c).lower(): c for c in df.columns}
    for name in candidates:
        found = lower_to_original.get(name.lower())
        if found is not None:
            return found
    return None


def add_run_labels(df: pd.DataFrame) -> pd.DataFrame:
    """Add labels used for filtering/grouping in the explorer."""
    out = df.copy()

    id_series = out["ID"].astype(str) if "ID" in out.columns else pd.Series("", index=out.index)
    prefix = id_series.str.extract(r"^([A-Z]{2})", expand=False).fillna("")
    assist_code = prefix.str[1].fillna("")

    assist_map = {
        "F": "Off",
        "W": "WoC",
        "P": "Spline assist",
    }
    out["assist_label"] = assist_code.map(assist_map).fillna("Unknown")
    out["assist_on"] = assist_code.isin(["W", "P"])

    # Health/pathology is determined from the model sheet's muscle column.
    # Empty muscles -> healthy, non-empty muscles -> pathology.
    muscle_col = _find_column_case_insensitive(out, ["muscle", "muscles"])
    if muscle_col is not None:
        out["health_label"] = out[muscle_col].apply(lambda v: "Healthy" if _is_empty_value(v) else "Pathology")
    else:
        out["health_label"] = "Unknown"

    # Symmetric/asymmetric is determined from Gaitmode/gaitMode.
    gait_mode_col = _find_column_case_insensitive(out, ["Gaitmode", "gaitMode", "gait_mode"])
    if gait_mode_col is not None:
        normalized = out[gait_mode_col].astype(str).str.strip().str.lower()
        out["symmetry_label"] = "Unknown"
        out.loc[normalized.str.contains("sym", na=False) & ~normalized.str.contains("asym", na=False), "symmetry_label"] = "Symmetric"
        out.loc[normalized.str.contains("asym", na=False), "symmetry_label"] = "Asymmetric"
    else:
        out["symmetry_label"] = "Unknown"

    # Backward-compatible combined label, but the UI no longer relies on this.
    out["gait_label"] = out["health_label"].astype(str) + " / " + out["symmetry_label"].astype(str)

    if "optMode_type" in out.columns:
        out["opt_mode_label"] = out["optMode_type"].replace({
            "modeOff": "Off",
            "modeWoC": "WoC",
            "modeSpline": "Spline assist",
        })

    return out


def build_master_table(queue_file: str | Path, metric_file: str | Path) -> pd.DataFrame:
    """
    Build one analysis table from completed_queue + models + metric data.

    Join policy:
      - completed_queue.ID <-> metric_run.data.ID
      - completed_queue.model <-> models.model
    """
    _, completed, models = load_queue_tables(queue_file)
    metric = read_metric_data(metric_file)
    metric_source_columns = [c for c in metric.columns if c not in {"ID", "metric_iter"}]

    if "ID" not in completed.columns:
        raise KeyError("completed_queue sheet must have an 'ID' column.")
    if "ID" not in metric.columns:
        raise KeyError("metric data sheet must have an 'ID' column.")

    if "iter" in completed.columns:
        completed = completed.rename(columns={"iter": "queue_iter"})

    master = completed.merge(metric, on="ID", how="inner", suffixes=("_queue", "_metric"))

    if "model" in master.columns and "model" in models.columns:
        master = master.merge(models, on="model", how="left", suffixes=("", "_model"))

    master = add_run_labels(master)

    text_columns = {
        "ID", "model", "result_name", "resume_name", "optMode_type", "gaitMode",
        "Gaitmode", "gait_mode", "health_label", "symmetry_label", "gait_label",
        "assist_label", "opt_mode_label",
    }

    for col in master.columns:
        if col in text_columns:
            continue

        if master[col].dtype == object:
            converted = pd.to_numeric(master[col], errors="coerce")
            original_non_empty = master[col].notna() & (master[col].astype(str).str.strip() != "")
            converted_non_na = converted.notna()

            if converted_non_na.sum() > 0 and converted_non_na.sum() >= 0.5 * original_non_empty.sum():
                master[col] = converted

    master.attrs["metric_columns"] = [c for c in metric_source_columns if c in master.columns]

    return master


def get_metric_columns(df: pd.DataFrame) -> list[str]:
    """Return numeric columns that originated from metric_run.xlsx."""
    metric_cols = df.attrs.get("metric_columns", [])
    return [c for c in metric_cols if c in df.columns and pd.api.types.is_numeric_dtype(df[c])]


def get_numeric_columns(df: pd.DataFrame) -> list[str]:
    return [c for c in df.columns if pd.api.types.is_numeric_dtype(df[c])]


def get_categorical_columns(df: pd.DataFrame) -> list[str]:
    preferred_first = [
        "health_label", "symmetry_label", "assist_label", "opt_mode_label", "model",
        "gaitMode", "optMode_type",
    ]

    cols = []
    for c in df.columns:
        if c == "ID":
            continue
        if not pd.api.types.is_numeric_dtype(df[c]):
            cols.append(c)
        elif df[c].nunique(dropna=True) <= 12:
            cols.append(c)

    ordered = [c for c in preferred_first if c in cols]
    ordered += [c for c in cols if c not in ordered]
    return ordered
