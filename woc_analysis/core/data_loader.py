from __future__ import annotations

from pathlib import Path
import re
import pandas as pd


def _normalize_col_name(name: object) -> str:
    """Make Excel column names safe and consistent enough for pandas usage."""
    if name is None or (isinstance(name, float) and pd.isna(name)):
        return ""
    text = str(name).strip()
    text = re.sub(r"\s+", "_", text)
    return text


def read_endheader_sheet(path: str | Path, sheet_name: str) -> pd.DataFrame:
    """
    Read sheets where metadata rows come first, followed by an 'endheader' row,
    then the actual table header row.

    Example:
        row 1-7: metadata
        row 8:   endheader
        row 9:   ID, Date, model, ...
        row 10+: data
    """
    raw = pd.read_excel(path, sheet_name=sheet_name, header=None, engine="openpyxl")

    first_col = raw.iloc[:, 0].astype(str).str.strip().str.lower()
    matches = first_col[first_col == "endheader"]
    if matches.empty:
        raise ValueError(f"Sheet '{sheet_name}' does not contain an 'endheader' row.")

    header_idx = int(matches.index[0]) + 1
    if header_idx >= len(raw):
        raise ValueError(f"Sheet '{sheet_name}' has no header row after 'endheader'.")

    headers = [_normalize_col_name(v) for v in raw.iloc[header_idx].tolist()]
    data = raw.iloc[header_idx + 1 :].copy()
    data.columns = headers

    # Drop empty unnamed columns and fully empty rows.
    data = data.loc[:, [c for c in data.columns if c != ""]]
    data = data.dropna(how="all").reset_index(drop=True)
    return data


def read_metric_data(path: str | Path, sheet_name: str = "data") -> pd.DataFrame:
    """Read metric_run.xlsx data sheet."""
    df = pd.read_excel(path, sheet_name=sheet_name, engine="openpyxl")
    df.columns = [_normalize_col_name(c) for c in df.columns]
    df = df.dropna(how="all").reset_index(drop=True)

    # Avoid name collision with completed_queue.iter.
    if "Iter" in df.columns:
        df = df.rename(columns={"Iter": "metric_iter"})
    elif "iter" in df.columns:
        df = df.rename(columns={"iter": "metric_iter"})
    return df


def load_queue_tables(queue_file: str | Path) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    pending = read_endheader_sheet(queue_file, "simulation_queue")
    completed = read_endheader_sheet(queue_file, "completed_queue")
    models = read_endheader_sheet(queue_file, "models")

    if "Names" in models.columns:
        models = models.rename(columns={"Names": "model"})

    return pending, completed, models
