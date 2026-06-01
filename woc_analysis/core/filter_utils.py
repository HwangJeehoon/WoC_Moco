from __future__ import annotations

import re
import pandas as pd


def _parse_number_ranges(range_text: str | None) -> list[tuple[int, int]] | None:
    """
    Parse a compact numeric range expression.

    Examples
    --------
    "001-025" -> [(1, 25)]
    "006~012" -> [(6, 12)]
    "001-005, 010, 020-025" -> [(1, 5), (10, 10), (20, 25)]

    Returns
    -------
    None
        Empty input. This means "all numbers for this prefix".
    list[tuple[int, int]]
        Parsed inclusive ranges. Invalid tokens are ignored.
    """
    if range_text is None or str(range_text).strip() == "":
        return None

    ranges: list[tuple[int, int]] = []
    tokens = re.split(r"[,;]+", str(range_text).strip())

    for token in tokens:
        token = token.strip()
        if not token:
            continue

        range_match = re.match(r"^(\d+)\s*(?:-|~|:)\s*(\d+)$", token)
        single_match = re.match(r"^(\d+)$", token)

        if range_match:
            start = int(range_match.group(1))
            end = int(range_match.group(2))
            if start > end:
                start, end = end, start
            ranges.append((start, end))
        elif single_match:
            value = int(single_match.group(1))
            ranges.append((value, value))

    return ranges


def apply_id_prefix_range_filter(
    df: pd.DataFrame,
    id_range_rules: dict[str, str] | None,
    id_col: str = "ID",
) -> pd.DataFrame:
    """
    Filter rows by ID prefix and numeric suffix.

    Parameters
    ----------
    id_range_rules
        Dict like {"sp": "001-025", "af": "006-012"}.
        Multiple prefix rules are OR-combined.
        Empty range text means all IDs with that prefix.
    """
    if not id_range_rules or id_col not in df.columns:
        return df

    id_text = df[id_col].astype(str).str.strip()
    parsed = id_text.str.extract(r"^([A-Za-z]+)(\d+)", expand=True)

    id_prefix = parsed[0].str.lower()
    id_number = pd.to_numeric(parsed[1], errors="coerce")

    keep_mask = pd.Series(False, index=df.index)

    for prefix, range_text in id_range_rules.items():
        prefix = str(prefix).strip().lower()
        if not prefix:
            continue

        prefix_mask = id_prefix == prefix
        ranges = _parse_number_ranges(range_text)

        if ranges is None:
            keep_mask |= prefix_mask
            continue

        range_mask = pd.Series(False, index=df.index)
        for start, end in ranges:
            range_mask |= id_number.between(start, end, inclusive="both")

        keep_mask |= prefix_mask & range_mask

    return df[keep_mask]


def _filter_by_selected_values(
    df: pd.DataFrame,
    column: str,
    selected_values: list[str] | None,
) -> pd.DataFrame:
    """
    Apply a checkbox-style categorical filter.

    None means the UI/control does not exist, so do not filter.
    [] means the user unchecked everything, so return an empty table.
    """
    if selected_values is None or column not in df.columns:
        return df
    return df[df[column].isin(selected_values)]


def apply_basic_filters(
    df: pd.DataFrame,
    assist_values: list[str] | None = None,
    health_values: list[str] | None = None,
    symmetry_values: list[str] | None = None,
    opt_values: list[str] | None = None,
    model_values: list[str] | None = None,
    id_range_rules: dict[str, str] | None = None,
    # Backward-compatible argument. Older app versions may still call this.
    assist_filter: str | None = None,
) -> pd.DataFrame:
    out = df.copy()

    # New checkbox-style assist filter.
    if assist_values is not None and "assist_on" in out.columns:
        selected_bool_values: list[bool] = []
        if "Assist On" in assist_values:
            selected_bool_values.append(True)
        if "Assist Off" in assist_values:
            selected_bool_values.append(False)
        out = out[out["assist_on"].isin(selected_bool_values)]

    # Backward compatibility for older dropdown-style assist filter.
    elif assist_filter is not None and "assist_on" in out.columns:
        if assist_filter == "Assist only":
            out = out[out["assist_on"] == True]
        elif assist_filter == "No assist only":
            out = out[out["assist_on"] == False]

    out = _filter_by_selected_values(out, "health_label", health_values)
    out = _filter_by_selected_values(out, "symmetry_label", symmetry_values)
    out = _filter_by_selected_values(out, "assist_label", opt_values)

    # Model filter: None means all models. Empty list is also treated as all models,
    # because app.py sends None when the model multiselect is empty.
    if model_values and "model" in out.columns:
        out = out[out["model"].isin(model_values)]

    out = apply_id_prefix_range_filter(out, id_range_rules=id_range_rules, id_col="ID")

    return out
