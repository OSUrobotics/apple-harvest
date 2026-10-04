from __future__ import annotations

import csv
import re
from pathlib import Path
from typing import Any, Mapping


PICK_LOG_FIELDS = (
    "timestamp",
    "batch_number",
    "apple_number",
    "pick_number",
    "status",
    "bag_path",
    "batch_directory",
    "recorded_at",
    "profile",
    "mode",
)

_STRUCTURED_CONTEXT = re.compile(
    r"HARVEST_CONTEXT\s+batch=(?P<batch>\d+)"
    r"(?:\s+apple=(?P<apple>\d+))?"
    r"(?:\s+batch_dir=(?P<batch_dir>\S+))?",
    re.IGNORECASE,
)
_BATCH_PATH = re.compile(
    r"(?P<batch_dir>/\S*?/batch_(?P<batch>\d+)/)"
    r"(?:apple_(?P<apple>\d+)/)?",
    re.IGNORECASE,
)
_APPLE_TEXT = re.compile(
    r"(?:freedrive apple|approaching apple|start with apple)\s+(?P<apple>\d+)",
    re.IGNORECASE,
)
_STRUCTURED_BAG = re.compile(
    r"BAG_CONTEXT\s+timestamp=(?P<timestamp>\d{8}_\d{6})"
    r"\s+bag_path=(?P<bag_path>\S+)",
    re.IGNORECASE,
)
_LEGACY_BAG = re.compile(
    r"Started recording topics:.*?\s+to\s+"
    r"(?P<bag_path>\S*?_(?P<timestamp>\d{8}_\d{6})\.db3)",
    re.IGNORECASE,
)


def extract_harvest_context(text: str) -> dict[str, Any]:
    """Extract the most recent batch/apple context from harvest output."""
    context: dict[str, Any] = {}
    for match in _STRUCTURED_CONTEXT.finditer(text):
        context["batch_number"] = int(match.group("batch"))
        if match.group("apple") is not None:
            context["apple_number"] = int(match.group("apple"))
        if match.group("batch_dir"):
            context["batch_directory"] = match.group("batch_dir")

    if not context:
        for match in _BATCH_PATH.finditer(text):
            context["batch_number"] = int(match.group("batch"))
            context["batch_directory"] = match.group("batch_dir")
            if match.group("apple") is not None:
                context["apple_number"] = int(match.group("apple"))

    apple_matches = list(_APPLE_TEXT.finditer(text))
    if apple_matches and "apple_number" not in context:
        context["apple_number"] = int(apple_matches[-1].group("apple"))
    return context


def extract_bag_context(text: str) -> dict[str, str]:
    """Extract the exact timestamp and path written by the bag recorder."""
    matches = list(_STRUCTURED_BAG.finditer(text))
    if not matches:
        matches = list(_LEGACY_BAG.finditer(text))
    if not matches:
        return {}
    match = matches[-1]
    return {
        "timestamp": match.group("timestamp"),
        "bag_path": match.group("bag_path"),
    }


def append_pick_record(path: str | Path, record: Mapping[str, Any]) -> Path:
    """Append one spreadsheet-compatible CSV record, creating its header."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            existing_rows = list(reader)
            existing_fields = tuple(reader.fieldnames or ())
        if existing_fields != PICK_LOG_FIELDS:
            temporary = path.with_suffix(path.suffix + ".tmp")
            with temporary.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=PICK_LOG_FIELDS)
                writer.writeheader()
                for existing in existing_rows:
                    writer.writerow({field: existing.get(field, "") for field in PICK_LOG_FIELDS})
            temporary.replace(path)
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=PICK_LOG_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow({field: record.get(field, "") for field in PICK_LOG_FIELDS})
    return path
