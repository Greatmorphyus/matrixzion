#!/usr/bin/env python3
"""Offline deterministic smoke check for the Hermes earning integration."""

from __future__ import annotations

import json
import os
from pathlib import Path


ROOT = Path(os.environ.get("WORKSPACE_ROOT", Path(__file__).resolve().parents[1]))
FIXTURES = ROOT / "integrations" / "hermes" / "fixtures"


def next_action(item: dict[str, object]) -> str:
    if item.get("stale") is True:
        return "refresh_canonical_feed"
    if item.get("canonical") is not True or item.get("status") != "claimable":
        return "skip_unfunded"
    if item.get("verification_ready") is not True:
        return "skip_verification_unavailable"
    return "inspect_and_prepare_claim"


for name in ("claimable", "unfunded", "stale"):
    path = FIXTURES / f"{name}.json"
    item = json.loads(path.read_text(encoding="utf-8"))
    actual = next_action(item)
    expected = item.get("expected_action")
    if actual != expected:
        raise SystemExit(f"{name}: expected {expected}, got {actual}")

print("Hermes integration smoke checks passed")
