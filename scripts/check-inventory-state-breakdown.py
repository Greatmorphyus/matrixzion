#!/usr/bin/env python3
"""Build and self-test the inventory-state-breakdown-v1 response."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "scripts" / "fixtures" / "inventory-state-breakdown"
STATES = ("ready_to_earn", "in_progress", "submitted", "paid", "verification_unavailable")


def breakdown(snapshot: dict) -> dict:
    """Count one accepted canonical snapshot, failing closed when it is unusable."""
    source = snapshot.get("source")
    generated_at = snapshot.get("generated_at")
    observed_at = snapshot.get("observed_at")
    if not all(isinstance(value, str) and value for value in (source, generated_at, observed_at)):
        raise ValueError("source, generated_at, and observed_at are required")

    unavailable = bool(snapshot.get("degraded") or snapshot.get("stale"))
    counts = {state: 0 for state in STATES}
    items = snapshot.get("items", [])
    if not isinstance(items, list):
        raise ValueError("items must be a list")

    if unavailable:
        counts["verification_unavailable"] = len(items)
    else:
        for item in items:
            if not isinstance(item, dict):
                raise ValueError("inventory items must be objects")
            lifecycle = item.get("lifecycle")
            verification_ready = item.get("verification_ready") is True
            # Preserve the strict earning gate: merely claiming readiness is
            # insufficient when verification is unavailable.
            if lifecycle == "ready_to_earn" and not verification_ready:
                counts["verification_unavailable"] += 1
            elif lifecycle in STATES:
                counts[lifecycle] += 1
            else:
                counts["verification_unavailable"] += 1

    return {
        "schema": "inventory-state-breakdown-v1",
        "generated_at": generated_at,
        "observed_at": observed_at,
        "source": source,
        "source_available": not unavailable,
        "counts": counts,
    }


def main() -> None:
    for name in ("empty", "mixed", "degraded", "stale"):
        fixture = json.loads((FIXTURES / f"{name}.json").read_text(encoding="utf-8"))
        actual = breakdown(fixture["snapshot"])
        if actual != fixture["expected"]:
            raise SystemExit(f"{name}: expected {fixture['expected']!r}, got {actual!r}")
    print("inventory-state-breakdown-v1 fixtures passed")


if __name__ == "__main__":
    main()
