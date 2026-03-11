# response_handler.py
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


VALID_ACTIONS = {"stop_cart", "alert_admin", "none"}
VALID_SEVERITIES = {"low", "medium", "high", "unknown"}


@dataclass(frozen=True)
class Decision:
    """Internal contract between LLM output and ROS2 downstream usage."""
    anomaly: bool
    severity: str          # low | medium | high | unknown
    action: str            # stop_cart | alert_admin | none
    summary: str           # short explanation (human readable)
    raw: Optional[Dict[str, Any]] = None  # keep original dict when possible


def _derive_severity(action: str, anomaly: bool) -> str:
    """Fallback mapping when severity is not explicitly provided."""
    if not anomaly:
        return "low"
    if action == "stop_cart":
        return "high"
    if action == "alert_admin":
        return "medium"
    return "unknown"


def _validate_payload(d: Dict[str, Any]) -> Tuple[bool, str]:
    """
    Validate required fields and allowed values.
    Returns (ok, error_message).
    """
    if "anomaly" not in d:
        return False, "Missing required field: anomaly"
    if not isinstance(d["anomaly"], bool):
        return False, "Field 'anomaly' must be a boolean"

    # Support either "action" or legacy "response"
    action = d.get("action", d.get("response"))
    if action is None:
        return False, "Missing required field: action (or legacy response)"
    if action not in VALID_ACTIONS:
        return False, f"Invalid action '{action}'. Must be one of {sorted(VALID_ACTIONS)}"

    # Only if we want to enforce severity values when provided (optional field)
    #if "severity" in d and d["severity"] not in VALID_SEVERITIES:
        #return False, f"Invalid severity '{d['severity']}'. Must be one of {sorted(VALID_SEVERITIES)}"

    # summary or reason are acceptable (we’ll normalize)
    if "summary" not in d and "reason" not in d:
        return False, "Missing required field: summary (or legacy reason)"

    return True, ""


def parse_llm_response(resp: Any) -> Decision:
    """
    Accepts:
      - Pydantic model instance (e.g., AnomalyResponse from openai_call.py)
      - dict
      - JSON string
      - plain text (treated as malformed -> fallback)

    Produces a validated Decision with safe fallbacks on errors.
    """
    # 1) Normalize to dict where possible
    payload: Optional[Dict[str, Any]] = None

    # Pydantic v1/v2 compatibility: model_dump() or dict()
    if hasattr(resp, "model_dump"):
        payload = resp.model_dump()
    elif hasattr(resp, "dict"):
        payload = resp.dict()
    elif isinstance(resp, dict):
        payload = resp
    elif isinstance(resp, str):
        s = resp.strip()
        # Try JSON parse if it looks like JSON
        if s.startswith("{") and s.endswith("}"):
            try:
                payload = json.loads(s)
            except Exception:
                payload = None
        else:
            payload = None
    else:
        payload = None

    # 2) Validate + build Decision
    if payload is None:
        return Decision(
            anomaly=False,
            severity="unknown",
            action="none",
            summary="Invalid/malformed LLM response (non-JSON). Falling back to safe default.",
            raw=None,
        )

    ok, err = _validate_payload(payload)
    if not ok:
        return Decision(
            anomaly=False,
            severity="unknown",
            action="none",
            summary=f"Validation error: {err}. Falling back to safe default.",
            raw=payload,
        )

    anomaly = payload["anomaly"]
    action = payload.get("action", payload.get("response"))  # support legacy field name
    summary = payload.get("summary", payload.get("reason", ""))
    severity = payload.get("severity") or _derive_severity(action, anomaly)

    return Decision(
        anomaly=anomaly,
        severity=severity,
        action=action,
        summary=summary,
        raw=payload,
    )
