"""Response handler for LLM outputs. 
Ensures consistent Decision objects with validation given LLM responses.

Author: AAD Team Spring 26'
Version: 4/21/26
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

VALID_ACTIONS = {"stop_cart", "alert_admin", "none"}
VALID_SEVERITIES = {"low", "medium", "high", "unknown"}

## Definition of the `Decision` type that is actually returned.s
@dataclass(frozen=True)
class Decision:
    """Internal contract between LLM output and ROS2 downstream usage."""
    anomaly: bool
    severity: str                         # low | medium | high | unknown
    action: str                           # stop_cart | alert_admin | none
    summary: str                          # short explanation (human readable)
    raw: Optional[Dict[str, Any]] = None  # keep original dict when possible


def _derive_severity(action: str, anomaly: bool) -> str:
    """
    Util function to associate severity with the action with severity level.
    
    Args
    ----
        action (str): the recommended action returned from the LLM.

        anomaly (bool): whether the LLM detected an anomaly.
    
    Returns
    -------
        str: associated severity.
            >>> str : ["low", "medium", "high", "unknown"]
    
    """
    if not anomaly:
        return "low"
    if action == "stop_cart":
        return "high"
    if action == "alert_admin":
        return "medium"
    return "unknown"


def _validate_payload(d: Dict[str, Any]) -> Tuple[bool, str]:
    """
    Validates the LLM response against expected schema.
    
    Args
    ----
        d (dict): the LLM response payload to validate.
    
    Returns
    -------
        tuple[bool, str]: A tuple containing a boolean indicating validation success and a string with an error message if validation fails.
    
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

    # summary or reason are acceptable (we’ll normalize)
    if "summary" not in d and "reason" not in d:
        return False, "Missing required field: summary (or legacy reason)"

    return True, ""


def parse_llm_response(resp: Any) -> Decision:
    """
    Main method for parsing LLM responses.
    
    Args
    ----
        resp (Any): The LLM response to parse. Should be a string representation of a JSON object, but some 
        validation is used to handle dict, Pydantic model instances, or malformed strings.
    
    Returns
    -------
        Decision: The parsed decision object.
    
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
