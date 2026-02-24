"""OpenAI call point for anomaly detection. Requires cached data from manager.py.

Author: John Rosario Cruz
Version: 2/18/2026
"""
from openai import OpenAI
import os
from pydantic import BaseModel
from typing import Literal


## reason is included mostly for debugging. May give hints into the cause of false positives/negatives.
class AnomalyResponse(BaseModel):
    anomaly: bool
    response: Literal["stop_cart", "alert_admin", "none"]
    reason: str

def call_openai(context: str) -> AnomalyResponse:
    """Calls ChatGPT to make a determination about whether an anomaly is present given cart context.
    
    Args:
        context (str): String representation of cache log in manager.py. Contains ROS2 messages post-processing (when applicable).
    
    Returns:
        AnomalyResponse: A response determining if a an anomaly was detected, and the appropriate response.
            Response is currently defaulting to three messages, may be modified in the future as of 2/17.
            >>> {
                "anomaly": bool,
                "response" : "stop_cart" | "alert_admin" | "none"
                }
    
    """
    ## REQUIRES dev acc info
    client = OpenAI(
        api_key=os.environ.get("OA_SECRET")
    )

    SYSTEM_PROMPT = f'''
    You are an anomaly classifier for an autonomous ROS2 golf cart system.

    You will receive a single string called CONTEXT containing system logs, passenger data, sensor summaries, or error messages.
    These logs are coming from anomaly_msg.msg.AnomalyLog, but this doesn't guarantee an anomaly. These are simply consolidated logs for you to make a determination for an anomaly.

    Your task:
    Determine whether there is an anomaly that requires intervention.

    Rules:
    - If there is NO anomaly, set:
        "anomaly": false
        "response": "none"
        "reason": "your reason"

    - If there is a safety-critical issue (collision risk, incapacitated passenger, control failure, braking failure, obstacle detection failure), return:
        "anomaly": true
        "response": "stop_cart"
        "reason": "your reason"

    - If there is a non-critical but abnormal condition (sensor degradation, unusual passenger behavior, repeated warnings, system errors without immediate danger), return:
        "anomaly": true
        "response": "alert_admin"
        "reason": "your reason"

    '''

    resp = client.responses.parse(
        model="gpt-5-mini-2025-08-07",
        input=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": f"CONTEXT:\n{context}"},
        ],
        text_format=AnomalyResponse,
    )

    
    # Depending on SDK version, parsed output is available in one of these places.
    if getattr(resp, "output_parsed", None) is not None:
        return resp.output_parsed

    # Fallback: first message content parsed payload
    return resp.output[0].content[0].parsed

if __name__ == '__main__':
    ## just so no accidental calls, change to true if you are serious about testing
    call = False
    payload = "Dummy payload"

    with open("OA_SECRET.txt", "r") as f:
        os.environ["OA_SECRET"] = f.read().strip()

    if call:
        response = call_openai(payload)
        print(response)
