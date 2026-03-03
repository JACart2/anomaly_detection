from anomaly_detection.response_handler import parse_llm_response

def main():
    valid_v1 = {"anomaly": True, "response": "alert_admin", "reason": "Repeated sensor dropout warnings."}
    valid_v2 = {"anomaly": True, "action": "stop_cart", "severity": "high", "summary": "Obstacle detection failure."}
    invalid_1 = {"response": "stop_cart", "reason": "Missing anomaly field"}  # missing anomaly
    invalid_2 = "not json at all"

    for label, sample in [
        ("valid_v1", valid_v1),
        ("valid_v2", valid_v2),
        ("invalid_1", invalid_1),
        ("invalid_2", invalid_2),
    ]:
        decision = parse_llm_response(sample)
        print(f"{label}: {decision}")

if __name__ == "__main__":
    main()