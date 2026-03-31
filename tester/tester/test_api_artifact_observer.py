import json
import os
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ApiArtifactObserver(Node):
    """Watches the trigger/cache/response topics and validates a full artifact set."""

    def __init__(self):
        super().__init__("api_artifact_observer")

        self.trigger_topic = os.getenv("AAD_TRIGGER_TOPIC", "/aad/api_bag_trigger")
        self.cache_topic = os.getenv("AAD_CACHE_TOPIC", "/aad/cache_snapshot")
        self.response_topic = os.getenv("AAD_RESPONSE_TOPIC", "/aad/api_response")
        self.stop_after_pass = os.getenv("AAD_OBSERVER_STOP_AFTER_PASS", "true").lower() == "true"

        self.received = defaultdict(set)
        self.latest_payloads = defaultdict(dict)
        self.passed_artifact_ids = set()

        self.create_subscription(String, self.trigger_topic, self._trigger_callback, 10)
        self.create_subscription(String, self.cache_topic, self._cache_callback, 10)
        self.create_subscription(String, self.response_topic, self._response_callback, 10)

        self.get_logger().info(
            "ApiArtifactObserver ready. Watching "
            f"trigger={self.trigger_topic}, cache={self.cache_topic}, response={self.response_topic}"
        )

    def _trigger_callback(self, msg: String):
        self._handle_message("trigger", msg.data)

    def _cache_callback(self, msg: String):
        self._handle_message("cache", msg.data)

    def _response_callback(self, msg: String):
        self._handle_message("response", msg.data)

    def _handle_message(self, kind: str, raw_data: str):
        try:
            payload = json.loads(raw_data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid JSON on {kind} topic: {exc} | payload={raw_data}")
            return

        artifact_id = payload.get("artifact_id")
        if not artifact_id:
            self.get_logger().error(f"Missing artifact_id in {kind} payload: {payload}")
            return

        self.received[artifact_id].add(kind)
        self.latest_payloads[artifact_id][kind] = payload

        self.get_logger().info(
            f"Received {kind} payload for artifact_id={artifact_id} | "
            f"seen={sorted(self.received[artifact_id])}"
        )

        self._evaluate_artifact(artifact_id)

    def _evaluate_artifact(self, artifact_id: str):
        required = {"trigger", "cache", "response"}
        if artifact_id in self.passed_artifact_ids:
            return

        if not required.issubset(self.received[artifact_id]):
            return

        cache_payload = self.latest_payloads[artifact_id]["cache"]
        response_payload = self.latest_payloads[artifact_id]["response"]

        cache_ok = isinstance(cache_payload.get("cache"), list)
        response_ok = "response" in response_payload and "decision" in response_payload

        if cache_ok and response_ok:
            self.passed_artifact_ids.add(artifact_id)
            self.get_logger().info(
                f"[PASS] artifact_id={artifact_id} has trigger/cache/response and required fields present"
            )
            if self.stop_after_pass:
                self.get_logger().info("Observer stopping after first pass.")
                raise SystemExit(0)
        else:
            self.get_logger().warn(
                f"[FAIL] artifact_id={artifact_id} received all topics but payload shape was unexpected | "
                f"cache_ok={cache_ok}, response_ok={response_ok}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ApiArtifactObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
