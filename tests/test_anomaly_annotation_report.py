"""Tests for the standalone anomaly bag report and annotation safety."""

import json
import tempfile
import unittest
from collections import Counter
from pathlib import Path

from scripts.anomaly_annotation_report import json_for_script, write_html
from scripts.extract_anomaly_bag import prepare_output_directory


def sample_record() -> dict:
    """Return one representative extracted AnomalyMsg record."""
    return {
        "index": 184,
        "topic": "/ai_anomaly_logging",
        "recorded_timestamp_ns": 1785178501234567890,
        "timestamp": "2026-07-27T18:55:01.234567890Z",
        "node_name": "zed_camera",
        "importance_name": "ERROR",
        "type_name": "IMAGE",
        "message": "Driver is no longer visible",
        "data_type": "",
        "data_base64": "",
        "data_length": 0,
        "image": {"file": "", "raw_file": "", "error": ""},
    }


class AnnotationReportTests(unittest.TestCase):
    """Verify generated report features and sidecar data safety."""

    def test_generated_report_contains_annotation_identity_and_workflow(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            write_html(
                output,
                output / "driver_missing_001.mcap",
                [sample_record()],
                Counter({2: 1}),
                Counter({1: 1}),
                0,
            )

            report = (output / "index.html").read_text(encoding="utf-8")
            self.assertIn('data-message-index="184"', report)
            self.assertIn(
                'data-recorded-timestamp-ns="1785178501234567890"', report
            )
            self.assertIn('data-topic="/ai_anomaly_logging"', report)
            self.assertIn('data-node-name="zed_camera"', report)
            self.assertIn('id="export-annotations"', report)
            self.assertIn('id="import-annotations"', report)
            self.assertIn('class="small set-range-start"', report)
            self.assertIn('class="small set-range-end"', report)
            self.assertIn("driver_missing_001.annotations.json", report)
            self.assertIn("window.localStorage.setItem", report)
            self.assertIn("parseLosslessJson", report)

    def test_script_json_encoding_cannot_close_script_element(self):
        encoded = json_for_script("bag</script><script>alert(1)</script>.mcap")
        self.assertNotIn("</script>", encoded)
        self.assertIn("\\u003c/script>", encoded)

    def test_regeneration_refuses_to_delete_annotation_sidecar(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "report"
            report.mkdir()
            sidecar = report / "recording.annotations.json"
            sidecar.write_text("{}", encoding="utf-8")

            with self.assertRaisesRegex(SystemExit, "Refusing to regenerate"):
                prepare_output_directory(report)

            self.assertTrue(sidecar.is_file())

    def test_annotation_schema_is_version_one(self):
        schema_path = (
            Path(__file__).parents[1] / "docs" / "anomaly_annotations.schema.json"
        )
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        self.assertEqual(schema["properties"]["schema_version"]["const"], 1)
        self.assertEqual(
            schema["required"], ["schema_version", "bag", "ranges", "annotations"]
        )
        self.assertIn(
            "recorded_timestamp_ns",
            schema["$defs"]["messageAnnotation"]["required"],
        )


if __name__ == "__main__":
    unittest.main()
