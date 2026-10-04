import csv
import tempfile
import unittest
from pathlib import Path

from harvest_ui.pick_ledger import append_pick_record, extract_bag_context, extract_harvest_context


class PickLedgerTests(unittest.TestCase):
    def test_structured_context_is_parsed(self):
        context = extract_harvest_context(
            "[INFO] HARVEST_CONTEXT batch=12 apple=3 batch_dir=/data/batch_12/"
        )
        self.assertEqual(context["batch_number"], 12)
        self.assertEqual(context["apple_number"], 3)
        self.assertEqual(context["batch_directory"], "/data/batch_12/")

    def test_legacy_output_is_parsed(self):
        context = extract_harvest_context(
            "Created new directory: /data/batch_4/\nApproaching apple 2: Coord ..."
        )
        self.assertEqual(context["batch_number"], 4)
        self.assertEqual(context["apple_number"], 2)

    def test_exact_bag_timestamp_and_path_are_parsed(self):
        context = extract_bag_context(
            "BAG_CONTEXT timestamp=20261001_143015 "
            "bag_path=/data/batch_4/apple_2/final_approach_20261001_143015.db3"
        )
        self.assertEqual(context["timestamp"], "20261001_143015")
        self.assertTrue(context["bag_path"].endswith("20261001_143015.db3"))

    def test_legacy_recorder_log_is_parsed(self):
        context = extract_bag_context(
            "Started recording topics: ['/joint_states'] to "
            "/data/batch_4/apple_2/final_approach_20261001_143015.db3"
        )
        self.assertEqual(context["timestamp"], "20261001_143015")

    def test_blank_pick_number_is_written_as_discarded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pick_log.csv"
            append_pick_record(
                path,
                {
                    "timestamp": "2026-09-29T12:00:00-07:00",
                    "batch_number": 4,
                    "apple_number": 2,
                    "pick_number": "",
                    "status": "discarded",
                },
            )
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["pick_number"], "")
        self.assertEqual(rows[0]["status"], "discarded")

    def test_existing_csv_header_is_upgraded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pick_log.csv"
            path.write_text(
                "timestamp,batch_number,apple_number,pick_number,status\n"
                "old,1,2,7,bagged\n",
                encoding="utf-8",
            )
            append_pick_record(path, {"timestamp": "new", "batch_number": 2})
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
                fields = tuple(rows[0].keys())
        self.assertIn("bag_path", fields)
        self.assertIn("recorded_at", fields)
        self.assertEqual(len(rows), 2)


if __name__ == "__main__":
    unittest.main()
