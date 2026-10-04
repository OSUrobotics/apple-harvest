import ast
import unittest
from pathlib import Path
from threading import Condition
from types import SimpleNamespace

# Load only the pure selection state so this test runs without a sourced ROS environment.
script = Path(__file__).resolve().parents[2] / "harvest" / "scripts" / "start_harvest_abort.py"
tree = ast.parse(script.read_text(encoding="utf-8"))
selector = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "AppleSelection")
namespace = {"Condition": Condition}
exec(compile(ast.Module(body=[selector], type_ignores=[]), str(script), "exec"), namespace)
AppleSelection = namespace["AppleSelection"]


class HarvestAborted(Exception):
    pass


flow_node = next(
    node for node in tree.body
    if isinstance(node, ast.ClassDef) and node.name == "StartHarvestAbort"
)
flow_method = next(
    node for node in flow_node.body
    if isinstance(node, ast.FunctionDef) and node.name == "_run_full_batch_flow"
)
flow_namespace = {
    "rclpy": SimpleNamespace(ok=lambda: True),
    "HarvestAborted": HarvestAborted,
    "input": lambda prompt: "",
    "print": lambda *args, **kwargs: None,
}
exec(compile(ast.Module(body=[flow_method], type_ignores=[]), str(script), "exec"), flow_namespace)
run_full_batch_flow = flow_namespace["_run_full_batch_flow"]


class AppleSelectionTests(unittest.TestCase):
    def test_targets_can_be_repeated_and_invalid_ids_are_rejected(self):
        selection = AppleSelection()
        self.assertFalse(selection.request(1)[0])
        selection.set_available(4)
        self.assertEqual(selection.selectable_ids(), [0, 1, 2, 3])
        self.assertFalse(selection.request(8)[0])
        self.assertTrue(selection.request(3)[0])
        self.assertEqual(selection.take_next(0), 3)
        self.assertEqual(selection.selectable_ids(), [0, 1, 2, 3])
        self.assertTrue(selection.request(3)[0])
        self.assertEqual(selection.take_next(0), 3)
        self.assertTrue(selection.request(0)[0])
        self.assertEqual(selection.take_next(0), 0)
        self.assertEqual(selection.selectable_ids(), [0, 1, 2, 3])
        self.assertFalse(selection.done)

    def test_only_the_latest_pending_target_is_used(self):
        selection = AppleSelection()
        selection.set_available(3)
        self.assertTrue(selection.request(1)[0])
        self.assertTrue(selection.request(2)[0])
        self.assertEqual(selection.take_next(0), 2)
        self.assertIsNone(selection.take_next(0))

    def test_abort_clears_pending_choice_but_allows_reentry(self):
        selection = AppleSelection()
        selection.set_available(2)
        selection.request(1)
        selection.clear_pending()
        self.assertIsNone(selection.take_next(0))
        self.assertTrue(selection.request(1)[0])
        self.assertEqual(selection.take_next(0), 1)

    def test_finish_discards_queued_target(self):
        selection = AppleSelection()
        selection.set_available(2)
        self.assertTrue(selection.request(1)[0])
        self.assertTrue(selection.finish()[0])
        self.assertIsNone(selection.take_next(0))
        self.assertTrue(selection.done)

    def test_empty_prediction_finishes(self):
        selection = AppleSelection()
        selection.set_available(0)
        self.assertTrue(selection.done)
        self.assertFalse(selection.request(0)[0])

    def test_autonomous_abort_returns_to_same_id_selection(self):
        pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0))
        choices = [0, 0, None]
        approaches = []
        final_approach_count = 0

        def run_stage(topics, prefix, **kwargs):
            nonlocal final_approach_count
            if prefix.endswith("final_approach_and_pick"):
                final_approach_count += 1
                if final_approach_count == 1:
                    raise HarvestAborted("first attempt")
            if kwargs.get("action_fn"):
                kwargs["action_fn"]()

        node = SimpleNamespace(
            enable_apple_prediction=True,
            enable_recording=False,
            enable_visual_servo=False,
            enable_pressure_servo=True,
            enable_picking=False,
            manual_apple_selection=True,
            abort_recovery_mode="freedrive",
            use_optimal_trajectory=False,
            batch_dir="/tmp/batch_1/",
            batch_number=1,
            prediction_topics=[],
            prediction_file_name_prefix="prediction",
            pressure_servo_and_pick_controller_topics=[],
            final_approach_and_pick_file_name_prefix="final_approach_and_pick",
            PICK_PATTERN="stiffness-seeking",
            apple_selection=AppleSelection(),
            get_logger=lambda: SimpleNamespace(
                info=lambda message: None,
                warn=lambda message: None,
                error=lambda message: None,
            ),
            go_to_scan_position=lambda: None,
            start_apple_prediction=lambda: SimpleNamespace(poses=[pose]),
            run_stage=run_stage,
            switch_controller=lambda **kwargs: None,
            _wait_for_selected_apple=lambda: choices.pop(0),
            _publish_available_apple_ids=lambda: None,
            trigger_move_arm_to_pose=lambda target: approaches.append(target),
            _raise_if_aborted=lambda stage: None,
            configure_servo=lambda frame: None,
            grasp_controller_action=lambda: None,
            release_controller=lambda: None,
        )
        run_full_batch_flow(node)
        self.assertEqual(final_approach_count, 2)
        self.assertEqual(approaches, [pose, pose])
        self.assertEqual(node.apple_selection.selectable_ids(), [0])

    def test_prediction_abort_retries_prediction_without_freedrive_pick_loop(self):
        pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0))
        prediction_calls = 0
        scan_calls = []

        def run_stage(topics, prefix, **kwargs):
            nonlocal prediction_calls
            prediction_calls += 1
            if prediction_calls == 1:
                raise HarvestAborted("prediction")
            kwargs["action_fn"]()

        node = SimpleNamespace(
            enable_apple_prediction=True,
            enable_recording=False,
            manual_apple_selection=True,
            abort_recovery_mode="freedrive",
            batch_dir="/tmp/batch_1/",
            batch_number=1,
            prediction_topics=[],
            prediction_file_name_prefix="prediction",
            apple_selection=AppleSelection(),
            get_logger=lambda: SimpleNamespace(
                info=lambda message: None,
                warn=lambda message: None,
                error=lambda message: None,
            ),
            go_to_scan_position=lambda: scan_calls.append(True),
            start_apple_prediction=lambda: SimpleNamespace(poses=[pose]),
            run_stage=run_stage,
            switch_controller=lambda **kwargs: None,
            _wait_for_selected_apple=lambda: None,
            _publish_available_apple_ids=lambda: None,
        )
        run_full_batch_flow(node)
        self.assertEqual(prediction_calls, 2)
        self.assertEqual(len(scan_calls), 2)
        self.assertEqual(node.apple_selection.selectable_ids(), [0])


if __name__ == "__main__":
    unittest.main()
