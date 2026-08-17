import tempfile
import unittest
from pathlib import Path

from harvest_ui.config import DEFAULT_CONFIG, build_process_specs, load_config, normalize_config, save_config, validate_config


class ConfigTests(unittest.TestCase):
    def test_freedrive_keeps_optional_vision_but_disables_vision_stages(self):
        config = normalize_config({"mode": "freedrive", "components": {"vision": True}})
        self.assertTrue(config["components"]["vision"])
        self.assertFalse(config["harvest"]["enable_visual_servo"])
        self.assertFalse(config["harvest"]["enable_apple_prediction"])
        keys = [spec.key for spec in build_process_specs(config)]
        self.assertIn("vision", keys)

    def test_freedrive_can_disable_vision(self):
        config = normalize_config({"mode": "freedrive", "components": {"vision": False}})
        keys = [spec.key for spec in build_process_specs(config)]
        self.assertNotIn("vision", keys)

    def test_rviz_is_configurable(self):
        config = normalize_config(DEFAULT_CONFIG)
        config["arm"]["view_rviz"] = False
        arm = next(spec for spec in build_process_specs(config) if spec.key == "arm")
        self.assertIn("view_rviz:=false", arm.argv)

    def test_real_arm_starts_headless_with_scaled_controller_active(self):
        arm = next(spec for spec in build_process_specs(DEFAULT_CONFIG) if spec.key == "arm")
        self.assertIn("headless_mode:=true", arm.argv)
        self.assertIn("activate_joint_controller:=true", arm.argv)
        self.assertIn("source_frame:=cart_base", arm.argv)

    def test_pose_listener_source_frame_can_be_changed(self):
        config = normalize_config({"arm": {"source_frame": "amiga__base"}})
        arm = next(spec for spec in build_process_specs(config) if spec.key == "arm")
        self.assertIn("source_frame:=amiga__base", arm.argv)

    def test_sweep_pick_controller_parameters_are_forwarded(self):
        config = normalize_config({"harvest": {"pick_pattern": "sweep", "sweep_theta_deg": 75.0}})
        self.assertEqual([], validate_config(config))
        harvest = next(spec for spec in build_process_specs(config) if spec.key == "harvest")
        self.assertIn("pick_pattern:=sweep", harvest.argv)
        self.assertIn("sweep_theta_deg:=75.0", harvest.argv)

    def test_camera_serial_is_passed_to_single_vision_launch(self):
        specs = build_process_specs(DEFAULT_CONFIG)
        vision = next(spec for spec in specs if spec.key == "vision")
        self.assertIn("mast_serial:=040322070611", vision.argv)
        self.assertIn("launch_realsense:=true", vision.argv)
        self.assertNotIn("vision_camera", [spec.key for spec in specs])

    def test_profile_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.yaml"
            save_config(DEFAULT_CONFIG, path)
            loaded = load_config(path)
        self.assertEqual(loaded["arm"]["robot_ip"], "169.254.177.230")
        self.assertEqual(loaded["harvest"]["pick_pattern"], "stiffness-seeking")

    def test_dashboard_uses_all_required_image_and_status_topics(self):
        topics = normalize_config(None)["topics"]
        self.assertEqual(topics["mast_rgb"], "/camera/gripper_camera/color/image_raw")
        self.assertEqual(topics["mast_depth"], "/camera/gripper_camera/aligned_depth_to_color/image_raw")
        self.assertEqual(topics["palm_image"], "/gripper/rgb_palm_camera/image_raw")
        self.assertEqual(topics["apple_prediction_image"], "/apple_annotated")
        self.assertEqual(topics["gripper_imu"], "/microROS/imu1")
        self.assertEqual(topics["can_status"], "/microROS/can_status")

    def test_controls_match_active_firmware_services(self):
        services = normalize_config(None)["services"]
        self.assertEqual(services["gripper_actuate"], "/microROS/actuate_odrive")
        self.assertEqual(services["valve"], "/microROS/toggle_valve")
        self.assertEqual(services["clear_odrive_errors"], "/microROS/clear_odrive_errors")
        self.assertEqual(services["home_gripper"], "/microROS/home_odrive")
        self.assertEqual(services["harvest_freedrive"], "/set_harvest_freedrive")


if __name__ == "__main__":
    unittest.main()
