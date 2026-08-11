# A simple set of functions to determine what distances/error metrics to use for visual servoing

import numpy as np


class CameraDistances():
    def __init__(self):
        # See set methods 
        self.name = "None"

        self.in_hand_camera_angle = np.pi * 2
        self.camera_height = 1080
        self.camera_width = 1920

        self.camera_distance = 0.76   # Figure this out for best valeus

        self.apple_width = 0.075  # In meters
        self.apple_spacing = 0.0508  # In meters
        self.row_spacing = 0.46      # In meters (18 inches)

        self.row_width = 4.5   # In meters
        self.amiga_width = 1   # In meters
        self.arm_length = 0.8  # In meters
        self.feasible_distance_range = (0.16, 1.0)

    def set_azure(self):
        self.name = "Azure"

        # D45s and Azure are roughly 90 horizontally, and 60 vertically
        self.camera_angle = np.pi * 2
        # Azure is much higher (3840x2160), realsense is 1280x720
        self.camera_height = 2160
        self.camera_width = 3840

    def set_intel(self):
        self.name = "Intel"
        # D45s and Azure are roughly 90 horizontally, and 60 vertically
        self.camera_angle = np.pi * 2

        # Azure is much higher (3840x2160), realsense is 1280x720
        self.camera_height = 720
        self.camera_width = 1280

    def set_generic_rgb(self):
        self.name = "Generic"
        # Assume 90 degrees horizontal
        self.camera_angle = np.pi * 2

        # Azure is much higher (3840x2160), realsense is 1280x720
        self.camera_height = 720
        self.camera_width = 1080

    def set_base_location(self):
        self.camera_distance = 0.76   # Figure this out for best valeus

    def set_in_hand_location(self):
        self.camera_distance = 0.76   # Figure this out for best valeus

    def _aspect_ratio(self):
        return self.camera_height / self.camera_width
    
    def _camera_angle_h(self):
        return self._aspect_ratio() * self.camera_angle
    
    def _camera_project_pixels(self, width_in_real_world: float):
        angle_subtended = np.arctan2(width_in_real_world, self.camera_distance)
        pix_subtended_w = (angle_subtended / self.camera_angle) * self.camera_width
        pix_subtended_h = (angle_subtended / self._camera_angle_h()) * self.camera_height
        return pix_subtended_w, pix_subtended_h
    
    def _estimated_size_of_apple_in_pix(self):
        n_pixs_apple_w, n_pixs_apple_h = self._camera_project_pixels(self.apple_width) 
        return n_pixs_apple_w, n_pixs_apple_h

    def _estimated_number_of_apples_visible_camera(self):
        n_pixs_apple_w, n_pixs_apple_h = self._estimated_size_of_apple_in_pix() 
        n_pixs_apple_sp_w, _ = self._camera_project_pixels(self.apple_spacing)
        _, n_pixs_apple_sp_h = self._camera_project_pixels(self.row_spacing)
        n_apples_x = np.floor(float(self.camera_width) / (n_pixs_apple_w + n_pixs_apple_sp_w))
        n_apples_y = np.floor(float(self.camera_height)) / (n_pixs_apple_h + n_pixs_apple_sp_h)
        return n_apples_x * n_apples_y

    def find_apple_overlap(self, n_apples: int):
        """Find the range of distances where at least n_apples are visible"""

        d_min = self.feasible_distance_range[1]
        d_max = self.feasible_distance_range[0]

        save_distance = self.camera_distance    
        for d in np.linspace(self.feasible_distance_range[0], self.feasible_distance_range[1], 40):
            self.camera_distance = d
            n_total = self._estimated_number_of_apples_visible_camera()
            if n_total > n_apples:
                if d < d_min:
                    d_min = d
                if d > d_max:
                    d_max = d
        self.camera_distance = save_distance
        return d_min, d_max
    

def main():
    cam_intel = CameraDistances()
    cam_intel.set_intel()
    cam_intel.set_base_location()

    cam_azure = CameraDistances()
    cam_azure.set_azure()
    cam_azure.set_base_location()

    cam_in_hand = CameraDistances()
    cam_in_hand.set_generic_rgb()
    cam_in_hand.set_in_hand_location()

    for cam in (cam_intel, cam_azure, cam_in_hand):
        save_distance = cam.camera_distance    
        for d in np.linspace(cam.feasible_distance_range[0], cam.feasible_distance_range[1], 10):
            cam.camera_distance = d
            print(f"{cam.name}: Distance {d} n apples {cam._estimated_number_of_apples_visible_camera()}")

        print(f"Distance range {cam.find_apple_overlap(6)}")
        cam.camera_distance = save_distance


if __name__ == '__main__':
    main()
