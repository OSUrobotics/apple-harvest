import rclpy
from rclpy.node import Node
from harvest_interfaces.srv import VoxelGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from ament_index_python.packages import get_package_share_directory
import open3d as o3d
import numpy as np
import os
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import cv2


class VoxelGridService(Node):
    def __init__(self):
        super().__init__('voxel_grid_node')
        # Parameters
        self.declare_parameter("vision_experiment", "NA")
        self.vision_experiment = self.get_parameter("vision_experiment").get_parameter_value().string_value

        # Retrieve point cloud data from vision experiment
        package_name = 'harvest_vision'
        try:
            # Get the package's share directory
            share_directory = get_package_share_directory(package_name)
            
            # Access a file or subdirectory within the share directory
            self.point_cloud_path = os.path.join(share_directory, 'data/', f'prosser_{self.vision_experiment}/', 'pointcloud.ply')
            self.rgb_path = os.path.join(share_directory, 'data/', f'prosser_{self.vision_experiment}/', 'color_raw.png')
            self.depth_path = os.path.join(share_directory, 'data/', f'prosser_{self.vision_experiment}/', 'depth_to_color.png')
        except Exception as e:
            self.get_logger().error(f"Error accessing share directory: {e}")
        
        ### AZURE CAMERA INTRINSICS
        self.azure_color_intrinsic = [902.98, 0, 956.55, 0, 902.77, 547.68, 0, 0, 1]
        self.fx = self.azure_color_intrinsic[0]  # Focal length in x
        self.fy = self.azure_color_intrinsic[4]  # Focal length in y
        self.cx = self.azure_color_intrinsic[2]  # Principal point x
        self.cy = self.azure_color_intrinsic[5]  # Principal point y
        self.target_size = (1920, 1080)

        # Load RGB and depth images
        self.rgb_image = cv2.imread(self.rgb_path)
        self.depth_image = cv2.imread(self.depth_path, cv2.IMREAD_UNCHANGED)
        self.rgb_image = cv2.resize(self.rgb_image, self.target_size, interpolation=cv2.INTER_LINEAR) # Resize RGB image
        self.depth_image = cv2.resize(self.depth_image, self.target_size, interpolation=cv2.INTER_NEAREST) # Resize depth image

        # Publishers
        self.publisher = self.create_publisher(Point, 'voxel_centers', 10)
        
        # Services
        self.srv = self.create_service(VoxelGrid, 'voxel_grid', self.voxel_grid_callback)

        # Coordinate frame change
        ## Microsoft Azure Kinect DK coordinate frame (when looking out from the camera)
        ### x-axis: right 
        ### y-axis: down
        ### z-axis: forward
        self.R = np.array([
            [1, 0, 0],  # X remains X
            [0, 0, 1],  # Z becomes Y
            [0, -1, 0]  # -Y becomes Z
        ])

        # Voxel distance thresholding
        self.y_lower_threshold = 0.1 # meters
        self.y_upper_threshold = 1.5 # meters

        # TF2 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('VoxelGrid Service ready.')

    def transform_points(self, pcd_points, rotation, translation):
        # Scale points from mm to m
        pcd_points = np.asarray(pcd_points) / 1000  

        # Combine rotation and translation into a single transformation matrix
        transformation_matrix = np.eye(4)  # 4x4 identity matrix
        transformation_matrix[:3, :3] = rotation  # Set the rotation
        transformation_matrix[:3, 3] = translation  # Set the translation

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_points)
        pcd.transform(transformation_matrix)  # Apply the transformation

        return np.asarray(pcd.points)

    def voxelize_point_cloud(self, voxel_size, transform):
        rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        # Use the entire depth image without segmentation
        rgb_pc = o3d.geometry.Image(rgb_image)
        depth_pc = o3d.geometry.Image(self.depth_image)

        # Create RGBD image (ensure RGB format for correct visualization)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_pc, depth_pc, convert_rgb_to_intensity=False
        )
        # Point cloud with Azure camera intrinsics
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                width=self.target_size[0], height=self.target_size[1],
                fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy
            )
        )

        # Perform voxelization
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

        # Extract voxel centers
        voxels = voxel_grid.get_voxels()
        # voxel_centers = np.array([voxel_grid.get_voxel_center_coordinate(voxel.grid_index) for voxel in voxels])


        ###################### ATTEMPTING TO GET A BLEND OF COLORS FOR THE VOXELS ########################
        # Retrieve point colors and coordinates
        point_colors = np.asarray(pcd.colors)
        point_coords = np.asarray(pcd.points)
        voxel_centers = []
        voxel_colors = []

        # Create a mapping of voxel indices to points
        voxel_map = {}
        for idx, point in enumerate(point_coords):
            voxel_index = tuple(voxel_grid.get_voxel(point))  # Convert to tuple
            if voxel_index not in voxel_map:
                voxel_map[voxel_index] = []
            voxel_map[voxel_index].append(idx)

        # Calculate the center and blended color for each voxel
        for voxel_index, point_indices in voxel_map.items():
            voxel_center = voxel_grid.get_voxel_center_coordinate(voxel_index)
            voxel_centers.append(voxel_center)

            # Blend colors of points in the voxel
            voxel_color = np.mean(point_colors[point_indices], axis=0)
            voxel_colors.append(voxel_color)

        voxel_centers = np.array(voxel_centers)
        voxel_colors = np.array(voxel_colors)
        # voxel_colors = []

        #################################################################################################


        # Convert the quaternion into a rotation matrix using tf2
        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        rot_matrix = R.from_quat(quaternion).as_matrix()

        # Translation vector
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # Apply the transformation to voxel centers
        transformed_voxel_centers = (rot_matrix @ voxel_centers.T).T + translation

        # Filter points within the specified range
        transformed_voxel_centers = np.array([
            point for point in transformed_voxel_centers
            if self.y_lower_threshold <= point[1] <= self.y_upper_threshold
        ])

        return transformed_voxel_centers, voxel_colors

    def voxel_grid_callback(self, request, response):
        try:
            # Use the voxel size from the request
            voxel_size = request.voxel_size

            # Get the transform from camera_link to the world frame
            target_frame = 'base_link'
            source_frame = 'camera_link'
            
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f"Transform lookup failed: {e}")
                return response

            # Voxelize the point cloud
            voxel_centers, voxel_colors = self.voxelize_point_cloud(voxel_size, transform)

            # Populate the response
            response.voxel_centers = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in voxel_centers]
            response.voxel_colors = [
                ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0) for c in voxel_colors
            ]

            # Publish voxel centers
            for center in voxel_centers:
                point_msg = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
                self.publisher.publish(point_msg)

            self.get_logger().info(f"Voxel grid generated with voxel size {voxel_size}.")
        except Exception as e:
            self.get_logger().error(f"Error during voxel grid generation: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VoxelGridService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()