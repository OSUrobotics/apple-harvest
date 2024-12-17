import rclpy
from rclpy.node import Node
from harvest_interfaces.srv import VoxelGrid
from geometry_msgs.msg import Point
import open3d as o3d
import numpy as np
from tf2_ros import Buffer, TransformListener


class VoxelGridService(Node):
    def __init__(self):
        super().__init__('voxel_grid_node')
        self.srv = self.create_service(VoxelGrid, 'voxel_grid', self.voxel_grid_callback)
        self.publisher = self.create_publisher(Point, 'voxel_centers', 10)

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

        # Convert points to homogeneous coordinates (Nx4)
        homogeneous_points = np.hstack((pcd_points, np.ones((pcd_points.shape[0], 1))))

        # Apply the transformation
        transformed_points = homogeneous_points @ transformation_matrix.T

        # Return only the 3D points
        return transformed_points[:, :3]

    def voxelize_point_cloud(self, file_path, voxel_size, transform):
        # Load the point cloud
        pcd = o3d.io.read_point_cloud(file_path)

        # # Translation vector (new origin relative to old origin)
        # translation = np.array([0, 0.25, 0.45])
        translation = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]

        # Transform the points
        transformed_points = self.transform_points(pcd.points, self.R, translation)

        pcd.points = o3d.utility.Vector3dVector(transformed_points)  # Convert back to Open3D format

        # Perform voxelization
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

        # Get the voxel data
        voxels = voxel_grid.get_voxels()
        
        # Extract voxel centers and their indices
        voxel_centers, voxel_indices = zip(*[(voxel_grid.get_voxel_center_coordinate(voxel.grid_index), voxel.grid_index) for voxel in voxels])

        # Filter points within the specified range
        points = np.array(voxel_centers)
        voxel_centers = np.array([
            point for point in points
            if self.y_lower_threshold <= point[1] <= self.y_upper_threshold
        ])

        return voxel_centers, voxel_indices

    def voxel_grid_callback(self, request, response):
        try:
            # Use the voxel size from the request
            voxel_size = request.voxel_size

            file_path = "/home/marcus/orchard_template_ws/src/apple-harvest/harvest_vision/point_cloud/pointcloud_1.ply"

            # Get the transform from camera_link to the world frame
            target_frame = 'base_link'
            source_frame = 'camera_link'
            
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f"Transform lookup failed: {e}")
                return response

            # Voxelize the point cloud
            voxel_centers, _ = self.voxelize_point_cloud(file_path, voxel_size, transform)

            # Populate the response
            response.voxel_centers = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in voxel_centers]

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

    # rclpy.spin(node)
    # rclpy.shutdown()


if __name__ == '__main__':
    main()

    # file_path = "/home/marcus/orchard_template_ws/src/apple-harvest/harvest_vision/harvest_vision/point_cloud/pointcloud_1.ply"
    # voxel_size = 0.01

    # # Load the point cloud
    # pcd = o3d.io.read_point_cloud(file_path)
    
    # # Convert points from mm to m
    # pcd_points = np.asarray(pcd.points)  # Convert to NumPy array
    # pcd_points /= 1000  # Scale points from mm to m
    # pcd.points = o3d.utility.Vector3dVector(pcd_points)  # Convert back to Open3D format

    # # Perform voxelization
    # voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

    # # Get the voxel data
    # voxels = voxel_grid.get_voxels()
    
    # # Extract voxel centers and their indices
    # voxel_centers, voxel_indices = zip(*[(voxel_grid.get_voxel_center_coordinate(voxel.grid_index), voxel.grid_index) for voxel in voxels])

    # print(voxel_centers[0])
    # print(len(voxel_centers))