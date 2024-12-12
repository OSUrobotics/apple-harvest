import rclpy
from rclpy.node import Node
from harvest_interfaces.srv import VoxelGrid
from geometry_msgs.msg import Point
import open3d as o3d
import numpy as np


class VoxelGridService(Node):
    def __init__(self):
        super().__init__('voxel_grid_node')
        self.srv = self.create_service(VoxelGrid, 'voxel_grid', self.voxel_grid_callback)
        self.publisher = self.create_publisher(Point, 'voxel_centers', 10)
        self.get_logger().info('VoxelGrid Service ready.')

    def voxelize_point_cloud(self, file_path, voxel_size):
        # Load the point cloud
        pcd = o3d.io.read_point_cloud(file_path)
        
        # Convert points from mm to m
        pcd_points = np.asarray(pcd.points)  # Convert to NumPy array
        pcd_points /= 1000  # Scale points from mm to m

        # # Coordinate frame change
        # new_points = np.copy(pcd_points)
        # new_points[:, 1] = pcd_points[:, 2]
        # new_points[:, 2] = pcd_points[:, 1]

        pcd.points = o3d.utility.Vector3dVector(pcd_points)  # Convert back to Open3D format

        # Perform voxelization
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

        # Get the voxel data
        voxels = voxel_grid.get_voxels()
        
        # Extract voxel centers and their indices
        voxel_centers, voxel_indices = zip(*[(voxel_grid.get_voxel_center_coordinate(voxel.grid_index), voxel.grid_index) for voxel in voxels])
        return voxel_centers, voxel_indices

    def voxel_grid_callback(self, request, response):
        try:
            # Use the voxel size from the request
            voxel_size = request.voxel_size

            file_path = "/home/marcus/orchard_template_ws/src/apple-harvest/harvest_vision/harvest_vision/point_cloud/pointcloud_1.ply"

            # Voxelize the point cloud
            voxel_centers, _ = self.voxelize_point_cloud(file_path, voxel_size)

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

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

    rclpy.spin(node)
    rclpy.shutdown()


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