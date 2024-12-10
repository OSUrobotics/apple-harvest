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
        self.get_logger().info('VoxelGrid Service ready.')

    def voxelize_point_cloud(self, file_path, voxel_size):
        # Load the point cloud
        pcd = o3d.io.read_point_cloud(file_path)

        # Perform voxelization
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

        # Extract voxel centers
        voxels = voxel_grid.get_voxels()
        voxel_centers = [voxel.grid_index for voxel in voxels]
        voxel_centers = np.array(voxel_centers) * voxel_size  # Convert grid indices to coordinates

        return voxel_centers

    def voxel_grid_callback(self, request, response):
        try:
            # Use the voxel size from the request
            voxel_size = request.voxel_size

            # Voxelize the point cloud
            voxel_centers = self.voxelize_point_cloud("/home/marcus/orchard_template_ws/src/apple-harvest/harvest_vision/harvest_vision/point_cloud/pointcloud_1.ply", voxel_size)

            # Populate the response
            response.voxel_centers = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in voxel_centers]
            self.get_logger().info(f"Voxel grid generated with voxel size {voxel_size}.")
        except Exception as e:
            self.get_logger().error(f"Error during voxel grid generation: {e}")


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
