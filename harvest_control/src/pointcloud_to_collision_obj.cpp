#include <rclcpp/rclcpp.hpp>
#include <harvest_interfaces/srv/voxel_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.h>

class VoxelCollisionNode : public rclcpp::Node {
public:
    VoxelCollisionNode() : Node("voxel_collision_node") {
        client_ = this->create_client<harvest_interfaces::srv::VoxelGrid>("voxel_grid");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VoxelCollisionNode::callVoxelGridService, this)
        );
    }

private:
    void callVoxelGridService() {
        auto request = std::make_shared<harvest_interfaces::srv::VoxelGrid::Request>();
        request->voxel_size = 0.1; 

        using ServiceResponseFuture = 
            rclcpp::Client<harvest_interfaces::srv::VoxelGrid>::SharedFuture;
        auto result_callback = [this](ServiceResponseFuture future) {
            auto response = future.get();
            this->addCollisionObjects(response->voxel_centers);
        };

        client_->async_send_request(request, result_callback);
    }

    void addCollisionObjects(const std::vector<geometry_msgs::msg::Point> &voxel_centers) {
        moveit::planning_interface::PlanningSceneInterface psi;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        for (size_t i = 0; i < voxel_centers.size(); ++i) {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.id = "voxel_" + std::to_string(i);
            collision_object.header.frame_id = "world";

            // Define the shape and size
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {0.1, 0.1, 0.1};  // Voxel size

            // Define the pose
            geometry_msgs::msg::Pose box_pose;
            box_pose.position = voxel_centers[i];
            box_pose.orientation.w = 1.0;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            collision_objects.push_back(collision_object);
        }

        psi.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(this->get_logger(), "Added %zu collision objects to planning scene", voxel_centers.size());
    }

    rclcpp::Client<harvest_interfaces::srv::VoxelGrid>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoxelCollisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
