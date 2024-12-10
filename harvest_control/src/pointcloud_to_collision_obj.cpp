#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include "harvest_interfaces/srv/voxel_grid.hpp"

class CollisionObjectGenerator : public rclcpp::Node
{
public:
    CollisionObjectGenerator() : Node("collision_object_generator")
    {
        service_ = this->create_service<harvest_interfaces::srv::VoxelGrid>(
            "voxel_grid",
            std::bind(&CollisionObjectGenerator::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_service(const std::shared_ptr<harvest_interfaces::srv::VoxelGrid::Request> request,
                        const std::shared_ptr<harvest_interfaces::srv::VoxelGrid::Response> response)
    {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        for (const auto &center : request->voxel_centers)
        {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.id = "voxel_" + std::to_string(collision_objects.size());
            collision_object.header.frame_id = "world";

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {request->voxel_size, request->voxel_size, request->voxel_size};

            geometry_msgs::msg::Pose pose;
            pose.position = center;
            pose.orientation.w = 1.0;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);
            collision_object.operation = collision_object.ADD;

            collision_objects.push_back(collision_object);
        }

        planning_scene_interface.applyCollisionObjects(collision_objects);
        response->success = true;
    }

    rclcpp::Service<harvest_interfaces::srv::VoxelGrid>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionObjectGenerator>());
    rclcpp::shutdown();
    return 0;
}
