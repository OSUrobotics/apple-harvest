// generate_tree_collision_objects.cpp
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shapes.h>  // For shape definitions
#include <geometry_msgs/msg/pose.hpp>

void generateTreeCollisionObjects(std::vector<moveit_msgs::msg::CollisionObject> &collision_objects)
{
    moveit_msgs::msg::CollisionObject leader_branch;
    leader_branch.id = "leader_branch";
    leader_branch.header.frame_id = "world";  // Replace with appropriate frame

    shape_msgs::msg::SolidPrimitive leader_primitive;
    leader_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    leader_primitive.dimensions = {4.0, 0.08}; // Length, Radius

    geometry_msgs::msg::Pose leader_pose;
    leader_pose.orientation.w = 1.0;
    leader_pose.position.z = 2.0;  // Leader branch length / 2

    leader_branch.primitives.push_back(leader_primitive);
    leader_branch.primitive_poses.push_back(leader_pose);
    leader_branch.operation = leader_branch.ADD;

    // Add leader branch to the collision objects
    collision_objects.push_back(leader_branch);

    // Horizontal branches
    double horizontal_branch_spacing = 1.0;
    double branch_diameter = 0.04;
    double branch_length = 4.0;

    for (int i = 1; i <= 4; ++i)
    {
        moveit_msgs::msg::CollisionObject horizontal_branch;
        horizontal_branch.id = "horizontal_branch_" + std::to_string(i);
        horizontal_branch.header.frame_id = "world";  // Replace with appropriate frame

        shape_msgs::msg::SolidPrimitive branch_primitive;
        branch_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        branch_primitive.dimensions = {branch_length, branch_diameter};

        geometry_msgs::msg::Pose branch_pose;
        branch_pose.orientation.x = 1.0;  // Rotate 90 degrees around x-axis
        branch_pose.position.z = horizontal_branch_spacing * i;

        horizontal_branch.primitives.push_back(branch_primitive);
        horizontal_branch.primitive_poses.push_back(branch_pose);
        horizontal_branch.operation = horizontal_branch.ADD;

        // Add each horizontal branch to the collision objects
        collision_objects.push_back(horizontal_branch);
    }
}
