#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

void addTreeToScene()
{
    // Create a PlanningSceneInterface instance
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create a CollisionObject for the tree structure
    moveit_msgs::msg::CollisionObject tree_object;
    tree_object.id = "v_trellis_tree";  // Unique identifier
    tree_object.header.frame_id = "world";  // Root frame

    // Define the leader branch (cylinder)
    shape_msgs::msg::SolidPrimitive leader_branch;
    leader_branch.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    leader_branch.dimensions = {4.0, 0.08};  // Height, radius


    geometry_msgs::msg::Pose leader_pose;
    leader_pose.position.x = 0.0;
    leader_pose.position.y = 0.0;
    leader_pose.position.z = 2.0;  // Centered vertically
    leader_pose.orientation.w = 1;

    // Add the leader branch to the tree object
    tree_object.primitives.push_back(leader_branch);
    tree_object.primitive_poses.push_back(leader_pose);

    // Define and add horizontal branches
    double branch_spacing = 1.0;
    for (int i = 1; i <= 4; ++i)
    {
        shape_msgs::msg::SolidPrimitive horizontal_branch;
        horizontal_branch.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        horizontal_branch.dimensions = {4.0, 0.04};  // Height, radius

        geometry_msgs::msg::Pose branch_pose;
        branch_pose.position.x = 0.0;
        branch_pose.position.y = 0.0;
        branch_pose.position.z = i * branch_spacing;  // Spaced vertically
        branch_pose.orientation.x = 0.7071;  // 90-degree rotation around X-axis
        branch_pose.orientation.w = 0.7071;

        tree_object.primitives.push_back(horizontal_branch);
        tree_object.primitive_poses.push_back(branch_pose);
    }

    // Set the pose of the canopy relative to the base of the robot
    tree_object.pose.position.x = 2.0;
    tree_object.pose.position.y = 0.0;
    tree_object.pose.position.z = 0.0;

    tf2::Quaternion canopy_orientation;
    canopy_orientation.setRPY(0, -0.321751448, 0);
    tree_object.pose.orientation.x = canopy_orientation.x();
    tree_object.pose.orientation.y = canopy_orientation.y();
    tree_object.pose.orientation.z = canopy_orientation.z();
    tree_object.pose.orientation.w = canopy_orientation.w();

    // Set the operation to ADD
    tree_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Apply the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(tree_object);

    RCLCPP_INFO(rclcpp::get_logger("add_tree_to_scene"), "Tree added to the scene.");
}

int main(int argc, char** argv)
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_tree_to_scene");

    // Allow some time for MoveIt to initialize
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Add the tree structure to the planning scene
    addTreeToScene();

    // Spin to keep the node alive for interaction
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}