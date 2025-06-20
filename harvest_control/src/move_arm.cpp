#include <rclcpp/rclcpp.hpp>
#include "harvest_interfaces/srv/move_to_pose.hpp"
#include "harvest_interfaces/srv/send_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_srvs/srv/empty.hpp"
#include <std_srvs/srv/trigger.hpp>

// Read data packages
#include <iostream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <cmath> // For M_PI

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class MoveArmNode : public rclcpp::Node
{
public:
    MoveArmNode();

private:
    rclcpp::Service<harvest_interfaces::srv::SendTrajectory>::SharedPtr arm_trajectory_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_to_home_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_to_config_service_;
    rclcpp::Service<harvest_interfaces::srv::MoveToPose>::SharedPtr arm_to_pose_service_;

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr gripper_tip_subscription_;
    geometry_msgs::msg::TransformStamped current_gripper_pose_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    moveit::planning_interface::MoveGroupInterface move_group_;
    std::vector<double> home_joint_positions = {
        M_PI / 2,
        -M_PI / 2,
        2 * M_PI / 3,
        5 * M_PI / 6,
        -M_PI / 2,
        0};
    // std::vector<double> home_joint_positions = {
    //     M_PI / 2,
    //     -2.36,
    //     2 * M_PI / 3,
    //     3.40,
    //     -M_PI / 2,
    //     0};

    std::vector<double> scan_joint_positions = {
        M_PI / 2,
        -M_PI / 2,
        2 * M_PI / 3,
        5 * M_PI / 6,
        -M_PI / 2,
        0};

    void execute_trajectory(const std::shared_ptr<harvest_interfaces::srv::SendTrajectory::Request> request,
                            std::shared_ptr<harvest_interfaces::srv::SendTrajectory::Response> response);
    void move_to_home(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void move_to_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void move_to_pose(const std::shared_ptr<harvest_interfaces::srv::MoveToPose::Request> request,
                      const std::shared_ptr<harvest_interfaces::srv::MoveToPose::Response> response);
    void gripper_tip_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
};

MoveArmNode::MoveArmNode()
    : Node("move_arm_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ur_manipulator")
{
    // Initialize the subscription to /gripper_tip
    gripper_tip_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/gripper_tip", 10, std::bind(&MoveArmNode::gripper_tip_callback, this, std::placeholders::_1));

    // Set up services
    arm_trajectory_service_ = this->create_service<harvest_interfaces::srv::SendTrajectory>(
        "send_arm_trajectory", std::bind(&MoveArmNode::execute_trajectory, this, _1, _2));

    arm_to_home_service_ = this->create_service<std_srvs::srv::Trigger>(
        "move_arm_to_home", std::bind(&MoveArmNode::move_to_home, this, _1, _2));

    arm_to_config_service_ = this->create_service<std_srvs::srv::Trigger>(
        "move_arm_to_config", std::bind(&MoveArmNode::move_to_config, this, _1, _2));

    arm_to_pose_service_ = this->create_service<harvest_interfaces::srv::MoveToPose>(
        "move_arm_to_pose", std::bind(&MoveArmNode::move_to_pose, this, _1, _2));

    // Set up parameters
    double max_accel = this->get_parameter("max_accel").as_double();
    double max_vel = this->get_parameter("max_vel").as_double();

    // Set velocity and acceleration limits
    // NEED TO RESET TO 0.05 FOR HARDWARE!!!
    this->move_group_.setMaxAccelerationScalingFactor(max_accel);
    this->move_group_.setMaxVelocityScalingFactor(max_vel);

    RCLCPP_INFO(this->get_logger(), "Move arm server ready");
}

void MoveArmNode::gripper_tip_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    current_gripper_pose_ = *msg;

//     RCLCPP_INFO(this->get_logger(), "Gripper tip pose received: [%f, %f, %f]",
//                 current_gripper_pose_.pose.position.x,
//                 current_gripper_pose_.pose.position.y,
//                 current_gripper_pose_.pose.position.z);
}

void MoveArmNode::move_to_home(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Suppress unused parameter warning

    // Set the home configuration as the target for the MoveGroup
    move_group_.setJointValueTarget(home_joint_positions);

    // Plan and execute to move to the home position
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_.plan(plan));

    if (success)
    {
        move_group_.execute(plan);
        RCLCPP_INFO(this->get_logger(), "Moved to home configuration.");
        response->success = true;
        response->message = "Successfully moved to home configuration.";
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to home configuration.");
        response->success = false;
        response->message = "Failed to move to home configuration.";
    }
}

void MoveArmNode::move_to_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Suppress unused parameter warning

    std::vector<double> target_config = {
        1.5,
        -2.775,
        1.72,
        4.55,
        -1.58,
        0};

    // Set the target configuration as the target for the MoveGroup
    move_group_.setJointValueTarget(target_config);

    // Plan and execute to move to the target position
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_.plan(plan));

    if (success)
    {
        move_group_.execute(plan);
        RCLCPP_INFO(this->get_logger(), "Moved to target configuration.");
        response->success = true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to target configuration.");
        response->success = false;
    }
}

void MoveArmNode::move_to_pose(const std::shared_ptr<harvest_interfaces::srv::MoveToPose::Request> request,
                               const std::shared_ptr<harvest_interfaces::srv::MoveToPose::Response> response)
{
    // Set the current state as the start
    this->move_group_.setStartStateToCurrentState();

    // Make PoseStamped message
    tf2::Quaternion orientation;
    orientation.setRPY(3.14 / 2, 3.14, 3.14);  // Set desired orientation
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.pose.orientation = tf2::toMsg(orientation);
    msg.pose.position.x = request->position.x;
    msg.pose.position.y = request->position.y;
    msg.pose.position.z = request->position.z;

    // Set pose and joint tolerances
    this->move_group_.setPoseTarget(msg, "gripper_link");
    // this->move_group_.setGoalOrientationTolerance(0.35);
    this->move_group_.setGoalOrientationTolerance(1.05);
    // this->move_group_.setGoalJointTolerance(0.001); // Minimize joint changes

    // Use an optimization-aware planner
    // this->move_group_.setPlannerId("RRTstarkConfigDefault");
    this->move_group_.setPlannerId("RRTConnectkConfigDefault");
    // this->move_group_.setPlanningTime(20.0);
    // this->move_group_.setNumPlanningAttempts(50);
    this->move_group_.setPlanningTime(20.0);
    this->move_group_.setNumPlanningAttempts(1000);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan goal;
    if (move_group_.plan(goal))
    {
        this->move_group_.execute(goal);
        response->result = true;

        // Save the reverse trajectory as Float32MultiArray
        std_msgs::msg::Float32MultiArray reverse_traj;
        reverse_traj.layout.dim.resize(2);
        reverse_traj.layout.dim[0].label = "waypoints";
        reverse_traj.layout.dim[0].size = goal.trajectory_.joint_trajectory.points.size();
        reverse_traj.layout.dim[0].stride = goal.trajectory_.joint_trajectory.points.size() * goal.trajectory_.joint_trajectory.joint_names.size();
        reverse_traj.layout.dim[1].label = "joints";
        reverse_traj.layout.dim[1].size = goal.trajectory_.joint_trajectory.joint_names.size();
        reverse_traj.layout.dim[1].stride = goal.trajectory_.joint_trajectory.joint_names.size();

        
        for (auto it = goal.trajectory_.joint_trajectory.points.rbegin(); it != goal.trajectory_.joint_trajectory.points.rend(); ++it)
        {
            for (double position : it->positions)
            {
                reverse_traj.data.push_back(position);
            }
        }

        response->reverse_traj = reverse_traj;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        response->result = false;
    }
}

void MoveArmNode::execute_trajectory(const std::shared_ptr<harvest_interfaces::srv::SendTrajectory::Request> request,
                                     std::shared_ptr<harvest_interfaces::srv::SendTrajectory::Response> response)
{
    // Extract the layout dimensions from the Float32MultiArray message
    const auto &layout = request->waypoints.layout;
    if (layout.dim.size() < 2)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid dimensions in waypoints array");
        response->success = false;
        return;
    }

    int num_waypoints = layout.dim[0].size;
    int num_joints = layout.dim[1].size;

    // Ensure the flattened data size matches the expected dimensions
    if (static_cast<int>(request->waypoints.data.size()) != num_waypoints * num_joints)
    {
        RCLCPP_ERROR(this->get_logger(), "Mismatch between data size and dimensions");
        response->success = false;
        return;
    }

    // Create a 2D vector to store the reconstructed data
    std::vector<std::vector<double>> path(num_waypoints, std::vector<double>(num_joints));

    // Copy data into the 2D vector
    size_t index = 0;
    for (int row = 0; row < num_waypoints; ++row)
    {
        for (int col = 0; col < num_joints; ++col)
        {
            path[row][col] = request->waypoints.data[index++];
        }
    }

    // Logging each waypoint from the trajectory
    for (int row = 0; row < num_waypoints; ++row)
    {
        std::stringstream ss;
        ss << "Waypoint " << row << ": ";
        for (int col = 0; col < num_joints; ++col)
        {
            ss << path[row][col];
            if (col < num_joints - 1)
            {
                ss << ", ";
            }
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    // Prepare the JointTrajectory message
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.frame_id = move_group_.getPlanningFrame();
    joint_trajectory.joint_names = move_group_.getJointNames();

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;

    double current_time = 0.0; // Start time
    // double time_step = 0.05;   // Time step between waypoints (seconds)
    double traj_time_step = this->get_parameter("traj_time_step").as_double();

    for (int i = 0; i < num_waypoints; ++i)
    {
        // Set joint positions for this point
        point.positions.clear();
        for (int j = 0; j < num_joints; ++j)
        {
            point.positions.push_back(path[i][j]);
        }

        // Set time_from_start for this point
        builtin_interfaces::msg::Duration duration;
        duration.sec = static_cast<uint32_t>(current_time);
        duration.nanosec = static_cast<uint32_t>((current_time - static_cast<uint32_t>(current_time)) * 1e9);
        point.time_from_start = duration;

        joint_trajectory.points.push_back(point);

        current_time += traj_time_step; // Increment time for the next waypoint
    }

    // Convert JointTrajectory to RobotTrajectory
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory = joint_trajectory;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;

    // Plan and execute the trajectory
    bool plan_success = static_cast<bool>(move_group_.execute(plan));
    if (plan_success)
    {
        response->success = true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute trajectory");
        response->success = false;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto move_service = std::make_shared<MoveArmNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_service);
    executor.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
