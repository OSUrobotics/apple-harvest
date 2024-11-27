#include <rclcpp/rclcpp.hpp>
#include "harvest_interfaces/srv/move_to_pose.hpp"
#include "harvest_interfaces/action/send_trajectory.hpp"
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
#include <rclcpp_action/rclcpp_action.hpp> 
// #include "generate_tree_collision_objects.hpp"


// Read data packages
#include <iostream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <cmath>               // For M_PI

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class MoveArmNode : public rclcpp::Node
{
public:
    using SendTrajectory = harvest_interfaces::action::SendTrajectory;
    using GoalHandleSendTrajectory = rclcpp_action::ServerGoalHandle<SendTrajectory>;

    MoveArmNode();

private:
    rclcpp_action::Server<harvest_interfaces::action::SendTrajectory>::SharedPtr send_trajectory_action_server_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_to_home_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_to_config_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_arm_motion_service_;

    std::shared_ptr<GoalHandleSendTrajectory> current_goal_handle_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    moveit::planning_interface::MoveGroupInterface move_group_;

    void handle_send_trajectory(const std::shared_ptr<GoalHandleSendTrajectory> goal_handle);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SendTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSendTrajectory> goal_handle);
    void feedback_callback(const std::shared_ptr<GoalHandleSendTrajectory> goal_handle, double percentage);

    void move_to_home(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void move_to_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void stop_arm_motion(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);


    std::vector<double> home_joint_positions = {
        M_PI / 2,
        - M_PI / 2, 
        2 * M_PI / 3,
        5 * M_PI / 6,
        - M_PI / 2,
        0};
};

MoveArmNode::MoveArmNode()
    : Node("move_arm_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ur_manipulator")
{
    RCLCPP_INFO(this->get_logger(), "Setting up send trajectory action server");
    // Set up action server
    send_trajectory_action_server_ = rclcpp_action::create_server<harvest_interfaces::action::SendTrajectory>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "send_arm_trajectory",
        std::bind(&MoveArmNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveArmNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&MoveArmNode::handle_send_trajectory, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Send trajectory action server set up successfully");

    // Set up services
    arm_to_home_service_ = this->create_service<std_srvs::srv::Trigger>(
        "move_arm_to_home", std::bind(&MoveArmNode::move_to_home, this, _1, _2));

    arm_to_config_service_ = this->create_service<std_srvs::srv::Trigger>(
        "move_arm_to_config", std::bind(&MoveArmNode::move_to_config, this, _1, _2));

    stop_arm_motion_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_arm_motion", std::bind(&MoveArmNode::stop_arm_motion, this, _1, _2));

    // Set up parameters
    double max_accel = this->get_parameter("max_accel").as_double();
    double max_vel = this->get_parameter("max_vel").as_double();

    // Set velocity and acceleration limits
    // NEED TO RESET TO 0.05 FOR HARDWARE!!!
    this->move_group_.setMaxAccelerationScalingFactor(max_accel);
    this->move_group_.setMaxVelocityScalingFactor(max_vel);

    // // Add tree collision objects to the planning scene
    // RCLCPP_INFO(this->get_logger(), "Generating tree collision objects...");
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // std::vector<moveit_msgs::msg::CollisionObject> tree_collision_objects = generateTreeCollisionObjects();
    // planning_scene_interface.applyCollisionObjects(tree_collision_objects);
    // RCLCPP_INFO(this->get_logger(), "Tree collision objects added to the planning scene.");

    RCLCPP_INFO(this->get_logger(), "Move arm server ready");
}

rclcpp_action::GoalResponse MoveArmNode::handle_goal(const rclcpp_action::GoalUUID & /*uuid*/, 
                                                     std::shared_ptr<const SendTrajectory::Goal> /*goal*/)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request to execute trajectory");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveArmNode::handle_cancel(std::shared_ptr<GoalHandleSendTrajectory> /*goal_handle*/)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel trajectory execution");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveArmNode::stop_arm_motion(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;  // Suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Stopping arm motion");

    // Stop any active motion
    move_group_.stop();  // This will stop the ongoing motion if any

    response->success = true;
    response->message = "Arm motion stopped successfully.";
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
        -2.35,
        4.88,
        -0.89,
        -3.65,
        3.75,
        1.62};

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

void MoveArmNode::handle_send_trajectory(const std::shared_ptr<GoalHandleSendTrajectory> goal_handle)
{
    current_goal_handle_ = goal_handle;  // Store the active goal handle

    const auto &waypoints = goal_handle->get_goal()->waypoints;
    
    // Extract the layout dimensions from the Float32MultiArray message
    const auto &layout = waypoints.layout;
    if (layout.dim.size() < 2)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid dimensions in waypoints array");

        auto result = std::make_shared<SendTrajectory::Result>();
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    int num_waypoints = layout.dim[0].size;
    int num_joints = layout.dim[1].size;

    // Ensure the flattened data size matches the expected dimensions
    if (static_cast<int>(waypoints.data.size()) != num_waypoints * num_joints)
    {
        RCLCPP_ERROR(this->get_logger(), "Mismatch between data size and dimensions");
        auto result = std::make_shared<SendTrajectory::Result>();
        result->success = false;
        goal_handle->abort(result);
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
            path[row][col] = waypoints.data[index++];
        }
    }

    // Prepare the JointTrajectory message
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.frame_id = move_group_.getPlanningFrame();
    joint_trajectory.joint_names = move_group_.getJointNames();

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;

    double current_time = 0.0; // Start time
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
    bool plan_success = static_cast<bool>(move_group_.asyncExecute(plan));
    if (plan_success)
    {
        for (int i = 0; i <= num_waypoints; ++i)
        {
            // Sending feedback during execution
            double percentage = static_cast<double>(i) / num_waypoints * 100.0;
            feedback_callback(goal_handle, percentage);

            // Convert the double (seconds) to std::chrono::milliseconds
            auto delay_duration = std::chrono::duration<int64_t, std::nano>(static_cast<int64_t>((traj_time_step - (traj_time_step * 0.15)) * 1e9));
            rclcpp::sleep_for(delay_duration);
        }

        auto result = std::make_shared<SendTrajectory::Result>();
        result->success = true;  // Set the result data properly
        goal_handle->succeed(result);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute trajectory");
        auto result = std::make_shared<SendTrajectory::Result>();
        result->success = false;
        goal_handle->abort(result);
    }
}

void MoveArmNode::feedback_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<harvest_interfaces::action::SendTrajectory>> goal_handle, 
    double percentage) {
    auto feedback = std::make_shared<SendTrajectory::Feedback>();
    feedback->progress = percentage;
    goal_handle->publish_feedback(feedback);
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
