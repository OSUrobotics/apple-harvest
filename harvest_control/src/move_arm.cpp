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

#include <iostream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class MoveArmNode : public rclcpp::Node
{
public:
    MoveArmNode();
    void init_moveit();

private:
    rclcpp::Service<harvest_interfaces::srv::SendTrajectory>::SharedPtr arm_trajectory_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_to_home_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_to_config_service_;
    rclcpp::Service<harvest_interfaces::srv::MoveToPose>::SharedPtr arm_to_pose_service_;

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr gripper_tip_subscription_;
    geometry_msgs::msg::TransformStamped current_gripper_pose_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    const std::vector<double> home_joint_positions_ = {
        M_PI / 4, -M_PI / 2, 2 * M_PI / 3, 5 * M_PI / 6, -M_PI / 2, 0};

    const std::vector<double> scan_joint_positions_ = {
        M_PI / 2, -M_PI / 2, 2 * M_PI / 3, 5 * M_PI / 6, -M_PI / 2, 0};

    const std::vector<double> target_joint_positions_ = {
        1.5, -2.775, 1.72, 4.55, -1.58, 0};

    // Plans and executes a joint-space goal; returns true on success
    bool plan_and_execute_joints(const std::vector<double> &joint_positions);

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
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
    gripper_tip_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/gripper_tip", 10, std::bind(&MoveArmNode::gripper_tip_callback, this, std::placeholders::_1));

    arm_trajectory_service_ = this->create_service<harvest_interfaces::srv::SendTrajectory>(
        "send_arm_trajectory", std::bind(&MoveArmNode::execute_trajectory, this, _1, _2));

    arm_to_home_service_ = this->create_service<std_srvs::srv::Trigger>(
        "move_arm_to_home", std::bind(&MoveArmNode::move_to_home, this, _1, _2));

    arm_to_config_service_ = this->create_service<std_srvs::srv::Trigger>(
        "move_arm_to_config", std::bind(&MoveArmNode::move_to_config, this, _1, _2));

    arm_to_pose_service_ = this->create_service<harvest_interfaces::srv::MoveToPose>(
        "move_arm_to_pose", std::bind(&MoveArmNode::move_to_pose, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Move arm server ready");
}

void MoveArmNode::init_moveit()
{
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur5e_manipulator");

    move_group_->setMaxAccelerationScalingFactor(this->get_parameter("max_accel").as_double());
    move_group_->setMaxVelocityScalingFactor(this->get_parameter("max_vel").as_double());

    RCLCPP_INFO(this->get_logger(), "MoveIt MoveGroupInterface initialized");
}

void MoveArmNode::gripper_tip_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    current_gripper_pose_ = *msg;
}

bool MoveArmNode::plan_and_execute_joints(const std::vector<double> &joint_positions)
{
    move_group_->setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (static_cast<bool>(move_group_->plan(plan)))
    {
        move_group_->execute(plan);
        return true;
    }
    return false;
}

void MoveArmNode::move_to_home(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                               const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = plan_and_execute_joints(home_joint_positions_);
    response->message = response->success
        ? "Successfully moved to home configuration."
        : "Failed to move to home configuration.";
    RCLCPP_INFO(this->get_logger(), response->message.c_str());
}

void MoveArmNode::move_to_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                 const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = plan_and_execute_joints(target_joint_positions_);
    if (!response->success)
        RCLCPP_ERROR(this->get_logger(), "Failed to move to target configuration.");
}

void MoveArmNode::move_to_pose(const std::shared_ptr<harvest_interfaces::srv::MoveToPose::Request> request,
                               const std::shared_ptr<harvest_interfaces::srv::MoveToPose::Response> response)
{
    move_group_->setStartStateToCurrentState();

    geometry_msgs::msg::PoseStamped msg = request->pose_stamped;

    const auto &ori = request->pose_stamped.pose.orientation;
    if (std::isnan(ori.x) || std::isnan(ori.y) || std::isnan(ori.z) || std::isnan(ori.w))
    {
        RCLCPP_WARN(this->get_logger(), "No valid orientation provided, using default.");
        tf2::Quaternion q;
        q.setRPY(M_PI / 2, M_PI, M_PI);
        msg.pose.orientation = tf2::toMsg(q);
    }

    move_group_->setPoseTarget(msg, "gripper_link");
    move_group_->setGoalOrientationTolerance(1.05);
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(20.0);
    move_group_->setNumPlanningAttempts(1000);

    moveit::planning_interface::MoveGroupInterface::Plan goal;
    if (move_group_->plan(goal))
    {
        move_group_->execute(goal);
        response->result = true;

        // Build reverse trajectory for caller
        const auto &traj = goal.trajectory_.joint_trajectory;
        std_msgs::msg::Float32MultiArray reverse_traj;
        reverse_traj.layout.dim.resize(2);
        reverse_traj.layout.dim[0].label  = "waypoints";
        reverse_traj.layout.dim[0].size   = traj.points.size();
        reverse_traj.layout.dim[0].stride = traj.points.size() * traj.joint_names.size();
        reverse_traj.layout.dim[1].label  = "joints";
        reverse_traj.layout.dim[1].size   = traj.joint_names.size();
        reverse_traj.layout.dim[1].stride = traj.joint_names.size();

        for (auto it = traj.points.rbegin(); it != traj.points.rend(); ++it)
            for (double pos : it->positions)
                reverse_traj.data.push_back(pos);

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
    const auto &layout = request->waypoints.layout;
    if (layout.dim.size() < 2)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid dimensions in waypoints array");
        response->success = false;
        return;
    }

    const int num_waypoints = layout.dim[0].size;
    const int num_joints    = layout.dim[1].size;

    if (static_cast<int>(request->waypoints.data.size()) != num_waypoints * num_joints)
    {
        RCLCPP_ERROR(this->get_logger(), "Mismatch between data size and dimensions");
        response->success = false;
        return;
    }

    // Reconstruct 2-D path from flat array
    std::vector<std::vector<double>> path(num_waypoints, std::vector<double>(num_joints));
    size_t index = 0;
    for (int row = 0; row < num_waypoints; ++row)
        for (int col = 0; col < num_joints; ++col)
            path[row][col] = request->waypoints.data[index++];

    // Log waypoints
    for (int row = 0; row < num_waypoints; ++row)
    {
        std::stringstream ss;
        ss << "Waypoint " << row << ": ";
        for (int col = 0; col < num_joints; ++col)
        {
            ss << path[row][col];
            if (col < num_joints - 1) ss << ", ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // Build JointTrajectory
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.frame_id = move_group_->getPlanningFrame();
    joint_trajectory.joint_names     = move_group_->getJointNames();

    const double traj_time_step = this->get_parameter("traj_time_step").as_double();
    double current_time = 0.0;

    for (int i = 0; i < num_waypoints; ++i)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (int j = 0; j < num_joints; ++j)
            point.positions.push_back(path[i][j]);

        builtin_interfaces::msg::Duration duration;
        duration.sec    = static_cast<uint32_t>(current_time);
        duration.nanosec = static_cast<uint32_t>((current_time - duration.sec) * 1e9);
        point.time_from_start = duration;

        joint_trajectory.points.push_back(point);
        current_time += traj_time_step;
    }

    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory = joint_trajectory;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;

    response->success = static_cast<bool>(move_group_->execute(plan));
    if (!response->success)
        RCLCPP_ERROR(this->get_logger(), "Failed to execute trajectory");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto move_service = std::make_shared<MoveArmNode>();
    move_service->init_moveit();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_service);
    executor.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}