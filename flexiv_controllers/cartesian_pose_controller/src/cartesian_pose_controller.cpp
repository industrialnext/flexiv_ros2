/**
 * @file cartesian_pose_controller.cpp
 * @brief Cartesian pose controller as ROS 2 controller for NRT cartesian streaming.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "cartesian_pose_controller/cartesian_pose_controller.hpp"

#include <cmath>
#include <string>

namespace cartesian_pose_controller {

CartesianPoseController::CartesianPoseController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn CartesianPoseController::on_init()
{
    try {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
        pose_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianPoseController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Use raw robot_sn (with dashes) to match hardware interface naming.
    // Dash-to-underscore replacement is only for ROS topic names (in on_configure).
    const std::string& robot_sn = params_.robot_sn;

    const std::string names[] = {
        "pose_x", "pose_y", "pose_z", "pose_qw", "pose_qx", "pose_qy", "pose_qz"};
    for (const auto& name : names) {
        config.names.emplace_back(robot_sn + "_cartesian/" + name);
    }

    return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string& robot_sn = params_.robot_sn;

    const std::string names[] = {"tcp_pose_x", "tcp_pose_y", "tcp_pose_z", "tcp_pose_qw",
        "tcp_pose_qx", "tcp_pose_qy", "tcp_pose_qz"};
    for (const auto& name : names) {
        config.names.emplace_back(robot_sn + "_cartesian/" + name);
    }

    return config;
}

controller_interface::return_type CartesianPoseController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!has_received_command_) {
        return controller_interface::return_type::OK;
    }

    std::array<double, kCartPoseSize> cmd;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        cmd = pose_cmd_;
    }

    // Write to command interfaces (7 values in order: x,y,z,qw,qx,qy,qz)
    for (size_t i = 0; i < kCartPoseSize; ++i) {
        command_interfaces_[i].set_value(cmd[i]);
    }

    // Publish commanded pose for debugging
    if (commanded_pose_pub_) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = get_node()->get_clock()->now();
        msg.header.frame_id = "world";
        msg.pose.position.x = cmd[0];
        msg.pose.position.y = cmd[1];
        msg.pose.position.z = cmd[2];
        msg.pose.orientation.w = cmd[3];
        msg.pose.orientation.x = cmd[4];
        msg.pose.orientation.y = cmd[5];
        msg.pose.orientation.z = cmd[6];
        commanded_pose_pub_->publish(msg);
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CartesianPoseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    params_ = param_listener_->get_params();

    std::string robot_sn = params_.robot_sn;
    if (robot_sn.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'robot_sn' parameter has to be specified.");
        return CallbackReturn::ERROR;
    } else {
        // Replace "-" with "_" in robot_sn to match the topic name
        std::replace(robot_sn.begin(), robot_sn.end(), '-', '_');
    }

    try {
        // register publisher
        commanded_pose_pub_
            = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/" + robot_sn + "/cartesian_pose_controller/commanded_pose",
                rclcpp::SystemDefaultsQoS());

        // register subscriber
        target_pose_sub_
            = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + robot_sn + "/cartesian_pose_controller/target_pose",
                rclcpp::SystemDefaultsQoS(),
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    pose_cmd_[0] = msg->pose.position.x;
                    pose_cmd_[1] = msg->pose.position.y;
                    pose_cmd_[2] = msg->pose.position.z;
                    pose_cmd_[3] = msg->pose.orientation.w;  // qw first in RDK
                    pose_cmd_[4] = msg->pose.orientation.x;
                    pose_cmd_[5] = msg->pose.orientation.y;
                    pose_cmd_[6] = msg->pose.orientation.z;
                    has_received_command_ = true;
                });
    } catch (...) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    try {
        // reset publisher
        commanded_pose_pub_.reset();
    } catch (...) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    has_received_command_ = false;
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace cartesian_pose_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    cartesian_pose_controller::CartesianPoseController,
    controller_interface::ControllerInterface)
