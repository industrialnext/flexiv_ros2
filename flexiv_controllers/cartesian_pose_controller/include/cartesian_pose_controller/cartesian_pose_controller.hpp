/**
 * @file cartesian_pose_controller.hpp
 * @brief Cartesian pose controller as ROS 2 controller for NRT cartesian streaming.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef CARTESIAN_POSE_CONTROLLER__CARTESIAN_POSE_CONTROLLER_HPP_
#define CARTESIAN_POSE_CONTROLLER__CARTESIAN_POSE_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "cartesian_pose_controller/cartesian_pose_controller_parameters.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cartesian_pose_controller {

/** Cartesian pose size: x, y, z, qw, qx, qy, qz */
constexpr size_t kCartPoseSize = 7;

class CartesianPoseController : public controller_interface::ControllerInterface
{
public:
    CartesianPoseController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    // Buffered pose command from subscription [x,y,z,qw,qx,qy,qz] in RDK ordering
    std::array<double, kCartPoseSize> pose_cmd_;
    std::mutex pose_mutex_;
    bool has_received_command_ = false;

    // Publisher
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> commanded_pose_pub_;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
};

} /* namespace cartesian_pose_controller */
#endif /* CARTESIAN_POSE_CONTROLLER__CARTESIAN_POSE_CONTROLLER_HPP_ */
