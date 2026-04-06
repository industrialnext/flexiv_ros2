/**
 * @file flexiv_hardware_interface.hpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/include/ros2_control_demo_example_3/rrbot_system_multi_interface.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

// ROS
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

// ros2_control hardware_interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "flexiv_hardware/visibility_control.h"

// Flexiv
#include "flexiv/rdk/robot.hpp"
#include "flexiv/rdk/tool.hpp"

namespace flexiv_hardware {

/** Robot joint space degree of freedoms */
constexpr size_t kJointDoF = 7;

/** Cartesian DOF (x,y,z,rx,ry,rz) */
constexpr size_t kCartDoF = 6;

/** Cartesian pose size (x,y,z,qw,qx,qy,qz) */
constexpr size_t kPoseSize = 7;

enum StoppingInterface
{
    NONE,
    STOP_POSITION,
    STOP_VELOCITY,
    STOP_EFFORT,
    STOP_CARTESIAN
};

class FlexivHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FlexivHardwareInterface)

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    FLEXIV_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    FLEXIV_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    FLEXIV_HARDWARE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Flexiv RDK
    std::unique_ptr<flexiv::rdk::Robot> robot_;

    // Tool profile name for gravity compensation. Empty = no switch.
    std::string tool_name_;

    // RDK control mode for joint position and velocity interfaces
    flexiv::rdk::Mode rdk_control_mode_;

    // ── Joint commands ──────────────────────────────────────────────
    std::vector<double> hw_commands_joint_positions_;
    std::vector<double> hw_commands_joint_velocities_;
    std::vector<double> hw_commands_joint_efforts_;

    // ── Joint states ────────────────────────────────────────────────
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // ── Robot States (full struct, exposed via pointer) ─────────────
    flexiv::rdk::RobotStates hw_flexiv_robot_states_;
    flexiv::rdk::RobotStates* hw_flexiv_robot_states_addr_ = &hw_flexiv_robot_states_;

    // ── GPIO commands and states ────────────────────────────────────
    std::vector<double> hw_commands_gpio_out_;
    std::vector<double> hw_states_gpio_in_;
    std::map<unsigned int, bool> current_digital_outputs_;

    // ── Cartesian command interfaces (for RT_CARTESIAN_MOTION_FORCE) ─
    //
    // Target TCP pose: [x, y, z, qw, qx, qy, qz]
    std::array<double, kPoseSize> hw_cmd_cart_pose_;

    // Target TCP wrench: [fx, fy, fz, mx, my, mz]
    // Phase 2: feed-forward wrench for force-controlled axes
    std::array<double, kCartDoF> hw_cmd_cart_wrench_;

    // Cartesian impedance stiffness: [kx, ky, kz, krx, kry, krz]
    // Valid range: [0, K_x_nom]. Unit: [N/m]:[Nm/rad]
    std::array<double, kCartDoF> hw_cmd_cart_stiffness_;

    // Cartesian impedance damping ratio: [zx, zy, zz, zrx, zry, zrz]
    // Valid range: [0.3, 0.8]. Nominal = 0.7
    std::array<double, kCartDoF> hw_cmd_cart_damping_ratio_;

    // Maximum contact wrench: [fx, fy, fz, mx, my, mz]
    // Inf = disabled. Unit: [N]:[Nm]
    std::array<double, kCartDoF> hw_cmd_cart_max_wrench_;

    // Per-axis force control enable (0.0 = motion, 1.0 = force)
    // Phase 2: per-axis motion/force switching
    std::array<double, kCartDoF> hw_cmd_cart_force_ctrl_axis_;

    // Nullspace reference joint positions for Cartesian mode
    std::vector<double> hw_cmd_cart_nullspace_q_;

    // ── Cartesian state interfaces ──────────────────────────────────
    // Current TCP pose from robot: [x, y, z, qw, qx, qy, qz]
    std::array<double, kPoseSize> hw_state_cart_pose_;

    // Nominal stiffness from robot info (read-only)
    std::array<double, kCartDoF> hw_state_cart_K_x_nom_;

    // Active tool TCP in flange frame: [x,y,z,qw,qx,qy,qz] (read-only)
    std::array<double, kPoseSize> hw_state_tool_tcp_;

    // ── Cartesian dirty flags (Option C: only call blocking APIs on change)
    std::array<double, kCartDoF> prev_cart_stiffness_;
    std::array<double, kCartDoF> prev_cart_damping_ratio_;
    std::array<double, kCartDoF> prev_cart_max_wrench_;
    std::array<double, kCartDoF> prev_cart_force_ctrl_axis_;
    std::vector<double> prev_cart_nullspace_q_;

    bool cart_stiffness_dirty_{false};
    bool cart_max_wrench_dirty_{false};
    bool cart_force_ctrl_axis_dirty_{false};
    bool cart_nullspace_dirty_{false};

    /// Check if Cartesian config command interfaces changed since last write.
    void check_cartesian_dirty_flags();

    static rclcpp::Logger getLogger();

    // ── Control mode tracking ───────────────────────────────────────
    bool controllers_initialized_;
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool torque_controller_running_;
    bool cartesian_controller_running_;
};

} /* namespace flexiv_hardware */
#endif /* FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_ */
