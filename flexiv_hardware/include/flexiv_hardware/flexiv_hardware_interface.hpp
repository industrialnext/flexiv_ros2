/**
 * @file flexiv_hardware_interface.hpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/include/ros2_control_demo_example_3/rrbot_system_multi_interface.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_srvs/srv/trigger.hpp>

// ros2_control hardware_interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "flexiv_hardware/visibility_control.h"

// Flexiv
#include "flexiv/rdk/robot.hpp"

namespace flexiv_hardware {

/** Robot joint space degree of freedoms */
constexpr size_t kJointDoF = 7;

enum StoppingInterface
{
    NONE,
    STOP_POSITION,
    STOP_VELOCITY,
    STOP_EFFORT
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

    // RDK control mode for joint position and velocity interfaces
    flexiv::rdk::Mode rdk_control_mode_;

    // Joint commands
    std::vector<double> hw_commands_joint_positions_;
    std::vector<double> hw_commands_joint_velocities_;
    std::vector<double> hw_commands_joint_efforts_;

    // Joint states
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // Robot States
    flexiv::rdk::RobotStates hw_flexiv_robot_states_;
    flexiv::rdk::RobotStates* hw_flexiv_robot_states_addr_ = &hw_flexiv_robot_states_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;
    std::vector<double> hw_states_gpio_in_;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_;

    static rclcpp::Logger getLogger();

    void handle_zero_ft_sensor(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Internal node + executor for the /zero_ft_sensor service.
    // Runs in a background thread; uses robot_ directly.
    std::shared_ptr<rclcpp::Node> zero_ft_node_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_ft_srv_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> zero_ft_executor_;
    std::thread zero_ft_thread_;

    // Set to true while ZeroFTSensor is executing so write() skips
    // SendJointPosition. Cleared when the primitive finishes.
    std::atomic<bool> zero_ft_in_progress_{false};

    // control modes
    bool controllers_initialized_;
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool torque_controller_running_;
};

} /* namespace flexiv_hardware */
#endif /* FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_ */
