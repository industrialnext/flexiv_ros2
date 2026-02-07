/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"

namespace {

constexpr double kMaxJointVelocity = 2.0;
constexpr double kMaxJointAcceleration = 3.0;

}

namespace flexiv_hardware {

hardware_interface::CallbackReturn FlexivHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info)
        != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_efforts_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    stop_modes_ = {StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE};
    start_modes_ = {};
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    torque_controller_running_ = false;
    controllers_initialized_ = false;

    // Initialize cartesian arrays
    hw_commands_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_states_tcp_pose_.fill(0.0);
    cartesian_send_elapsed_s_ = 0.0;

    // Parse optional cartesian hardware params with safe defaults
    auto get_param = [&](const std::string& name, double default_val) -> double {
        auto it = info_.hardware_parameters.find(name);
        if (it != info_.hardware_parameters.end()) {
            return std::stod(it->second);
        }
        return default_val;
    };
    cartesian_send_period_s_ = get_param("cartesian_send_period_s", 0.01);
    cartesian_max_linear_vel_ = get_param("cartesian_max_linear_vel", 0.05);
    cartesian_max_angular_vel_ = get_param("cartesian_max_angular_vel", 0.20);
    cartesian_max_linear_acc_ = get_param("cartesian_max_linear_acc", 0.20);
    cartesian_max_angular_acc_ = get_param("cartesian_max_angular_acc", 0.50);

    if (info_.joints.size() != kJointDoF) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(), kJointDoF);
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld state interfaces found. 3 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    std::string robot_sn;
    try {
        robot_sn = info_.hardware_parameters["robot_sn"];
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'robot_sn' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        auto rdk_control_mode_str = info_.hardware_parameters.at("rdk_control_mode");
        if (rdk_control_mode_str == "joint_position") {
            rdk_control_mode_ = flexiv::rdk::Mode::NRT_JOINT_POSITION;
        } else if (rdk_control_mode_str == "joint_impedance") {
            rdk_control_mode_ = flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE;
        } else {
            RCLCPP_FATAL(getLogger(),
                "Parameter 'rdk_control_mode' has invalid value '%s'. Options: joint_position, "
                "joint_impedance",
                rdk_control_mode_str.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'rdk_control_mode' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        RCLCPP_INFO(getLogger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robot");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
    return hardware_interface::CallbackReturn::SUCCESS;
}

rclcpp::Logger FlexivHardwareInterface::getLogger()
{
    return rclcpp::get_logger("FlexivHardwareInterface");
}

std::vector<hardware_interface::StateInterface> FlexivHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_joint_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_joint_efforts_[i]));
    }

    std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        robot_sn, "flexiv_robot_states", reinterpret_cast<double*>(&hw_flexiv_robot_states_addr_)));

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (std::size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix + "gpio", "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
    }

    // Cartesian TCP pose state interfaces
    const std::string cart_state_names[] = {
        "tcp_pose_x", "tcp_pose_y", "tcp_pose_z",
        "tcp_pose_qw", "tcp_pose_qx", "tcp_pose_qy", "tcp_pose_qz"};
    for (size_t i = 0; i < kCartPoseSize; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix + "cartesian", cart_state_names[i], &hw_states_tcp_pose_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FlexivHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_joint_positions_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocities_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT, &hw_commands_joint_efforts_[i]));
    }

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "gpio", "digital_output_" + std::to_string(i), &hw_commands_gpio_out_[i]));
    }

    // Cartesian pose command interfaces
    const std::string cart_cmd_names[] = {
        "pose_x", "pose_y", "pose_z",
        "pose_qw", "pose_qx", "pose_qy", "pose_qz"};
    for (size_t i = 0; i < kCartPoseSize; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "cartesian", cart_cmd_names[i], &hw_commands_cartesian_pose_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Starting... please wait...");

    try {
        // Clear fault on robot server if any
        if (robot_->fault()) {
            RCLCPP_WARN(getLogger(), "Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            if (!robot_->ClearFault()) {
                RCLCPP_FATAL(getLogger(), "Fault cannot be cleared, exiting ...");
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(getLogger(), "Fault on robot server is cleared");
        }

        // Check the DoF of the robot
        if (robot_->info().DoF != kJointDoF) {
            RCLCPP_FATAL(getLogger(),
                "Robot has %ld DoF. Expected %ld. External axes control is not supported in ROS 2 "
                "yet.",
                robot_->info().DoF, kJointDoF);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Enable the robot
        RCLCPP_INFO(getLogger(), "Enabling robot ...");
        robot_->Enable();

        // Wait for the robot to become operational
        while (!robot_->operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(getLogger(), "Robot is now operational");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not enable robot.");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "System successfully started!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Stopping... please wait...");

    robot_->Stop();

    RCLCPP_INFO(getLogger(), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FlexivHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (robot_->operational()) {

        hw_flexiv_robot_states_ = robot_->states();

        // Read joint states
        for (size_t i = 0; i < info_.joints.size(); i++) {
            hw_states_joint_positions_[i] = robot_->states().q[i];
            hw_states_joint_velocities_[i] = robot_->states().dtheta[i];
            hw_states_joint_efforts_[i] = robot_->states().tau[i];
        }

        // Read GPIO input states
        auto gpio_in = robot_->digital_inputs();
        for (size_t i = 0; i < hw_states_gpio_in_.size(); i++) {
            hw_states_gpio_in_[i] = static_cast<double>(gpio_in[i]);
        }

        // Copy TCP pose to cartesian state interfaces
        const auto& tcp_pose = hw_flexiv_robot_states_.tcp_pose;
        for (size_t i = 0; i < kCartPoseSize && i < tcp_pose.size(); i++) {
            hw_states_tcp_pose_[i] = tcp_pose[i];
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    // Initialize target vectors to hold position
    std::vector<double> target_pos(robot_->info().DoF);
    std::vector<double> target_vel(robot_->info().DoF);

    std::vector<double> max_vel(robot_->info().DoF, kMaxJointVelocity);
    std::vector<double> max_acc(robot_->info().DoF, kMaxJointAcceleration);

    bool isNanPos = false;
    bool isNanVel = false;
    bool isNanEff = false;
    for (std::size_t i = 0; i < robot_->info().DoF; i++) {
        if (hw_commands_joint_positions_[i] != hw_commands_joint_positions_[i]) {
            isNanPos = true;
        }
        if (hw_commands_joint_velocities_[i] != hw_commands_joint_velocities_[i]) {
            isNanVel = true;
        }
        if (hw_commands_joint_efforts_[i] != hw_commands_joint_efforts_[i]) {
            isNanEff = true;
        }
    }

    if (position_controller_running_ && robot_->mode() == rdk_control_mode_ && !isNanPos) {
        target_pos = hw_commands_joint_positions_;
        robot_->SendJointPosition(target_pos, target_vel, max_vel, max_acc);
    } else if (velocity_controller_running_ && robot_->mode() == rdk_control_mode_ && !isNanVel) {
        target_pos = hw_states_joint_positions_;
        target_vel = hw_commands_joint_velocities_;
        robot_->SendJointPosition(target_pos, target_vel, max_vel, max_acc);
    } else if (torque_controller_running_ && robot_->mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE
               && !isNanEff) {
        std::vector<double> target_torque(robot_->info().DoF);
        target_torque = hw_commands_joint_efforts_;
        robot_->StreamJointTorque(target_torque, true, true);
    }

    // Cartesian motion-force streaming (NRT, rate-limited to ~100Hz)
    if (cartesian_controller_running_
        && robot_->mode() == flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE) {
        bool isNanCart = false;
        for (size_t i = 0; i < kCartPoseSize; i++) {
            if (std::isnan(hw_commands_cartesian_pose_[i])) {
                isNanCart = true;
                break;
            }
        }
        if (!isNanCart) {
            cartesian_send_elapsed_s_ += period.seconds();
            if (cartesian_send_elapsed_s_ >= cartesian_send_period_s_) {
                cartesian_send_elapsed_s_ = 0.0;
                std::array<double, kCartPoseSize> target;
                std::copy(hw_commands_cartesian_pose_.begin(),
                          hw_commands_cartesian_pose_.end(), target.begin());
                robot_->SendCartesianMotionForce(
                    target,
                    {},
                    {},
                    cartesian_max_linear_vel_,
                    cartesian_max_angular_vel_,
                    cartesian_max_linear_acc_,
                    cartesian_max_angular_acc_);
            }
        }
    }

    // Write digital output
    std::map<unsigned int, bool> digital_outputs;
    for (size_t i = 0; i < hw_commands_gpio_out_.size(); i++) {
        if (hw_commands_gpio_out_[i] != hw_commands_gpio_out_[i]) {
            continue;
        }
        digital_outputs[i] = static_cast<bool>(hw_commands_gpio_out_[i]);
    }
    // Check if there are changes in the digital output values
    bool digital_outputs_changed = false;
    for (const auto& [index, value] : digital_outputs) {
        if (current_digital_outputs_[index] != value) {
            current_digital_outputs_[index] = value;
            digital_outputs_changed = true;
        }
    }
    current_digital_outputs_.clear();
    for (const auto& [index, value] : digital_outputs) {
        current_digital_outputs_[index] = value;
    }

    // Set digital outputs
    if (digital_outputs_changed && !digital_outputs.empty()) {
        robot_->SetDigitalOutputs(digital_outputs);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    start_modes_.clear();
    stop_modes_.clear();
    cartesian_start_requested_ = false;
    cartesian_stop_requested_ = false;

    // Starting interfaces
    for (const auto& key : start_interfaces) {
        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                start_modes_.push_back(hardware_interface::HW_IF_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                start_modes_.push_back(hardware_interface::HW_IF_EFFORT);
            }
        }
    }

    // Detect cartesian start interfaces
    size_t cartesian_start_count = 0;
    for (const auto& key : start_interfaces) {
        if (key.find("cartesian/pose_") != std::string::npos) {
            cartesian_start_count++;
        }
    }
    if (cartesian_start_count > 0) {
        if (cartesian_start_count != kCartPoseSize) {
            RCLCPP_ERROR(getLogger(),
                "Cartesian start requires all %zu interfaces, got %zu",
                kCartPoseSize, cartesian_start_count);
            return hardware_interface::return_type::ERROR;
        }
        // Reject mixed mode (joint + cartesian simultaneously)
        if (start_modes_.size() > 0) {
            RCLCPP_ERROR(getLogger(),
                "Cannot start joint and cartesian interfaces simultaneously");
            return hardware_interface::return_type::ERROR;
        }
        cartesian_start_requested_ = true;
    }

    // All joints must be given new command mode at the same time
    // (skip this check when only cartesian interfaces are being started)
    if (!cartesian_start_requested_) {
        if (start_modes_.size() != 0 && start_modes_.size() != info_.joints.size()) {
            return hardware_interface::return_type::ERROR;
        }
        // All joints must have the same command mode
        if (start_modes_.size() != 0
            && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
            return hardware_interface::return_type::ERROR;
        }
    }

    // Stop motion on all relevant joints that are stopping
    for (const auto& key : stop_interfaces) {
        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                stop_modes_.push_back(StoppingInterface::STOP_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                stop_modes_.push_back(StoppingInterface::STOP_EFFORT);
            }
        }
    }

    // Detect cartesian stop interfaces
    size_t cartesian_stop_count = 0;
    for (const auto& key : stop_interfaces) {
        if (key.find("cartesian/pose_") != std::string::npos) {
            cartesian_stop_count++;
        }
    }
    if (cartesian_stop_count > 0) {
        if (cartesian_stop_count != kCartPoseSize) {
            RCLCPP_ERROR(getLogger(),
                "Cartesian stop requires all %zu interfaces, got %zu",
                kCartPoseSize, cartesian_stop_count);
            return hardware_interface::return_type::ERROR;
        }
        cartesian_stop_requested_ = true;
    }

    // stop all interfaces at the same time
    if (!cartesian_stop_requested_) {
        if (stop_modes_.size() != 0
            && (stop_modes_.size() != info_.joints.size()
                || !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin()))) {
            return hardware_interface::return_type::ERROR;
        }
    }

    controllers_initialized_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    // STOP cartesian
    if (cartesian_stop_requested_) {
        cartesian_controller_running_ = false;
        robot_->Stop();
    }

    if (stop_modes_.size() != 0
        && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION)
               != stop_modes_.end()) {
        position_controller_running_ = false;
        robot_->Stop();
    } else if (stop_modes_.size() != 0
               && std::find(
                      stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY)
                      != stop_modes_.end()) {
        velocity_controller_running_ = false;
        robot_->Stop();
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_EFFORT)
                      != stop_modes_.end()) {
        torque_controller_running_ = false;
        robot_->Stop();
    }

    if (start_modes_.size() != 0
        && std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION)
               != start_modes_.end()) {
        velocity_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold joints before user commands arrives
        std::fill(hw_commands_joint_positions_.begin(), hw_commands_joint_positions_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        position_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_VELOCITY)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold joints before user commands arrives
        std::fill(hw_commands_joint_velocities_.begin(), hw_commands_joint_velocities_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        velocity_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_EFFORT)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;

        // Hold joints when starting joint torque controller before user
        // commands arrives
        std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint torque mode
        robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

        torque_controller_running_ = true;
    }

    // START cartesian
    if (cartesian_start_requested_) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;
        torque_controller_running_ = false;
        hw_commands_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
        cartesian_send_elapsed_s_ = 0.0;
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});
        cartesian_controller_running_ = true;
    }

    start_modes_.clear();
    stop_modes_.clear();
    cartesian_start_requested_ = false;
    cartesian_stop_requested_ = false;

    return hardware_interface::return_type::OK;
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivHardwareInterface, hardware_interface::SystemInterface)
