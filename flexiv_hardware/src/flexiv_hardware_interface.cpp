/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <vector>
#include <string>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"

namespace {

constexpr double kMaxJointVelocity = 2.0;
constexpr double kMaxJointAcceleration = 3.0;

// Cartesian command interface name prefix (used as a gpio-style component)
const std::string kCartPrefix = "tcp";

// Interface names for Cartesian commands
const std::array<std::string, 7> kPoseNames = {
    "pose_x", "pose_y", "pose_z", "pose_qw", "pose_qx", "pose_qy", "pose_qz"};
const std::array<std::string, 6> kWrenchNames = {
    "wrench_fx", "wrench_fy", "wrench_fz", "wrench_mx", "wrench_my", "wrench_mz"};
const std::array<std::string, 6> kStiffnessNames = {
    "stiffness_x", "stiffness_y", "stiffness_z",
    "stiffness_rx", "stiffness_ry", "stiffness_rz"};
const std::array<std::string, 6> kDampingRatioNames = {
    "damping_ratio_x", "damping_ratio_y", "damping_ratio_z",
    "damping_ratio_rx", "damping_ratio_ry", "damping_ratio_rz"};
const std::array<std::string, 6> kMaxWrenchNames = {
    "max_wrench_fx", "max_wrench_fy", "max_wrench_fz",
    "max_wrench_mx", "max_wrench_my", "max_wrench_mz"};
const std::array<std::string, 6> kForceCtrlAxisNames = {
    "force_ctrl_x", "force_ctrl_y", "force_ctrl_z",
    "force_ctrl_rx", "force_ctrl_ry", "force_ctrl_rz"};
const std::array<std::string, 7> kNullspaceNames = {
    "nullspace_q1", "nullspace_q2", "nullspace_q3", "nullspace_q4",
    "nullspace_q5", "nullspace_q6", "nullspace_q7"};

// State interface names
const std::array<std::string, 7> kStatePoseNames = {
    "state_pose_x", "state_pose_y", "state_pose_z",
    "state_pose_qw", "state_pose_qx", "state_pose_qy", "state_pose_qz"};
const std::array<std::string, 6> kStateKxNomNames = {
    "K_x_nom_x", "K_x_nom_y", "K_x_nom_z",
    "K_x_nom_rx", "K_x_nom_ry", "K_x_nom_rz"};

/// Check approximate equality for doubles
bool approx_eq(double a, double b, double eps = 1e-6)
{
    return std::abs(a - b) < eps;
}

} // namespace

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
    cartesian_controller_running_ = false;
    controllers_initialized_ = false;

    // Initialize Cartesian command arrays
    hw_cmd_cart_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_cmd_cart_wrench_.fill(0.0);
    hw_cmd_cart_stiffness_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_cmd_cart_damping_ratio_.fill(0.7);  // Flexiv nominal
    hw_cmd_cart_max_wrench_.fill(std::numeric_limits<double>::infinity());
    hw_cmd_cart_force_ctrl_axis_.fill(0.0);  // All motion-controlled
    hw_cmd_cart_nullspace_q_.resize(kJointDoF, std::numeric_limits<double>::quiet_NaN());
    hw_state_cart_pose_.fill(0.0);
    hw_state_cart_K_x_nom_.fill(0.0);

    // Dirty flag tracking
    prev_cart_stiffness_.fill(std::numeric_limits<double>::quiet_NaN());
    prev_cart_damping_ratio_.fill(std::numeric_limits<double>::quiet_NaN());
    prev_cart_max_wrench_.fill(std::numeric_limits<double>::quiet_NaN());
    prev_cart_force_ctrl_axis_.fill(std::numeric_limits<double>::quiet_NaN());
    prev_cart_nullspace_q_.resize(kJointDoF, std::numeric_limits<double>::quiet_NaN());

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

    // Tool profile name for gravity compensation (mass/CoM/inertia/TCP).
    // If non-empty, Tool::Switch() will be called during on_activate() while
    // the robot is still in IDLE mode. This allows setting the correct tool
    // profile independently of the gripper node.
    auto tool_name_it = info_.hardware_parameters.find("tool_name");
    if (tool_name_it != info_.hardware_parameters.end()) {
        tool_name_ = tool_name_it->second;
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

    // Raise the RT timeliness failure limit so that blocking mode
    // switches (Stop + SwitchMode) don't trip the default threshold
    // of 3 failures within 60 seconds.
    robot_->SetTimelinessFailureLimit(20);

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

    // Joint states
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_joint_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_joint_efforts_[i]));
    }

    // Flexiv robot states (raw struct pointer)
    std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        robot_sn, "flexiv_robot_states", reinterpret_cast<double*>(&hw_flexiv_robot_states_addr_)));

    // GPIO inputs
    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (std::size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix + "gpio", "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
    }

    // Cartesian state: current TCP pose [x,y,z,qw,qx,qy,qz]
    for (std::size_t i = 0; i < kPoseSize; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            kCartPrefix, kStatePoseNames[i], &hw_state_cart_pose_[i]));
    }

    // Cartesian state: nominal stiffness K_x_nom (read-only, from robot info)
    for (std::size_t i = 0; i < kCartDoF; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            kCartPrefix, kStateKxNomNames[i], &hw_state_cart_K_x_nom_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FlexivHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Joint commands
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_joint_positions_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocities_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT, &hw_commands_joint_efforts_[i]));
    }

    // GPIO outputs
    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "gpio", "digital_output_" + std::to_string(i), &hw_commands_gpio_out_[i]));
    }

    // ── Cartesian command interfaces ────────────────────────────────
    // Target pose [x,y,z,qw,qx,qy,qz]
    for (std::size_t i = 0; i < kPoseSize; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            kCartPrefix, kPoseNames[i], &hw_cmd_cart_pose_[i]));
    }

    // Target wrench [fx,fy,fz,mx,my,mz] (Phase 2: force-controlled axes)
    for (std::size_t i = 0; i < kCartDoF; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            kCartPrefix, kWrenchNames[i], &hw_cmd_cart_wrench_[i]));
    }

    // Impedance stiffness [kx,ky,kz,krx,kry,krz]
    for (std::size_t i = 0; i < kCartDoF; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            kCartPrefix, kStiffnessNames[i], &hw_cmd_cart_stiffness_[i]));
    }

    // Impedance damping ratio [zx,zy,zz,zrx,zry,zrz]
    for (std::size_t i = 0; i < kCartDoF; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            kCartPrefix, kDampingRatioNames[i], &hw_cmd_cart_damping_ratio_[i]));
    }

    // Maximum contact wrench [fx,fy,fz,mx,my,mz]
    for (std::size_t i = 0; i < kCartDoF; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            kCartPrefix, kMaxWrenchNames[i], &hw_cmd_cart_max_wrench_[i]));
    }

    // Per-axis force control enable (Phase 2)
    for (std::size_t i = 0; i < kCartDoF; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            kCartPrefix, kForceCtrlAxisNames[i], &hw_cmd_cart_force_ctrl_axis_[i]));
    }

    // Nullspace reference joint positions
    for (std::size_t i = 0; i < kJointDoF; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            kCartPrefix, kNullspaceNames[i], &hw_cmd_cart_nullspace_q_[i]));
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

        // Switch tool profile for gravity compensation if configured.
        // Must happen while the robot is in IDLE mode (after Enable, before
        // SwitchMode), so we do it here before any controller starts.
        if (!tool_name_.empty()) {
            RCLCPP_INFO(
                getLogger(), "Switching robot tool to '%s' ...", tool_name_.c_str());
            auto tool = std::make_unique<flexiv::rdk::Tool>(*robot_);
            tool->Switch(tool_name_);

            auto tp = tool->params();
            RCLCPP_INFO(getLogger(),
                "Active tool '%s': mass=%.3f kg, CoM=[%.4f, %.4f, %.4f] m, "
                "TCP=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                tool_name_.c_str(), tp.mass,
                tp.CoM[0], tp.CoM[1], tp.CoM[2],
                tp.tcp_location[0], tp.tcp_location[1], tp.tcp_location[2],
                tp.tcp_location[3], tp.tcp_location[4], tp.tcp_location[5],
                tp.tcp_location[6]);
        }

        // Cache nominal stiffness from robot info for state interfaces
        auto K_nom = robot_->info().K_x_nom;
        for (std::size_t i = 0; i < kCartDoF; i++) {
            hw_state_cart_K_x_nom_[i] = K_nom[i];
        }
        RCLCPP_INFO(getLogger(),
            "Robot nominal Cartesian stiffness K_x_nom: "
            "[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
            K_nom[0], K_nom[1], K_nom[2], K_nom[3], K_nom[4], K_nom[5]);

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
    cartesian_controller_running_ = false;

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

        // Read current TCP pose for state interface
        const auto& tcp = robot_->states().tcp_pose;
        for (std::size_t i = 0; i < kPoseSize; i++) {
            hw_state_cart_pose_[i] = tcp[i];
        }
    }

    return hardware_interface::return_type::OK;
}

void FlexivHardwareInterface::check_cartesian_dirty_flags()
{
    // Check stiffness + damping ratio (always sent together)
    cart_stiffness_dirty_ = false;
    for (std::size_t i = 0; i < kCartDoF; i++) {
        if (!approx_eq(hw_cmd_cart_stiffness_[i], prev_cart_stiffness_[i]) ||
            !approx_eq(hw_cmd_cart_damping_ratio_[i], prev_cart_damping_ratio_[i])) {
            cart_stiffness_dirty_ = true;
            break;
        }
    }

    // Check max wrench
    cart_max_wrench_dirty_ = false;
    for (std::size_t i = 0; i < kCartDoF; i++) {
        if (!approx_eq(hw_cmd_cart_max_wrench_[i], prev_cart_max_wrench_[i])) {
            cart_max_wrench_dirty_ = true;
            break;
        }
    }

    // Check force control axis (Phase 2)
    cart_force_ctrl_axis_dirty_ = false;
    for (std::size_t i = 0; i < kCartDoF; i++) {
        if (!approx_eq(hw_cmd_cart_force_ctrl_axis_[i], prev_cart_force_ctrl_axis_[i])) {
            cart_force_ctrl_axis_dirty_ = true;
            break;
        }
    }

    // Check nullspace
    cart_nullspace_dirty_ = false;
    for (std::size_t i = 0; i < kJointDoF; i++) {
        if (!approx_eq(hw_cmd_cart_nullspace_q_[i], prev_cart_nullspace_q_[i]) &&
            !std::isnan(hw_cmd_cart_nullspace_q_[i])) {
            cart_nullspace_dirty_ = true;
            break;
        }
    }
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // ── Joint-level control modes ───────────────────────────────────

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

    // ── Cartesian motion-force control mode ─────────────────────────
    if (cartesian_controller_running_
        && robot_->mode() == flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {

        // Check if pose command is valid (not NaN)
        bool pose_valid = true;
        for (std::size_t i = 0; i < kPoseSize; i++) {
            if (std::isnan(hw_cmd_cart_pose_[i])) {
                pose_valid = false;
                break;
            }
        }

        if (pose_valid) {
            // Build the pose array for StreamCartesianMotionForce
            std::array<double, flexiv::rdk::kPoseSize> target_pose;
            for (std::size_t i = 0; i < kPoseSize; i++) {
                target_pose[i] = hw_cmd_cart_pose_[i];
            }

            // Build wrench array (Phase 2: will be non-zero for force-controlled axes)
            std::array<double, flexiv::rdk::kCartDoF> target_wrench;
            for (std::size_t i = 0; i < kCartDoF; i++) {
                target_wrench[i] = hw_cmd_cart_wrench_[i];
            }

            // RT path: stream pose + wrench every cycle
            robot_->StreamCartesianMotionForce(target_pose, target_wrench);

            // ── Dirty-flag path: blocking calls only when config changes ──
            check_cartesian_dirty_flags();

            if (cart_stiffness_dirty_) {
                // Clamp stiffness to [0, K_x_nom] as required by Flexiv
                std::array<double, flexiv::rdk::kCartDoF> K_x;
                std::array<double, flexiv::rdk::kCartDoF> Z_x;
                for (std::size_t i = 0; i < kCartDoF; i++) {
                    double k_max = hw_state_cart_K_x_nom_[i];
                    double k_cmd = hw_cmd_cart_stiffness_[i];
                    if (k_cmd > k_max) {
                        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                        RCLCPP_WARN_THROTTLE(getLogger(), steady_clock, 2000,
                            "Cartesian stiffness axis %zu: %.1f clamped to K_x_nom %.1f",
                            i, k_cmd, k_max);
                        k_cmd = k_max;
                    }
                    if (k_cmd < 0.0) k_cmd = 0.0;
                    K_x[i] = k_cmd;

                    // Clamp damping ratio to [0.3, 0.8]
                    double z_cmd = hw_cmd_cart_damping_ratio_[i];
                    Z_x[i] = std::clamp(z_cmd, 0.3, 0.8);
                }

                try {
                    robot_->SetCartesianImpedance(K_x, Z_x);
                    RCLCPP_INFO(getLogger(),
                        "SetCartesianImpedance: K=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f] "
                        "Z=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                        K_x[0], K_x[1], K_x[2], K_x[3], K_x[4], K_x[5],
                        Z_x[0], Z_x[1], Z_x[2], Z_x[3], Z_x[4], Z_x[5]);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(getLogger(), "SetCartesianImpedance failed: %s", e.what());
                }

                prev_cart_stiffness_ = hw_cmd_cart_stiffness_;
                prev_cart_damping_ratio_ = hw_cmd_cart_damping_ratio_;
            }

            if (cart_max_wrench_dirty_) {
                std::array<double, flexiv::rdk::kCartDoF> max_wrench;
                for (std::size_t i = 0; i < kCartDoF; i++) {
                    max_wrench[i] = hw_cmd_cart_max_wrench_[i];
                    if (max_wrench[i] < 0.0) {
                        max_wrench[i] = std::numeric_limits<double>::infinity();
                    }
                }

                try {
                    robot_->SetMaxContactWrench(max_wrench);
                    RCLCPP_INFO(getLogger(),
                        "SetMaxContactWrench: [%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
                        max_wrench[0], max_wrench[1], max_wrench[2],
                        max_wrench[3], max_wrench[4], max_wrench[5]);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(getLogger(), "SetMaxContactWrench failed: %s", e.what());
                }

                prev_cart_max_wrench_ = hw_cmd_cart_max_wrench_;
            }

            if (cart_force_ctrl_axis_dirty_) {
                std::array<bool, flexiv::rdk::kCartDoF> enabled;
                for (std::size_t i = 0; i < kCartDoF; i++) {
                    enabled[i] = hw_cmd_cart_force_ctrl_axis_[i] > 0.5;
                }

                try {
                    robot_->SetForceControlAxis(enabled);
                    RCLCPP_INFO(getLogger(),
                        "SetForceControlAxis: [%d,%d,%d,%d,%d,%d]",
                        enabled[0], enabled[1], enabled[2],
                        enabled[3], enabled[4], enabled[5]);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(getLogger(), "SetForceControlAxis failed: %s", e.what());
                }

                prev_cart_force_ctrl_axis_ = hw_cmd_cart_force_ctrl_axis_;
            }

            if (cart_nullspace_dirty_) {
                std::vector<double> ref_q(kJointDoF);
                for (std::size_t i = 0; i < kJointDoF; i++) {
                    ref_q[i] = hw_cmd_cart_nullspace_q_[i];
                }

                try {
                    robot_->SetNullSpacePosture(ref_q);
                    RCLCPP_INFO(getLogger(),
                        "SetNullSpacePosture: [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                        ref_q[0], ref_q[1], ref_q[2], ref_q[3],
                        ref_q[4], ref_q[5], ref_q[6]);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(getLogger(), "SetNullSpacePosture failed: %s", e.what());
                }

                prev_cart_nullspace_q_ = hw_cmd_cart_nullspace_q_;
            }
        }
    }

    // ── Digital output (runs regardless of control mode) ────────────
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

    // Check if any Cartesian (tcp/*) interfaces are being started
    bool starting_cartesian = false;
    for (const auto& key : start_interfaces) {
        if (key.find(kCartPrefix + "/") == 0) {
            starting_cartesian = true;
            break;
        }
    }

    // Check if any Cartesian interfaces are being stopped
    bool stopping_cartesian = false;
    for (const auto& key : stop_interfaces) {
        if (key.find(kCartPrefix + "/") == 0) {
            stopping_cartesian = true;
            break;
        }
    }

    // If starting Cartesian mode, treat this as a Cartesian-only switch.
    // The FlexivCartesianController also claims joint effort+position
    // interfaces as an exclusion lock (so no other controller can activate
    // at the same time), but the actual RDK mode is RT_CARTESIAN_MOTION_FORCE.
    // Joint interfaces in the start list are ignored when tcp/* is present.
    if (starting_cartesian) {
        start_modes_.push_back("cartesian");
    }

    if (stopping_cartesian) {
        stop_modes_.push_back(StoppingInterface::STOP_CARTESIAN);
    }

    // Starting joint interfaces.
    // Controllers may claim multiple interface types for mutual exclusion
    // (e.g. CartesianController claims effort+position so that position-based
    // controllers can't run simultaneously). We determine the primary mode
    // by counting which type has the most claims — the primary mode is the
    // one that covers all joints, the rest are exclusion locks.
    if (!starting_cartesian) {
        size_t n_pos = 0, n_vel = 0, n_eff = 0;
        for (const auto& key : start_interfaces) {
            for (std::size_t i = 0; i < info_.joints.size(); i++) {
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                    n_pos++;
                }
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                    n_vel++;
                }
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                    n_eff++;
                }
            }
        }

        // Determine primary mode: the interface type that has all joints claimed.
        // When a controller claims multiple types (e.g. effort+position), we pick
        // effort > position > velocity as the primary mode to use with the RDK,
        // since effort-based controllers need RT_JOINT_TORQUE while position-based
        // need NRT_JOINT_POSITION/IMPEDANCE.
        std::string primary_mode;
        if (n_eff == info_.joints.size()) {
            primary_mode = hardware_interface::HW_IF_EFFORT;
        } else if (n_pos == info_.joints.size()) {
            primary_mode = hardware_interface::HW_IF_POSITION;
        } else if (n_vel == info_.joints.size()) {
            primary_mode = hardware_interface::HW_IF_VELOCITY;
        }

        if (!primary_mode.empty()) {
            for (std::size_t i = 0; i < info_.joints.size(); i++) {
                start_modes_.push_back(primary_mode);
            }
        } else if (n_pos > 0 || n_vel > 0 || n_eff > 0) {
            // Some joints have interfaces but no single type covers all joints
            RCLCPP_ERROR(getLogger(),
                "Not all joints have the same command interface type "
                "(pos=%zu, vel=%zu, eff=%zu, expected %zu)",
                n_pos, n_vel, n_eff, info_.joints.size());
            return hardware_interface::return_type::ERROR;
        }
    }

    // Stop motion on all relevant joints that are stopping.
    // Same logic as start: determine primary mode from the stop list,
    // ignoring exclusion-lock interfaces.
    if (!stopping_cartesian) {
        size_t n_pos = 0, n_vel = 0, n_eff = 0;
        for (const auto& key : stop_interfaces) {
            for (std::size_t i = 0; i < info_.joints.size(); i++) {
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                    n_pos++;
                }
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                    n_vel++;
                }
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                    n_eff++;
                }
            }
        }

        // Determine which mode to stop (effort > position > velocity)
        if (n_eff == info_.joints.size()) {
            for (std::size_t i = 0; i < info_.joints.size(); i++)
                stop_modes_.push_back(StoppingInterface::STOP_EFFORT);
        } else if (n_pos == info_.joints.size()) {
            for (std::size_t i = 0; i < info_.joints.size(); i++)
                stop_modes_.push_back(StoppingInterface::STOP_POSITION);
        } else if (n_vel == info_.joints.size()) {
            for (std::size_t i = 0; i < info_.joints.size(); i++)
                stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
        }
    }

    controllers_initialized_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    bool starting_new_mode = (start_modes_.size() != 0);

    // ── Handle stopping ─────────────────────────────────────────────

    // Check if stopping Cartesian
    bool stopping_cartesian = false;
    for (const auto& m : stop_modes_) {
        if (m == StoppingInterface::STOP_CARTESIAN) {
            stopping_cartesian = true;
            break;
        }
    }

    if (stopping_cartesian) {
        cartesian_controller_running_ = false;
        if (!starting_new_mode) {
            robot_->Stop();
        }
    } else if (stop_modes_.size() != 0
        && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION)
               != stop_modes_.end()) {
        position_controller_running_ = false;
        if (!starting_new_mode) {
            robot_->Stop();
        }
    } else if (stop_modes_.size() != 0
               && std::find(
                      stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY)
                      != stop_modes_.end()) {
        velocity_controller_running_ = false;
        if (!starting_new_mode) {
            robot_->Stop();
        }
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_EFFORT)
                      != stop_modes_.end()) {
        torque_controller_running_ = false;
        if (!starting_new_mode) {
            robot_->Stop();
        }
    }

    // ── Handle starting ─────────────────────────────────────────────

    // Check if starting Cartesian mode
    bool starting_cartesian = false;
    for (const auto& m : start_modes_) {
        if (m == "cartesian") {
            starting_cartesian = true;
            break;
        }
    }

    if (starting_cartesian) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;
        torque_controller_running_ = false;

        // Initialize Cartesian commands to NaN (hold-at-current until controller writes)
        hw_cmd_cart_pose_.fill(std::numeric_limits<double>::quiet_NaN());
        hw_cmd_cart_wrench_.fill(0.0);
        hw_cmd_cart_force_ctrl_axis_.fill(0.0);

        // Reset dirty flag tracking so initial config is applied
        prev_cart_stiffness_.fill(std::numeric_limits<double>::quiet_NaN());
        prev_cart_damping_ratio_.fill(std::numeric_limits<double>::quiet_NaN());
        prev_cart_max_wrench_.fill(std::numeric_limits<double>::quiet_NaN());
        prev_cart_force_ctrl_axis_.fill(std::numeric_limits<double>::quiet_NaN());
        prev_cart_nullspace_q_.assign(kJointDoF, std::numeric_limits<double>::quiet_NaN());

        // Switch to RT Cartesian motion-force mode
        RCLCPP_INFO(getLogger(), "Switching to RT_CARTESIAN_MOTION_FORCE mode");
        robot_->SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Configure initial settings (non-RT context — blocking is fine here)
        // All axes motion-controlled (Phase 1)
        robot_->SetForceControlAxis(
            std::array<bool, flexiv::rdk::kCartDoF>{false, false, false, false, false, false});

        RCLCPP_INFO(getLogger(), "RT_CARTESIAN_MOTION_FORCE mode active");

        cartesian_controller_running_ = true;

    } else if (start_modes_.size() != 0
        && std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION)
               != start_modes_.end()) {
        velocity_controller_running_ = false;
        torque_controller_running_ = false;
        cartesian_controller_running_ = false;

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
        cartesian_controller_running_ = false;

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
        cartesian_controller_running_ = false;

        // Hold joints when starting joint torque controller before user
        // commands arrives
        std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint torque mode
        robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

        torque_controller_running_ = true;
    }

    start_modes_.clear();
    stop_modes_.clear();

    return hardware_interface::return_type::OK;
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivHardwareInterface, hardware_interface::SystemInterface)
