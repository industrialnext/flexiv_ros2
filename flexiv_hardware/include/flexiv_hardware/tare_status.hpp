#pragma once

/**
 * @file tare_status.hpp
 * @brief Shared constants for the F/T sensor tare command/state interfaces.
 *
 * The Flexiv hardware interface exposes a tare/request command interface and
 * a tare/status state interface.  Both are doubles (the only type ros2_control
 * supports).  These constants give symbolic names to the status values so
 * that the hardware interface and any controller using tare don't rely on
 * magic numbers.
 */

namespace flexiv_hardware {
namespace TareStatus {

/// Ready to accept a new tare request.
constexpr double kIdle       = 0.0;

/// Tare sequence is running (mode switch + ZeroFTSensor primitive).
constexpr double kInProgress = 1.0;

/// Tare completed successfully; previous RT mode has been restored.
constexpr double kSuccess    = 2.0;

/// Tare failed (exception during mode switch or primitive execution).
constexpr double kFailed     = 3.0;

}  // namespace TareStatus
}  // namespace flexiv_hardware
