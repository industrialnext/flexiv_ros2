#pragma once

/**
 * @file flexiv_executor.hpp
 * @brief Self-spinning multi-threaded executor for Flexiv hardware service nodes.
 *
 * Identical pattern to Franka's FrankaExecutor: spawns a background thread
 * that runs spin(), and joins on destruction.
 */

#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace flexiv_hardware {

class FlexivExecutor : public rclcpp::executors::MultiThreadedExecutor {
public:
  FlexivExecutor();
  ~FlexivExecutor() override;

  FlexivExecutor(const FlexivExecutor &) = delete;
  FlexivExecutor &operator=(const FlexivExecutor &) = delete;
  FlexivExecutor(FlexivExecutor &&) = delete;
  FlexivExecutor &operator=(FlexivExecutor &&) = delete;

private:
  std::thread spin_thread_;
};

}  // namespace flexiv_hardware
