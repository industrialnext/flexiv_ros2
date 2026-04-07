#pragma once

/**
 * @file flexiv_tare_service_node.hpp
 * @brief Lightweight ROS node hosting the F/T sensor tare service.
 *
 * Created by the FlexivHardwareInterface and spun on a dedicated executor
 * thread so that the service callback runs independently of the
 * controller manager's executor.
 */

#include <atomic>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "flexiv/rdk/robot.hpp"

namespace flexiv_hardware {

class FlexivTareServiceNode : public rclcpp::Node {
public:
  /**
   * @param options  Node options (typically default).
   * @param robot    Reference to the Flexiv RDK robot unique_ptr.
   * @param tare_in_progress  Pointer to the hardware interface's atomic
   *                          flag so read()/write() can skip during tare.
   * @param tare_fn  Callable that performs the actual tare sequence.
   *                 Called synchronously from the service callback.
   */
  FlexivTareServiceNode(const rclcpp::NodeOptions &options,
                        std::unique_ptr<flexiv::rdk::Robot> &robot,
                        std::atomic<bool> *tare_in_progress,
                        std::function<void()> tare_fn);

private:
  std::atomic<bool> *tare_in_progress_;
  std::function<void()> tare_fn_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;
};

}  // namespace flexiv_hardware
