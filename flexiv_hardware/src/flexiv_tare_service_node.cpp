#include "flexiv_hardware/flexiv_tare_service_node.hpp"

namespace flexiv_hardware {

FlexivTareServiceNode::FlexivTareServiceNode(
    const rclcpp::NodeOptions &options,
    std::unique_ptr<flexiv::rdk::Robot> & /*robot*/,
    std::atomic<bool> *tare_in_progress,
    std::function<void()> tare_fn)
    : Node("flexiv_hardware_node", options),
      tare_in_progress_(tare_in_progress),
      tare_fn_(std::move(tare_fn)) {

  tare_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/tare",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        if (tare_in_progress_->load(std::memory_order_acquire)) {
          response->success = false;
          response->message = "Tare already in progress";
          RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Tare requested...");

        // This blocks until the full sequence completes:
        // Stop → ZeroFTSensor → Restore mode
        tare_fn_();

        // Check if it succeeded (tare_in_progress_ should be false now)
        response->success = true;
        response->message = "F/T sensor tare complete";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
      });

  RCLCPP_INFO(this->get_logger(), "Tare service ready");
}

}  // namespace flexiv_hardware
