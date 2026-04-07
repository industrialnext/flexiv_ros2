#include "flexiv_hardware/flexiv_executor.hpp"

namespace flexiv_hardware {

using namespace std::chrono_literals;

FlexivExecutor::FlexivExecutor()
    : spin_thread_([this] { spin(); }) {
  // Wait until the executor is actually spinning
  while (!this->spinning) {
    std::this_thread::sleep_for(100ms);
  }
}

FlexivExecutor::~FlexivExecutor() {
  if (this->spinning) {
    this->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

}  // namespace flexiv_hardware
