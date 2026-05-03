#include "spot_micro_motion_cmd.h"
#include <iostream>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpotMicroMotionCmd>();

  rclcpp::Rate rate(1.0 / node->getNodeConfig().dt);

  if (node->publishServoConfiguration()) {
    bool debug_mode = node->getNodeConfig().debug_mode;
    rclcpp::Time begin;

    while (rclcpp::ok()) {
      if (debug_mode) {
        begin = node->now();
      }

      node->runOnce();
      rclcpp::spin_some(node);
      rate.sleep();

      if (debug_mode) {
        std::cout << (node->now() - begin).seconds() << std::endl;
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}
