#include "odom_pointlio_wheel/odom_fusion.h"

// ── main ─────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFusionNode>());
  rclcpp::shutdown();
  return 0;
}