#include "pointlio_bridge/pointlio_lego_processor.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointLIOLegoProcessor>());
    rclcpp::shutdown();
    return 0;
}