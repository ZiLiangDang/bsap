#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared</* namespace_name::ClassName */>())
    rclcpp::shutdown();
    return 0;
}