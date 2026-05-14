#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class PlanningProcess : public rclcpp::Node
    {
    public:
        PlanningProcess() : Node("planning_node")
        {
        }

        bool process(){
            return true;
        } // 总流程
    private:
    };
}  // namespace Planning
#endif  // PLANNING_PROCESS_H_

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PlanningProcess>();
    RCLCPP_INFO(rclcpp::get_logger("planning_node"), "Planning start");
    if (!node->process())
    {
        RCLCPP_ERROR(rclcpp::get_logger("planning_node"), "Planning failed");
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}