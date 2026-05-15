#include "planning_process.h"

namespace Planning
{
     PlanningProcess::PlanningProcess(): Node("planning_node")
        {
            RCLCPP_INFO(this->get_logger(), "Planning_node  ceated");
        }

        bool PlanningProcess::process()
        {
            return true;
        }
}
