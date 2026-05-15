#include "planning_process.h"

namespace Planning
{
     PlanningProcess::PlanningProcess(): Node("planning_node")//规划总流程
        {
            RCLCPP_INFO(this->get_logger(), "Planning_node  ceated");
        }

        bool PlanningProcess::process()//总流程
        {
            return true;
        }
}
