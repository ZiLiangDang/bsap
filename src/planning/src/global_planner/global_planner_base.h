#ifndef GLOBAL_PLANNER_BASE_H_
#define GLOBAL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "config_reader.h"

namespace Planning
{
    class GlobalPlannerBase // 全局路径规划器基类
    {
    public:
    };
} // namespace Planning
#endif // GLOBAL_PLANNER_BASE_H_
