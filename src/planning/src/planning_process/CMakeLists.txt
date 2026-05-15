cmake_minimum_required(VERSION 3.8)
project(planning_process)
#设置依赖
set(DEPENDS
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "base_msgs"
  "tf2"
)

#添加节点
add_executable(planning_node
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_process.cpp
)

set(target
  planning_node
)
#添加依赖
ament_target_dependencies(${target}
  ${DEPENDS}
)

#target_link_libraries(${target}

#)
#安装动态库、节点和相关目录
install(TARGETS
  ${target}
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)