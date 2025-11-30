cmake_minimum_required(VERSION 3.22)
project(mppi)

# 设置依赖，放到一个自定义变量中
set(MPPI_DEPENDS
  "Eigen3"
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# 添加动态库
add_library(mppi SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/mppi.cpp
)
# 让动态库关联上依赖
ament_target_dependencies(mppi
  ${MPPI_DEPENDS}
)
target_link_libraries(mppi
  Eigen3::Eigen
  mppi_config_reader
)
# 安装动态库、节点和相关目录
install(TARGETS
  mppi
  LIBRARY DESTINATION lib
)

# 添加节点
add_executable(tracking_node
  ${CMAKE_CURRENT_SOURCE_DIR}/demo_tracking.cpp
)
ament_target_dependencies(tracking_node
  ${MPPI_DEPENDS}
)
target_link_libraries(tracking_node
  # Eigen3::Eigen
  # mppi_config_reader
  mppi
)
# 安装节点
install(TARGETS
  tracking_node
  DESTINATION lib/${PROJECT_NAME} # 这里需要是外面的PACKAGE_NAME，如果是PROJECT_NAME，则会安装到main目录下
)
