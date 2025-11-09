cmake_minimum_required(VERSION 3.22)
project(bicycle)

set(BICYCLE_DEPENDS
    "rclcpp"
    "geometry_msgs"
    "nav_msgs"
    "std_msgs"
    "Eigen3"
    "tf2"
    "tf2_ros"
    "tf2_geometry_msgs"
    # "Eigen3"
)

# 添加节点
add_executable(odom_bicycle
    ${CMAKE_CURRENT_SOURCE_DIR}/bicycle.cpp
)

# 让动态库关联上依赖
ament_target_dependencies(odom_bicycle
    ${BICYCLE_DEPENDS}
)
target_link_libraries(odom_bicycle
    Eigen3::Eigen
    #   osqp::osqp
    #   OsqpEigen::OsqpEigen
    #   config_reader
)

# 安装节点
install(TARGETS
    odom_bicycle
    DESTINATION lib/${AGENTS_CPP} # 这里需要是外面的PACKAGE_NAME，如果是PROJECT_NAME，则会安装到main目录下
)
