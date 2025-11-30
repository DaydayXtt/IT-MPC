cmake_minimum_required(VERSION 3.22)
project(common)

# 设置依赖，放到一个自定义变量中
set(CONFIG_DEPENDS
  "rclcpp"
  "yaml-cpp"
  "ament_index_cpp"
)

# 添加动态库
add_library(mppi_config_reader SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/config_reader/config_reader.cpp
)

# 让动态库关联上依赖
ament_target_dependencies(mppi_config_reader
  ${CONFIG_DEPENDS}
)

target_link_libraries(mppi_config_reader
  yaml-cpp
)

# 安装动态库和相关目录
install(TARGETS
  mppi_config_reader
  LIBRARY DESTINATION lib
)