cmake_minimum_required(VERSION 3.22)
project(agents_cpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CMAKE_COMPILER_IS_GNUCXX已经废弃，改用 CMAKE_CXX_COMPILER_ID MATCHES "GNU"
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置预编译指令，生成compile_commands.json
# 然后让c_cpp_properties.json读取这个文件，防止预编译找不到cmake中定义的宏而报错
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# 设置包名，让子目录节点安装时使用
set(AGENTS_CPP ${PROJECT_NAME})


# 包含头文件目录
include_directories(
  /usr/local/include/eigen-3.4.0
  # src/common
  # src/common/config_reader
  src/bicycle
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(src/bicycle)

# 安装辅助目录
install(DIRECTORY # 作用：把这里列的目录全部复制到install文件夹下
  urdf launch rviz config   
  DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/odom_to_tf.py
  src/ign_pose_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
