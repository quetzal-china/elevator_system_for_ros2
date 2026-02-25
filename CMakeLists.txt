cmake_minimum_required(VERSION 3.5)
project(elevator_system)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Elevator.action"
)

# 导出依赖和包含目录
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 添加服务端可执行文件
add_executable(elevator_server src/elevator_server.cpp)
# 添加 include 目录
target_include_directories(elevator_server PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# 链接依赖库
ament_target_dependencies(elevator_server
  rclcpp
  rclcpp_action
)
# 链接自动生成的 Action 接口库
target_link_libraries(elevator_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
)
# 安装可执行文件
install(TARGETS elevator_server
  DESTINATION lib/${PROJECT_NAME})

# 客户端可执行文件
add_executable(elevator_client src/elevator_client.cpp)
# 添加 include 目录
target_include_directories(elevator_client PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# 链接依赖库
ament_target_dependencies(elevator_client
  rclcpp
  rclcpp_action
)
# 链接自动生成的 Action 接口库
target_link_libraries(elevator_client
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
)
# 安装可执行文件
install(TARGETS elevator_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
