cmake_minimum_required(VERSION 3.8)
project(jtop_orinnx_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cpu/CpuFrequency.msg"
  "msg/Cpu/IdleState.msg"
  "msg/Cpu/Cpu.msg"
  "msg/Gpu/GpuFrequency.msg"
  "msg/Gpu/GpuStatus.msg"
  "msg/Gpu/Gpu.msg"
  "msg/Temp/Temp.msg"
  "msg/Temp/TempList.msg"
  "msg/Mem/Ram.msg"
  "msg/Mem/Zram.msg"
  "msg/Mem/Swap.msg"
  "msg/Mem/Emc.msg"
  "msg/Mem/Mem.msg"
  "msg/Jtop.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
