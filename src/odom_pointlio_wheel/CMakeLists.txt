cmake_minimum_required(VERSION 3.8...3.28)   # 修复 Deprecation Warning
project(odom_pointlio_wheel)#<-----------包名

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ==================== 库 ====================
add_library(odom_fusion SHARED
  src/odom_fusion.cpp
)

target_include_directories(odom_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include># ← 构建时使用这个路径
  $<INSTALL_INTERFACE:include># ← 安装后（给其他包使用）使用这个路径
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(odom_fusion
  ${dependencies}
)

# ==================== 可执行文件 ====================
add_executable(odom_pointlio_wheel_node#<----------节点名
  src/main.cpp
)

target_include_directories(odom_pointlio_wheel_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(odom_pointlio_wheel_node
  ${dependencies}
)

# 必须链接你自己的库（解决 undefined reference 的关键）
target_link_libraries(odom_pointlio_wheel_node
  odom_fusion
)

# ==================== 安装 ====================
install(TARGETS 
  odom_pointlio_wheel_node
  odom_fusion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 可选：导出头文件（如果其他包要使用你的库）
# install(DIRECTORY include/
#   DESTINATION include/
# )

ament_package()