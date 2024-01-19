cmake_minimum_required(VERSION 3.22)
project(px4_ros_com)

# include(FetchContent)

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
# add Altro
# FetchContent_Declare(
#         Altro
#         # SOURCE_DIR /home/REXOperator/support_files/altro
#         GIT_REPOSITORY https://github.com/PrakritTyagi/ALTRO_.git
#         GIT_TAG 624303c806236ac0be66fa3ac1d0f1a89d8bf71a
# )
# FetchContent_MakeAvailable(Altro)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mocap_msgs REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Add quaternion_math lib
add_library(quaternion_math src/lib/quaternion_math.cpp)
ament_target_dependencies(quaternion_math Eigen3)
target_include_directories(quaternion_math PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Add quadcopter_model lib
add_library(quadcopter_model src/lib/quadcopter_model.cpp)
ament_target_dependencies(quadcopter_model Eigen3)
target_link_libraries(quadcopter_model PRIVATE quaternion_math)
target_include_directories(quadcopter_model PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Add mpc_utils lib
# add_library(mpc_utils src/lib/mpc_utils.cpp)
# target_link_libraries(mpc_utils altro::altro fmt::fmt Eigen3::Eigen)
# target_include_directories(mpc_utils PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
# 	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )
# Add ricatti lib
add_library(ricatti src/lib/ricatti.cpp)
ament_target_dependencies(ricatti Eigen3)
# target_link_libraries(ricatti PRIVATE quaternion_math)
target_include_directories(ricatti PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# examples/listeners/sensor_combined_listener
add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

# examples/listeners/vehicle_gps_position_listener
add_executable(vehicle_gps_position_listener src/examples/listeners/vehicle_gps_position_listener.cpp)
ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)
install(TARGETS vehicle_gps_position_listener DESTINATION lib/${PROJECT_NAME})

# examples/advertisers/debug_vect_advertiser
add_executable(debug_vect_advertiser src/examples/advertisers/debug_vect_advertiser.cpp)
ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)
install(TARGETS debug_vect_advertiser DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control
add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# example/advertisers/actuator_publisher
add_executable(actuator_publisher src/examples/advertisers/actuator_publisher.cpp)
ament_target_dependencies(actuator_publisher rclcpp px4_msgs)
install(TARGETS actuator_publisher DESTINATION lib/${PROJECT_NAME})

# examples/listeners/optitrack_listener
add_executable(optitrack_listener src/examples/listeners/optitrack_listener.cpp)
ament_target_dependencies(optitrack_listener rclcpp px4_msgs mocap_msgs)
install(TARGETS optitrack_listener DESTINATION lib/${PROJECT_NAME})

# examples/lqr/lqr_sim
add_executable(lqr_sim src/examples/lqr/lqr_sim.cpp)
target_link_libraries(lqr_sim quaternion_math quadcopter_model ricatti)
ament_target_dependencies(lqr_sim rclcpp px4_msgs Eigen3)
install(TARGETS lqr_sim DESTINATION lib/${PROJECT_NAME})

# examples/lqr/lqr
add_executable(lqr src/examples/lqr/lqr.cpp)
target_link_libraries(lqr quaternion_math quadcopter_model ricatti)
ament_target_dependencies(lqr rclcpp px4_msgs Eigen3)
install(TARGETS lqr DESTINATION lib/${PROJECT_NAME})

# examples/lqr/lqr_sim
add_executable(lqr_circle_sim src/examples/lqr/lqr_circle_sim.cpp)
target_link_libraries(lqr_circle_sim quaternion_math quadcopter_model ricatti)
ament_target_dependencies(lqr_circle_sim rclcpp px4_msgs Eigen3)
install(TARGETS lqr_circle_sim DESTINATION lib/${PROJECT_NAME})

# examples/mpc/mpc_sim
# add_executable(mpc_sim src/examples/mpc/mpc_sim.cpp)
# target_link_libraries(mpc_sim altro::altro fmt::fmt Eigen3::Eigen quaternion_math quadcopter_model mpc_utils ricatti)
# ament_target_dependencies(mpc_sim rclcpp px4_msgs Eigen3)
# install(TARGETS mpc_sim DESTINATION lib/${PROJECT_NAME})

# examples/FrequencyCalculator
add_executable(FrequencyCalculator src/examples/FrequencyCalculator.cpp)
ament_target_dependencies(FrequencyCalculator rclcpp px4_msgs)
install(TARGETS FrequencyCalculator DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs mocap_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

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

ament_package()
