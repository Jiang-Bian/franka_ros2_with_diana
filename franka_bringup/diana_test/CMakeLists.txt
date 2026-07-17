#
# This CMakeLists is included from
# franka_bringup/CMakeLists.txt
#

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Franka 0.18.2 REQUIRED)

set(DIANA_TEST_COMMON_SRC
    src/compatibility_report.cpp
    src/action_helper.cpp
)

set(DIANA_TEST_COMMON_INCLUDE
    include
)

###########################################################
#
# Diana7 libfranka Compatibility Test
#
###########################################################

add_executable(
    diana7_libfranka_compatibility_test
    diana7_libfranka_compatibility_test.cpp
    ${DIANA_TEST_COMMON_SRC}
)

target_include_directories(
    diana7_libfranka_compatibility_test
    PRIVATE
    ${DIANA_TEST_COMMON_INCLUDE}
)

ament_target_dependencies(
    diana7_libfranka_compatibility_test
    Franka
    rclcpp
)

###########################################################
#
# Diana7 JointTrajectory Validation
#
###########################################################

add_executable(
    diana7_joint_trajectory_test
    diana7_joint_trajectory_test.cpp
    ${DIANA_TEST_COMMON_SRC}
)

target_include_directories(
    diana7_joint_trajectory_test
    PRIVATE
    ${DIANA_TEST_COMMON_INCLUDE}
)

ament_target_dependencies(
    diana7_joint_trajectory_test
    rclcpp
    rclcpp_action
    sensor_msgs
    trajectory_msgs
    control_msgs
)

###########################################################
#
# Install
#
###########################################################

install(
    TARGETS
    diana7_libfranka_compatibility_test
    diana7_joint_trajectory_test
    DESTINATION
    lib/${PROJECT_NAME}
)
