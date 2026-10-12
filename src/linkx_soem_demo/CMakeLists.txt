cmake_minimum_required(VERSION 3.10)
project(linkx_soem_demo C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ============================================================================
# 复用源列表：SOEM 底层（主控依赖）
# ============================================================================
set(SOEM_SRC
    src/vehicle_control/1_Middleware/soem/ec_base.c
    src/vehicle_control/1_Middleware/soem/ec_coe.c
    src/vehicle_control/1_Middleware/soem/ec_config.c
    src/vehicle_control/1_Middleware/soem/ec_dc.c
    src/vehicle_control/1_Middleware/soem/ec_eoe.c
    src/vehicle_control/1_Middleware/soem/ec_foe.c
    src/vehicle_control/1_Middleware/soem/ec_main.c
    src/vehicle_control/1_Middleware/soem/ec_print.c
    src/vehicle_control/1_Middleware/soem/ec_soe.c
    src/vehicle_control/1_Middleware/soem/nicdrv.c
    src/vehicle_control/1_Middleware/soem/osal.c
    src/vehicle_control/1_Middleware/soem/oshw.c
)

# 复用源列表：LinkX + EtherCAT manager（主控 CAN 桥依赖）
set(LINKX_BRIDGE_SRC
    src/vehicle_control/1_Middleware/linkx/linkx.c
    src/vehicle_control/2_Device/ethercat/ecat_manager/ecat_manager.c
    src/vehicle_control/2_Device/ethercat/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/2_Device/rt_timing/rt_timing.c
)

set(PROJECT_INCLUDE_DIRS
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/diagnostics/can_terminal
    include/linkx_soem_demo/diagnostics/chassis_trace
    include/linkx_soem_demo/diagnostics/task_terminal
    include/linkx_soem_demo/diagnostics/timing_terminal
    include/linkx_soem_demo/tof
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/1_Middleware
    include/linkx_soem_demo/vehicle_control/1_Middleware/algorithm
    include/linkx_soem_demo/vehicle_control/1_Middleware/linkx
    include/linkx_soem_demo/vehicle_control/1_Middleware/math
    include/linkx_soem_demo/vehicle_control/1_Middleware/soem
    include/linkx_soem_demo/vehicle_control/2_Device
    include/linkx_soem_demo/vehicle_control/2_Device/ethercat
    include/linkx_soem_demo/vehicle_control/2_Device/ethercat/ecat_io_thread
    include/linkx_soem_demo/vehicle_control/2_Device/ethercat/ecat_manager
    include/linkx_soem_demo/vehicle_control/2_Device/ethercat/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/2_Device/motor
    include/linkx_soem_demo/vehicle_control/2_Device/rt_timing
    include/linkx_soem_demo/vehicle_control/3_Chariot
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis/omni_wheel
    include/linkx_soem_demo/vehicle_control/3_Chariot/imu_heading_hold
    include/linkx_soem_demo/vehicle_control/3_Chariot/lift
    include/linkx_soem_demo/vehicle_control/3_Chariot/gripper
    include/linkx_soem_demo/vehicle_control/4_Interaction/robot
    include/linkx_soem_demo/vehicle_control/5_Task/task
)

# ============================================================================
# 1) 主控可执行：vehicle_control（SOEM/EtherCAT 主站 + LinkX-4C CAN 桥 + r2 整车控制）
# ============================================================================

set(VEHICLE_CONTROL_SRC
    src/vehicle_control/main.cpp
    src/vehicle_control/5_Task/task/task.cpp
    src/vehicle_control/4_Interaction/robot/robot.cpp
    src/diagnostics/chassis_trace/chassis_trace_logger.cpp
    src/diagnostics/can_terminal/can_terminal_printer.cpp
    src/diagnostics/task_terminal/task_terminal_diagnostics.cpp
    src/diagnostics/timing_terminal/timing_terminal_printer.cpp
    src/tof/tfmini_s_range_publisher.cpp

    # 3_Chariot —— 全向轮工况
    src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
    src/vehicle_control/3_Chariot/imu_heading_hold/imu_heading_hold.cpp
    src/vehicle_control/3_Chariot/lift/crt_lift.cpp
    src/vehicle_control/3_Chariot/gripper/crt_gripper.cpp

    # 2_Device
    src/vehicle_control/2_Device/motor/dvc_motor_dm.cpp
    src/vehicle_control/2_Device/ethercat/ecat_io_thread/ecat_io_thread.cpp
    ${LINKX_BRIDGE_SRC}

    # 1_Middleware
    src/vehicle_control/1_Middleware/algorithm/alg_pid.cpp
    src/vehicle_control/1_Middleware/algorithm/ramp.c
    ${SOEM_SRC}
)

add_executable(${PROJECT_NAME} ${VEHICLE_CONTROL_SRC})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    pthread
    rt
    m
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    std_msgs
    std_srvs
    sensor_msgs
    rcl_interfaces
)

# ============================================================================
# 2) 遥控解算节点：joy → /chassis/remote_cmd_vel + /robot_buttons
# ============================================================================
add_executable(remote_node_cpp
    src/remote/ros2/remote_node.cpp
    src/remote/ros2/joystick_mapper.cpp
    src/remote/device/dvc_logF710.cpp
)
target_include_directories(remote_node_cpp PRIVATE ${PROJECT_INCLUDE_DIRS})
ament_target_dependencies(remote_node_cpp
    rclcpp sensor_msgs geometry_msgs std_msgs)

# ============================================================================
# 3) 通用话题转发节点：/cmd_vel → /chassis/cmd_vel；/robot_buttons → /chassis/buttons
# ============================================================================
add_executable(topic_relay_cpp
    src/remote/ros2/topic_relay.cpp
)
ament_target_dependencies(topic_relay_cpp
    rclcpp geometry_msgs std_msgs)

set(LINKX_SOEM_INSTALL_TARGETS
    ${PROJECT_NAME}
    remote_node_cpp
    topic_relay_cpp
)

# ============================================================================
# 4) tools：硬件联调 / 标定 / 诊断工具（源文件存在时自动注册，未落地工具不阻断整包配置）
# ============================================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/lift_3519_readonly_main.cpp")
    add_executable(lift_3519_readonly
        src/tools/lift_3519_readonly_main.cpp
        src/vehicle_control/2_Device/motor/dvc_motor_dm.cpp
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(lift_3519_readonly PRIVATE ${PROJECT_INCLUDE_DIRS})
    target_link_libraries(lift_3519_readonly pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS lift_3519_readonly)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/dm_motor_param_calib_main.cpp")
    add_executable(dm_motor_param_calib
        src/tools/dm_motor_param_calib_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/motor/dvc_motor_dm.cpp
        src/vehicle_control/1_Middleware/algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(dm_motor_param_calib PRIVATE ${PROJECT_INCLUDE_DIRS})
    target_link_libraries(dm_motor_param_calib pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS dm_motor_param_calib)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/omni_motion_record_main.cpp")
    add_executable(omni_motion_record
        src/tools/omni_motion_record_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/motor/dvc_motor_dm.cpp
        src/vehicle_control/1_Middleware/algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(omni_motion_record PRIVATE ${PROJECT_INCLUDE_DIRS})
    target_link_libraries(omni_motion_record pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS omni_motion_record)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/omni_accel_response_main.cpp")
    add_executable(omni_accel_response
        src/tools/omni_accel_response_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/motor/dvc_motor_dm.cpp
        src/vehicle_control/1_Middleware/algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(omni_accel_response PRIVATE ${PROJECT_INCLUDE_DIRS})
    target_link_libraries(omni_accel_response pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS omni_accel_response)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/imu_heading_hold_sweep_main.cpp")
    add_executable(imu_heading_hold_sweep
        src/tools/imu_heading_hold_sweep_main.cpp
    )
    target_include_directories(imu_heading_hold_sweep PRIVATE ${PROJECT_INCLUDE_DIRS})
    ament_target_dependencies(imu_heading_hold_sweep
        rclcpp
        geometry_msgs
        sensor_msgs
        rcl_interfaces
    )
    list(APPEND LINKX_SOEM_INSTALL_TARGETS imu_heading_hold_sweep)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/remote_chassis_yaw_tune_main.cpp")
    add_executable(remote_chassis_yaw_tune
        src/tools/remote_chassis_yaw_tune_main.cpp
    )
    target_include_directories(remote_chassis_yaw_tune PRIVATE ${PROJECT_INCLUDE_DIRS})
    ament_target_dependencies(remote_chassis_yaw_tune
        rclcpp
        geometry_msgs
        sensor_msgs
        rcl_interfaces
    )
    list(APPEND LINKX_SOEM_INSTALL_TARGETS remote_chassis_yaw_tune)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/lift_3519_feedback_main.cpp")
    add_executable(lift_3519_feedback
        src/tools/lift_3519_feedback_main.cpp
        src/vehicle_control/3_Chariot/lift/crt_lift.cpp
        src/vehicle_control/2_Device/motor/dvc_motor_dm.cpp
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(lift_3519_feedback PRIVATE ${PROJECT_INCLUDE_DIRS})
    target_link_libraries(lift_3519_feedback pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS lift_3519_feedback)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/lift_raise_drive_test_main.cpp")
    add_executable(lift_raise_drive_test
        src/tools/lift_raise_drive_test_main.cpp
        src/vehicle_control/3_Chariot/lift/crt_lift.cpp
        src/vehicle_control/2_Device/motor/dvc_motor_dm.cpp
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(lift_raise_drive_test PRIVATE ${PROJECT_INCLUDE_DIRS})
    target_link_libraries(lift_raise_drive_test pthread rt m)
    ament_target_dependencies(lift_raise_drive_test
        rclcpp
        sensor_msgs
    )
    list(APPEND LINKX_SOEM_INSTALL_TARGETS lift_raise_drive_test)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/lift_tof_trigger_test_main.cpp")
    add_executable(lift_tof_trigger_test
        src/tools/lift_tof_trigger_test_main.cpp
        src/vehicle_control/3_Chariot/lift/crt_lift.cpp
        src/vehicle_control/2_Device/motor/dvc_motor_dm.cpp
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(lift_tof_trigger_test PRIVATE ${PROJECT_INCLUDE_DIRS})
    target_link_libraries(lift_tof_trigger_test pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS lift_tof_trigger_test)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/tfmini_s_can_read_main.cpp")
    add_executable(tfmini_s_can_read
        src/tools/tfmini_s_can_read_main.cpp
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(tfmini_s_can_read PRIVATE ${PROJECT_INCLUDE_DIRS})
    target_link_libraries(tfmini_s_can_read pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS tfmini_s_can_read)
endif()

install(TARGETS
    ${LINKX_SOEM_INSTALL_TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
