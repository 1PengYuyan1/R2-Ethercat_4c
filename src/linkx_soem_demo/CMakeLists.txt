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
find_package(sensor_msgs REQUIRED)

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
    src/vehicle_control/2_Device/Ethercat/ecat_manager/ecat_manager.c
    src/vehicle_control/2_Device/Ethercat/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/2_Device/rt_timing/rt_timing.c
)

set(EXTRA_TEST_INCLUDE_DIRS
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/1_Middleware
    include/linkx_soem_demo/vehicle_control/1_Middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/1_Middleware/linkx
    include/linkx_soem_demo/vehicle_control/1_Middleware/math
    include/linkx_soem_demo/vehicle_control/1_Middleware/npcap
    include/linkx_soem_demo/vehicle_control/1_Middleware/soem
    include/linkx_soem_demo/vehicle_control/2_Device
    include/linkx_soem_demo/vehicle_control/2_Device/Motor
    include/linkx_soem_demo/vehicle_control/2_Device/OPS
    include/linkx_soem_demo/vehicle_control/2_Device/Ethercat
    include/linkx_soem_demo/vehicle_control/2_Device/Ethercat/ecat_manager
    include/linkx_soem_demo/vehicle_control/2_Device/Ethercat/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/2_Device/rt_timing
    include/linkx_soem_demo/vehicle_control/3_Chariot
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis/steer_wheel
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis/omni_wheel
    include/linkx_soem_demo/vehicle_control/3_Chariot/lift
)

# ============================================================================
# 1) 主控可执行：vehicle_control（SOEM/EtherCAT 主站 + LinkX-4C CAN 桥 + r2 整车控制）
# ============================================================================

set(VEHICLE_CONTROL_SRC
    src/vehicle_control/main.cpp
    src/vehicle_control/5_Task/task/task.cpp
    src/vehicle_control/4_Interaction/robot/robot.cpp

    # 3_Chariot —— 全向轮工况
    src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
    src/vehicle_control/3_Chariot/lift/crt_lift.cpp
    src/vehicle_control/3_Chariot/navigation/crt_navigation.cpp

    # 2_Device
    src/vehicle_control/2_Device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/2_Device/OPS/dvc_ops.cpp
    ${LINKX_BRIDGE_SRC}

    # 1_Middleware
    src/vehicle_control/1_Middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/1_Middleware/Algorithm/ramp.c
    ${SOEM_SRC}
)

add_executable(${PROJECT_NAME} ${VEHICLE_CONTROL_SRC})

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    include/linkx_soem_demo
    include/linkx_soem_demo/vehicle_control
    include/linkx_soem_demo/vehicle_control/1_Middleware
    include/linkx_soem_demo/vehicle_control/1_Middleware/Algorithm
    include/linkx_soem_demo/vehicle_control/1_Middleware/linkx
    include/linkx_soem_demo/vehicle_control/1_Middleware/math
    include/linkx_soem_demo/vehicle_control/1_Middleware/npcap
    include/linkx_soem_demo/vehicle_control/1_Middleware/soem
    include/linkx_soem_demo/vehicle_control/2_Device
    include/linkx_soem_demo/vehicle_control/2_Device/Motor
    include/linkx_soem_demo/vehicle_control/2_Device/OPS
    include/linkx_soem_demo/vehicle_control/2_Device/Ethercat
    include/linkx_soem_demo/vehicle_control/2_Device/Ethercat/ecat_manager
    include/linkx_soem_demo/vehicle_control/2_Device/Ethercat/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/2_Device/rt_timing
    include/linkx_soem_demo/vehicle_control/3_Chariot
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis/steer_wheel
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis/omni_wheel
    include/linkx_soem_demo/vehicle_control/3_Chariot/lift
    include/linkx_soem_demo/vehicle_control/3_Chariot/navigation
    include/linkx_soem_demo/vehicle_control/4_Interaction/robot
    include/linkx_soem_demo/vehicle_control/5_Task/task
)

target_link_libraries(${PROJECT_NAME}
    pcap
    pthread
    rt
    m
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    std_msgs
    sensor_msgs
)

# ============================================================================
# 2) 遥控解算节点：joy → /chassis/remote_cmd_vel + /robot_buttons
# ============================================================================
add_executable(remote_node_cpp
    src/remote/ros2/remote_node.cpp
    src/remote/ros2/joystick_mapper.cpp
    src/remote/device/Remote/dvc_logF710.cpp
)
target_include_directories(remote_node_cpp PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
ament_target_dependencies(remote_node_cpp
    rclcpp sensor_msgs geometry_msgs std_msgs)

# ============================================================================
# 3) 通用话题转发节点：/cmd_vel → /chassis/cmd_vel；/robot_buttons → /chassis/buttons
# ============================================================================
add_executable(stm32_node_cpp
    src/remote/ros2/stm32_node.cpp
)
ament_target_dependencies(stm32_node_cpp
    rclcpp geometry_msgs std_msgs)

set(LINKX_SOEM_INSTALL_TARGETS
    ${PROJECT_NAME}
    remote_node_cpp
    stm32_node_cpp
)

# ============================================================================
# 4) extra_tests：源文件存在时自动注册，避免未落地测试阻断整包配置
# ============================================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/dm_motor_param_calib_main.cpp")
    add_executable(dm_motor_param_calib
        src/extra_tests/dm_motor_param_calib_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/Motor/dvc_motor_dm.cpp
        src/vehicle_control/1_Middleware/Algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/Algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(dm_motor_param_calib PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(dm_motor_param_calib pcap pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS dm_motor_param_calib)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/omni_mit_param_sweep_main.cpp")
    add_executable(omni_mit_param_sweep
        src/extra_tests/omni_mit_param_sweep_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/Motor/dvc_motor_dm.cpp
        src/vehicle_control/1_Middleware/Algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/Algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(omni_mit_param_sweep PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(omni_mit_param_sweep pcap pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS omni_mit_param_sweep)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/omni_motion_record_main.cpp")
    add_executable(omni_motion_record
        src/extra_tests/omni_motion_record_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/Motor/dvc_motor_dm.cpp
        src/vehicle_control/1_Middleware/Algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/Algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(omni_motion_record PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(omni_motion_record pcap pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS omni_motion_record)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/omni_accel_response_main.cpp")
    add_executable(omni_accel_response
        src/extra_tests/omni_accel_response_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/Motor/dvc_motor_dm.cpp
        src/vehicle_control/1_Middleware/Algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/Algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(omni_accel_response PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(omni_accel_response pcap pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS omni_accel_response)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/lift_3519_feedback_main.cpp")
    add_executable(lift_3519_feedback
        src/extra_tests/lift_3519_feedback_main.cpp
        src/vehicle_control/3_Chariot/lift/crt_lift.cpp
        src/vehicle_control/2_Device/Motor/dvc_motor_dm.cpp
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(lift_3519_feedback PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(lift_3519_feedback pcap pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS lift_3519_feedback)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/ops_yaw_pd_sweep_main.cpp")
    add_executable(ops_yaw_pd_sweep
        src/extra_tests/ops_yaw_pd_sweep_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/Motor/dvc_motor_dm.cpp
        src/vehicle_control/2_Device/OPS/dvc_ops.cpp
        src/vehicle_control/1_Middleware/Algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/Algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(ops_yaw_pd_sweep PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(ops_yaw_pd_sweep pcap pthread rt m)
    ament_target_dependencies(ops_yaw_pd_sweep
        rclcpp
        geometry_msgs
        std_msgs
    )
    list(APPEND LINKX_SOEM_INSTALL_TARGETS ops_yaw_pd_sweep)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/ops_terminal_probe_main.cpp")
    add_executable(ops_terminal_probe
        src/extra_tests/ops_terminal_probe_main.cpp
        src/vehicle_control/2_Device/OPS/dvc_ops.cpp
    )
    target_include_directories(ops_terminal_probe PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    ament_target_dependencies(ops_terminal_probe
        rclcpp
        geometry_msgs
        std_msgs
    )
    list(APPEND LINKX_SOEM_INSTALL_TARGETS ops_terminal_probe)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/ops_can_probe_main.cpp")
    add_executable(ops_can_probe
        src/extra_tests/ops_can_probe_main.cpp
        src/vehicle_control/2_Device/OPS/dvc_ops.cpp
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(ops_can_probe PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(ops_can_probe pcap pthread rt m)
    ament_target_dependencies(ops_can_probe
        rclcpp
        geometry_msgs
        std_msgs
    )
    list(APPEND LINKX_SOEM_INSTALL_TARGETS ops_can_probe)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/ops_can_command_main.cpp")
    add_executable(ops_can_command
        src/extra_tests/ops_can_command_main.cpp
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(ops_can_command PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(ops_can_command pcap pthread rt m)
    list(APPEND LINKX_SOEM_INSTALL_TARGETS ops_can_command)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/extra_tests/omni_slope_param_sweep_main.cpp")
    add_executable(omni_slope_param_sweep
        src/extra_tests/omni_slope_param_sweep_main.cpp
        src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp
        src/vehicle_control/2_Device/Motor/dvc_motor_dm.cpp
        src/vehicle_control/2_Device/OPS/dvc_ops.cpp
        src/vehicle_control/1_Middleware/Algorithm/alg_pid.cpp
        src/vehicle_control/1_Middleware/Algorithm/ramp.c
        ${SOEM_SRC}
        ${LINKX_BRIDGE_SRC}
    )
    target_include_directories(omni_slope_param_sweep PRIVATE ${EXTRA_TEST_INCLUDE_DIRS})
    target_link_libraries(omni_slope_param_sweep pcap pthread rt m)
    ament_target_dependencies(omni_slope_param_sweep
        rclcpp
        geometry_msgs
        std_msgs
    )
    list(APPEND LINKX_SOEM_INSTALL_TARGETS omni_slope_param_sweep)
endif()

install(TARGETS
    ${LINKX_SOEM_INSTALL_TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
