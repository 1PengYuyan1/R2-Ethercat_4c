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
# 复用源列表：SOEM 底层（所有 test_main + 主控都依赖）
# ============================================================================
set(SOEM_SRC
    src/vehicle_control/middleware/soem/ec_base.c
    src/vehicle_control/middleware/soem/ec_coe.c
    src/vehicle_control/middleware/soem/ec_config.c
    src/vehicle_control/middleware/soem/ec_dc.c
    src/vehicle_control/middleware/soem/ec_eoe.c
    src/vehicle_control/middleware/soem/ec_foe.c
    src/vehicle_control/middleware/soem/ec_main.c
    src/vehicle_control/middleware/soem/ec_print.c
    src/vehicle_control/middleware/soem/ec_soe.c
    src/vehicle_control/middleware/soem/nicdrv.c
    src/vehicle_control/middleware/soem/osal.c
    src/vehicle_control/middleware/soem/oshw.c
)

# 复用源列表：LinkX + EtherCAT manager（需要 CAN 收发的 test_main 用得到）
set(LINKX_BRIDGE_SRC
    src/vehicle_control/middleware/linkx/linkx.c
    src/vehicle_control/device/ecat_manager/ecat_manager.c
    src/vehicle_control/device/linkx4c_handler/linkx4c_handler.c
    src/vehicle_control/device/rt_timing/rt_timing.c
)

# 复用 include 目录（test_main 共用）
set(TEST_MAIN_INCLUDE_DIRS
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
    include/linkx_soem_demo/vehicle_control/2_Device/Buzzer
    include/linkx_soem_demo/vehicle_control/2_Device/Suction
    include/linkx_soem_demo/vehicle_control/2_Device/OPS
    include/linkx_soem_demo/vehicle_control/2_Device/ecat_manager
    include/linkx_soem_demo/vehicle_control/2_Device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/2_Device/rt_timing
    include/linkx_soem_demo/vehicle_control/3_Chariot
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis
    include/linkx_soem_demo/vehicle_control/3_Chariot/gantry
    include/linkx_soem_demo/vehicle_control/3_Chariot/arm
    include/linkx_soem_demo/vehicle_control/3_Chariot/navigation
    include/linkx_soem_demo/vehicle_control/4_Interaction/robot
    include/linkx_soem_demo/vehicle_control/5_Task/task
)

# ============================================================================
# 1) 主控可执行：vehicle_control（SOEM/EtherCAT 主站 + LinkX-4C CAN 桥 + r2 整车控制）
# ============================================================================

set(VEHICLE_CONTROL_SRC
    src/vehicle_control/main.cpp
    src/vehicle_control/task/task/task.cpp
    src/vehicle_control/interaction/robot/robot.cpp

    # 3_Chariot
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/gantry/crt_gantry.cpp
    src/vehicle_control/chariot/arm/crt_arm.cpp
    src/vehicle_control/chariot/navigation/crt_navigation.cpp

    # 2_Device
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/Buzzer/dvc_buzzer.cpp
    src/vehicle_control/device/Suction/dvc_suction.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    ${LINKX_BRIDGE_SRC}

    # 1_Middleware
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/Algorithm/ramp.c
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
    include/linkx_soem_demo/vehicle_control/2_Device/Buzzer
    include/linkx_soem_demo/vehicle_control/2_Device/Suction
    include/linkx_soem_demo/vehicle_control/2_Device/OPS
    include/linkx_soem_demo/vehicle_control/2_Device/ecat_manager
    include/linkx_soem_demo/vehicle_control/2_Device/linkx4c_handler
    include/linkx_soem_demo/vehicle_control/2_Device/rt_timing
    include/linkx_soem_demo/vehicle_control/3_Chariot
    include/linkx_soem_demo/vehicle_control/3_Chariot/chassis
    include/linkx_soem_demo/vehicle_control/3_Chariot/gantry
    include/linkx_soem_demo/vehicle_control/3_Chariot/arm
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
# 2) 遥控解算节点：joy → /cmd_vel + /robot_buttons
# ============================================================================
add_executable(remote_node_cpp
    src/remote/ros2/remote_node.cpp
    src/remote/ros2/joystick_mapper.cpp
    src/remote/device/Remote/dvc_logF710.cpp
)
target_include_directories(remote_node_cpp PRIVATE include)
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

# ============================================================================
# 4) test_main: linkx_set_alias —— EEPROM Station Alias 配置工具（最小依赖）
# ============================================================================
add_executable(linkx_set_alias
    src/test_mains/linkx_set_alias_main.cpp
    ${SOEM_SRC}
)
target_include_directories(linkx_set_alias PRIVATE ${TEST_MAIN_INCLUDE_DIRS})
target_link_libraries(linkx_set_alias pcap pthread rt m)

# ============================================================================
# 5) test_main: can_link_test —— 4 通道经典 CAN 1Mbps 链路冒烟测试
# ============================================================================
add_executable(can_link_test
    src/test_mains/can_link_test_main.cpp
    ${SOEM_SRC}
    ${LINKX_BRIDGE_SRC}
)
target_include_directories(can_link_test PRIVATE ${TEST_MAIN_INCLUDE_DIRS})
target_link_libraries(can_link_test pcap pthread rt m)

# ============================================================================
# 6) test_main: motor_calib —— DM6225 / DM3519 电机参数标定（friction/stiction/inertia）
# ============================================================================
add_executable(motor_calib
    src/test_mains/motor_calib_main.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/Algorithm/ramp.c
    ${SOEM_SRC}
    ${LINKX_BRIDGE_SRC}
)
target_include_directories(motor_calib PRIVATE ${TEST_MAIN_INCLUDE_DIRS})
target_link_libraries(motor_calib pcap pthread rt m)

# ============================================================================
# 7) test_main: steer_tuning —— 舵向 PID 网格扫参（用 DM6225 内部反馈，无 BRT 编码器）
# ============================================================================
add_executable(steer_tuning
    src/test_mains/steer_tuning_main.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/Algorithm/ramp.c
    ${SOEM_SRC}
    ${LINKX_BRIDGE_SRC}
)
target_include_directories(steer_tuning PRIVATE ${TEST_MAIN_INCLUDE_DIRS})
target_link_libraries(steer_tuning pcap pthread rt m)

# ============================================================================
# 8) test_main: robot_test —— 整车回归 (Chassis + Gantry + Arm + ROS 桥接)
# 注意：不链接 task.cpp/main.cpp（避免 robot/master/linkx_dev 重定义）
# ============================================================================
add_executable(robot_test
    src/test_mains/robot_test_main.cpp
    src/vehicle_control/interaction/robot/robot.cpp
    src/vehicle_control/chariot/chassis/crt_chassis.cpp
    src/vehicle_control/chariot/gantry/crt_gantry.cpp
    src/vehicle_control/chariot/arm/crt_arm.cpp
    src/vehicle_control/chariot/navigation/crt_navigation.cpp
    src/vehicle_control/device/Motor/dvc_motor_dm.cpp
    src/vehicle_control/device/Buzzer/dvc_buzzer.cpp
    src/vehicle_control/device/Suction/dvc_suction.cpp
    src/vehicle_control/device/OPS/dvc_ops.cpp
    src/vehicle_control/middleware/Algorithm/alg_pid.cpp
    src/vehicle_control/middleware/Algorithm/ramp.c
    ${SOEM_SRC}
    ${LINKX_BRIDGE_SRC}
)
target_include_directories(robot_test PRIVATE ${TEST_MAIN_INCLUDE_DIRS})
target_link_libraries(robot_test pcap pthread rt m)
ament_target_dependencies(robot_test rclcpp geometry_msgs std_msgs sensor_msgs)

install(TARGETS
    ${PROJECT_NAME}
    remote_node_cpp
    stm32_node_cpp
    linkx_set_alias
    can_link_test
    motor_calib
    steer_tuning
    robot_test
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
