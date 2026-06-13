cmake_minimum_required(VERSION 3.10)
project(tof C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# This package is imported into the Ethercat-R2 workspace. Reuse the local
# LinkX/SOEM implementation from the sibling linkx_soem_demo package instead
# of depending on the external dm_linkx4c_ethercat package used by the source
# project this was copied from.
set(LINKX_SOEM_DEMO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../linkx_soem_demo")
if(NOT EXISTS "${LINKX_SOEM_DEMO_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "Expected sibling package not found: ${LINKX_SOEM_DEMO_DIR}")
endif()

set(SOEM_SRC
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_base.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_coe.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_config.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_dc.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_eoe.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_foe.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_main.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_print.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/ec_soe.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/nicdrv.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/osal.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/soem/oshw.c
)

set(LINKX_BRIDGE_SRC
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/1_Middleware/linkx/linkx.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/2_Device/Ethercat/ecat_manager/ecat_manager.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/2_Device/Ethercat/linkx4c_handler/linkx4c_handler.c
  ${LINKX_SOEM_DEMO_DIR}/src/vehicle_control/2_Device/rt_timing/rt_timing.c
)

set(LINKX_INCLUDE_DIRS
  ${LINKX_SOEM_DEMO_DIR}/include
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/1_Middleware
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/1_Middleware/linkx
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/1_Middleware/npcap
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/1_Middleware/soem
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/2_Device
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/2_Device/Ethercat
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/2_Device/Ethercat/ecat_manager
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/2_Device/Ethercat/linkx4c_handler
  ${LINKX_SOEM_DEMO_DIR}/include/linkx_soem_demo/vehicle_control/2_Device/rt_timing
)

# ==========================================
# 上位机雷达节点：EtherCAT -> LinkX-4C -> CAN -> Range topics
# ==========================================
add_executable(can_link_test
  src/can_link_test.cpp
  src/dvc_tfmini_s.cpp
  src/dvc_low_freq.cpp
  ${SOEM_SRC}
  ${LINKX_BRIDGE_SRC}
)
target_include_directories(can_link_test PRIVATE ${LINKX_INCLUDE_DIRS})
ament_target_dependencies(can_link_test rclcpp sensor_msgs)
target_link_libraries(can_link_test pcap pthread rt m)

# ==========================================
# 四路上/下高频测距订阅示例节点
# ==========================================
add_executable(up_range_subscriber
  src/up_range_subscriber.cpp
)
ament_target_dependencies(up_range_subscriber rclcpp sensor_msgs)

# ==========================================
# 安装
# ==========================================
install(TARGETS
  can_link_test
  up_range_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
