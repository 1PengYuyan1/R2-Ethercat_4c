# 第三方组件与许可说明 (Third-Party Notices)

本文件列出 R2-Ethercat_4c 仓库中包含的第三方代码及其许可，供分发与合规检查使用。
根 `LICENSE` 中的专有条款不覆盖本文件列出的组件。

## 1. SOEM（Simple Open EtherCAT Master）

- 位置：
  - `src/linkx_soem_demo/include/linkx_soem_demo/vehicle_control/1_Middleware/soem/`
  - `src/linkx_soem_demo/src/vehicle_control/1_Middleware/soem/`
- 上游：https://github.com/OpenEtherCATsociety/SOEM
- 版权：Copyright (C) 2005-2023 Arthur Ketels 与 rt-labs AB
- 许可：上游采用 **GPLv3 与商业许可双重授权**。仓库内 SOEM 源码文件头保留了原始声明：
  `This software is dual-licensed under GPLv3 and a commercial license. See the file LICENSE.md distributed with this software for full license information.`
- 现状：本仓库未附带 SOEM 上游的 `LICENSE.md`，完整条款请从上游仓库获取。
- 合规提醒：以 GPLv3 方式使用 SOEM 时，相应的源码分发与衍生作品义务由使用者承担；
  若需要闭源商业分发整车主控，应向 rt-labs 取得 SOEM 商业许可。

## 2. HiPNUC 解码库与 IMU 驱动（Apache-2.0）

- 位置：
  - `src/hipnuc_lib_package/`（`hipnuc_dec.*`、`nmea_dec.*`）
  - `src/hipnuc_imu/`（`hipnuc_imu_node`、示例与配置）
- 上游：HiPNUC IMU 官方 SDK 与 ROS 2 驱动
- 版权：Copyright (c) 2006-2024, HiPNUC
- 许可：Apache License 2.0（文件头 SPDX 标识为 `Apache-2.0`）
- 对应 `package.xml` 的 `<license>` 字段：`src/hipnuc_imu/package.xml` 为 `Apache License 2.0`，
  `src/hipnuc_lib_package/package.xml` 为 `Apache-2.0`。

## 3. USTC-RoboWalker 相关算法文件

- 位置：
  - `src/linkx_soem_demo/include/linkx_soem_demo/vehicle_control/1_Middleware/algorithm/alg_pid.h`
  - `src/linkx_soem_demo/src/vehicle_control/1_Middleware/algorithm/alg_pid.cpp`
  - `src/linkx_soem_demo/**/1_Middleware/algorithm/ramp.h` 与 `ramp.c`（文件头署名 Yan Yuanbin，沿用 RoboWalker 工程模板）
- 版权：`alg_pid.*` 文件头保留 `Copyright USTC-RoboWalker (c) 2023`
- 处理方式：保留上游文件头注释，未删除版权信息；如需再分发请遵循上游项目声明。

## 4. 其他

- Logitech F710 遥控解算（`src/linkx_soem_demo/src/remote/device/dvc_log_f710.cpp`）按公开的手柄按键
  映射协议独立实现，未包含第三方库代码。
- `src/linkx_soem_demo` 通过 `rclcpp`、`std_msgs`、`sensor_msgs`、`geometry_msgs`、`std_srvs`、
  `rcl_interfaces` 依赖 ROS 2（Apache-2.0）与系统库（glibc、pthread、rt），未内嵌其源码。
- 本文件仅为工程内合规记录，不构成法律意见；对外正式分发前建议由法务复核。
