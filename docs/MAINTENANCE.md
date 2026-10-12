# 维护指南

本文档面向本仓库的日常改动、构建、联调和布局维护。代码分层与接口约定见
[ARCHITECTURE.md](ARCHITECTURE.md)。

## 1. 环境与依赖

```bash
sudo apt update
sudo apt install -y ros-humble-joy
```

已假定 ROS 2 Humble 安装在 `/opt/ros/humble`。运行手柄链路时需确认 F710（或兼容手柄）已接入，
并能被 `/dev/input/js*` 识别。

## 2. 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

一键脚本会自动执行 Release 构建，并在检测到陈旧 CMake cache 时清理 package cache：

```bash
./start_upper_computer.sh --ifname enp4s0
```

构建产物：`build/`（中间产物）、`install/<package>/lib/<package>/`（可执行）、`log/`（日志）、
`var_data/ops_terminal.log`（主控运行时输出，脚本启动时会 tail）。

## 3. 运行权限

SOEM 需要直接打开网卡收发 raw Ethernet frame，整车主控和 ToF 相关节点需要 root 或 capability。

```bash
# 方式一：直接 sudo
sudo ./start_upper_computer.sh --ifname enp4s0

# 方式二：授予 capability 后免 sudo
sudo setcap cap_net_raw,cap_net_admin+ep \
  install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo
./start_upper_computer.sh --ifname enp4s0 --no-sudo
```

每次重新构建后可执行文件会被覆盖，需要重新 `setcap`。`src/linkx_soem_demo/src/tools/` 下的联调工具
独立运行时同样需要 raw 权限。网卡建议专用，不要同时承担上网、SSH、远程桌面流量。

## 4. 启动

```bash
./start_upper_computer.sh --ifname enp4s0              # IMU + joy + remote + relay + EtherCAT 主控
./start_upper_computer.sh --ifname enp4s0 --no-vehicle # 仅 ROS 话题链路
./start_upper_computer.sh --ifname enp4s0 --no-sudo    # 已配置 setcap
./start_upper_computer.sh --ifname enp4s0 --gimbal     # 同时启动云台 relay
```

也可直接用 launch：

```bash
ros2 launch linkx_bringup full_system.launch.py ifname:=enp4s0 start_vehicle_control:=true start_gimbal_bridge:=false start_imu:=true
ros2 launch linkx_bringup teleop.launch.py
```

网卡默认值统一为 `enp4s0`：`start_upper_computer.sh`、`full_system.launch.py`、主控与各工具一致；
优先级为 `--ifname` > `IFNAME` 环境变量 > `enp4s0`。换机器或换网口时建议显式指定。

## 5. 排错速查

| 现象 | 排查方向 |
| --- | --- |
| `Failed to open interface` | 网卡名错误、网卡未 UP、没有 sudo/setcap raw 权限 |
| `slave count = 0` | LinkX-4C 未上电、网线错误、不是专用 EtherCAT 网口、从站状态异常 |
| WKC 连续异常 | 线缆、从站供电、PDO 映射、LinkX 状态和 EtherCAT 状态机 |
| 手柄无输出 | F710 接收器、D/X 档位、`/joy` 是否有数据 |
| `/chassis/cmd_vel` 有数据但底盘不动 | 是否已 START 或调用 `/vehicle/enable`，主控是否进入 OP |
| `setcap` 后仍权限不足 | 重新构建后 capability 丢失，重新对 install 下可执行执行 `setcap` |
| 找不到 ROS 包 | 确认当前目录是工作区根目录，并 `source install/setup.bash` |
| `/IMU_data` 没有数据 | `/dev/ttyUSB0`、波特率、串口权限和 `hipnuc_config.yaml` |

## 6. 改动流程

### 新增源文件

1. 放到对应分层目录（见 ARCHITECTURE 第 3 节的层级表）；联调工具只放 `src/linkx_soem_demo/src/tools/`。
2. 在 `src/linkx_soem_demo/CMakeLists.txt` 中把它加入相应的 `add_executable`/源列表；新可执行还需追加到
   `LINKX_SOEM_INSTALL_TARGETS`。
3. 运行 `python3 scripts/check_repo_layout.py .`：检查项 B/E 会拦住「CMake 引用了不存在的文件」
   和「源文件没有被 CMake 引用」两类问题。

### 新增头文件

1. 放在 `include/linkx_soem_demo/` 下、与 `src/linkx_soem_demo/src/` 同构的目录中，并在 CMake
   `PROJECT_INCLUDE_DIRS` 中存在对应 include 目录（检查项 C 会校验）。
2. 项目头文件不要再起与 C 标准库同名的名字（`math.h`、`string.h`、`stdio.h` 等），检查项 G 会报错。
3. 被别人 `#include` 之前先确认大小写完全一致（检查项 C 在 Linux 上大小写敏感）。

### 修改文件名 / 目录名

1. 使用 `git mv` 保留历史，不要手动删除再新建。
2. 全仓库搜索旧名字（含 `CMakeLists.txt`、launch、`package.xml`、文档）：

   ```bash
   git grep -n -e old_name -e OldName
   ```

3. 改名后重新运行布局校验器：检查项 C/D/E/F 会覆盖头文件解析、孤儿头文件、未引用源文件、launch 目标。
4. 头文件内部 guard 宏（`XXX_H`）与文件用途保持一致，便于排查重复包含问题。

### 历史改名记录

| 旧路径 | 新路径 | 原因 |
| --- | --- | --- |
| `include/linkx_soem_demo/vehicle_control/1_Middleware/math/math.h` | `.../1_Middleware/math/r2_math.h` | 与 C 标准库 `math.h` 同名：在 `-I` 顺序靠前时会覆盖系统头，且该文件依赖 `#include_next <math.h>` 这种脆弱写法。改名后 12 处 `#include "math.h"` 同步改为 `#include "r2_math.h"`，guard 改为 `LINKX_R2_PROJECT_R2_MATH_H`。 |
| `include/linkx_soem_demo/remote/device/dvc_logF710.h` | `.../remote/device/dvc_log_f710.h` | 统一「小写 + 下划线」命名；类名 `Class_LogF710` 等业务符号保持不变。 |
| `src/linkx_soem_demo/src/remote/device/dvc_logF710.cpp` | `.../remote/device/dvc_log_f710.cpp` | 同上；同步更新 `CMakeLists.txt` 与 2 处 include。 |

目录名 `math/` 保持不变：它只是「数学工具」目录，不会与标准库冲突。

## 7. 持续集成

`.github/workflows/ci.yml` 在 push 到 `main`、PR 和手动触发时运行两个 job：

| job | 环境 | 内容 |
| --- | --- | --- |
| `layout` | `ubuntu-22.04` | `python3 scripts/check_repo_layout.py .`（无需 ROS 与硬件） |
| `build` | `ros:humble` 容器 | 安装 `ros-humble-joy` 等依赖后执行 `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` |

本机提交前建议至少执行一次布局校验器，它比编译更快，能拦住绝大多数路径/引用类回归。

## 8. 许可与元数据

- 根 `LICENSE`：项目自有代码为专有许可（proprietary），第三方组件例外。
- `THIRD_PARTY_NOTICES.md`：第三方组件的版权与许可清单（SOEM、HiPNUC、USTC-RoboWalker 相关文件）。
- `package.xml` 的 `<license>` 必须与文件实际声明一致：
  - `hipnuc_imu`、`hipnuc_lib_package`：`Apache-2.0`（HiPNUC 官方代码）；
  - `linkx_soem_demo`、`linkx_bringup`：`Proprietary`。
- 引入新的第三方源码或库时，同步更新 `THIRD_PARTY_NOTICES.md` 与对应 `package.xml`；
  SOEM 采用 GPLv3/商业双重许可，闭源分发前需确认授权方式。

## 9. 提交前检查清单

1. `python3 scripts/check_repo_layout.py .` 输出 `OK`（problems: 0）。
2. `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过。
3. 涉及话题/服务/参数的改动已同步到 `README.md` 与 `docs/`。
4. 涉及文件改名时使用 `git mv`，且全仓库无旧名残留。
5. 涉及硬件行为（速度上限、PID、限位、CAN ID）的改动先在台架上低速验证。
