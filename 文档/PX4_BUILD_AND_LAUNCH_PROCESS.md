# PX4 构建和启动流程详解

## 文档说明

本文档详细说明 `make px4_sitl gz_hnuter` 命令的完整执行流程，从编译到启动 Gazebo 显示无人机，直到 PX4 命令终端打开。

---

## 目录

1. [命令解析](#命令解析)
2. [编译阶段](#编译阶段)
3. [启动阶段](#启动阶段)
4. [机型加载](#机型加载)
5. [Gazebo 启动](#gazebo-启动)
6. [模块初始化](#模块初始化)
7. [完整流程图](#完整流程图)
8. [关键文件索引](#关键文件索引)

---

## 命令解析

### 命令格式

```bash
make px4_sitl gz_hnuter
```

**参数分解**:
- `make` - GNU Make 构建工具
- `px4_sitl` - 目标板配置 (Software In The Loop)
- `gz_hnuter` - 机型名称 (Gazebo + hnuter)

### Makefile 解析流程

**文件**: `Makefile` (顶层)

#### 步骤 1: 目标识别

```makefile
# 第 225-227 行
px4_sitl: px4_sitl_default
px4_sitl_default: $(BUILD_DIR_SUFFIX)
	$(call cmake-build,$@)
```

**映射关系**:
- `px4_sitl` → `px4_sitl_default`
- 调用 `cmake-build` 函数

#### 步骤 2: cmake-build 函数

**文件**: `Makefile` (第 173-191 行)

```makefile
define cmake-build
	@$(eval BUILD_DIR = $(BUILD_DIR_SUFFIX)$(call lowercase,$(1)))
	@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ]; then \
		mkdir -p $(BUILD_DIR) \
		&& cd $(BUILD_DIR) \
		&& cmake $(PX4_CMAKE_GENERATOR) -DCONFIG=$(1) \
		   -DPYTHON_EXECUTABLE=$(PYTHON) \
		   $(CMAKE_ARGS) .. \
		|| (rm -rf $(BUILD_DIR)); \
	fi
	@$(PX4_MAKE) -C $(BUILD_DIR) $(PX4_MAKE_ARGS)
endef
```

**执行步骤**:
1. 创建构建目录: `build/px4_sitl_default/`
2. 运行 CMake 配置: `cmake -G"Ninja" -DCONFIG=px4_sitl_default ...`
3. 运行构建: `ninja` (或 `make`)

#### 步骤 3: 额外参数处理

```bash
# gz_hnuter 参数传递
PX4_MAKE_ARGS = gz_hnuter
```

这个参数会传递给构建系统和启动脚本。

---

## 编译阶段

### 阶段 1: CMake 配置

#### 1.1 顶层 CMakeLists.txt

**文件**: `CMakeLists.txt`

**关键配置** (第 1-450 行):

```cmake
cmake_minimum_required(VERSION 3.22)

# 第 36-40 行: 项目定义
project(px4 CXX C ASM)

# 第 48-93 行: 解析 CONFIG
if(NOT CONFIG)
    set(CONFIG "px4_sitl_default" CACHE STRING "PX4 config" FORCE)
endif()

# 从 CONFIG 提取板信息
px4_parse_config(${CONFIG})  # → px4/sitl/default

# 第 102-107 行: 设置源和二进制目录
set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# 第 140-142 行: 包含板配置
set(config_module_list)
include(${PX4_BOARD_DIR}/default.px4board)
```

#### 1.2 板配置文件

**文件**: `boards/px4/sitl/default.px4board`

**关键配置**:

```cmake
CONFIG_PLATFORM_POSIX=y               # POSIX 平台
CONFIG_BOARD_TESTING=y                # 启用测试

# Gazebo 支持
CONFIG_MODULES_SIMULATION_GZ_MSGS=y   # Gazebo 消息
CONFIG_MODULES_SIMULATION_GZ_BRIDGE=y # Gazebo 桥接
CONFIG_MODULES_SIMULATION_GZ_PLUGINS=y # Gazebo 插件

# 仿真公共模块
CONFIG_COMMON_SIMULATION=y

# 驱动和模块
CONFIG_DRIVERS_ADC_BOARD_ADC=y
CONFIG_DRIVERS_BAROMETER_SIM=y
CONFIG_DRIVERS_GPS_SIM=y
CONFIG_DRIVERS_IMU_SIM=y
# ... 更多驱动
```

#### 1.3 平台配置

**文件**: `boards/px4/sitl/sitl.cmake`

```cmake
# 第 1-20 行: 平台设置
set(PX4_PLATFORM "posix")
set(PX4_BOARD "px4_sitl")

# 第 22-40 行: 编译器设置
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

# 第 42-60 行: Gazebo 环境变量
set(PX4_GZ_MODELS "${PX4_SOURCE_DIR}/Tools/simulation/gz/models")
set(PX4_GZ_WORLDS "${PX4_SOURCE_DIR}/Tools/simulation/gz/worlds")

# 第 62-80 行: 包含 POSIX 平台配置
include(platforms/posix/cmake/sitl_target.cmake)
```

### 阶段 2: 模块编译

#### 2.1 核心模块列表

基于 `default.px4board` 配置，以下模块被编译：

| 模块 | 源码路径 | 功能 |
|------|---------|------|
| **主程序** | `platforms/posix/src/px4/` | PX4 SITL 主程序 |
| **Gazebo 桥接** | `src/modules/simulation/gz_bridge/` | PX4 ↔ Gazebo 通信 |
| **Gazebo 消息** | `src/modules/simulation/gz_msgs/` | Gazebo 消息定义 |
| **Gazebo 插件** | `src/modules/simulation/gz_plugins/` | Gazebo 仿真插件 |
| **电池仿真** | `src/modules/simulation/battery_simulator/` | 电池状态仿真 |
| **传感器仿真** | `src/modules/simulation/sensor_*_sim/` | GPS/气压计/IMU 等 |
| **Commander** | `src/modules/commander/` | 飞行模式和安全管理 |
| **Navigator** | `src/modules/navigator/` | 航线和自主飞行 |
| **Control Allocator** | `src/modules/control_allocator/` | 控制分配 |
| **MC Attitude Control** | `src/modules/mc_att_control/` | 多旋翼姿态控制 |
| **MC Position Control** | `src/modules/mc_pos_control/` | 多旋翼位置控制 |
| **EKF2** | `src/modules/ekf2/` | 扩展卡尔曼滤波器 |
| **MAVLink** | `src/modules/mavlink/` | MAVLink 通信协议 |
| **Logger** | `src/modules/logger/` | 飞行日志 |
| **uORB** | `src/modules/uORB/` | 微对象请求代理 |

#### 2.2 主程序编译

**文件**: `platforms/posix/src/px4/common/main.cpp`

**关键函数**:

```cpp
// 第 121-150 行: main() 入口
int main(int argc, char **argv)
{
    bool is_client = false;
    std::string absolute_binary_path;
    int instance = 0;

    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--instance") == 0) {
            instance = atoi(argv[i + 1]);
            i++;
        }
    }

    // 获取绝对二进制路径
    absolute_binary_path = get_absolute_binary_path(argv[0]);

    // 检查是否为客户端模式（px4- 前缀）
    std::string binary_name = basename(argv[0]);
    const char *prefix = "px4-";
    if (binary_name.compare(0, strlen(prefix), prefix) == 0) {
        is_client = true;
    }

    if (is_client) {
        // 客户端模式：连接到 PX4 服务器
        return client_main(argc, argv, absolute_binary_path, instance);
    } else {
        // 服务器模式：启动 PX4 主进程
        return server_main(argc, argv, absolute_binary_path, instance);
    }
}
```

**server_main() 函数** (第 200-250 行):

```cpp
static int server_main(int argc, char **argv,
                      const std::string &absolute_binary_path,
                      int instance)
{
    // 1. 初始化 PX4 平台
    px4::init(argc, argv, &px4::print_usage);

    // 2. 设置工作目录
    const char *px4_rootfsdir = getenv("PX4_ROOTFS");
    if (px4_rootfsdir) {
        chdir(px4_rootfsdir);
    }

    // 3. 挂载 ROMFS
    px4::init_once();

    // 4. 运行启动脚本
    std::string startup_script = "etc/init.d-posix/rcS";
    run_startup_script(startup_script, absolute_binary_path, instance);

    // 5. 进入主循环
    px4::px4_daemon_loop();

    return 0;
}
```

### 阶段 3: ROMFS 生成

**文件**: `ROMFS/CMakeLists.txt`

**流程**:

```cmake
# 第 1-50 行: 复制 ROMFS 文件到构建目录

# 1. 复制标准初始化脚本
file(COPY px4fmu_common/init.d/
     DESTINATION ${PX4_BINARY_DIR}/etc/init.d/)

# 2. 复制 POSIX SITL 脚本
file(COPY px4fmu_common/init.d-posix/
     DESTINATION ${PX4_BINARY_DIR}/etc/init.d-posix/)

# 3. 复制机型配置文件（83 个）
file(GLOB airframe_files "px4fmu_common/init.d-posix/airframes/*")
file(COPY ${airframe_files}
     DESTINATION ${PX4_BINARY_DIR}/etc/init.d-posix/airframes/)
```

**输出结构**:

```
build/px4_sitl_default/etc/
├── init.d/                      # 标准初始化脚本
│   ├── rc.mc_defaults           # 多旋翼默认配置
│   ├── rc.fw_defaults           # 固定翼默认配置
│   └── ...
├── init.d-posix/                # POSIX SITL 专用脚本
│   ├── rcS                      # 主启动脚本 ⭐
│   ├── px4-alias.sh             # 模块别名
│   ├── px4-rc.simulator         # 仿真器选择脚本
│   ├── px4-rc.gzsim             # Gazebo 启动脚本 ⭐
│   ├── px4-rc.mavlinksim        # MAVLink 仿真脚本
│   └── airframes/               # 机型配置文件（83 个）
│       ├── 4051_gz_hnuter       # Hnuter Gazebo 配置（独立完整）⭐
│       └── ...
└── parameters.bson              # 参数数据库
```

### 阶段 4: 编译输出

**构建目录**: `build/px4_sitl_default/`

**主要产物**:

```
build/px4_sitl_default/
├── bin/
│   ├── px4                      # 主可执行文件 ⭐
│   ├── px4-alias.sh
│   ├── px4-commander            # 符号链接 → px4
│   ├── px4-ekf2                 # 符号链接 → px4
│   ├── px4-navigator            # 符号链接 → px4
│   └── px4-*                    # 其他模块符号链接
├── etc/                         # ROMFS 文件系统
├── src/
│   └── modules/
│       ├── gz_bridge/
│       │   └── libmodules__simulation__gz_bridge.a
│       ├── commander/
│       │   └── libmodules__commander.a
│       └── ...                  # 其他模块库
├── CMakeCache.txt
├── build.ninja                  # Ninja 构建文件
└── compile_commands.json
```

---

## 启动阶段

### 阶段 1: PX4 主程序启动

#### 命令执行

编译完成后，Make 会自动执行：

```bash
cd build/px4_sitl_default
./bin/px4 gz_hnuter
```

**参数**:
- `gz_hnuter` - 机型名称，设置环境变量 `PX4_SIM_MODEL=hnuter`

#### main() 执行

**文件**: `platforms/posix/src/px4/common/main.cpp` (第 121 行)

```cpp
int main(int argc, char **argv) {
    // argc = 2
    // argv[0] = "./bin/px4"
    // argv[1] = "gz_hnuter"

    // 二进制名称 = "px4" (不含 px4- 前缀)
    // → is_client = false
    // → 进入服务器模式

    return server_main(argc, argv, absolute_binary_path, 0);
}
```

#### server_main() 执行

**文件**: `platforms/posix/src/px4/common/main.cpp` (第 200 行)

```cpp
static int server_main(...) {
    // 1. 初始化 PX4 平台
    px4::init(argc, argv, &px4::print_usage);
    // → 解析命令行参数
    // → 设置信号处理器
    // → 初始化 uORB

    // 2. 设置工作目录
    chdir(build_path);  // → build/px4_sitl_default/

    // 3. 挂载 ROMFS（虚拟文件系统）
    px4::init_once();
    // → 将 etc/ 目录映射到内存

    // 4. 解析机型参数
    if (argc > 1 && argv[1][0] != '-') {
        // argv[1] = "gz_hnuter"
        setenv("PX4_SIM_MODEL", "hnuter", 1);  // 去掉 "gz_" 前缀
    }

    // 5. 运行启动脚本
    run_startup_script("etc/init.d-posix/rcS", absolute_binary_path, 0);

    // 6. 进入主循环
    px4::px4_daemon_loop();
}
```

### 阶段 2: 启动脚本执行（rcS）

**文件**: `ROMFS/px4fmu_common/init.d-posix/rcS`

#### 脚本结构（376 行）

```bash
#!/bin/sh
#
# PX4 启动脚本（POSIX SITL）
#

# ============================================================
# 第 1-50 行: 初始化
# ============================================================

# 设置路径
R=${R:=etc}                          # ROMFS 根目录

# 加载别名
. ${R}/init.d/px4-alias.sh

# 解析 SIM_MODEL
if [ -n "$PX4_SIM_MODEL" ]; then
    REQUESTED_MODEL=$PX4_SIM_MODEL
else
    # 从命令行参数提取
    REQUESTED_MODEL=$(echo "$1" | sed 's/^gz_//')
fi

# ============================================================
# 第 51-64 行: 机型识别
# ============================================================

# 查找匹配的自启动文件
# 格式: [数字]_${REQUESTED_MODEL}

REQUESTED_AUTOSTART=$(ls "${R}etc/init.d-posix/airframes" | \
    sed -n 's/^\([0-9][0-9]*\)_'${REQUESTED_MODEL}'$/\1/p')

# 对于 REQUESTED_MODEL=hnuter:
# 查找文件: *_hnuter
# 找到: 4051_gz_hnuter
# 提取: REQUESTED_AUTOSTART=4051

if [ -n "$REQUESTED_AUTOSTART" ]; then
    SYS_AUTOSTART=$REQUESTED_AUTOSTART
    echo "Found autostart ID: $SYS_AUTOSTART"
fi

# ============================================================
# 第 66-104 行: 参数加载
# ============================================================

# 参数文件路径
PARAM_FILE="${R}/parameters.bson"

# 加载参数
param select $PARAM_FILE

# 如果需要重置参数
if [ -n "$PARAM_DEFAULTS_VER" ]; then
    param import -d $PARAM_FILE
fi

# 设置自启动 ID
param set SYS_AUTOSTART $SYS_AUTOSTART
param set SYS_AUTOCONFIG 1

# ============================================================
# 第 143-239 行: 机型配置加载
# ============================================================

# 查找机型配置文件
# 格式: [SYS_AUTOSTART]_*

autostart_file=""
for f in ${R}etc/init.d-posix/airframes/"$(param show -q SYS_AUTOSTART)"_*
do
    autostart_file="$f"
    break
done

if [ -n "$autostart_file" ]; then
    echo "Loading autostart file: $autostart_file"
    . "$autostart_file"  # ← 执行 4051_gz_hnuter
fi

# ============================================================
# 第 241-280 行: 仿真器启动
# ============================================================

# 启动数据管理器
dataman start

# 运行仿真器启动脚本
. ${R}/init.d-posix/px4-rc.simulator

# ============================================================
# 第 282-340 行: 系统模块启动
# ============================================================

# 电池仿真
battery_simulator start

# 系统电源仿真
system_power_simulator start

# 声音警告
tone_alarm start

# 遥控更新
rc_update start

# 传感器模块
sensors start

# ============================================================
# 第 342-375 行: 控制和导航模块启动
# ============================================================

# 指挥官（模式管理和安全）
commander start -h

# EKF2（状态估计）
ekf2 start

# 多旋翼姿态控制
mc_att_control start

# 多旋翼位置控制
mc_pos_control start

# 多旋翼速率控制
mc_rate_control start

# 控制分配器
control_allocator start

# 导航器
navigator start

# DDS 客户端（ROS2 通信）
uxrce_dds_client start

echo "PX4 startup complete!"
```

---

## 机型加载

### 阶段 1: 机型文件查找

**rcS 脚本** (第 218-239 行):

```bash
# SYS_AUTOSTART = 4051

for f in ${R}etc/init.d-posix/airframes/4051_*
do
    autostart_file="$f"
    # 找到: etc/init.d-posix/airframes/4051_gz_hnuter
    break
done

. "$autostart_file"  # 执行机型配置
```

### 阶段 2: 4051_gz_hnuter 配置

**文件**: `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`

```bash
#!/bin/sh
#
# @name hnuter tiltrotor (Gazebo)
# @type tiltrotor
# @maintainer hnuter <hnuter@example.com>
#

# 第 9 行: 加载多旋翼默认配置
. ${R}etc/init.d/rc.mc_defaults

# 第 11-17 行: 仿真器配置
PX4_SIMULATOR=${PX4_SIMULATOR:=gz}      # Gazebo 仿真
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}   # 默认世界
PX4_SIM_MODEL=${PX4_SIM_MODEL:=hnuter}  # 模型名称

# 第 20 行: 启用 Gazebo 桥接
param set-default SIM_GZ_EN 1

# 第 23-24 行: 启用传感器
param set-default SIM_GZ_EN_GPS 1
param set-default SIM_GZ_EN_BARO 1

# 第 27 行: 启用电池仿真
param set-default SIM_BAT_ENABLE 1

# 第 30-33 行: 绕过安全检查（仿真用）
param set-default CBRK_SUPPLY_CHK 894281  # 电源检查
param set-default CBRK_USB_CHK 197848     # USB 检查

# 第 36-40 行: 5 个电机的执行器映射
param set-default SIM_GZ_EC_FUNC1 101  # Motor 1 (xy1)
param set-default SIM_GZ_EC_FUNC2 102  # Motor 2 (xy2)
param set-default SIM_GZ_EC_FUNC3 103  # Motor 3 (xy3)
param set-default SIM_GZ_EC_FUNC4 104  # Motor 4 (xy4)
param set-default SIM_GZ_EC_FUNC5 105  # Motor 5 (xy5)

# 第 43-53 行: 电机速度限制
param set-default SIM_GZ_EC_MIN1 10    # 最小 10 RPM
param set-default SIM_GZ_EC_MAX1 1500  # 最大 1500 RPM
# ... (其他 4 个电机类似)

# 第 56-59 行: 4 个倾转舵机的映射
param set-default SIM_GZ_SV_FUNC1 201  # Servo 1 (rj2)
param set-default SIM_GZ_SV_FUNC2 202  # Servo 2 (lj2)
param set-default SIM_GZ_SV_FUNC3 203  # Servo 3 (rj1)
param set-default SIM_GZ_SV_FUNC4 204  # Servo 4 (lj1)

# 第 62-69 行: 舵机角度限制（弧度）
param set-default SIM_GZ_SV_MAXA1 1.57   # +90°
param set-default SIM_GZ_SV_MINA1 -1.57  # -90°
# ... (其他 3 个舵机类似)

# 第 72-73 行: 禁用 ESC 故障检测
param set-default COM_ARM_CHK_ESCS 0
param set-default FD_ESCS_EN 0

# 第 76-90 行: 手动控制和飞行模式配置
param set-default COM_RC_IN_MODE 1      # 启用 RC 输入
param set-default RC_MAP_MODE_SW 5      # 模式切换通道
param set-default COM_FLTMODE1 1        # 位置模式
param set-default COM_FLTMODE2 2        # 高度模式
param set-default COM_FLTMODE3 3        # 手动模式
param set-default COM_FLTMODE4 6        # 稳定模式
param set-default COM_LOW_BAT_ACT 0     # 低电量动作: 无
param set-default NAV_RCL_ACT 0         # RC 丢失动作: 无

# 第 93-241 行: 车辆和控制配置（4051 是独立完整的配置）
# 包含以下完整配置：
# - 车辆类型 (MAV_TYPE=2)
# - 控制分配器配置 (CA_AIRFRAME=16, CA_ROTOR_COUNT=5)
# - 5 个电机的物理参数 (位置和力矩系数)
# - 倾转舵机配置 (CA_SV_TL_COUNT=0, 禁用但配置保留)
# - 多旋翼控制器参数 (MC_ROLL_P, MC_PITCH_P, MC_YAW_P 等)
# - VTOL 配置 (VT_TYPE=1, 倾转旋翼)
# - 传感器配置 (SYS_HAS_NUM_ASPD=0)
# - EKF2 配置 (EKF2_GPS_CHECK, EKF2_REQ_GPS_H)
# - 解锁和安全检查参数 (COM_ARM_WO_GPS, COM_ARM_EKF_*)
# - 任务参数 (MIS_TAKEOFF_ALT=10)

# 详细的电机配置示例:
# Motor 0: xy1 (右臂上方, CCW)
param set-default CA_ROTOR0_PX 0.0
param set-default CA_ROTOR0_PY 0.2
param set-default CA_ROTOR0_PZ 0.05
param set-default CA_ROTOR0_KM -0.05

# Motor 1-4: (类似配置)
# ...

# VTOL 配置
param set-default VT_TYPE 1              # 倾转旋翼
param set-default VT_FWD_THRUST_EN 4     # 电机 5 用于前向推力
param set-default VT_FWD_THRUST_SC 0.6
param set-default VT_TILT_TRANS 0.6

# 其他参数...
```

**重要说明**: `4051_gz_hnuter` 现在是一个**独立完整**的机型配置文件（243行），不再依赖其他机型文件。它包含了所有必要的配置参数。详细配置请参考 `AIRFRAME_4051_FIX.md` 文档。

---

## Gazebo 启动

### 阶段 1: px4-rc.simulator 选择

**文件**: `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator` (第 1-26 行)

```bash
#!/bin/sh
#
# 仿真器选择脚本
#

if [ "$PX4_SIMULATOR" = "sihsim" ]; then
    # SIH (Simulator In Hardware)
    . ${R}/init.d-posix/px4-rc.sihsim

elif [ "$PX4_SIMULATOR" = "gz" ] || [ "$(param show -q SIM_GZ_EN)" = "1" ]; then
    # Gazebo ← 对于 gz_hnuter，执行此分支
    . ${R}/init.d-posix/px4-rc.gzsim

elif [ "$PX4_SIM_MODEL" = "jmavsim_iris" ]; then
    # jMAVSim
    . ${R}/init.d-posix/px4-rc.jmavsim

else
    # 默认: MAVLink SITL
    . ${R}/init.d-posix/px4-rc.mavlinksim
fi
```

### 阶段 2: px4-rc.gzsim 执行

**文件**: `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim` (207 行)

#### 步骤 1: Gazebo 版本检查 (第 4-29 行)

```bash
#!/bin/sh
#
# Gazebo 启动脚本
#

# 检查 Gazebo 版本
gz_version=$(gz sim --versions | grep -m 1 "version" | sed 's/.*version \([0-9]*\.[0-9]*\).*/\1/')

# 要求最低版本: 8.0.0
min_gz_version="8.0"

if [ "$(printf '%s\n' "$min_gz_version" "$gz_version" | sort -V | head -n1)" != "$min_gz_version" ]; then
    echo "ERROR: Gazebo version $gz_version is too old!"
    echo "Required minimum version: $min_gz_version"
    exit 1
fi

echo "Gazebo version: $gz_version ✓"
```

#### 步骤 2: 环境变量设置 (第 31-50 行)

```bash
# 设置 Gazebo 资源路径
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${PX4_GZ_MODELS}

# 设置世界和模型路径
PX4_GZ_WORLDS=${PX4_GZ_WORLDS:=${PX4_SOURCE_DIR}/Tools/simulation/gz/worlds}
PX4_GZ_MODELS=${PX4_GZ_MODELS:=${PX4_SOURCE_DIR}/Tools/simulation/gz/models}

# 默认世界
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}

echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "PX4_GZ_WORLDS=$PX4_GZ_WORLDS"
echo "PX4_GZ_MODELS=$PX4_GZ_MODELS"
echo "PX4_GZ_WORLD=$PX4_GZ_WORLD"
```

#### 步骤 3: Gazebo 世界启动 (第 52-80 行)

```bash
# 检查是否已有 Gazebo 世界运行
gz_world=$(gz topic -l | grep -m 1 "^/world/.*/clock" | sed 's/\/world\///g' | sed 's/\/clock//g')

if [ -z "${gz_world}" ]; then
    # 没有运行的世界，启动新世界
    echo "Starting Gazebo world: ${PX4_GZ_WORLD}"

    # 启动 Gazebo 服务器（无头模式，带物理）
    gz sim --verbose=1 -r -s "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf" &

    # 等待世界就绪
    sleep 2

    # 启动 Gazebo GUI（可选）
    if [ "$HEADLESS" != "1" ]; then
        gz sim -g &
    fi

    # 等待世界完全启动
    sleep 3

    # 重新获取世界名称
    gz_world=$(gz topic -l | grep -m 1 "^/world/.*/clock" | sed 's/\/world\///g' | sed 's/\/clock//g')

    echo "Gazebo world ready: ${gz_world}"
else
    echo "Using existing Gazebo world: ${gz_world}"
fi

# 更新世界名称
PX4_GZ_WORLD=${gz_world}
```

#### 步骤 4: 模型名称解析 (第 82-110 行)

```bash
# 从 PX4_SIM_MODEL 提取模型名称
# 格式: gz_<model_name> 或 <model_name>

MODEL_NAME="${PX4_SIM_MODEL#*gz_}"  # 去掉 "gz_" 前缀

# 对于 PX4_SIM_MODEL=hnuter 或 gz_hnuter:
# MODEL_NAME = "hnuter"

echo "Model name: ${MODEL_NAME}"

# 实例号（多机仿真）
px4_instance=${px4_instance:=0}

# 模型实例名称
MODEL_NAME_INSTANCE="${MODEL_NAME}_${px4_instance}"
# MODEL_NAME_INSTANCE = "hnuter_0"

echo "Model instance: ${MODEL_NAME_INSTANCE}"
```

#### 步骤 5: 模型生成 (第 112-146 行)

```bash
# 构建 SDF 字符串
sdf_str="<sdf version=\"1.6\">
    <include>
        <uri>file://${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf</uri>
        <name>${MODEL_NAME_INSTANCE}</name>
        <pose>0 0 0.3 0 0 0</pose>
    </include>
</sdf>"

echo "Spawning model..."
echo "Model SDF: ${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf"

# 使用 Gazebo 服务生成模型
gz service -s "/world/${PX4_GZ_WORLD}/create" \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "name: \"${MODEL_NAME_INSTANCE}\", sdf: '${sdf_str}'"

# 检查生成结果
if [ $? -eq 0 ]; then
    echo "✓ Model spawned successfully: ${MODEL_NAME_INSTANCE}"
else
    echo "✗ ERROR: Failed to spawn model!"
    exit 1
fi
```

**等效命令**:
```bash
gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --req "name: \"hnuter_0\", sdf: '<sdf version=\"1.6\"><include><uri>file:///home/hnuter/PX4-Autopilot-Hnuter/Tools/simulation/gz/models/hnuter/model.sdf</uri></include></sdf>'"
```

#### 步骤 6: gz_bridge 启动 (第 148-207 行)

```bash
# 启动 Gazebo 桥接模块
# 桥接 PX4 uORB ↔ Gazebo Transport

echo "Starting gz_bridge..."

gz_bridge start -w "${PX4_GZ_WORLD}" -n "${MODEL_NAME_INSTANCE}"

# 等待桥接就绪
sleep 1

# 检查桥接状态
gz_bridge status

if [ $? -eq 0 ]; then
    echo "✓ gz_bridge started successfully"
else
    echo "✗ ERROR: gz_bridge failed to start!"
    exit 1
fi
```

**gz_bridge 功能**:
- 订阅 PX4 uORB 消息 (`actuator_motors`, `actuator_servos`)
- 发布到 Gazebo 话题 (`/hnuter/command/motor_speed`, `/model/hnuter/servo_*`)
- 订阅 Gazebo 话题 (传感器数据、姿态等)
- 发布到 PX4 uORB 消息 (`sensor_gps`, `sensor_baro`, `vehicle_attitude_groundtruth`)

### 阶段 3: Gazebo 模型加载

#### model.sdf 结构

**文件**: `Tools/simulation/gz/models/hnuter/model.sdf` (784 行)

```xml
<sdf version='1.11'>
  <model name='hnuter'>
    <!-- 第 3 行: 初始位姿 -->
    <pose>0 0 0.3 0 0 0</pose>  <!-- X Y Z Roll Pitch Yaw -->

    <!-- 第 4-136 行: 主体链接 (base_link) -->
    <link name='base_link'>
      <!-- 惯性参数 -->
      <inertial>
        <pose>0.036 -0.0005 1.039 0 0 0</pose>
        <mass>5.956</mass>
        <inertia>
          <ixx>0.0053</ixx>
          <ixy>2.89e-06</ixy>
          <ixz>8.78e-06</ixz>
          <iyy>0.0172</iyy>
          <iyz>-1.16e-07</iyz>
          <izz>0.0203</izz>
        </inertia>
      </inertial>

      <!-- 碰撞几何 -->
      <collision name='base_link_collision'>
        <geometry>
          <mesh>
            <uri>meshes/base_link.STL</uri>
          </mesh>
        </geometry>
      </collision>

      <!-- 视觉几何 -->
      <visual name='base_link_visual'>
        <geometry>
          <mesh>
            <uri>meshes/base_link.STL</uri>
          </mesh>
        </geometry>
      </visual>

      <!-- 传感器 -->
      <sensor name="air_pressure_sensor" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
      </sensor>

      <sensor name="magnetometer_sensor" type="magnetometer">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
      </sensor>

      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>250</update_rate>
      </sensor>

      <sensor name="navsat_sensor" type="navsat">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>

    <!-- 第 138-631 行: 倾转关节和链接 -->

    <!-- 左臂主倾转关节 (lj2) -->
    <joint name='lj2' type='revolute'>
      <parent>base_link</parent>
      <child>l2</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>  <!-- -90° -->
          <upper>1.57</upper>   <!-- +90° -->
        </limit>
      </axis>
    </joint>
    <link name='l2'>...</link>

    <!-- 左臂副倾转关节 (lj1) -->
    <joint name='lj1' type='revolute'>
      <parent>l2</parent>
      <child>l1</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>
    <link name='l1'>...</link>

    <!-- 左臂电机 (xy3, xy4) -->
    <joint name='xyj3' type='revolute'>
      <parent>l1</parent>
      <child>xy3</child>
    </joint>
    <link name='xy3'>...</link>

    <joint name='xyj4' type='revolute'>
      <parent>l1</parent>
      <child>xy4</child>
    </joint>
    <link name='xy4'>...</link>

    <!-- 右臂倾转关节 (rj2, rj1) -->
    <joint name='rj2' type='revolute'>...</joint>
    <link name='r2'>...</link>
    <joint name='rj1' type='revolute'>...</joint>
    <link name='r1'>...</link>

    <!-- 右臂电机 (xy1, xy2) -->
    <joint name='xyj1' type='revolute'>...</joint>
    <link name='xy1'>...</link>
    <joint name='xyj2' type='revolute'>...</joint>
    <link name='xy2'>...</link>

    <!-- 尾部电机 (xy5) -->
    <joint name='xyj5' type='revolute'>
      <parent>base_link</parent>
      <child>xy5</child>
    </joint>
    <link name='xy5'>...</link>

    <!-- 第 633-722 行: 电机插件（5 个） -->

    <!-- Motor 0: xy1 (右臂上方, CCW) -->
    <plugin filename="gz-sim-multicopter-motor-model-system"
            name="gz::sim::systems::MulticopterMotorModel">
      <jointName>xyj1</jointName>
      <linkName>xy1</linkName>
      <turningDirection>ccw</turningDirection>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
    </plugin>

    <!-- Motor 1-4: 类似配置 -->

    <!-- 第 725-780 行: 舵机插件（4 个） -->

    <!-- Servo 0: rj2 (右臂主倾转) -->
    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>rj2</joint_name>
      <sub_topic>servo_0</sub_topic>
      <p_gain>20</p_gain>
      <i_gain>0</i_gain>
      <d_gain>0.5</d_gain>
    </plugin>

    <!-- Servo 1-3: 类似配置 -->

  </model>
</sdf>
```

**关键 Gazebo 插件**:

1. **MulticopterMotorModel** (电机):
   - 订阅: `/<model_name>/command/motor_speed`
   - 功能: 根据速度命令生成推力和扭矩
   - 参数: 推力系数、力矩系数、最大转速

2. **JointPositionController** (舵机):
   - 订阅: `/model/<model_name>/<sub_topic>`
   - 功能: PID 位置控制关节角度
   - 参数: P/I/D 增益

---

## 模块初始化

### 启动顺序

**rcS 脚本** (第 282-375 行)

```bash
# 1. 数据管理器
dataman start

# 2. 仿真器（已在 px4-rc.gzsim 中启动）

# 3. 电池和电源仿真
battery_simulator start
system_power_simulator start

# 4. 声音和遥控
tone_alarm start
rc_update start

# 5. 传感器融合
sensors start

# 6. 模式管理和安全
commander start -h

# 7. 状态估计
ekf2 start

# 8. 控制器
mc_att_control start      # 姿态控制
mc_pos_control start      # 位置控制
mc_rate_control start     # 速率控制

# 9. 控制分配
control_allocator start

# 10. 自主飞行
navigator start

# 11. ROS2 通信
uxrce_dds_client start

# 12. 日志
logger start -t -b 1000

# 13. MAVLink 通信
mavlink start -x -u 14556 -r 4000000
mavlink start -x -u 14557 -r 4000000 -m onboard -o 14540

# 14. MAVLink 流配置
mavlink stream -d /dev/mavlink0 -s HIGHRES_IMU -r 50
mavlink stream -d /dev/mavlink1 -s HIGHRES_IMU -r 50

echo "All modules started ✓"
```

### 关键模块说明

| 模块 | 功能 | 订阅消息 | 发布消息 |
|------|------|---------|---------|
| **dataman** | 航点和任务数据管理 | - | - |
| **battery_simulator** | 仿真电池状态 | - | `battery_status` |
| **sensors** | 传感器数据整合 | `sensor_accel`, `sensor_gyro`, ... | `sensor_combined` |
| **commander** | 飞行模式管理、安全检查 | `vehicle_command` | `vehicle_status`, `vehicle_command_ack` |
| **ekf2** | 扩展卡尔曼滤波器 | `sensor_combined`, `vehicle_gps_position` | `vehicle_attitude`, `vehicle_local_position` |
| **mc_att_control** | 多旋翼姿态控制 | `vehicle_attitude_setpoint` | `vehicle_rates_setpoint` |
| **mc_pos_control** | 多旋翼位置控制 | `trajectory_setpoint` | `vehicle_attitude_setpoint` |
| **control_allocator** | 控制分配 | `vehicle_torque_setpoint`, `vehicle_thrust_setpoint` | `actuator_motors`, `actuator_servos` |
| **navigator** | 自主飞行和航线 | `position_setpoint_triplet` | `vehicle_command` |
| **gz_bridge** | Gazebo 桥接 | `actuator_motors`, `actuator_servos` | `sensor_gps`, `sensor_baro`, ... |
| **mavlink** | MAVLink 通信 | 所有 uORB 消息 | MAVLink 消息 |

---

## 完整流程图

```
┌─────────────────────────────────────────────────────────────┐
│  1. 命令行输入                                               │
│  make px4_sitl gz_hnuter                                    │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  2. Makefile 解析                                            │
│  - px4_sitl → px4_sitl_default                              │
│  - 调用 cmake-build 函数                                     │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  3. CMake 配置阶段                                           │
│  ├─ 读取 boards/px4/sitl/default.px4board                   │
│  ├─ 设置 PX4_PLATFORM=posix, PX4_BOARD=px4_sitl            │
│  ├─ 配置 Gazebo 支持 (GZ_BRIDGE, GZ_MSGS, GZ_PLUGINS)      │
│  ├─ 解析所有模块依赖                                         │
│  └─ 生成 build.ninja                                        │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  4. 编译阶段 (Ninja)                                         │
│  ├─ 编译 200+ 模块和库                                       │
│  ├─ 链接主程序: bin/px4                                     │
│  ├─ 生成模块符号链接: px4-commander, px4-ekf2, ...         │
│  └─ 复制 ROMFS 到 etc/                                      │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  5. 启动 PX4 主程序                                          │
│  ./bin/px4 gz_hnuter                                        │
│                                                              │
│  main() → server_main()                                     │
│  ├─ px4::init() - 初始化平台                                 │
│  ├─ chdir(build_path) - 切换工作目录                        │
│  ├─ px4::init_once() - 挂载 ROMFS                           │
│  ├─ setenv("PX4_SIM_MODEL", "hnuter")                       │
│  └─ run_startup_script("etc/init.d-posix/rcS")              │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  6. 执行 rcS 启动脚本                                        │
│  ├─ 加载 px4-alias.sh                                       │
│  ├─ 识别机型: PX4_SIM_MODEL=hnuter                          │
│  │   → 查找 4051_gz_hnuter                                  │
│  │   → SYS_AUTOSTART=4051                                   │
│  ├─ 加载参数文件 parameters.bson                            │
│  └─ 执行机型配置文件                                         │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  7. 加载机型配置                                             │
│  └─ 4051_gz_hnuter (独立完整配置)                           │
│      ├─ Gazebo 仿真设置                                      │
│      │   ├─ PX4_SIMULATOR=gz                                │
│      │   ├─ PX4_GZ_WORLD=default                            │
│      │   ├─ SIM_GZ_EN=1                                     │
│      │   ├─ SIM_GZ_EC_FUNC1-5 (5 个电机)                    │
│      │   └─ SIM_GZ_SV_FUNC1-4 (4 个舵机)                    │
│      ├─ 车辆和控制配置                                       │
│      │   ├─ MAV_TYPE=2                                      │
│      │   ├─ CA_AIRFRAME=16, CA_ROTOR_COUNT=5               │
│      │   ├─ CA_ROTOR0-4_* (电机位置和力矩系数)              │
│      │   ├─ CA_SV_TL_COUNT=0 (倾转舵机已禁用)               │
│      │   ├─ MC_*_P (控制器增益)                             │
│      │   ├─ VT_TYPE=1 (倾转旋翼)                            │
│      │   └─ EKF2/COM_* (状态估计和安全)                     │
│      └─ 共 243 行完整配置                                    │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  8. 执行仿真器启动脚本                                       │
│  ├─ px4-rc.simulator                                        │
│  │   └─ 检测 PX4_SIMULATOR=gz                               │
│  │       → 执行 px4-rc.gzsim                                 │
│  │                                                           │
│  └─ px4-rc.gzsim                                            │
│      ├─ 检查 Gazebo 版本 (>= 8.0)                           │
│      ├─ 设置环境变量 (GZ_SIM_RESOURCE_PATH)                 │
│      ├─ 启动 Gazebo 世界                                     │
│      │   gz sim -r -s default.sdf &                         │
│      │   gz sim -g &  (GUI)                                 │
│      ├─ 生成 hnuter 模型                                     │
│      │   gz service /world/default/create ...               │
│      └─ 启动 gz_bridge                                       │
│          gz_bridge start -w default -n hnuter_0             │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  9. Gazebo 仿真启动                                          │
│  ├─ 加载世界文件: default.sdf                               │
│  ├─ 启动物理引擎                                             │
│  ├─ 加载 hnuter 模型                                         │
│  │   └─ model.sdf                                           │
│  │       ├─ 链接和关节 (base_link, l2, l1, r2, r1, xy1-5)  │
│  │       ├─ 传感器 (IMU, GPS, 气压计, 磁力计)               │
│  │       ├─ 5 个电机插件 (MulticopterMotorModel)            │
│  │       └─ 4 个舵机插件 (JointPositionController)          │
│  └─ 启动 GUI 窗口 (显示 3D 场景)                            │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  10. PX4 模块启动                                            │
│  ├─ dataman start                                           │
│  ├─ battery_simulator start                                 │
│  ├─ sensors start                                           │
│  ├─ commander start                                         │
│  ├─ ekf2 start                                              │
│  ├─ mc_att_control start                                    │
│  ├─ mc_pos_control start                                    │
│  ├─ control_allocator start                                 │
│  ├─ navigator start                                         │
│  ├─ mavlink start (端口 14556, 14557)                       │
│  └─ uxrce_dds_client start (ROS2)                           │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  11. gz_bridge 连接                                          │
│  ├─ PX4 → Gazebo                                            │
│  │   ├─ actuator_motors → /hnuter/command/motor_speed      │
│  │   └─ actuator_servos → /model/hnuter/servo_*            │
│  │                                                           │
│  └─ Gazebo → PX4                                            │
│      ├─ /hnuter/imu → sensor_combined                       │
│      ├─ /hnuter/gps → vehicle_gps_position                  │
│      ├─ /hnuter/baro → sensor_baro                          │
│      └─ /hnuter/pose → vehicle_attitude_groundtruth         │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│  12. 系统就绪                                                │
│  ├─ Gazebo 窗口显示 hnuter 无人机                           │
│  ├─ PX4 命令终端显示 "nsh>" 提示符                          │
│  ├─ MAVLink 监听端口 14556, 14557                           │
│  └─ 等待用户命令 (commander arm, commander takeoff, ...)    │
└─────────────────────────────────────────────────────────────┘
```

---

## 关键文件索引

### 构建系统

| 文件 | 路径 | 功能 |
|------|------|------|
| **顶层 Makefile** | `Makefile` | 构建入口 |
| **顶层 CMakeLists** | `CMakeLists.txt` | CMake 配置入口 |
| **板配置** | `boards/px4/sitl/default.px4board` | SITL 板配置 |
| **板 CMake** | `boards/px4/sitl/sitl.cmake` | SITL CMake 设置 |
| **POSIX 平台** | `platforms/posix/CMakeLists.txt` | POSIX 平台配置 |

### 主程序

| 文件 | 路径 | 功能 |
|------|------|------|
| **主入口** | `platforms/posix/src/px4/common/main.cpp` | main() 函数 |
| **启动脚本** | `ROMFS/px4fmu_common/init.d-posix/rcS` | 主启动脚本 |
| **仿真器选择** | `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator` | 仿真器分发 |
| **Gazebo 启动** | `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim` | Gazebo 启动脚本 |

### 机型配置

| 文件 | 路径 | 功能 |
|------|------|------|
| **Gazebo 机型** | `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter` | Hnuter Gazebo 独立完整配置 |
| **多旋翼默认** | `ROMFS/px4fmu_common/init.d/rc.mc_defaults` | 多旋翼默认参数 |

### Gazebo 资源

| 文件 | 路径 | 功能 |
|------|------|------|
| **模型 SDF** | `Tools/simulation/gz/models/hnuter/model.sdf` | Hnuter 模型定义 |
| **模型网格** | `Tools/simulation/gz/models/hnuter/meshes/*.STL` | 3D 网格文件 |
| **世界文件** | `Tools/simulation/gz/worlds/default.sdf` | 默认世界 |

### 关键模块

| 模块 | 路径 | 功能 |
|------|------|------|
| **Gazebo 桥接** | `src/modules/simulation/gz_bridge/` | PX4 ↔ Gazebo 通信 |
| **Commander** | `src/modules/commander/` | 飞行模式和安全 |
| **Navigator** | `src/modules/navigator/` | 自主飞行 |
| **Control Allocator** | `src/modules/control_allocator/` | 控制分配 |
| **Attitude Control** | `src/modules/mc_att_control/` | 姿态控制 |
| **Position Control** | `src/modules/mc_pos_control/` | 位置控制 |
| **EKF2** | `src/modules/ekf2/` | 状态估计 |

---

## 时间线

从执行命令到系统就绪的典型时间线：

```
T+0s     make px4_sitl gz_hnuter
T+0.1s   Makefile 解析完成
T+0.2s   CMake 配置开始
T+1s     CMake 配置完成
T+2s     Ninja 编译开始
T+30s    编译完成 (首次编译，后续约 5-10s)
T+31s    启动 px4 主程序
T+31.5s  rcS 脚本开始执行
T+32s    机型配置加载完成
T+33s    Gazebo 世界启动
T+35s    Gazebo GUI 打开
T+36s    hnuter 模型生成
T+37s    gz_bridge 启动
T+38s    PX4 模块全部启动
T+39s    系统就绪，显示 "nsh>" 提示符
```

**总耗时**: 约 30-40 秒（首次编译约 1-2 分钟）

---

## 常见问题

### Q1: 如何更改机型？

修改命令行参数：
```bash
# 使用不同的机型
make px4_sitl gz_x500          # X500 四旋翼
make px4_sitl gz_standard_vtol # 标准 VTOL
```

### Q2: 如何禁用 GUI？

设置环境变量：
```bash
HEADLESS=1 make px4_sitl gz_hnuter
```

### Q3: 如何更改世界？

设置环境变量：
```bash
PX4_GZ_WORLD=warehouse make px4_sitl gz_hnuter
```

### Q4: 如何查看详细日志？

```bash
# 构建详细日志
VERBOSE=1 make px4_sitl gz_hnuter

# Gazebo 详细日志
gz sim --verbose=4 ...
```

### Q5: 如何重新编译？

```bash
# 清理构建
make clean

# 重新编译
make px4_sitl gz_hnuter

# 或完全重置
make distclean
make px4_sitl gz_hnuter
```

---

**文档版本**: 1.0
**创建日期**: 2026-02-24
**适用版本**: PX4 v1.14+, Gazebo Harmonic 8.0+
**机型**: Hnuter Tiltrotor
