# Gazebo 仿真完整流程代码分析报告

本文档对 PX4 Gazebo 仿真流程进行完整代码级分析，识别存在的问题和缺失环节。

---

## 一、仿真启动流程（代码链路）

### 1.1 入口与工作目录

```
main() [platforms/posix/src/px4/common/main.cpp]
  └─ chdir(working_directory)  // 默认: build/px4_sitl_default/rootfs
  └─ 执行 rcS 脚本
```

**关键**：PX4 启动时会将工作目录切换到 `rootfs`，后续脚本需能在此目录找到 `gz_env.sh`。

### 1.2 启动脚本链

```
rcS [ROMFS/init.d-posix/rcS]
  ├─ . px4-alias.sh          // 设置 px4_instance=0 (默认)
  ├─ 根据 PX4_SYS_AUTOSTART 或 PX4_SIM_MODEL 确定 SYS_AUTOSTART
  ├─ 加载机型配置: . ${R}etc/init.d-posix/airframes/${SYS_AUTOSTART}_*
  │     └─ 4051_gz_hnuter → 设置 PX4_SIMULATOR=gz, PX4_SIM_MODEL=hnuter 等
  └─ . px4-rc.simulator
```

### 1.3 仿真器选择

```
px4-rc.simulator
  ├─ PX4_SIMULATOR=gz 或 SIM_GZ_EN=1 → . px4-rc.gzsim
  └─ 其他 → sihsim / jmavsim / mavlinksim
```

### 1.4 Gazebo 启动 (px4-rc.gzsim)

```
px4-rc.gzsim
  ├─ 1. 检查 gz sim 版本 (>= 8.0.0)
  ├─ 2. 无 PX4_GZ_STANDALONE 时:
  │     ├─ 检查是否已有 Gazebo 运行 (gz topic -l)
  │     ├─ 若无: 加载 gz_env.sh (./gz_env.sh 或 ../gz_env.sh)
  │     ├─ 启动 Gazebo 服务器: gz sim -r -s ${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf &
  │     └─ 非 HEADLESS 时: gz sim -g &  (启动 GUI)
  ├─ 3. 等待 world 就绪 (check_scene_info, 最多 30 秒)
  ├─ 4. 生成模型:
  │     ├─ MODEL_NAME="${PX4_SIM_MODEL#*gz_}"  // gz_hnuter→hnuter
  │     ├─ MODEL_NAME_INSTANCE="${MODEL_NAME}_${px4_instance}"  // hnuter_0
  │     ├─ sdf_str="<include><uri>file://${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf</uri></include>"
  │     └─ gz service -s /world/${PX4_GZ_WORLD}/create ...
  └─ 5. 启动 gz_bridge: gz_bridge start -w ${PX4_GZ_WORLD} -n ${MODEL_NAME_INSTANCE}
```

### 1.5 gz_env.sh 内容（由 CMake 生成）

```bash
# 位置: build/px4_sitl_default/rootfs/gz_env.sh
export PX4_GZ_MODELS=.../Tools/simulation/gz/models
export PX4_GZ_WORLDS=.../Tools/simulation/gz/worlds
export PX4_GZ_PLUGINS=.../build/.../gz_plugins
export PX4_GZ_SERVER_CONFIG=.../gz_bridge/server.config
export GZ_SIM_RESOURCE_PATH=...:$PX4_GZ_MODELS:$PX4_GZ_WORLDS
export GZ_SIM_SYSTEM_PLUGIN_PATH=...:$PX4_GZ_PLUGINS
export GZ_SIM_SERVER_CONFIG_PATH=$PX4_GZ_SERVER_CONFIG
```

---

## 二、GZBridge 模块（传感器与执行器桥接）

### 2.1 初始化顺序

```
GZBridge::init()
  ├─ subscribeClock()      // 必需
  ├─ 等待 _realtime_clock_set
  ├─ subscribePoseInfo()   // 必需
  ├─ subscribeImu()        // 必需
  ├─ subscribeMag()        // 必需
  ├─ subscribeNavsat()     // 若 SIM_GZ_EN_GPS=1
  ├─ subscribeAirPressure() // 若 SIM_GZ_EN_BARO=1
  ├─ subscribeAirPressure/Lidar/Flow/Odom 等
  ├─ _mixing_interface_esc.init(_model_name)
  ├─ _mixing_interface_servo.init(_model_name)
  ├─ _mixing_interface_wheel.init(_model_name)
  └─ _gimbal.init()
```

### 2.2 话题订阅（硬编码路径）

| 数据类型 | 话题路径 | hnuter 模型匹配 |
|---------|----------|-----------------|
| IMU | `/world/{world}/model/{model}/link/base_link/sensor/imu_sensor/imu` | ✅ 有 imu_sensor |
| 磁力计 | `.../sensor/magnetometer_sensor/magnetometer` | ✅ 有 magnetometer_sensor |
| 气压 | `.../sensor/air_pressure_sensor/air_pressure` | ✅ 有 air_pressure_sensor |
| GPS | `.../sensor/navsat_sensor/navsat` | ✅ 有 navsat_sensor |
| 姿态 | `/world/{world}/pose/info` (Pose_V 中按 model 名过滤) | ✅ 需 model 名 hnuter_0 |

### 2.3 执行器接口

**GZMixingInterfaceESC**（电机）：
- 发布: `/{model_name}/command/motor_speed` (如 `/hnuter_0/command/motor_speed`)
- 订阅: 同上（用于 ESC 反馈）
- 依赖: 模型内 **MulticopterMotorModel** 插件需订阅 `command/motor_speed`

**GZMixingInterfaceServo**（舵机）：
- 发布: `/model/{model_name}/servo_{0-7}` (如 `/model/hnuter_0/servo_0`)
- 依赖: 模型内 **JointPositionController** 插件需订阅 `servo_N`

---

## 三、已发现的问题

### 问题 1：hnuter 模型缺少仿真插件（严重）

**位置**：`Tools/simulation/gz/models/hnuter/model.sdf`

**现状**：
- 仅有 IMU、气压、磁力计、GPS 传感器（与 GZBridge 订阅匹配）
- **无** MulticopterMotorModel 插件
- **无** JointPositionController 插件

**影响**：
- 电机不会响应 PX4 的 `command/motor_speed`
- 倾转舵机不会响应
- 模型可加载、传感器可工作，但无法飞行

**对比**：tiltrotor 模型有 4 个 MulticopterMotorModel、多个 JointPositionController

### 问题 2：rcS 中 PX4_SIM_MODEL 与机型匹配逻辑

**位置**：`ROMFS/init.d-posix/rcS` 第 56 行

```sh
REQUESTED_AUTOSTART=$(ls ... | sed -n 's/^\([0-9][0-9]*\)_'${PX4_SIM_MODEL}'$/\1/p')
```

- `PX4_SIM_MODEL=gz_hnuter` → 匹配 `4051_gz_hnuter` ✅
- `PX4_SIM_MODEL=hnuter` → 匹配 `*_hnuter`，但 `4051_gz_hnuter` 不以 `_hnuter` 结尾，`9001_hnuter_tiltrotor` 也不以 `_hnuter` 结尾 → **无匹配** ❌

**结论**：手动启动时必须设置 `PX4_SYS_AUTOSTART=4051`，不能仅用 `PX4_SIM_MODEL=hnuter`。

### 问题 3：server.config 依赖可能缺失的插件

**位置**：`src/modules/simulation/gz_bridge/server.config`

```xml
<plugin entity_name="*" entity_type="world" filename="libOpticalFlowSystem.so" .../>
<plugin entity_name="*" entity_type="world" filename="libGstCameraSystem.so" .../>
```

**依赖**：OpticalFlowSystem 依赖 PX4-OpticalFlow 子模块（需从 GitHub 下载）

**影响**：若构建失败（如网络问题），Gazebo 启动时可能因缺少 `.so` 而报错。

### 问题 4：gz_env.sh 查找路径

**位置**：`px4-rc.gzsim` 第 40-45 行

```sh
if [ -f ./gz_env.sh ]; then
    . ./gz_env.sh
elif [ -f ../gz_env.sh ]; then
    . ../gz_env.sh
fi
```

**依赖**：脚本执行时当前目录必须为 `rootfs`，才能找到 `./gz_env.sh`。

**验证**：CMake 目标 `gz_hnuter` 的 `WORKING_DIRECTORY` 为 `SITL_WORKING_DIR`（即 rootfs），满足要求 ✅

### 问题 5：模型生成时的 SDF 路径

**位置**：`px4-rc.gzsim` 第 136 行

```sh
sdf_str="<sdf version=\"1.6\"> <include> <uri>file://${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf</uri> ..."
```

**要求**：`${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf` 必须存在

**验证**：`Tools/simulation/gz/models/hnuter/model.sdf` 存在 ✅

---

## 四、流程完整性检查清单

| 环节 | 状态 | 说明 |
|------|------|------|
| rcS → px4-rc.simulator | ✅ | 逻辑正确 |
| px4-rc.simulator → px4-rc.gzsim | ✅ | PX4_SIMULATOR=gz 或 SIM_GZ_EN=1 → gz |
| gz_env.sh 加载 | ✅ | 从 rootfs 执行时可找到 |
| Gazebo 服务器启动 | ✅ | gz sim -r -s ${world}.sdf |
| Gazebo GUI 启动 | ✅ | 非 HEADLESS 时 gz sim -g |
| 模型生成 | ✅ | EntityFactory create，路径正确 |
| gz_bridge 启动 | ✅ | gz_bridge start -w -n |
| GZBridge 传感器订阅 | ✅ | hnuter 传感器名称匹配 |
| GZBridge 执行器发布 | ⚠️ | 话题存在，但模型无对应插件 |
| 电机控制 | ❌ | 无 MulticopterMotorModel |
| 舵机控制 | ❌ | 无 JointPositionController |

---

## 五、修复建议

### 5.1 模型层面（必须）

在 `hnuter/model.sdf` 中补充：

1. **MulticopterMotorModel**（每个旋翼）：
   - 确定对应旋翼的 joint/link（如 rotor_0_joint, rotor_0）
   - 配置 `commandSubTopic` 为 `command/motor_speed`
   - 参考 `tiltrotor/model.sdf` 或 `x500/model.sdf`

2. **JointPositionController**（每个倾转关节）：
   - 确定倾转关节（如 lj1, lj2 等）
   - 配置 `sub_topic` 为 `servo_0`, `servo_1` 等
   - 与 4051_gz_hnuter 中 SIM_GZ_EC_FUNC 映射一致

### 5.2 启动方式

- 使用 `make px4_sitl gz_hnuter`，或
- 手动启动时设置 `PX4_SYS_AUTOSTART=4051`，不要只用 `PX4_SIM_MODEL=hnuter` 推断机型

### 5.3 构建

- 若网络导致 OpticalFlow 下载失败，需解决网络或代理
- 或临时修改 server.config，注释掉 OpticalFlowSystem、GstCameraSystem（若不需要）

---

## 六、参考文件

- `tiltrotor/model.sdf`：MulticopterMotorModel、JointPositionController 配置示例
- `x500/model.sdf`：多旋翼电机配置示例
- `4051_gz_hnuter`：机型与仿真参数
