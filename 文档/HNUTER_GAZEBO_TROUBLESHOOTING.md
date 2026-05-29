# HNUTER Gazebo 仿真问题梳理与解决方案

本文档梳理了 HNUTER 倾转旋翼机型在 PX4 Gazebo 仿真中无法打开画面的问题，并给出完整的配置检查清单和解决方案。

## 一、正确的启动流程

### 方法 1：使用 make 命令（推荐）

```bash
cd /home/hnuter/PX4-Autopilot-Hnuter

# 1. 构建项目（需确保网络畅通，会下载子模块）
make px4_sitl_default

# 2. 启动 Gazebo 仿真（一条命令完成）
make px4_sitl gz_hnuter
```

**注意**：`make px4_sitl gz_hnuter` 会：
- 自动设置 `PX4_SIM_MODEL=gz_hnuter`
- 从 `build/px4_sitl_default/rootfs` 目录启动 px4
- PX4 会自动启动 Gazebo 服务器和 GUI 窗口

### 方法 2：手动启动（当 make 目标不可用时）

```bash
cd /home/hnuter/PX4-Autopilot-Hnuter/build/px4_sitl_default

# 从 rootfs 目录运行（重要！gz_env.sh 在此目录）
cd rootfs
PX4_SYS_AUTOSTART=4051 PX4_SIMULATOR=gz PX4_GZ_WORLD=default PX4_SIM_MODEL=hnuter ../bin/px4
```

### 方法 3：使用 cmake 目标

```bash
cd /home/hnuter/PX4-Autopilot-Hnuter/build/px4_sitl_default
cmake --build . --target gz_hnuter
```

---

## 二、已发现的问题清单

### 问题 1：文档中的错误路径

| 文档 | 错误内容 | 正确内容 |
|------|----------|----------|
| USING_HNUTER_MODEL.md | `./build_new/bin/px4` | `./build/px4_sitl_default/bin/px4` 且需从 rootfs 目录运行 |
| GZ_HNUTER_STARTUP_GUIDE.md | 建议单独运行 `gz sim -g` | PX4 启动时会自动启动 GUI，无需单独运行 |

### 问题 2：工作目录要求

PX4 必须从 `build/px4_sitl_default/rootfs` 目录启动，因为：
- `px4-rc.gzsim` 脚本会查找 `./gz_env.sh` 或 `../gz_env.sh`
- `gz_env.sh` 由 CMake 生成在 `rootfs/` 目录
- 该脚本设置 `GZ_SIM_RESOURCE_PATH`，使 Gazebo 能找到 models 和 worlds

### 问题 3：构建依赖网络

当前构建失败于 OpticalFlow 子模块下载（无法连接 GitHub）。解决方法：
- 配置网络代理（如需要）
- 或临时禁用 OpticalFlow 相关功能后构建
- 确保 `git submodule update --init --recursive` 能成功执行

### 问题 4：HNUTER 模型缺少 Gazebo 插件（关键）

**当前状态**：`Tools/simulation/gz/models/hnuter/model.sdf` 是纯机械 CAD 模型，**没有** PX4 所需的仿真插件。

**对比 tiltrotor 模型**，需要以下插件才能与 PX4 通信：

1. **MulticopterMotorModel**（每个旋翼一个）：
   - 订阅 `command/motor_speed` 话题
   - 将 PX4 的电机指令转换为 Gazebo 中的旋转
   - 需要 `jointName`、`linkName` 与模型中的关节/连杆对应

2. **JointPositionController**（每个倾转伺服一个）：
   - 控制倾转关节角度
   - 订阅位置指令话题

**影响**：即使 Gazebo 能启动并生成模型，电机也不会响应 PX4 指令，飞行器无法飞行。

### 问题 5：不存在的 setup_gz.bash

GZ_HNUTER_STARTUP_GUIDE.md 中提到 `source Tools/simulation/gz/setup_gz.bash`，但该文件在仓库中不存在。Gazebo 环境由 `rootfs/gz_env.sh` 在 PX4 启动时自动加载。

---

## 三、配置检查清单

在启动前请确认：

- [ ] **Gazebo Harmonic 已安装**：`gz sim --versions` 能输出 8.x 版本
- [ ] **hnuter 模型存在**：`ls Tools/simulation/gz/models/hnuter/model.sdf`
- [ ] **机型配置已注册**：`ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter` 存在
- [ ] **CMakeLists 已包含**：`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` 中有 `4051_gz_hnuter`
- [ ] **default 世界存在**：`Tools/simulation/gz/worlds/default.sdf` 存在
- [ ] **从正确目录启动**：在 `build/px4_sitl_default/rootfs` 下运行 px4
- [ ] **未设置 HEADLESS**：若设置了 `HEADLESS=1`，Gazebo GUI 不会打开

---

## 四、Gazebo 画面不显示的常见原因

1. **HEADLESS 模式**：检查是否设置了 `HEADLESS=1`，若有则去掉
2. **无显示环境**：SSH 远程时可能无 DISPLAY，需要 X11 转发或 VNC
3. **gz_env.sh 未加载**：必须从 rootfs 目录启动，确保能找到 gz_env.sh
4. **Gazebo 未安装**：需安装 `gz-harmonic`：https://gazebosim.org/docs/harmonic/install_ubuntu/

---

## 五、模型适配 PX4 的后续工作

要使 HNUTER 模型真正可飞，需要：

1. **识别模型中的旋翼**：确定对应 `rotor_0`~`rotor_4` 的 joint 和 link
2. **添加 MulticopterMotorModel 插件**：为每个旋翼添加，并正确配置 `jointName`、`linkName`、`motorNumber`
3. **识别倾转关节**：确定 lj1、lj2、xyj3 等中哪些是倾转伺服
4. **添加 JointPositionController**：为倾转关节配置位置控制
5. **对齐执行器映射**：4051_gz_hnuter 中 SIM_GZ_EC_FUNC1-7 需与模型中的电机/伺服一一对应

可参考 `Tools/simulation/gz/models/tiltrotor/model.sdf` 的插件配置方式。

---

## 六、快速验证命令

```bash
# 验证 Gazebo 安装
gz sim --versions

# 验证模型路径
ls -la /home/hnuter/PX4-Autopilot-Hnuter/Tools/simulation/gz/models/hnuter/

# 验证 gz_env.sh 内容
cat /home/hnuter/PX4-Autopilot-Hnuter/build/px4_sitl_default/rootfs/gz_env.sh

# 验证 gz_hnuter 目标存在（需先成功构建）
cd build/px4_sitl_default && ninja -t targets | grep gz_hnuter
```

---

## 七、参考文档

- [PX4 Gazebo 仿真官方文档](https://docs.px4.io/main/en/simulation/gazebo.html)
- [Gazebo Harmonic 安装](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- 项目内：`HNUTER_TILTROTOR_INTEGRATION.md`、`USING_HNUTER_MODEL.md`
