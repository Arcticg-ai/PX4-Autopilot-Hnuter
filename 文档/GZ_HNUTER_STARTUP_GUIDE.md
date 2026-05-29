# GZ_HNUTER 仿真启动指南

## 问题分析

通过分析代码库和测试，发现以下关键信息：

1. **机型配置文件存在**：
   - `/home/hnuter/PX4-Autopilot-Hnuter/ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter` - Gazebo仿真环境下的HNUTER倾转旋翼配置
   - `/home/hnuter/PX4-Autopilot-Hnuter/ROMFS/px4fmu_common/init.d-posix/airframes/9001_hnuter_tiltrotor` - 通用HNUTER倾转旋翼配置
   - 这些文件已在CMakeLists.txt中注册

2. **编译和启动流程**：
   - `make px4_sitl gz_hnuter`命令通过CMake在`build/px4_sitl_default`目录中构建
   - 正确的启动方式是使用`cmake --build . --target gz_hnuter`

3. **Gazebo窗口打开问题**：
   - PX4 启动时会**自动**启动 Gazebo 服务器和 GUI（`gz sim -g`）
   - 若画面不显示，请检查：未设置 `HEADLESS=1`、DISPLAY 环境变量正确、Gazebo Harmonic 已安装

## 正确的启动流程

### 步骤1：清理并重新构建项目

1. 清理现有的build目录（如果存在多个build空间）：
   ```bash
   rm -rf build/
   ```

2. 构建项目（需网络畅通，会下载子模块如 OpticalFlow）：
   ```bash
   make px4_sitl_default
   ```

### 步骤2：启动Gazebo仿真

**方式 A（推荐）**：从项目根目录一条命令启动
   ```bash
   cd /home/hnuter/PX4-Autopilot-Hnuter
   make px4_sitl gz_hnuter
   ```

**方式 B**：使用 CMake 目标
   ```bash
   cd build/px4_sitl_default
   cmake --build . --target gz_hnuter
   ```

以上命令会：
- 从 `build/px4_sitl_default/rootfs` 目录启动 PX4
- 自动加载 `gz_env.sh` 设置模型和世界路径
- 自动启动 Gazebo 服务器和 GUI 窗口
- 生成 hnuter 模型并连接 PX4

### 步骤3：Gazebo 可视化界面

**无需单独执行 `gz sim -g`**。PX4 的 `px4-rc.gzsim` 脚本会在启动时自动执行 `gz sim -g` 打开 GUI。若使用 SSH 远程连接且无图形界面，需配置 X11 转发或 VNC。

## 故障排查指南

### 问题1：CMake命令执行失败

**症状**：
- 执行`cmake --build . --target gz_hnuter`时出现错误

**解决方案**：
1. 确认build目录存在且包含正确的CMake配置
2. 重新构建项目：
   ```bash
   make px4_sitl_default
   ```
3. 检查机型配置文件是否正确注册在CMakeLists.txt中

### 问题2：Gazebo无法找到模型

**症状**：
- Gazebo启动时显示模型加载错误
- 错误信息包含"model not found"

**解决方案**：
1. 检查模型文件是否存在于正确位置：
   ```bash
   ls -la Tools/simulation/gz/models/hnuter/
   ```
2. 确保从正确目录启动：必须从 `build/px4_sitl_default/rootfs` 运行，以便自动加载 `gz_env.sh`（该文件由 CMake 生成，包含 PX4_GZ_MODELS 等路径）

### 问题3：PX4无法连接到Gazebo

**症状**：
- PX4启动时显示"gz_bridge failed to start"
- 错误信息包含"connection refused"

**解决方案**：
1. 确认Gazebo已经启动
2. 检查网络连接和端口设置
3. 重新启动Gazebo和PX4

### 问题4：地面站无法显示新机架

**症状**：
- QGroundControl中无法找到"hnuter tiltrotor"机架

**解决方案**：
1. 确认机型配置文件中的元数据正确：
   - `@name hnuter tiltrotor`
   - `@type tiltrotor`
   - `@class Tiltrotor`

2. 确认控制分配器中的机架类型定义正确：
   - `CA_AIRFRAME` 参数设置为 16（对应 `HNUTER_TILTROTOR`）

3. 重新构建并上传固件到地面站

## 技术要点

1. **CMake构建系统**：
   - 使用CMake命令构建和启动目标
   - 避免使用多个build空间，保持构建环境的一致性

2. **机型配置**：
   - 机型配置文件格式正确，包含必要的元数据
   - 控制分配器参数设置正确，对应正确的机架类型

3. **Gazebo集成**：
   - 理解PX4与Gazebo的集成方式，特别是启动流程
   - 正确设置环境变量，确保Gazebo能够找到模型和插件

4. **地面站兼容性**：
   - 确保新机架能够在地面站中正确显示和配置
   - 验证机型元数据和参数设置正确

## 启动脚本

以下是完整的启动脚本，可保存为`start_gz_hnuter.sh`并执行：

```bash
#!/bin/bash

# 进入项目根目录
cd "$(dirname "$0")"

# 构建项目
echo "构建项目..."
make px4_sitl_default

# 启动Gazebo仿真（推荐：一条命令完成）
echo "启动Gazebo仿真..."
make px4_sitl gz_hnuter
```

## 总结

正确的启动流程是：
1. 使用`make px4_sitl_default`构建项目
2. 使用`make px4_sitl gz_hnuter`启动仿真（从项目根目录）
3. Gazebo GUI 会由 PX4 自动启动，无需单独执行 `gz sim -g`

**重要**：必须从项目根目录或使用上述 make 命令启动，以确保工作目录为 `rootfs`，从而正确加载 `gz_env.sh`。

更多问题排查请参考 `HNUTER_GAZEBO_TROUBLESHOOTING.md`。