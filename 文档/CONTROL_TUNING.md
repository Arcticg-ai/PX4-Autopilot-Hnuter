# 控制参数调整 - 降低激进度

## 问题

✅ 推力方向已修复，所有电机推力向上
❌ 无人机起飞太激进，直接飞跑了

## 已完成的调整

### 1. 外部控制器增益（~/px4_ros2_ws/hnuter_external_controller.py）

#### 位置和速度控制增益
```python
# 旧值（过于激进）
self.Kp = np.diag([50, 50, 50])     # Position gain
self.Dp = np.diag([25, 25, 25])     # Velocity damping

# 新值（温和）
self.Kp = np.diag([8, 8, 10])       # Position gain - 降低6倍
self.Dp = np.diag([6, 6, 8])        # Velocity damping - 降低4倍
```

**效果**：
- 位置误差响应更平缓
- 减少超调和震荡
- Z轴略高保证能起飞

#### 姿态控制增益
```python
# 旧值
self.KR = np.array([8, 6.0, 1.5])
self.Domega = np.array([2.5, 2.0, 2.0])

# 新值
self.KR = np.array([4, 3.5, 1.0])
self.Domega = np.array([1.5, 1.2, 1.2])
```

**效果**：
- 姿态响应更柔和
- 减少姿态震荡

### 2. 推力限制

```python
# 旧值（太大）
T_max = 320.0   # 单旋翼组最大推力
T5_max = 40.0   # 尾推最大推力

# 新值（合理）
T_max = 80.0    # 降低到80N (单电机40N)
T5_max = 20.0   # 降低到20N
```

**悬停需求**：
- 质量: 5.956 kg
- 悬停推力: 5.956 × 9.81 = 58.4 N
- T_max = 80N 提供 **1.37倍推重比** ✓

### 3. 舵机角度限制

```python
# 旧值
alpha_max = np.radians(200.0)  # 200度
theta_max = np.radians(200.0)

# 新值
alpha_max = np.radians(90.0)   # 90度
theta_max = np.radians(90.0)
```

**效果**：防止舵机过度倾转

### 4. 起飞轨迹优化

```python
# 旧轨迹（太快）
0: 0~3s  - 0.5m
1: 3~6s  - 1.0m
2: 6~9s  - 1.5m
3: 9~12s - 2.0m

# 新轨迹（更平缓）
0: 0~4s  - 0.3m  ← 初始目标降低
1: 4~8s  - 0.8m  ← 延长时间
2: 8~12s - 1.3m
3: 12~16s - 2.0m
```

**改进**：
- 初始目标从0.5m降到0.3m
- 每阶段时间从3s延长到4s
- 总用时从12s延长到16s

### 5. 删除测试代码

- ✅ 删除舵机30度强制测试
- ✅ 恢复正常控制逻辑

## 预期飞行表现

### 起飞阶段（0-4s）
- 缓慢离地
- 平稳爬升到0.3m
- 姿态保持水平
- 无明显震荡

### 爬升阶段（4-16s）
- 分4个阶段逐步爬升
- 每个阶段用时4秒
- 最终到达2m高度

### 悬停阶段（16s+）
- 稳定悬停在2m
- 位置误差 < 0.1m
- 姿态角 < 5度

## 测试步骤

### 1. 重启 Gazebo
```bash
cd ~/PX4-Autopilot-Hnuter
make px4_sitl gz_hnuter
```

### 2. 运行控制器
```bash
cd ~/px4_ros2_ws
./run_hnuter_controller.sh
```

### 3. 观察飞行

**正常表现**：
- ✅ 平稳起飞，无突然冲刺
- ✅ 逐步爬升，无大幅震荡
- ✅ 姿态平稳，无剧烈晃动
- ✅ 最终稳定悬停在2m

**异常表现需要调整**：
- ❌ 起飞太慢/无法起飞 → 提高 Kp_z
- ❌ 起飞还是太猛 → 降低 Kp, Dp
- ❌ 悬停震荡 → 降低 Kp, 提高 Dp
- ❌ 姿态不稳 → 调整 KR, Domega

## PX4内部参数（可选调整）

如果外部控制器调整后还不理想，可以调整PX4内部参数：

### 降低姿态控制增益
编辑 `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`:

```bash
# 当前值
param set-default MC_ROLL_P 6.0
param set-default MC_PITCH_P 6.0
param set-default MC_YAW_P 4.0

# 可以降低到
param set-default MC_ROLL_P 4.0
param set-default MC_PITCH_P 4.0
param set-default MC_YAW_P 3.0
```

## 增益调整指南

### 如果起飞太慢
```python
# 只提高Z轴增益
self.Kp = np.diag([8, 8, 12])  # Z轴从10提高到12
```

### 如果起飞还是太快
```python
# 进一步降低增益
self.Kp = np.diag([6, 6, 8])   # 降低到6, 6, 8
self.Dp = np.diag([5, 5, 6])   # 降低到5, 5, 6
```

### 如果悬停震荡
```python
# 提高阻尼
self.Dp = np.diag([8, 8, 10])  # 提高阻尼
```

### 如果姿态不稳
```python
# 调整姿态增益
self.KR = np.array([3, 2.5, 0.8])     # 降低姿态增益
self.Domega = np.array([2.0, 1.5, 1.5])  # 提高角速度阻尼
```

## 完整参数表

### 外部控制器当前参数
```python
# 位置控制
Kp = [8, 8, 10]
Dp = [6, 6, 8]

# 姿态控制
KR = [4, 3.5, 1.0]
Domega = [1.5, 1.2, 1.2]

# 推力限制
T_max = 80.0 N
T5_max = 20.0 N

# 角度限制
alpha_max = 90°
theta_max = 90°

# 物理参数
mass = 5.956 kg
g = 9.81 m/s²
hover_thrust = 58.4 N
```

## 下一步

测试飞行后，根据实际表现微调参数。目标是：
1. 平稳起飞（无突然加速）
2. 稳定爬升（无大幅震荡）
3. 精确悬停（位置误差 < 10cm）
4. 姿态平稳（角度 < 5°）
