# Hnuter e0958bbd 实机调试修改与后续问题（2026-07-31）

## 本轮依据

- 对比日志：
  - `logs/log_48_2026-7-30-15-43-58.ulg`：`144bd9fe`
  - `logs/log_53_2026-7-30-21-32-46.ulg`：`e0958bbd`
- Pitch 在多段约 20--27 deg 姿态下能够保持厘米级位置误差。
- Roll 操作约 10 deg 后位置误差明显增大，并伴随约 3--4 Hz 的 roll
  torque/二级倾转差动分量。
- `e0958bbd` 日志没有飞行中参数修改，实飞保存值包括：
  `HNTR_ATT_KR_R=18.2`、`HNTR_ATT_D_R=9.6`、`HNTR_TAU_R=56.3`、
  `HNTR_VEL_I_XY=0.39`、`HNTR_LND_GC_R=0.75`、`HNTR_LND_MIN_R=0.60`。

## 本轮已修改

1. 实机 Roll 调试参数改为：

   ```text
   HNTR_ATT_KR_R = 10.0
   HNTR_ATT_D_R  = 4.0
   HNTR_TAU_R    = 15.0 Nm
   ```

2. `HNTR_VEL_I_XY` 实机默认值改为 `0.20`。
3. Hnuter 着陆阈值改为：

   ```text
   HNTR_MOT_HOV   = 0.50
   HNTR_LND_GC_R  = 0.85
   HNTR_LND_MIN_R = 0.80
   ```

4. `HnuterControl` 的消息新鲜度和解锁计时改用 `hrt_absolute_time()`。
   `vehicle_angular_velocity.timestamp_sample` 只继续作为输出采样时间，避免新发布的
   RC 消息比旧传感器采样时间略晚时，错误清空 AUX 姿态/航向保持状态。
5. 硬件 airframe 只对日志中确认的旧参数值做条件迁移；后续人工调参不会在每次开机
   被覆盖。刷写后仍应在 QGC 核对本节参数。

## 暂缓、下一阶段处理

1. **二级倾转实测角反馈**
   - 当前 `actuator_servos` 只有命令，没有实际角度。
   - 优先增加左右二级倾转输出轴角度、角速度和有效标志。
   - 识别正反向速率、纯延迟、带载下垂、回差和左右同步误差。

2. **Roll 侧向力的偏航力矩补偿**
   - 当前实机 allocator 未实现 `Tz_comp = HNTR_YAW_X * Fy`。
   - 先用小角度 Roll 和实测角度辨识符号及有效力臂，再加入分配器。
   - 禁止直接复制 Gazebo 的 `HNTR_YAW_X=0.027 m` 到实机。

3. **真实可达作用力和控制分配残差**
   - 当前 Hnuter allocator 的 unallocated wrench 为占位零。
   - 后续应使用实测/估计舵机角度和限速后的角度重算 achieved wrench，
     并将残差用于姿态和位置积分抗饱和。

4. **大扰动垂直推力安全裕度**
   - Position 模式增加垂直推力余量、舵机跟踪误差和 XY 误差保护。
   - 在反馈与几何补偿完成前，Roll 测试限制在正负 7--8 deg，
     `HNTR_RC_RATE_R` 限制在 5--8 deg/s。

5. **补充日志**
   - 记录实际/期望四舵机角、姿态目标、姿态积分、XY 速度积分、期望/实际 `Fy`、
     yaw 补偿量和真实分配残差。

## 下一次实机测试顺序

1. 刷写后读取并保存完整参数，确认本轮上述关键参数值。
2. 水平 Position 悬停 30 s，检查 Roll 角速度频谱和着陆自动上锁。
3. AUX1 单轴测试：`0 -> +5 -> 0 -> -5 -> 0 deg`，每段保持 5--10 s。
4. 无连续 3--4 Hz 振荡且 XY 误差可控后，再扩到正负 7--8 deg。
5. 暂不进行大 Roll、Fy--Yaw 补偿或无反馈的 180 deg 倾转测试。

## 本轮编译验证

- `make px4_sitl_default`：成功，产物
  `build/px4_sitl_default/bin/px4`，SHA-256
  `2bab2f4da799cabf65d50b8b9139d8791f4a670ce804cf34533351e5d033d7a1`。
- `CCACHE_DIR=/tmp/ccache make cuav_7-nano_default`：成功，FLASH
  使用 `1866428 / 1966080 bytes (94.93%)`，产物
  `build/cuav_7-nano_default/cuav_7-nano_default.px4`，SHA-256
  `2d0278d3e746d9653246304cb2de382ea9dad82e9a81dbb8b9d98ef95431993c`。
- 已从最终 `romfs_files.tar` 反向核对硬件 airframe，确认本轮参数和条件迁移
  已进入飞控固件包。
