# XUC 精简合并协议 v3

本文档供上位机开发使用，配合 `xuc.h` / `xuc.cpp` 一起阅读。

## 1. 概览

- 自瞄和导航合并成**一帧一个方向**。
- 去掉了 `bullet_speed_TJ`、`bullet_count_TJ`。
- 自瞄继续使用云台 IMU 的 `imu_yaw_TJ` / `imu_pitch_TJ`。
- 导航底盘航向由上位机用**雷达自带 IMU 的 yaw** + 下位机发的 `gimbal_relative_angle` 合成，不再下发/回传底盘 `wz`。

## 2. 通用约定

| 项目 | 约定 |
| --- | --- |
| 帧头 | `'S','P'` |
| 字节序 | 小端 |
| 浮点 | IEEE-754 32 位 `float` |
| 结构体对齐 | `#pragma pack(1)`，可整块 `memcpy` |
| CRC16 | CRC-16/MODBUS（反射多项式 0xA001，初值 `0xFFFF`），低字节在前 |
| CRC 范围 | 整帧除去最后 2 字节 |

## 3. 坐标系与方向（重点）

命令和反馈使用不同的坐标系：

| 量 | 坐标系 | 方向定义 |
| --- | --- | --- |
| `nav_vx` | **云台系**（= 雷达系） | 沿云台正前方，前为正 |
| `nav_vy` | **云台系**（= 雷达系） | 沿云台正右方，右为正 |
| `chassis_vx` | **底盘系**（固定） | 沿底盘正前方，前为正 |
| `chassis_vy` | **底盘系**（固定） | 沿底盘正左方，左为正 |
| `gimbal_relative_angle` | 云台相对底盘 | 正值 = 云台相对底盘**向左转** |

`gimbal_relative_angle` 说明：

- 由下位机的 6020 yaw 电机编码器相对上电初值的差值换算得到；
- **编码差值增大 = 云台左转**，因此该字段正值表示云台左转，负值表示右转；
- 单位 rad，范围约 ±π，已做回绕处理。

底盘航向合成（上位机）：

```text
chassis_yaw = radar_yaw + gimbal_relative_angle
```

其中 `radar_yaw` 是雷达自带 IMU 的 yaw（与云台 IMU yaw 一致）。本式基于：云台左转时 `radar_yaw` 减小、`gimbal_relative_angle` 增大，相加后底盘航向保持不变。若实车符号相反，把加号改成减号。

## 4. 下行帧：上位机 -> 下位机

`RxPacket_TJ`，共 **25 字节**。

| 偏移 | 大小 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | 2 | `head` | `'S','P'` |
| 2 | 1 | `control_TJ` | 云台控制，1 = 上位机在控 |
| 3 | 1 | `shoot_TJ` | 0 停 / 1 单发 |
| 4 | 4 | `yaw_TJ` | 云台目标 yaw，rad |
| 8 | 4 | `pitch_TJ` | 云台目标 pitch，rad |
| 12 | 1 | `nav_control` | 0 撤控 / 1 导航接管 |
| 13 | 2 | `nav_seq` | 导航命令序号 |
| 15 | 4 | `nav_vx` | m/s，云台系前为正 |
| 19 | 4 | `nav_vy` | m/s，云台系右为正 |
| 23 | 2 | `crc16_TJ` | 低字节在前 |

CRC 范围：偏移 0 ~ 22（共 23 字节）。

## 5. 上行帧：下位机 -> 上位机

`TxPacket_TJ`，共 **39 字节**。

| 偏移 | 大小 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | 2 | `head` | `'S','P'` |
| 2 | 1 | `mode_TJ` | 0 idle / 1 auto aim / 2 small buff / 3 big buff |
| 3 | 1 | `robot_id` | 3 红 / 103 蓝 |
| 4 | 4 | `imu_pitch_TJ` | 云台 pitch，rad（自瞄用） |
| 8 | 4 | `imu_yaw_TJ` | 云台 yaw，rad（自瞄用） |
| 12 | 2 | `fire_flag_TJ` | 1 = 本周期检测到发射 |
| 14 | 4 | `fire_tick_TJ` | 发射时刻 tick |
| 18 | 2 | `friction_current_TJ` | 摩擦轮电流 |
| 20 | 2 | `supply_speed_TJ` | 拨弹轮实时转速，rpm |
| 22 | 1 | `nav_allowed` | 0/1，是否允许导航控底盘 |
| 23 | 2 | `nav_valid_flags` | 有效性位，见下 |
| 25 | 4 | `chassis_vx` | m/s，底盘系前为正 |
| 29 | 4 | `chassis_vy` | m/s，底盘系左为正 |
| 33 | 4 | `gimbal_relative_angle` | rad，云台相对底盘转角，正=左转 |
| 37 | 2 | `crc16_TJ` | 低字节在前 |

CRC 范围：偏移 0 ~ 36（共 37 字节）。

## 6. 位定义

`nav_valid_flags`：

| 位 | 含义 |
| --- | --- |
| bit0 | `chassis_vx` 有效 |
| bit1 | `chassis_vy` 有效 |
| bit2 | `nav_allowed` 有效 |
| bit3 | `gimbal_relative_angle` 有效 |
| bit4~15 | 保留 |

`shoot_TJ`：

| 值 | 含义 |
| --- | --- |
| 0 | 停止 |
| 1 | 单发 |

## 7. 频率与超时

- 上行反馈由下位机按控制周期发送（约 5ms 一次）。
- 导航命令 `nav_control` 需要独立超时保护：建议 200ms 没有合法新命令即撤控，并清零 `nav_vx` / `nav_vy`。
- `nav_seq` 用于检测重复/丢包，回绕规则两端需一致。

## 8. 使用方法

C++ 直接包含：

```cpp
#include "xuc.h"
```

发送示例：

```cpp
RxPacket_TJ rx{};
rx.head[0] = 'S';
rx.head[1] = 'P';
rx.control_TJ = 1;
rx.shoot_TJ = XUC_SHOOT_SINGLE;
rx.yaw_TJ = 0.0f;
rx.pitch_TJ = 0.0f;
rx.nav_control = 1;
rx.nav_seq = seq++;
rx.nav_vx = 0.0f;   // 云台系，前为正
rx.nav_vy = 0.0f;   // 云台系，右为正

uint8_t buf[sizeof(RxPacket_TJ)];
xuc_encode_rx(rx, buf);       // 填头 + 补 CRC，得到 25 字节
serial_write(buf, sizeof(buf));
```

接收示例：

```cpp
uint8_t buf[sizeof(TxPacket_TJ)];
size_t n = serial_read(buf, sizeof(buf));

TxPacket_TJ tx;
if (xuc_decode_tx(buf, n, tx)) {   // 校验 CRC + 还原结构体
    // tx.imu_yaw_TJ：云台 yaw（自瞄）
    // tx.chassis_vx / tx.chassis_vy：底盘系速度（里程计）
    // tx.gimbal_relative_angle：云台相对底盘转角
    // chassis_yaw = radar_yaw + tx.gimbal_relative_angle
}
```

下位机方向同理：发送用 `xuc_encode_tx`，接收用 `xuc_decode_rx`。

## 9. 相对上一版的变更

1. 自瞄 + 导航合并为单帧；
2. 删除 `bullet_speed_TJ`、`bullet_count_TJ`；
3. 明确区分坐标系：`nav_vx/nav_vy` 为云台系，`chassis_vx/chassis_vy` 为底盘系；
4. 新增 `gimbal_relative_angle`，上位机配合雷达 yaw 合成底盘航向；
5. 自瞄继续使用云台 IMU 的 `imu_yaw_TJ`/`imu_pitch_TJ`，未删除。
