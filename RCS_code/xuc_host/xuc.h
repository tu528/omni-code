#ifndef XUC_HOST_H
#define XUC_HOST_H

// ============================================================================
// XUC 精简合并协议 v3（供上位机开发使用）
//
// 一帧一个方向：自瞄 + 导航合并。已去掉 bullet_speed_TJ / bullet_count_TJ。
// 自瞄继续使用云台 IMU 的 imu_yaw_TJ / imu_pitch_TJ；导航航向由上位机用
// 雷达 IMU yaw + gimbal_relative_angle 合成，不再发送底盘 wz。
//
// 共同约定：
//   - 帧头 'S','P'
//   - 多字节字段小端；float 为 IEEE-754 32 位
//   - CRC16 = CRC-16/MODBUS（反射多项式 0xA001，初值 0xFFFF），低字节在前
//     校验范围 = 整帧除去最后 2 字节
//   - 结构体 pack(1)，可整块 memcpy
//   - 命令 nav_vx/nav_vy：云台坐标系，x 前、y 右
//   - 反馈 chassis_vx/chassis_vy：底盘坐标系，x 前、y 左
//   - gimbal_relative_angle：云台相对底盘转角，正值 = 云台左转
// ============================================================================

#include <cstdint>
#include <cstddef>

#define XUC_HEAD0 'S'
#define XUC_HEAD1 'P'

/* shoot_TJ 取值：自瞄只用单发 */
#define XUC_SHOOT_STOP   0
#define XUC_SHOOT_SINGLE 1

/* nav_valid_flags 位定义 */
#define XUC_NAV_VX_VALID          (1u << 0)
#define XUC_NAV_VY_VALID          (1u << 1)
#define XUC_NAV_ALLOWED_VALID     (1u << 2)
#define XUC_NAV_GIMBAL_REL_VALID  (1u << 3)

#pragma pack(push, 1)

/* 上位机 -> 下位机：25 字节 */
struct RxPacket_TJ {
    uint8_t  head[2];
    uint8_t  control_TJ;   /* 云台控制：1=上位机在控 */
    uint8_t  shoot_TJ;     /* XUC_SHOOT_* */
    float    yaw_TJ;       /* rad，云台目标 yaw */
    float    pitch_TJ;     /* rad，云台目标 pitch */

    uint8_t  nav_control;  /* 0=撤控，1=导航接管 */
    uint16_t nav_seq;      /* 命令序号 */
    float    nav_vx;       /* m/s，云台前为正 */
    float    nav_vy;       /* m/s，云台右为正 */

    uint16_t crc16_TJ;     /* 低字节在前 */
};

/* 下位机 -> 上位机：39 字节 */
struct TxPacket_TJ {
    uint8_t  head[2];
    uint8_t  mode_TJ;      /* 0 idle,1 auto aim,2 small buff,3 big buff */
    uint8_t  robot_id;     /* 3 红,103 蓝 */
    float    imu_pitch_TJ; /* rad */
    float    imu_yaw_TJ;   /* rad，云台 yaw（自瞄用） */

    uint16_t fire_flag_TJ; /* 1 = 本周期检测到发射 */
    uint32_t fire_tick_TJ; /* 发射时刻 tick */
    int16_t  friction_current_TJ;
    int16_t  supply_speed_TJ; /* rpm */

    uint8_t  nav_allowed;  /* 0/1 */
    uint16_t nav_valid_flags;
    float    chassis_vx;   /* m/s，底盘前为正 */
    float    chassis_vy;   /* m/s，底盘左为正 */
    float    gimbal_relative_angle; /* rad，云台相对底盘转角，正=左转 */

    uint16_t crc16_TJ;     /* 低字节在前 */
};

#pragma pack(pop)

// CRC 原语
uint16_t xuc_crc16(const uint8_t *data, uint32_t len, uint16_t init);
void     xuc_append_crc16(uint8_t *frame, uint32_t size);
bool     xuc_verify_crc16(const uint8_t *frame, uint32_t size);

// 编码：把结构体打包成带 CRC 的字节帧
void xuc_encode_rx(const RxPacket_TJ &src, uint8_t out[sizeof(RxPacket_TJ)]);
void xuc_encode_tx(const TxPacket_TJ &src, uint8_t out[sizeof(TxPacket_TJ)]);

// 解码：校验 CRC 后从字节帧还原结构体
bool xuc_decode_rx(const uint8_t *buf, uint32_t len, RxPacket_TJ &dst);
bool xuc_decode_tx(const uint8_t *buf, uint32_t len, TxPacket_TJ &dst);

static_assert(sizeof(RxPacket_TJ) == 25, "RxPacket_TJ must be 25 bytes");
static_assert(sizeof(TxPacket_TJ) == 39, "TxPacket_TJ must be 39 bytes");
static_assert(offsetof(RxPacket_TJ, crc16_TJ) == 23, "RxPacket_TJ crc offset");
static_assert(offsetof(TxPacket_TJ, crc16_TJ) == 37, "TxPacket_TJ crc offset");

#endif /* XUC_HOST_H */
