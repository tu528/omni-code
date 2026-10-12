#ifndef XUC_NAV_PROTOCOL_H
#define XUC_NAV_PROTOCOL_H

// =============================================================================
// XUC 导航通信精简协议 v1
//
// 本文件是「独立草案」，用来给上位机（视觉/导航）确认协议是否可行。
// 不改动现有 STM32F405/xuc.h，也不改动原自瞄 14/20 字节协议。
//
// 命名沿用原自瞄 XUC 的习惯：Rx/Tx 都是站在「下位机」角度说的。
//   RxPacket_NAV : 上位机 -> 下位机，导航速度指令（下位机接收）
//   TxPacket_NAV : 下位机 -> 上位机，导航底盘反馈（下位机发送）
//
// 共同约定：
//   * 帧头 head[2] = 'S','P'
//   * 第 3 字节 type = 0x4E ('N')，用来和自瞄包区分
//     自瞄包第 3 字节是 control_TJ / mode_TJ，只会是 0~3，永远不会是 0x4E
//   * 多字节字段为小端；float 为 IEEE-754 32 位（sizeof(float) == 4）
//   * CRC16 与原自瞄 XUC 完全一致：CRC-16/ARC，初值 0xFFFF，低字节在前，
//     校验范围是「整帧除去最后 2 字节」
//   * 结构体 pack(1)，可整块 memcpy 到字节流或从字节流 memcpy 回来
//
// 方向的统一定义：x 向前、y 向左、z 向上；俯视逆时针为正。
//
// 需要上位机确认的问题见文件末尾 [待确认]。
// =============================================================================

#include <cstddef>
#include <cstdint>

namespace xuc_nav {

// -----------------------------------------------------------------------------
// CRC16：与下位机 CRC.cpp 的 GetCRC16CheckSum /
//        上位机 tools::get_crc16 是同一个算法
// -----------------------------------------------------------------------------
inline std::uint16_t Crc16(const std::uint8_t* data, std::uint32_t length,
                           std::uint16_t crc = 0xFFFFu)
{
    while (length-- != 0u) {
        crc ^= static_cast<std::uint16_t>(*data++);
        for (int bit = 0; bit < 8; ++bit) {
            crc = ((crc & 1u) != 0u)
                      ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001u)
                      : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

inline void AppendCrc16(std::uint8_t* frame, std::uint32_t size)
{
    if (frame == nullptr || size < 2u) {
        return;
    }
    const std::uint16_t crc = Crc16(frame, size - 2u);
    frame[size - 2u] = static_cast<std::uint8_t>(crc & 0xFFu);
    frame[size - 1u] = static_cast<std::uint8_t>((crc >> 8u) & 0xFFu);
}

inline bool VerifyCrc16(const std::uint8_t* frame, std::uint32_t size)
{
    if (frame == nullptr || size < 2u) {
        return false;
    }
    const std::uint16_t crc = Crc16(frame, size - 2u);
    return frame[size - 2u] == static_cast<std::uint8_t>(crc & 0xFFu) &&
           frame[size - 1u] == static_cast<std::uint8_t>((crc >> 8u) & 0xFFu);
}

// -----------------------------------------------------------------------------
// 公共常量与枚举
// -----------------------------------------------------------------------------
static const std::uint8_t XUC_HEAD0 = 'S';
static const std::uint8_t XUC_HEAD1 = 'P';
static const std::uint8_t XUC_TYPE_NAV = 0x4Eu;  // 'N'

// 底盘模式（TxPacket_NAV::mode）
enum XucNavMode : std::uint8_t {
    XUC_NAV_MODE_UNKNOWN  = 0,
    XUC_NAV_MODE_DISABLED = 1,
    XUC_NAV_MODE_MANUAL   = 2,
    XUC_NAV_MODE_AUTO     = 3,
    XUC_NAV_MODE_FAULT    = 4
};

// 有效性位（TxPacket_NAV::valid_flags）
enum XucNavValidFlag : std::uint16_t {
    XUC_NAV_VX_VALID          = 1u << 0,
    XUC_NAV_VY_VALID          = 1u << 1,
    XUC_NAV_WZ_VALID          = 1u << 2,
    XUC_NAV_MODE_VALID        = 1u << 3,
    XUC_NAV_ESTOP_VALID       = 1u << 4,
    XUC_NAV_FAULT_LIMIT_VALID = 1u << 5
};

// 故障位（TxPacket_NAV::fault_flags，示例，需电控最终确认）
enum XucNavFaultFlag : std::uint8_t {
    XUC_NAV_FAULT_MOTOR_OFFLINE  = 1u << 0,
    XUC_NAV_FAULT_MOTOR_OVERTEMP = 1u << 1,
    XUC_NAV_FAULT_FEEDBACK_LOST  = 1u << 2
};

// 限幅位（TxPacket_NAV::limit_flags，示例，需电控最终确认）
enum XucNavLimitFlag : std::uint8_t {
    XUC_NAV_LIMIT_SPEED    = 1u << 0,
    XUC_NAV_LIMIT_POWER    = 1u << 1,
    XUC_NAV_LIMIT_SUPERCAP = 1u << 2
};

// -----------------------------------------------------------------------------
// 数据包
// -----------------------------------------------------------------------------
#pragma pack(push, 1)

// 上位机 -> 下位机：导航速度指令（20 字节）
struct RxPacket_NAV {
    std::uint8_t  head[2];   // 0  'S','P'
    std::uint8_t  type;      // 2  固定 0x4E ('N')
    std::uint8_t  control;   // 3  0=撤控/停止，1=导航接管
    std::uint16_t seq;       // 4  命令序号，每帧 +1，回绕；用于检测重复/丢包
    float         vx;        // 6  m/s，向前为正
    float         vy;        // 10 m/s，向左为正
    float         wz;        // 14 rad/s，俯视逆时针为正
    std::uint16_t crc16;     // 18 CRC16，低字节在前
};

// 下位机 -> 上位机：导航底盘反馈（24 字节）
struct TxPacket_NAV {
    std::uint8_t  head[2];       // 0  'S','P'
    std::uint8_t  type;          // 2  固定 0x4E ('N')
    std::uint8_t  mode;          // 3  XucNavMode
    std::uint8_t  nav_allowed;   // 4  0/1：当前是否允许导航控底盘
    std::uint8_t  estop_active;  // 5  0/1：实际急停状态
    std::uint8_t  fault_flags;   // 6  XucNavFaultFlag
    std::uint8_t  limit_flags;   // 7  XucNavLimitFlag
    std::uint16_t valid_flags;   // 8  XucNavValidFlag
    float         chassis_vx;    // 10 m/s，实际/估计前后速度，向前为正
    float         chassis_vy;    // 14 m/s，实际/估计左右速度，向左为正
    float         chassis_wz;    // 18 rad/s，实际/估计角速度，逆时针为正
    std::uint16_t crc16;         // 22 CRC16，低字节在前
};

#pragma pack(pop)

// -----------------------------------------------------------------------------
// 编译期检查：布局必须和字节流完全一致
// -----------------------------------------------------------------------------
static_assert(sizeof(float) == 4, "XUC nav protocol requires 32-bit IEEE-754 float");
static_assert(sizeof(RxPacket_NAV) == 20, "RxPacket_NAV must be 20 bytes");
static_assert(sizeof(TxPacket_NAV) == 24, "TxPacket_NAV must be 24 bytes");
static_assert(offsetof(RxPacket_NAV, crc16) == 18, "RxPacket_NAV crc offset must be 18");
static_assert(offsetof(TxPacket_NAV, crc16) == 22, "TxPacket_NAV crc offset must be 22");
static_assert(XUC_HEAD0 == 'S' && XUC_HEAD1 == 'P' && XUC_TYPE_NAV == 0x4Eu,
              "XUC nav header constants");

}  // namespace xuc_nav

#endif  // XUC_NAV_PROTOCOL_H

// =============================================================================
// [说明] 关于 chassis_wz（底盘角速度）的来源
//
// 下位机目前只有一块装在云台上的 IMU（imu_pantile），没有底盘 IMU。
// 云台相对底盘会转（yaw 电机 M6020 就是底盘与云台之间的关节），所以：
//
//     云台航向     = 底盘航向 + 云台相对底盘的转角 phi
//     云台 IMU gyro_z = 底盘 wz + 云台关节转动角速度
//
// 因此云台 IMU 的 gyro_z 不能直接当作 chassis_wz，自瞄一转头就会污染导航。
// 可选来源按推荐顺序：
//
//   A. 四轮差速解算（不加硬件，先做这个）
//      angular_source = 1（轮速）
//      优点：零成本，不用接线。
//      缺点：打滑、轮径/减速比标定误差会直接进入 wz。
//
//   B. 加一块装在底盘上的 IMU（推荐，精度最好）
//      angular_source = 2（底盘 IMU）
//      注意：imu_chassis 目前只声明未定义，IMU 类也还没有 gyro_z 的 getter。
//
//   C. 云台 IMU + yaw 关节角反推（仅兜底，不推荐作为导航主来源）
//      chassis_wz = 云台 IMU 的 gyro_z - d(phi)/dt
//      angular_source = 3（其他），不能标成 2（底盘 IMU）。
//   D. 雷达自身是否有imu
//
// 无论用哪种，只要 wz 不可信，就必须把 valid_flags 的
// XUC_NAV_WZ_VALID 位清 0；不允许用 0 值冒充“角速度为 0”。
//
// =============================================================================
//
// =============================================================================
// [说明] 关于导航包和自瞄包：合并发送还是分开发送
//
// 两种包共用同一条 USART6，靠第 3 字节 type 区分，所以两种方式都可行：
//
//   方式一：分成两帧、各自独立发送（推荐先做）
//     - 自瞄包保持原频率（大约每个 ControlTask 循环一帧）；
//     - 导航包按分频发送（建议 50Hz），两者频率解耦；
//     - 上位机解析器要同时认两种帧：见到 'S','P' 后按第 3 字节分派。
//
//   方式二：合并成一次发送
//     - 把自瞄 20 字节和导航 24 字节拼进同一个发送缓冲，一次 UARTTransmit；
//     - 减少发送函数调用次数和帧间空隙；
//     - 但两包只能同频，导航无法单独降到 50Hz。
//
// 注意：下位机现在的 UARTTransmit 是阻塞发送。20 字节约 0.43ms，
// 24 字节约 0.52ms，44 字节约 0.95ms。如果两种包分开、又都在同一个
// ControlTask 循环里发，会占住控制任务接近 1ms；建议导航分频到 50Hz，
// 或者把发送移到低优先级的专用上报任务。
//
// =============================================================================
//
// =============================================================================
// [待确认] 请上位机（视觉/导航）侧确认以下几点：
//
// 1. 导航帧用 head='S','P' + type=0x4E，与自瞄包共用同一条串口，
//    由第 3 字节区分（自瞄第 3 字节是 0~3，不会撞 0x4E），是否可行？
//
// 2. 是否接受 20 字节命令包 / 24 字节反馈包及其字段布局？
//
// 3. 小端 + IEEE-754 float + CRC16（初值 0xFFFF，低字节在前，
//    范围 = 整帧除去最后 2 字节）是否与上位机现有实现一致？
//
// 4. 方向约定是否一致：x 前、y 左、z 上；vx/vy 前/左为正；
//    wz 俯视逆时针为正？
//
// 5. 命令下发频率与超时：建议 50~100Hz 下发，200ms 没有合法命令即撤控，
//    是否同意？
//
// 6. 本简版不包含 boot_id / feedback_seq / sample_time_ms / session / ACK。
//    对导航是否够用？如果上位机需要时间戳或会话确认，是否改用完整
//    56 字节版本？
//
// 7. 自瞄反馈与导航反馈是「合并成一次发送」还是「分成两帧、各自独立发送」？
//    合并 = 两包同频；分开 = 导航可以单独降到 50Hz。
//    分开时上位机解析器要同时认两种帧，按第 3 字节 type 分派。
//
// 8. 底盘 wz 的来源选哪种：四轮差速（angular_source=1）、加底盘 IMU
//    （angular_source=2），还是云台 IMU 减关节转速（angular_source=3，仅兜底）？
//    如果暂时给不出可信 wz，上位机是否接受 valid_flags 的 WZ 位为 0、自己算？
//
// 9. 确认后两端使用同一份结构体和同一套 CRC；任一端改字段都要同步。
// =============================================================================
