#pragma once

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "judgement.h"

#pragma pack(push, 1)
struct TxPacket_TJ  // lower board -> small computer
{
    uint8_t head[2];
    uint8_t mode_TJ;            // 0 idle, 1 auto aim, 2 small buff, 3 big buff
    uint8_t robot_id;           // 3 red, 103 blue, 0 unknown
    float bullet_speed_TJ;
    uint16_t bullet_count_TJ;
    float imu_pitch_TJ;         // rad
    float imu_yaw_TJ;           // rad
    uint16_t crc16_TJ;
};

struct RxPacket_TJ  // small computer -> lower board
{
    uint8_t head[2];
    uint8_t control_TJ;         //暂时无用
	uint8_t shoot_TJ;           //1连发；2单发；0停止
    float yaw_TJ;               // rad; bounded absolute yaw target
    float pitch_TJ;             // rad; bounded absolute pitch target in Phase 2D.5
    uint16_t crc16_TJ;
};
#pragma pack(pop)

class XUC
{
public:
    typedef struct { float yaw, pitch; } TargetAngle;

    TxPacket_TJ Tx_TJ{};
    RxPacket_TJ Rx_TJ{};

    void Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate);
    void Decode();
    void Encode();

    float GetTargetYaw() { return target.yaw; }
    float GetTargetPitch() { return target.pitch; }
    bool  RxFresh();
    uint8_t fire_auto = 0;

public:
    uint8_t tx_data[sizeof(TxPacket_TJ)] = { 0 };

private:
    uint8_t frame[UART_MAX_LEN]{};
    TargetAngle target{};
    UART* m_uart = nullptr;
    QueueHandle_t* queueHandler = NULL;
    TickType_t m_lastRxTick = 0;
};

extern XUC xuc;
