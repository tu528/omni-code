#include "xuc.h"
#include "judgement.h"
#include "imu.h"
#include "CRC.h"
#include "task.h"
#include <string.h>
#include <cmath>

XUC xuc;

void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
    m_uart = huart;
    if (m_uart != nullptr) {
        m_uart->Init(Instance, BaudRate).DMARxInit(nullptr);
        queueHandler = &m_uart->UartQueueHandler;
    }
    else {
        queueHandler = NULL;
    }

    target = {};
    fire_auto = 0;
    m_lastRxTick = 0;
    memset(&Tx_TJ, 0, sizeof(Tx_TJ));
}

void XUC::Decode()
{
    if (queueHandler == NULL || *queueHandler == NULL) {
        return;
    }

    if (xQueueReceive(*queueHandler, frame, 0) != pdPASS) {
        if (!RxFresh()) {
            Rx_TJ.control_TJ = 0;
            Rx_TJ.shoot_TJ = 0;
            target = {};
            fire_auto = 0;
        }
        return;
    }

    const uint32_t packLen = (uint32_t)sizeof(RxPacket_TJ);
    const uint32_t received_length =
        (m_uart->dataDmaNum <= UART_MAX_LEN) ? m_uart->dataDmaNum : UART_MAX_LEN;
    for (uint32_t i = 0; i + packLen <= received_length; ++i)
    {
        if (frame[i] != 'S' || frame[i + 1] != 'P') {
            continue;
        }

        uint8_t packet[sizeof(RxPacket_TJ)]{};
        memcpy(packet, &frame[i], sizeof(RxPacket_TJ));

        if (!VerifyCRC16CheckSum(packet, (uint32_t)sizeof(RxPacket_TJ))) {
            continue;
        }

        memcpy(&Rx_TJ, packet, sizeof(RxPacket_TJ));

        target.yaw = Rx_TJ.yaw_TJ;
        target.pitch = Rx_TJ.pitch_TJ;
        fire_auto = Rx_TJ.shoot_TJ;

        if (Rx_TJ.control_TJ == 0) {
            target.yaw = 0.0f;
            target.pitch = 0.0f;
        }

        m_lastRxTick = xTaskGetTickCount();
        return;
    }
}

bool XUC::RxFresh()
{
    return (TickType_t)(xTaskGetTickCount() - m_lastRxTick) <= pdMS_TO_TICKS(200);
}

void XUC::Encode()
{
    if (m_uart == nullptr) {
        return;
    }

    static uint16_t bullet_count = 0;

    Tx_TJ.head[0] = 'S';
    Tx_TJ.head[1] = 'P';

    // mode_TJ is updated by RC before Encode() is called.

    Tx_TJ.robot_id = 103U;

    const float bullet_speed = judgement.data.shoot_data_t.bullet_speed;
    Tx_TJ.bullet_speed_TJ = std::isfinite(bullet_speed) ? bullet_speed : 0.0f;
    Tx_TJ.bullet_count_TJ = bullet_count++;

    Tx_TJ.imu_pitch_TJ = imu_pantile.GetAnglePitch()/57.2957795f;
    Tx_TJ.imu_yaw_TJ = imu_pantile.GetAngleYaw()/57.2957795f;

    const uint32_t packet_size = (uint32_t)sizeof(TxPacket_TJ);

    memcpy(tx_data, &Tx_TJ, packet_size);
    AppendCRC16CheckSum(tx_data, packet_size);
    memcpy(&Tx_TJ, tx_data, packet_size);

    m_uart->UARTTransmit(tx_data, packet_size);
}
