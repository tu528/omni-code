#include "imu.h"

// Existing UART and FreeRTOS queue interfaces are supplied by imu.h.

void IMU::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE sensor_type)

{

	if (huart == nullptr || Instance == nullptr) return;
    be_data = BES102Data{};
    be_stats = BES102Stats{};
    be_have_data = false;
    be_rate_start_ms = HAL_GetTick();
    be_rate_start_samples = 0;
    be_config_step = 0;
    be_reply_retries = 0;
    be_awaiting_reply = false;
    be_command_sent_ms = 0;
    be_boot_ms = HAL_GetTick();
    if (sensor_type == BE_S102 && be_auto_configure)
        be_stats.config_state = BES102ConfigState::WAITING_BOOT;
    angle = Angle{};
    angularvelocity = AngularVelocity{};
    acceleration = Acceleration{};
    crc = 0;
    huart->Init(Instance, BaudRate).DMARxInit();

	m_uart = huart;

	this->type = sensor_type;

	queueHandler = &huart->UartQueueHandler;

}

void IMU::Decode()
{
    if (type == BE_S102) {
        UpdateBES102Rate();
        if (queueHandler != nullptr && *queueHandler != nullptr) {
            pd_Rx = xQueueReceive(*queueHandler, rxData, 0);
            if (pd_Rx == pdTRUE) DecodeBES102Block();
        }
        ProcessBES102Config();
        return;
    }

	if (queueHandler == NULL || *queueHandler == NULL) {

		return;  // 或者报错

	}

	else {

		pd_Rx = xQueueReceive(*queueHandler, rxData, 0);
        if (pd_Rx != pdTRUE) return;

	}

	if (type == IMU601)

	{

		if (rxData[0] == 0x55 && rxData[1] == 0x55)

		{

			uint8_t* data = rxData + 4;

			if (rxData[2] == 0x01 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))

			{

				angle.roll = (float)getword(data[1], data[0]) * 180.f / 32768.f;

				angle.pitch = (float)getword(data[3], data[2]) * 180.f / 32768.f;

				angle.yaw = (float)getword(data[5], data[4]) * 180.f / 32768.f;

			}

			else if (rxData[2] == 0x03 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))

			{

				int16_t Ax, Ay, Az, Gx, Gy, Gz;

				Ax = getword(data[1], data[0]);

				Ay = getword(data[3], data[2]);

				Az = getword(data[5], data[4]);

				Gx = getword(data[7], data[6]);

				Gy = getword(data[9], data[8]);

				Gz = getword(data[11], data[10]);

				acceleration.x = (float)Ax / 32768 * ACC_FSR;

				acceleration.y = (float)Ay / 32768 * ACC_FSR;

				acceleration.z = (float)Az / 32768 * ACC_FSR;

				angularvelocity.roll = (float)Gx / 32768 * GYRO_FSR;

				angularvelocity.pitch = (float)Gy / 32768 * GYRO_FSR;

				angularvelocity.yaw = (float)Gz / 32768 * GYRO_FSR;

			}

		}

	}

	else if (type == CH010)

	{

		if (rxData[0] == 0x5A && rxData[1] == 0xA5)

		{

			//Check(rxData, 4, 0);

			if (Check(rxData + 6, ((rxData[3] << 8) | rxData[2]), ((rxData[5] << 8) | rxData[4])))

			{

				if (rxData[6] == 0x91)

				{

					int offset = 6;

					acceleration.x = R4(rxData + offset + 12);

					acceleration.y = R4(rxData + offset + 16);

					acceleration.z = R4(rxData + offset + 20);

					angularvelocity.pitch = R4(rxData + offset + 24);

					angularvelocity.roll = R4(rxData + offset + 28);

					angularvelocity.yaw = R4(rxData + offset + 32);

					angle.pitch = R4(rxData + offset + 48);

					angle.roll = R4(rxData + offset + 52);

					angle.yaw = R4(rxData + offset + 56);

				}

			}

			crc = 0;

		}

	}

	else if (type == HI226)

	{

		if (rxData[0] == 0x5A && rxData[1] == 0xA5)

		{

			//Check(rxData, 4, 0);

			if (Check(rxData + 6, ((rxData[3] << 8) | rxData[2]), ((rxData[5] << 8) | rxData[4])))

			{

				if (rxData[6] == 0x91)

				{

					int offset = 6;

					acceleration.x = R4(rxData + offset + 12);

					acceleration.y = R4(rxData + offset + 16);

					acceleration.z = R4(rxData + offset + 20);

					angularvelocity.pitch = R4(rxData + offset + 24);

					angularvelocity.roll = R4(rxData + offset + 28);

					angularvelocity.yaw = R4(rxData + offset + 32);

					angle.pitch = R4(rxData + offset + 48);

					angle.roll = R4(rxData + offset + 52);

					angle.yaw = R4(rxData + offset + 56);

				}

			}

		}

	}

}

float IMU::GetAngleYaw()

{

	return angle.yaw;

}

float IMU::getangularvelocitypitch()

{

	return angularvelocity.pitch;

}

float IMU::GetAnglePitch()

{

	return angle.pitch;

}

float IMU::GetAngleRoll()

{

	return angle.roll;

}

float* IMU::GetAcceleration()
{
    acceleration_view[0] = acceleration.x;
    acceleration_view[1] = acceleration.y;
    acceleration_view[2] = acceleration.z;
    return acceleration_view;
}

bool IMU::Check(uint8_t* pdata, uint8_t len, uint32_t com)

{

	if (type == IMU601)

	{

		uint8_t t = 0;

		for (int i = 0; i < len; i++)

		{

			t += pdata[i];

		}

		return t == com;

	}

	else if (type == CH010)

	{

		for (int j = 0; j < len; ++j)

		{

			uint32_t i;

			uint32_t byte = pdata[j];

			crc ^= byte << 8;

			for (i = 0; i < 8; ++i)

			{

				uint32_t temp = crc << 1;

				if (crc & 0x8000)

				{

					temp ^= 0x1021;

				}

					crc = static_cast<uint16_t>(temp);

			}

		}

		return crc == com;

	}

    return false;
}

int16_t IMU::getword(uint8_t HighBit, uint8_t LowBits)

{

	return HighBit << 8 | LowBits;

}
namespace {
uint16_t ReadBE16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
uint32_t ReadBE32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
float ReadBEFloat(const uint8_t* p)
{
    uint32_t bits = ReadBE32(p);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}
uint32_t LockBEData()
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return previous;
}
void UnlockBEData(uint32_t previous)
{
    __DMB();
    __set_PRIMASK(previous);
}
}

bool IMU::DecodeBES102Block()
{
    // Validate whole LPBUS frames before recording diagnostics. Payload bytes
    // that happen to contain 0x3A must not overwrite the genuine header.
    static_assert(sizeof(float) == 4, "BE-S102 requires 32-bit float");
    static_assert(UART_MAX_LEN >= 67, "Existing UART queue item is too small");
    static_assert(BES102_OUTPUT_MASK == 0x1822, "This decoder uses mask 0x1822");
    static_assert(BES102_OUTPUT_HZ > 0 && BES102_OUTPUT_HZ <= 500 &&
                  BES102_TIMESTAMP_HZ % BES102_OUTPUT_HZ == 0,
                  "Invalid sensor timestamp/output frequency");
    bool decoded = false;
    for (uint16_t offset = 0; offset + 11U <= UART_MAX_LEN; ++offset) {
        if (rxData[offset] != 0x3A) continue;
        const uint8_t* frame = rxData + offset;
        // Restrict candidates to this sensor and known LPBUS commands first.
        const uint16_t command = ReadBE16(frame + 3);
        if (command != 9 && command != 0 && command != 1 &&
            command != 31 && command != 137) continue;
        const uint16_t length = ReadBE16(frame + 5);
        if (static_cast<uint32_t>(offset) + length + 11U > UART_MAX_LEN) {
            if (command == 9) ++be_stats.payload_errors;
            continue;
        }
        const uint16_t total = static_cast<uint16_t>(length + 11U);
        if (frame[total - 2] != 0x0D || frame[total - 1] != 0x0A) {
            ++be_stats.format_errors; continue;
        }
        uint16_t sum = 0;
        for (uint16_t i = 1; i < length + 7U; ++i)
            sum = static_cast<uint16_t>(sum + frame[i]);
        if (sum != ReadBE16(frame + length + 7)) {
            ++be_stats.checksum_errors; continue;
        }
        const uint16_t address = ReadBE16(frame + 1);
        const bool command_address = command != 9 &&
            address == BES102_COMMAND_ADDRESS;
        if (address != BES102_SENSOR_ID && !command_address) {
            ++be_stats.id_errors;
            offset = static_cast<uint16_t>(offset + total - 1U);
            continue;
        }
        if ((command == 0 || command == 1) && length != 0) {
            ++be_stats.format_errors; continue;
        }
        if (command == 9) {
            be_stats.last_frame_offset = offset;
            be_stats.last_payload_bytes = length;
            if (be_stats.config_state == BES102ConfigState::MANUAL ||
                be_stats.config_state == BES102ConfigState::DONE) {
                if (length != 56) ++be_stats.payload_errors;
                else decoded = DecodeBES102Data(frame) || decoded;
            }
        } else {
            ConsumeBES102ConfigReply(command, frame + 7, length);
        }
        // Only complete validated frames advance across the block.
        offset = static_cast<uint16_t>(offset + total - 1U);
    }
    return decoded;
}

bool IMU::DecodeBES102Data(const uint8_t* frame)
{
    for (uint16_t i = 11; i < 63; i += 4) {
        if ((ReadBE32(frame + i) & 0x7F800000UL) == 0x7F800000UL) {
            ++be_stats.payload_errors; return false;
        }
    }
    ++be_stats.valid_frames;
    const uint32_t timestamp = ReadBE32(frame + 7);
    if (be_have_data) {
        const uint32_t delta = timestamp - be_data.timestamp_ticks;
        if (delta == 0) { ++be_stats.duplicate_samples; return false; }
        if (delta > 5U * BES102_TIMESTAMP_HZ) ++be_stats.timestamp_resets;
        else {
            const uint32_t step = BES102_TIMESTAMP_HZ / BES102_OUTPUT_HZ;
            if (delta > step) be_stats.estimated_lost_samples += delta / step - 1;
        }
    }
    BES102Data next;
    next.timestamp_ticks = timestamp;
    next.sequence = be_stats.unique_samples + 1;
    next.received_ms = HAL_GetTick();
    for (uint16_t i = 0; i < 3; ++i) {
        next.accel_g[i] = ReadBEFloat(frame + 11 + 4 * i);
        next.gyro_dps[i] = ReadBEFloat(frame + 23 + 4 * i);
        next.euler_deg[i] = ReadBEFloat(frame + 51 + 4 * i);
    }
    for (uint16_t i = 0; i < 4; ++i)
        next.quaternion_wxyz[i] = ReadBEFloat(frame + 35 + 4 * i);

    const uint32_t key = LockBEData();
    // Commit to the ORIGINAL IMU fields: existing callers keep their getters.
    angle.roll = next.euler_deg[0];
    angle.pitch = next.euler_deg[1];
    angle.yaw = next.euler_deg[2];
    angularvelocity.roll = next.gyro_dps[0];
    angularvelocity.pitch = next.gyro_dps[1];
    angularvelocity.yaw = next.gyro_dps[2];
    acceleration.x = next.accel_g[0];
    acceleration.y = next.accel_g[1];
    acceleration.z = next.accel_g[2];
    be_data = next;
    be_have_data = true;
    be_stats.unique_samples = next.sequence;
    UnlockBEData(key);
    return true;
}

void IMU::UpdateBES102Rate()
{
    const uint32_t now = HAL_GetTick();
    const uint32_t elapsed = now - be_rate_start_ms;
    if (elapsed >= 1000) {
        be_stats.update_hz = (be_stats.unique_samples - be_rate_start_samples) *
                             1000.0f / static_cast<float>(elapsed);
        be_rate_start_ms = now;
        be_rate_start_samples = be_stats.unique_samples;
    }
}

bool IMU::GetBES102Data(BES102Data& data) const
{
    const uint32_t key = LockBEData();
    const bool valid = be_have_data;
    if (valid) data = be_data;
    UnlockBEData(key);
    return valid;
}

IMU::BES102Stats IMU::GetBES102Stats() const
{
    const uint32_t key = LockBEData();
    const BES102Stats stats = be_stats;
    UnlockBEData(key);
    return stats;
}

bool IMU::IsOnline(uint32_t timeout_ms) const
{
    if (type != BE_S102) return false; // New online check is BE-S102-specific.
    BES102Data data;
    return GetBES102Data(data) &&
           static_cast<uint32_t>(HAL_GetTick() - data.received_ms) <= timeout_ms;
}

namespace {
const uint16_t BEConfigCommands[] = {6, 31, 137, 136, 36, 30, 34, 31, 137, 7};
const uint8_t BEConfigCount = sizeof(BEConfigCommands) / sizeof(BEConfigCommands[0]);
uint16_t BEExpectedReply(uint8_t step)
{
    return (step == 1 || step == 7) ? 31 :
           ((step == 2 || step == 8) ? 137 : 0);
}
}

void IMU::SendBES102Command(uint16_t command, bool with_value, uint32_t value)
{
    uint8_t packet[15] = {};
    packet[0] = 0x3A;
    packet[1] = static_cast<uint8_t>(BES102_COMMAND_ADDRESS);
    packet[2] = static_cast<uint8_t>(BES102_COMMAND_ADDRESS >> 8);
    packet[3] = static_cast<uint8_t>(command);
    packet[4] = static_cast<uint8_t>(command >> 8);
    const uint16_t length = with_value ? 4 : 0;
    packet[5] = static_cast<uint8_t>(length);
    if (with_value) {
        for (uint8_t i = 0; i < 4; ++i)
            packet[7 + i] = static_cast<uint8_t>(value >> (8 * i));
    }
    uint16_t sum = 0;
    for (uint8_t i = 1; i < length + 7U; ++i)
        sum = static_cast<uint16_t>(sum + packet[i]);
    packet[length + 7] = static_cast<uint8_t>(sum);
    packet[length + 8] = static_cast<uint8_t>(sum >> 8);
    packet[length + 9] = 0x0D;
    packet[length + 10] = 0x0A;
    // Existing UART API only; no HAL handle exposure or new RX/DMA callbacks.
    // Its void return cannot report a TX failure; reply timeout covers no reply.
    m_uart->UARTTransmit(packet, length + 11U);
}

void IMU::FailBES102Config(BES102ConfigState state)
{
    be_stats.config_state = state;
    be_awaiting_reply = false;
    be_have_data = false;
    // Leave the sensor streaming rather than stranded in command mode.
    // Data remains invalid until a later successful Init/configuration.
    SendBES102Command(7, false, 0);
}

void IMU::ConsumeBES102ConfigReply(uint16_t command, const uint8_t* payload,
                                  uint16_t length)
{
    if (be_stats.config_state != BES102ConfigState::BUSY ||
        !be_awaiting_reply || be_config_step >= BEConfigCount) return;
    if (command == 1 && length == 0) {
        FailBES102Config(BES102ConfigState::NACK);
        return;
    }
    const uint16_t expected = BEExpectedReply(be_config_step);
    if (command != expected) return;
    if ((expected == 0 && length != 0) || (expected != 0 && length != 4)) {
        FailBES102Config(BES102ConfigState::READBACK_ERROR);
        return;
    }
    if (expected != 0) {
        const uint32_t value = ReadBE32(payload);
        if (be_config_step == 1) be_stats.initial_output_mask = value;
        if (be_config_step == 2) be_stats.initial_precision = value;
        if (be_config_step == 7) {
            be_stats.verified_output_mask = value;
            if (value != BES102_OUTPUT_MASK) {
                FailBES102Config(BES102ConfigState::READBACK_ERROR);
                return;
            }
        }
        if (be_config_step == 8) {
            be_stats.verified_precision = value;
            if (value != 1) {
                FailBES102Config(BES102ConfigState::READBACK_ERROR);
                return;
            }
        }
    }
    ++be_stats.config_replies;
    ++be_config_step;
    be_reply_retries = 0;
    be_awaiting_reply = false;
    if (be_config_step == BEConfigCount) {
        be_stats.config_state = BES102ConfigState::DONE;
        be_rate_start_ms = HAL_GetTick();
        be_rate_start_samples = be_stats.unique_samples;
    }
}

void IMU::ProcessBES102Config()
{
    if (!m_uart) return;
    const uint32_t now = HAL_GetTick();
    if (be_stats.config_state == BES102ConfigState::WAITING_BOOT) {
        if (now - be_boot_ms < 500) return;
        be_stats.config_state = BES102ConfigState::BUSY;
    }
    if (be_stats.config_state != BES102ConfigState::BUSY) return;
    if (be_awaiting_reply) {
        if (now - be_command_sent_ms < 300) return;
        // The original overwrite queue can lose a mode-switch ACK. Retry
        // idempotent queries/settings twice before reporting the timeout.
        if (be_reply_retries >= 2) {
            FailBES102Config(BES102ConfigState::TIMEOUT);
            return;
        }
        ++be_reply_retries;
        ++be_stats.config_retries;
        be_awaiting_reply = false;
    }
    if (be_config_step >= BEConfigCount) return;
    const uint16_t command = BEConfigCommands[be_config_step];
    const bool with_value = be_config_step >= 3 && be_config_step <= 6;
    uint32_t value = 0;
    if (be_config_step == 3) value = 1; // Float32
    if (be_config_step == 5) value = BES102_OUTPUT_MASK;
    if (be_config_step == 6) value = BES102_OUTPUT_HZ;
    be_stats.last_config_command = command;
    be_awaiting_reply = true;
    be_command_sent_ms = now;
    SendBES102Command(command, with_value, value);
}
