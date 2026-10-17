#pragma once
#include "stm32f4xx.h"
#include "usart.h"
#include <string.h>

enum IMU_TYPE { IMU601 = 0, CH010, HI226, BE_S102 };

class IMU
{
public:
    float ACC_FSR = 4.f, GYRO_FSR = 2000.f;
    typedef struct { float roll, yaw, pitch; } Angle, AngularVelocity;
    struct Acceleration { float x{}, y{}, z{}; };

    struct BES102Data {
        uint32_t timestamp_ticks = 0;
        uint32_t sequence = 0;
        uint32_t received_ms = 0;
        float accel_g[3] = {};
        float gyro_dps[3] = {};
        float quaternion_wxyz[4] = {};
        float euler_deg[3] = {};  // roll, pitch, yaw
    };
    enum class BES102ConfigState : uint8_t {
        MANUAL, WAITING_BOOT, BUSY, DONE, TIMEOUT, NACK, READBACK_ERROR
    };
    struct BES102Stats {
        uint32_t valid_frames = 0;
        uint32_t unique_samples = 0;
        uint32_t duplicate_samples = 0;
        uint32_t estimated_lost_samples = 0;
        uint32_t timestamp_resets = 0;
        uint32_t checksum_errors = 0;
        uint32_t format_errors = 0;
        uint32_t payload_errors = 0;
        uint32_t id_errors = 0;
        uint16_t last_payload_bytes = 0;  // Last complete, checksum-valid IMU frame
        uint16_t last_frame_offset = 0xFFFF; // 0xFFFF: no validated IMU frame yet
        float update_hz = 0;
        BES102ConfigState config_state = BES102ConfigState::MANUAL;
        uint16_t last_config_command = 0;
        uint32_t config_replies = 0;
        uint32_t initial_output_mask = 0;
        uint32_t initial_precision = 0;
        uint32_t verified_output_mask = 0;
        uint32_t verified_precision = 0;
        uint32_t config_retries = 0;
    };
    // Default auto-configuration: Float32, degrees, mask 0x1822, output rate below.
    // LPBUS, sensor ID and UART baud must already match the constants below.
    // This board's firmware was measured at 1000 timestamp ticks per second.
    // Check this constant when using firmware with a different timestamp unit.
    static constexpr uint16_t BES102_SENSOR_ID = 1;
    // Official SDK sends UART commands to address 0; data retains sensor ID.
    static constexpr uint16_t BES102_COMMAND_ADDRESS = 0;
    static constexpr uint32_t BES102_TIMESTAMP_HZ = 1000;
    static constexpr uint32_t BES102_OUTPUT_HZ = 500;
    static constexpr uint32_t BES102_OUTPUT_MASK = 0x1822;
    static constexpr uint32_t BES102_BAUD = 460800;

    void Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate, IMU_TYPE type);
    void Decode();
    bool Check(uint8_t* pdata, uint8_t len, uint32_t com);
    float GetAngleYaw();
    float GetAnglePitch();
    float GetAngleRoll();
    float getangularvelocitypitch();
    float* GetAcceleration();
    int16_t getword(uint8_t HighBit, uint8_t LowBits);

    bool IsOnline(uint32_t timeout_ms = 50) const;
    bool GetBES102Data(BES102Data& data) const;
    BES102Stats GetBES102Stats() const;
    // Call before Init() only. Default true: configure RAM with ACK/readback.
    void SetBES102AutoConfigure(bool enabled) { be_auto_configure = enabled; }

    BaseType_t pd_Rx = false;
    QueueHandle_t* queueHandler = NULL;
private:
    // Original fields and getter API remain the source of upper-layer data.
    Angle angle{};
    AngularVelocity angularvelocity{};
    Acceleration acceleration{};
    float acceleration_view[3]{}; // stable storage for original pointer getter
    uint16_t crc = 0;
    IMU_TYPE type = CH010;
    uint8_t rxData[UART_MAX_LEN]{};
    UART* m_uart = nullptr;

    BES102Data be_data{};
    BES102Stats be_stats{};
    bool be_have_data = false;
    uint32_t be_rate_start_ms = 0;
    uint32_t be_rate_start_samples = 0;
    bool DecodeBES102Block();
    bool DecodeBES102Data(const uint8_t* frame);
    void UpdateBES102Rate();
    bool be_auto_configure = true;
    uint8_t be_config_step = 0;
    uint8_t be_reply_retries = 0;
    bool be_awaiting_reply = false;
    uint32_t be_command_sent_ms = 0;
    uint32_t be_boot_ms = 0;
    void ProcessBES102Config();
    void ConsumeBES102ConfigReply(uint16_t command, const uint8_t* payload,
                                 uint16_t length);
    void SendBES102Command(uint16_t command, bool with_value, uint32_t value);
    void FailBES102Config(BES102ConfigState state);
};

static inline uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static inline uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static inline float R4(uint8_t* p) { float r; memcpy(&r, p, 4); return r; }
extern IMU imu_chassis, imu_pantile;
