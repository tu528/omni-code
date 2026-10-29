#pragma once

#include "gpio.h"
#include "usart.h"
#include "task.h"

#include <cstdint>

// A22 transparent-transmission UART. UART4 uses PA0 (TX) and PA1 (RX).
#define POWER_OFF_UART_INSTANCE UART4
#define POWER_OFF_UART_BAUD_RATE 38400U

// Power-cut control output. Change these macros to match the board schematic.
#define POWER_OFF_GPIO_PORT GPIOB
#define POWER_OFF_GPIO_PIN GPIO_PIN_0
#define POWER_OFF_GPIO_ACTIVE_STATE   GPIO_PIN_SET
#define POWER_OFF_GPIO_INACTIVE_STATE GPIO_PIN_RESET

#define POWER_OFF_TASK_STACK_SIZE 128U
#define POWER_OFF_TASK_PRIORITY 3U

class PowerOffController
{
public:
	void Init(UART* uart);
	void Update();
	void Trigger();
	bool IsTriggered() const { return triggered_; }

private:
	bool IsPowerOffCommand(const uint8_t* data, uint32_t size) const;

	UART* uart_ = nullptr;
	uint8_t rx_buffer_[UART_MAX_LEN]{};
	bool triggered_ = false;
};

extern PowerOffController power_off_controller;
extern TaskHandle_t PowerOffTask_Handler;
extern volatile BaseType_t PowerOffTaskCreateResult;

void PowerOffTask(void* argument);
