#include "power_off.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>

namespace
{
constexpr char kPowerOffCommand[] = "1";
}

PowerOffController power_off_controller;
TaskHandle_t PowerOffTask_Handler = nullptr;
volatile BaseType_t PowerOffTaskCreateResult = pdFAIL;

void PowerOffTask(void* argument)
{
	auto* controller = static_cast<PowerOffController*>(argument);
	configASSERT(controller != nullptr);

	while (true)
	{
		controller->Update();
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

void PowerOffController::Init(UART* uart)
{
	uart_ = uart;
	triggered_ = false;

	// Preload PB0 low before switching it to push-pull output mode.
	GPIO_CLK_ENABLE(POWER_OFF_GPIO_PORT);
	HAL_GPIO_WritePin(POWER_OFF_GPIO_PORT,
		POWER_OFF_GPIO_PIN,
		POWER_OFF_GPIO_INACTIVE_STATE);
	GPIO_Init(POWER_OFF_GPIO_PORT,
		GPIO_MODE_OUTPUT_PP,
		GPIO_NOPULL,
		POWER_OFF_GPIO_PIN);

	if (uart_ == nullptr)
	{
		return;
	}

	uart_->Init(POWER_OFF_UART_INSTANCE, POWER_OFF_UART_BAUD_RATE)
		.DMARxInit();
}

void PowerOffController::Update()
{
	// Keep PB0 high after triggering. Only an MCU reset clears the latch.
	if (triggered_)
	{
		HAL_GPIO_WritePin(POWER_OFF_GPIO_PORT,
			POWER_OFF_GPIO_PIN,
			POWER_OFF_GPIO_ACTIVE_STATE);
		return;
	}

	if (uart_ == nullptr || uart_->UartQueueHandler == nullptr)
	{
		return;
	}

	if (xQueueReceive(uart_->UartQueueHandler, rx_buffer_, 0) != pdTRUE)
	{
		return;
	}

	const uint32_t size =
		(uart_->dataDmaNum <= UART_MAX_LEN) ? uart_->dataDmaNum : UART_MAX_LEN;
	if (IsPowerOffCommand(rx_buffer_, size))
	{
		Trigger();
	}
}

void PowerOffController::Trigger()
{
	triggered_ = true;
	HAL_GPIO_WritePin(POWER_OFF_GPIO_PORT,
		POWER_OFF_GPIO_PIN,
		POWER_OFF_GPIO_ACTIVE_STATE);
}

bool PowerOffController::IsPowerOffCommand(const uint8_t* data, uint32_t size) const
{
	if (data == nullptr)
	{
		return false;
	}

	uint32_t begin = 0;
	while (begin < size &&
		(data[begin] == ' ' || data[begin] == '\r' || data[begin] == '\n' || data[begin] == '\t'))
	{
		++begin;
	}

	uint32_t end = size;
	while (end > begin &&
		(data[end - 1] == ' ' || data[end - 1] == '\r' || data[end - 1] == '\n' || data[end - 1] == '\t'))
	{
		--end;
	}

	const uint32_t command_size = sizeof(kPowerOffCommand) - 1U;
	return (end - begin == command_size) &&
		(std::memcmp(data + begin, kPowerOffCommand, command_size) == 0);
}
