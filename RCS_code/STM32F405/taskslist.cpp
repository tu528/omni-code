#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"
#include "power_off.h"
extern float Kp = 10;
extern float Kd = 0.6;
extern int start_flag;
void TASK::Init()
{
	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建任务

	xTaskCreate((TaskFunction_t)ArmTask,
		(const char*)"ArmTask",
		(uint16_t)LED_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)LED_TASK_PRIO,
		(TaskHandle_t*)&LedTask_Handler);

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&DecodeTask_Handler);

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&MotorTask_Handler);

	xTaskCreate((TaskFunction_t)CanTransimtTask,
		(const char*)"CanTransimtTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&CanTxTask_Handler);

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&ControlTask_Handler);

	PowerOffTaskCreateResult = xTaskCreate(
		(TaskFunction_t)PowerOffTask,
		(const char*)"PowerOffTask",
		(uint16_t)POWER_OFF_TASK_STACK_SIZE,
		(void*)&power_off_controller,
		(UBaseType_t)POWER_OFF_TASK_PRIORITY,
		(TaskHandle_t*)&PowerOffTask_Handler);
	configASSERT(PowerOffTaskCreateResult == pdPASS);

	taskEXIT_CRITICAL();            //退出临界区
	vTaskDelete(StartTask_Handler); //删除开始任务
}
int CNT = 0;
void MotorUpdateTask(void* pvParameters)
{
	
	while (1)
	{
	TickType_t xlastWakeTime = xTaskGetTickCount();
	
		for (auto& motor : can1_motor)motor.Ontimer(can1.data, can1.temp_data);

		for (auto& motor : can2_motor)motor.Ontimer(can2.data, can2.temp_data);

		int32_t cur = std::abs(can2_motor[0].torque_current) +
			std::abs(can2_motor[1].torque_current); // 取绝对值相加，正负抵消问题

		const int32_t THRESHOLD = 3000;   // 尖峰阈值，需要现场标定
		const uint32_t DEBOUNCE = 30;     // 不应期 ms，防止一次发射被重复触发

		uint32_t now = xTaskGetTickCount();
		if (cur > THRESHOLD && !ctrl.shooter.fire_detected &&
			(now - ctrl.shooter.last_fire_tick) > DEBOUNCE)
		{
			ctrl.shooter.fire_detected = true;   // 检测到一次发射
			ctrl.shooter.last_fire_tick = now;   // 记录发射时刻
		}
		else if (cur <= THRESHOLD)
		{
			ctrl.shooter.fire_detected = false;  // 尖峰回落，允许下次触发
		}

		DMmotor[0].State_Decode(can2, can2.jointidata)
			.DMmotor_Ontimer(can2, DMmotor[0].Kp, DMmotor[0].Kd, can2.jointpdata[0]);


	vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));//开始执行该任务之后1ms再执行该任务
}
}

void CanTransimtTask(void* pvParameters)
{
	while (true)
	{

		TickType_t xlastWakeTime1 = xTaskGetTickCount();

		switch ((timer.counter++) % 3)
		{
		case 0:
				DMmotor[0].DMmotor_transmit(1);
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8);
			can2.Transmit(0x1ff, can2.temp_data + 8);
			break;
		case 2:
			can1.Transmit(0x200, can1.temp_data);
			can2.Transmit(0x200, can2.temp_data);
		default:
			break;
		}
		
		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//开始执行该任务之后1ms再执行该任务

	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
		rc.Update();
		xuc.Encode();
		ctrl.chassis.Update();
		ctrl.pantile.Update();
		ctrl.shooter.Update();
		vTaskDelay(5);
	}
}


void DecodeTask(void* pvParameters)
{
	while (true)
	{
		rc.Decode();
		xuc.Decode();
		imu_pantile.Decode();
	
		vTaskDelay(5);
	}
}

void ArmTask(void* pvParameters)
{
	  //上电时只发一次，之后不再重复发送

	while (true)
	{
		DMmotor[0].DMmotorinit();
		power.Send();
		vTaskDelay(100);
	}
}





