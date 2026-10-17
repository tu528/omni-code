#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"


class CONTROL final
{
public:
	uint8_t init_DM = 0;
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};

	enum MODE { TEST, RC, AUTOAIM, RESET, ROTATION, SPINNING, FOLLOW, SEPARATE, FIRE, STOP, LOCK, DEAD, REAL } mode = TEST;
	struct CHASSIS
	{


		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};
		int32_t speed_x{}, speed_y{}, speed_z{};

		// DEAD模式是否已经完成起点记录；false表示下一次Update需要重新初始化
		bool  dead_started = false;
		// DEAD当前运动段：0前、1右、2后、3左、4全部完成
		uint8_t dead_part = 0;
		// DEAD进入时记录的IMU世界yaw，后续用它保持底盘航向
		float dead_yaw_target = 0.0f;
		// DEAD当前运动段开始时，四个底盘轮子的累计里程起点
		float dead_d0, dead_d1, dead_d2, dead_d3;

		// REAL模式是否已经完成起点记录；与DEAD完全独立
		bool  real_started = false;
		// REAL自主流程锁存：去程结束进入AUTOAIM后，防止遥控器拨杆覆盖当前阶段
		bool  real_auto_active = false;
		// REAL是否正在按原路线返程
		bool  real_returning = false;
		// REAL返程当前运动段：0右1750、1后900、2右1100、3后1100、4左1100
		// 5后1300、6右1300、7后3100、8左1300、9后1150、10返程完成
		uint8_t real_return_part = 0;
		// REAL返程结束锁存：等待遥控器拨杆离开REAL后才允许重新选择模式
		bool  real_completed = false;
		// REAL当前运动段：
		// 0前1150、1右1300、2前3100、3左1300、4前1300
		// 5右1100、6前1100、7左1100、8前900、9左1750、10全部完成
		uint8_t real_part = 0;
		// REAL进入时记录的IMU世界yaw，后续用它保持底盘航向
		float real_yaw_target = 0.0f;
		// REAL当前运动段开始时，四个底盘轮子的累计里程起点
		float real_d0, real_d1, real_d2, real_d3;

		void Keep_Direction();
		// 根据DEAD当前运动方向，读取四个轮子的实际转速并换算成底盘线速度
		float DeadMeasuredSpeed(uint8_t dead_part);
		// DEAD速度规划：前段加速，接近目标时根据实际速度计算动态刹车
		float DeadSpeedProfile(
			float traveled_distance,
			float target_distance,
			float max_speed,
			float measured_speed_mmps
		);
		// 根据REAL当前运动方向，读取四个轮子的实际转速并换算成底盘线速度
		float RealMeasuredSpeed(uint8_t real_part);
		// REAL速度规划：前段加速，接近目标时根据实际速度计算动态刹车
		float RealSpeedProfile(
			float traveled_distance,
			float target_distance,
			float max_speed,
			float measured_speed_mmps
		);
		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{

		enum TYPE { YAW, PITCH };
		float mark_pitch{}, mark_yaw{};
		float keep_angle1{}, keep_angle2{};

		PID pantile_PID[3] = { {0.1f,0.0f,0.01f},{0.05f,0.f,0.f}, {0.f,0.f,0.f} };
		const float sensitivity = 2.5f;
		bool aim = false;
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference);
		void Update();
	};

	struct SHOOTER
	{

		float now_bullet_speed = 0.f;
		bool fire_last_single = false;
		bool fire_now_single = false;

		bool auto_shoot = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		int16_t shoot_speed = 6000;
		bool fire_detected = false;     // 电流尖峰检测中
		uint32_t last_fire_tick = 0;    // 最近一次发射的 tick
		void Update();
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;

	static int16_t Setrange(const int16_t original, const int16_t range);
	void Control_Pantile(float ch_yaw, float ch_pitch);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
	void init_dm();
private:

};

extern CONTROL ctrl;
