#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include <math.h>
#include "RC.h"
#include "motor.h"
#include "xuc.h"
#include "task.h"

#define MAX_SPEED 3000.0f
#define PATH_FORWARD_SPEED 2500.0f   // DEAD和REAL共用的平移目标速度

#define PATH_YAW_KP 20.0f             // DEAD和REAL共用的yaw误差比例增益
#define PATH_YAW_MAX_SPEED 500.0f     // DEAD和REAL共用的最大speedz航向修正
#define PATH_YAW_DEADBAND 0.5f       // DEAD和REAL共用的yaw误差死区，单位：度

#define PATH_BRAKE_ACCEL_MMPS2 1000.0f // DEAD和REAL共用的初始 减速度（a），单位：mm/s^2
#define PATH_MIN_SPEED_COMMAND 120.0f  // DEAD和REAL共用的最低速度指令

#define REAL_START_SPEED_SCALE 0.40f  // REAL首次出发和射击后返程起步，从目标速度的40%开始

#define REAL_SEARCH_YAW_RANGE 600.0f   // REAL无目标时，以到位角为中心的搜索半径
#define REAL_SEARCH_YAW_STEP 2.0f      // 每个ControlTask周期的搜索步长，约17.6度/秒
#define REAL_SEARCH_DWELL_CYCLES 40    // 搜索端点停留约200ms（ControlTask周期约5ms）
#define REAL_RETURN_TIMEOUT_MS 180000UL // REAL启动后3分钟仍未收到返程指令时的保护时间

// REAL自动流程的计时起点，单位：FreeRTOS tick。
static uint32_t real_start_tick = 0;

float clamp_speed(float target)
{
	if (target > MAX_SPEED) return MAX_SPEED;
	if (target < -MAX_SPEED) return -MAX_SPEED;
	return target;
}

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			if (num1 < CHASSIS_MOTOR_NUM)
				chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			if (num2 < PANTILE_MOTOR_NUM)
				pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			if (num3 < SHOOTER_MOTOR_NUM)
				shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			if (num4 < SUPPLY_MOTOR_NUM)
			{
				supply_motor[num4] = motor[i];
				supply_motor[num4]->spinning = false;
				supply_motor[num4]->need_curcircle = false;
				num4++;
			}
			break;
		default:
			break;
		}
	}
	ctrl.pantile.mark_yaw = para.initial_yaw;//上电后的初始角度
	ctrl.pantile.mark_pitch = para.initial_pitch;
	DMmotor[0].setSpeed = 5.0f;//DM恒定速度
	DMmotor[0].setPos = para.initial_pitch;
	can1_motor[4].setangle = para.initial_yaw;
}

void CONTROL::Control_Pantile(float ch_yaw, float ch_pitch)//手动控制
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float adjangle = this->pantile.sensitivity * 3.f;
	const float pitch_adjangle = 0.003f;   // 每周期 pitch 增量，按手感调大调小

	ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);
	ctrl.pantile.mark_pitch -= (float)(pitch_adjangle * ch_pitch);
	if (ctrl.pantile.mark_pitch >= 0.35f)  ctrl.pantile.mark_pitch = 0.35f;//picth上下限幅
	if (ctrl.pantile.mark_pitch <= -0.324f) ctrl.pantile.mark_pitch = -0.324f;

}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	float delta = 0, adjust = sensitivity;
	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));
		if (delta <= -4096.f) delta += 8192.f;
		else if (delta >= 4096.f) delta -= 8192.f;
		if (abs(delta) >= 3.0f) mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta);
	}
	else if (type == PITCH)
	{
		delta = ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch());
		if (abs(delta) >= 1.0f)
		mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta) / 57.2957795f;//deg->rad
	}
}

void CONTROL::CHASSIS::Keep_Direction()//separate模式解算
{
	float mechanical_degree=can1_motor[4].angle[now]- para.initial_yaw;
	if (mechanical_degree > 8192.0f) mechanical_degree -= 8192.0f;
	if (mechanical_degree < 0.0f) mechanical_degree += 8192.0f;
	float deg = (mechanical_degree) / 8192.0f * 360.0f;
	float rad = deg / 57.2957795f;   // 换成弧度，给 sin/cos 用
	speedx =  (int32_t)(ctrl.chassis.speed_x * cosf(rad) + ctrl.chassis.speed_y * sinf(rad));
	speedy =  (int32_t)(-ctrl.chassis.speed_x * sinf(rad) + ctrl.chassis.speed_y * cosf(rad));

}

//// 根据四个M3508反馈的电机RPM，计算当前DEAD运动方向上的实际线速度。
//// 返回值单位为mm/s，并且只返回当前运动方向上的正速度。
//float CONTROL::CHASSIS::DeadMeasuredSpeed(uint8_t dead_part)
//{
//	float v0 = can1_motor[0].curspeed * RPM_TO_MMPS;
//	float v1 = can1_motor[1].curspeed * RPM_TO_MMPS;
//	float v2 = can1_motor[2].curspeed * RPM_TO_MMPS;
//	float v3 = can1_motor[3].curspeed * RPM_TO_MMPS;
//
//	// 与DEAD模式dx、dy正运动学保持相同的符号约定
//	float vx = (v0 + v1 - v2 - v3) * 0.70710678f / 2.0f;
//	float vy = (v0 - v1 - v2 + v3) * 0.70710678f / 2.0f;
//
//	float traveled_speed = 0.0f;
//	switch (dead_part)
//	{
//	case 0: // 向前
//		traveled_speed = -vx;
//		break;
//
//	case 1: // 向右
//		traveled_speed = -vy;
//		break;
//
//	case 2: // 向后
//		traveled_speed = vx;
//		break;
//
//	case 3: // 向左
//		traveled_speed = vy;
//		break;
//
//	default:
//		traveled_speed = 0.0f;
//		break;
//	}
//
//	// 编码器噪声或方向反向时，不返回负速度
//	if (traveled_speed < 0.0f)
//		traveled_speed = 0.0f;
//
//	return traveled_speed;
//}

// DEAD速度规划函数。
// traveled_distance：当前运动段已经走过的距离，单位mm。
// target_distance：当前运动段目标距离，单位mm。
// max_speed：允许的最大平移指令速度。
// measured_speed_mmps：由四轮编码器换算出的实际线速度，单位mm/s。
// 返回值：下一周期要给当前运动段的平移速度指令。
//float CONTROL::CHASSIS::DeadSpeedProfile(float traveled_distance,float target_distance,float max_speed,float measured_speed_mmps
//)
//{
//	if (target_distance <= 0.0f || max_speed <= 0.0f)
//		return 0.0f;
//
//	// 防止编码器噪声造成负距离
//	if (traveled_distance < 0.0f)
//		traveled_distance = 0.0f;
//
//	// 到达目标距离后不再继续给速度
//	if (traveled_distance >= target_distance)
//		return 0.0f;
//
//	float remaining_distance =
//		target_distance - traveled_distance;
//
//	// 根据当前实际速度计算刹车距离：d = v^2 / (2a)
//	float brake_distance = 0.0f;
//	if (measured_speed_mmps > 0.0f)
//	{
//		brake_distance =
//			measured_speed_mmps * measured_speed_mmps /
//			(2.0f * PATH_BRAKE_ACCEL_MMPS2);
//	}
//
//	// 进入动态刹车距离后，根据剩余距离降低目标速度
//	if (remaining_distance <= brake_distance &&
//		measured_speed_mmps > 0.0f)
//	{
//		float brake_speed_mmps = sqrtf(
//			2.0f * PATH_BRAKE_ACCEL_MMPS2 * remaining_distance
//		);
//
//		float brake_command =
//			brake_speed_mmps /
//			(0.70710678f * 2.0f * 3.1415926f * WHEEL_RADIUS_MM /
//			(60.0f * GEAR_RATIO));
//
//		if (brake_command < PATH_MIN_SPEED_COMMAND)
//			brake_command = PATH_MIN_SPEED_COMMAND;
//
//		if (brake_command > max_speed)
//			brake_command = max_speed;
//
//		return brake_command;
//	}
//
//	// 只保留前10%的平滑加速，后段由动态刹车控制
//	const float ramp_distance = target_distance * 0.10f;
//	const float min_speed_scale = 0.15f;
//
//	if (ramp_distance <= 0.0f)
//		return max_speed;
//
//	float speed_scale = 1.0f;
//
//	// 前 10%：平滑加速
//	if (traveled_distance < ramp_distance)
//	{
//		float progress = traveled_distance / ramp_distance;
//
//		speed_scale =
//			min_speed_scale +
//			(1.0f - min_speed_scale) *
//			(0.5f - 0.5f * cosf(3.1415926f * progress));
//	}
//	return max_speed * speed_scale;
//}

// REAL模式独立的实际速度计算函数，逻辑与DEAD对应函数一致，但状态入口完全独立。
float CONTROL::CHASSIS::RealMeasuredSpeed(uint8_t real_part)
{
	float v0 = can1_motor[0].curspeed * RPM_TO_MMPS;
	float v1 = can1_motor[1].curspeed * RPM_TO_MMPS;
	float v2 = can1_motor[2].curspeed * RPM_TO_MMPS;
	float v3 = can1_motor[3].curspeed * RPM_TO_MMPS;
	// 与REAL模式dx、dy正运动学保持相同的符号约定
	float vx = (v0 + v1 - v2 - v3) * 0.70710678f / 2.0f;
	float vy = (v0 - v1 - v2 + v3) * 0.70710678f / 2.0f;

	float traveled_speed = 0.0f;
	switch (real_part)
	{
	case 0: // 第1段：向前
	case 2: // 第3段：向前
	case 4: // 第5段：向前
	case 6: // 第7段：向前
	case 8: // 第9段：向前
		traveled_speed = -vx;
		break;
	case 1: // 第2段：向右
	case 5: // 第6段：向右
		traveled_speed = -vy;
		break;
	case 3: // 第4段：向左
	case 7: // 第8段：向左
	case 9: // 第10段：向左
		traveled_speed = vy;
		break;
	case 10: // 返程第1段：向右875
	case 12: // 返程第3段：向右550
	case 16: // 返程第7段：向右650
		traveled_speed = -vy;
		break;
	case 11: // 返程第2段：向后450
	case 13: // 返程第4段：向后550
	case 15: // 返程第6段：向后650
	case 17: // 返程第8段：向后1550
	case 19: // 返程第10段：向后575
		traveled_speed = vx;
		break;
	case 14: // 返程第5段：向左550
	case 18: // 返程第9段：向左650
		traveled_speed = vy;
		break;
	default:
		traveled_speed = 0.0f;
		break;
	}
	if (traveled_speed < 0.0f)
		traveled_speed = 0.0f;
	return traveled_speed;
}

// REAL速度规划函数。
// traveled_distance：当前REAL运动段已经走过的距离，单位mm。
// target_distance：当前REAL运动段目标距离，单位mm。
// max_speed：REAL允许的最大平移指令速度。
// measured_speed_mmps：由四轮编码器换算出的实际线速度，单位mm/s。
// 返回值：下一周期要给当前REAL运动段的平移速度指令。
float CONTROL::CHASSIS::RealSpeedProfile(
	float traveled_distance,
	float target_distance,
	float max_speed,
	float measured_speed_mmps
)
{
	if (target_distance <= 0.0f || max_speed <= 0.0f)
		return 0.0f;
	if (traveled_distance < 0.0f)
		traveled_distance = 0.0f;
	if (traveled_distance >= target_distance)
		return 0.0f;
	float remaining_distance =
		target_distance - traveled_distance;
	//--------------------------------------------------------------------------------------------
	// REAL模式根据当前实际速度计算刹车距离：d = v^2 / (2a)
	float brake_distance = 0.0f;
	if (measured_speed_mmps > 0.0f)
	{
		brake_distance =
			measured_speed_mmps * measured_speed_mmps /
			(2.0f * PATH_BRAKE_ACCEL_MMPS2);
	}
	// 进入动态刹车距离后，根据剩余距离降低目标速度
	if (remaining_distance <= brake_distance &&
		measured_speed_mmps > 0.0f)
	{
		float brake_speed_mmps = sqrtf(2.0f * PATH_BRAKE_ACCEL_MMPS2 * remaining_distance);//v=sqrt(2as)
		float brake_speed_rpm =
			brake_speed_mmps / (0.70710678f * RPM_TO_MMPS);//m/s->rpm
		if (brake_speed_rpm < PATH_MIN_SPEED_COMMAND)
			brake_speed_rpm = PATH_MIN_SPEED_COMMAND;
		if (brake_speed_rpm > max_speed)
			brake_speed_rpm = max_speed;

		return brake_speed_rpm;
	}//------------------------------------------------------------------------------
	// REAL模式只保留前10%的平滑加速，后段由动态刹车控制
	const float accelerate_distance = target_distance * 0.10f;
	// 只提高去程第一段和返程第一段的起步速度，中间换段仍按原比例加速。
	const bool real_start_segment = (!real_returning && real_part == 0) ||
		(real_returning && real_return_part == 0);
	const float min_speed_scale = real_start_segment ? REAL_START_SPEED_SCALE : 0.15f;
	if (accelerate_distance <= 0.0f)
		return 0.0f;
	if (traveled_distance < accelerate_distance)
	{
		float progress = traveled_distance / accelerate_distance;
		float speed_scale =
			min_speed_scale +(1.0f - min_speed_scale) *(0.5f - 0.5f *cosf(3.1415926f * progress));
		return max_speed * speed_scale;
	}
	return max_speed;
}

// 统一处理REAL返程启动，避免上位机指令和超时保护各写一套初始化逻辑。
static void StartRealReturn()
{
	if (ctrl.chassis.real_returning || ctrl.chassis.real_part < 20)
		return;

	ctrl.chassis.real_returning = true;
	ctrl.chassis.real_return_part = 0;
	ctrl.chassis.real_d0 = can1_motor[0].distance;
	ctrl.chassis.real_d1 = can1_motor[1].distance;
	ctrl.chassis.real_d2 = can1_motor[2].distance;
	ctrl.chassis.real_d3 = can1_motor[3].distance;
	ctrl.chassis.real_yaw_target = imu_pantile.GetAngleYaw();
	xuc.Tx_TJ.mode_TJ = 0;
	ctrl.shooter.return_started = true;
	ctrl.chassis.speedx = 0;
	ctrl.chassis.speedy = 0;
	ctrl.chassis.speedz = 0;
}

void CONTROL::CHASSIS::Update()
{
	// 上位机在REAL发射区发送control_TJ=2，表示停止射击并开始原路返程。
	const bool real_return_command =
		real_auto_active && real_part >= 20 && !real_returning &&
		xuc.RxFresh() && xuc.Rx_TJ.control_TJ == XUC_CONTROL_RETURN;

	// 计时从进入REAL开始，但只在到达射击区后触发，避免中途掉头走错路线。
	const bool real_return_timeout =
		real_auto_active && real_part >= 20 && !real_returning &&
		(uint32_t)(xTaskGetTickCount() - real_start_tick) >=
		pdMS_TO_TICKS(REAL_RETURN_TIMEOUT_MS);

	if (real_return_command || real_return_timeout)
	{
		StartRealReturn();
	}
	if (ctrl.mode == RESET)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;

		can1_motor[0].setspeed = 0;
		can1_motor[1].setspeed = 0;
		can1_motor[2].setspeed = 0;
		can1_motor[3].setspeed = 0;
	}
	else if (ctrl.mode == CONTROL::TEST)
	{
		speedx = ctrl.chassis.speedx;
		speedy = ctrl.chassis.speedy;
		speedz = ctrl.chassis.speedz;

		uint32_t ramp_slope;
		{
		ramp_slope = (fabsf(speedz) > (fabsf(speedx) + fabsf(speedy)))
			    ? 190 * 5
				: 150 * 5;
		}
		float target2 = -clamp_speed(-speedx * 0.707f - speedy * 0.707f + speedz);
		float target3 = -clamp_speed(-speedx * 0.707f + speedy * 0.707f + speedz);
		float target0 = -clamp_speed(speedx * 0.707f + speedy * 0.707f + speedz);
		float target1 = -clamp_speed(speedx * 0.707f - speedy * 0.707f + speedz);

		can1_motor[0].setspeed = (int32_t)Ramp(target0, can1_motor[0].setspeed, ramp_slope);
		can1_motor[1].setspeed = (int32_t)Ramp(target1, can1_motor[1].setspeed, ramp_slope);
		can1_motor[2].setspeed = (int32_t)Ramp(target2, can1_motor[2].setspeed, ramp_slope);
		can1_motor[3].setspeed = (int32_t)Ramp(target3, can1_motor[3].setspeed, ramp_slope);

	}
	else if (ctrl.mode == CONTROL::AUTOAIM)
	{
		// 射击区瞄准期间底盘保持停止，避免上一段平移速度残留。
		speedx = 0;
		speedy = 0;
		speedz = 0;
		can1_motor[0].setspeed = (int32_t)Ramp(0, can1_motor[0].setspeed, 150 * 5);
		can1_motor[1].setspeed = (int32_t)Ramp(0, can1_motor[1].setspeed, 150 * 5);
		can1_motor[2].setspeed = (int32_t)Ramp(0, can1_motor[2].setspeed, 150 * 5);
		can1_motor[3].setspeed = (int32_t)Ramp(0, can1_motor[3].setspeed, 150 * 5);
	}
	else if (ctrl.mode == CONTROL::SEPARATE)
	{
		Keep_Direction();
		uint32_t ramp_slope;//加速限制，防止加速过快，电流太大烧坏电机
		{
			ramp_slope = (fabsf(speedz) > (fabsf(speedx) + fabsf(speedy)))
				? 190 * 5
				: 150 * 5;
		}
		float target2 = -clamp_speed(-speedx * 0.707f - speedy * 0.707f + speedz);
		float target3 = -clamp_speed(-speedx * 0.707f + speedy * 0.707f + speedz);
		float target0 = -clamp_speed(speedx * 0.707f + speedy * 0.707f + speedz);
		float target1 = -clamp_speed(speedx * 0.707f - speedy * 0.707f + speedz);

		can1_motor[0].setspeed = (int32_t)Ramp(target0, can1_motor[0].setspeed, ramp_slope);
		can1_motor[1].setspeed = (int32_t)Ramp(target1, can1_motor[1].setspeed, ramp_slope);
		can1_motor[2].setspeed = (int32_t)Ramp(target2, can1_motor[2].setspeed, ramp_slope);
		can1_motor[3].setspeed = (int32_t)Ramp(target3, can1_motor[3].setspeed, ramp_slope);
	}
	else if (ctrl.mode == CONTROL::ROTATION)
	{
		speedx = ctrl.chassis.speedx;
		speedy = ctrl.chassis.speedy;
		speedz = ctrl.chassis.speedz;

		uint32_t ramp_slope;
		{
			ramp_slope = (fabsf(speedz) > (fabsf(speedx) + fabsf(speedy)))
				? 190 * 5
				: 150 * 5;
		}
		float target2 = clamp_speed(-speedx * 0.707f - speedy * 0.707f + speedz);
		float target3 = clamp_speed(-speedx * 0.707f + speedy * 0.707f + speedz);
		float target0 = clamp_speed(speedx * 0.707f + speedy * 0.707f + speedz);
		float target1 = clamp_speed(speedx * 0.707f - speedy * 0.707f + speedz);

		can1_motor[0].setspeed = (int32_t)Ramp(target0, can1_motor[0].setspeed, ramp_slope);
		can1_motor[1].setspeed = (int32_t)Ramp(target1, can1_motor[1].setspeed, ramp_slope);
		can1_motor[2].setspeed = (int32_t)Ramp(target2, can1_motor[2].setspeed, ramp_slope);
		can1_motor[3].setspeed = (int32_t)Ramp(target3, can1_motor[3].setspeed, ramp_slope);
	}
	// DEAD模式说明：以下代码保留为固定路线测试参考，目前整体注释，不参与编译。
	// DEAD原始路线为“前1 m → 右1 m → 后1 m → 左1 m”，方向由车头坐标定义。
	// DEAD使用四个底盘编码器计算dx、dy，并使用云台IMU的yaw修正底盘方向。
	//else if (ctrl.mode == CONTROL::DEAD)
	//{
	//	// DEAD一次完整流程：前1m → 右1m → 后1m → 左1m
	//	if (!dead_started)
	//	{
	//		dead_d0 = can1_motor[0].distance;
	//		dead_d1 = can1_motor[1].distance;
	//		dead_d2 = can1_motor[2].distance;
	//		dead_d3 = can1_motor[3].distance;
	//		dead_yaw_target = imu_pantile.GetAngleYaw();

	//		dead_part = 0;
	//		dead_started = true;
	//	}

	//	// dead_yaw_target是进入DEAD时的目标航向；当前yaw偏离后用speedz纠正底盘
	//	float yaw_error = ctrl.GetDelta(
	//		dead_yaw_target - imu_pantile.GetAngleYaw()
	//	);

	//	if (fabsf(yaw_error) < PATH_YAW_DEADBAND)
	//		yaw_error = 0.0f;

	//	float yaw_correction = PATH_YAW_KP * yaw_error;
	//	if (yaw_correction > PATH_YAW_MAX_SPEED)
	//		yaw_correction = PATH_YAW_MAX_SPEED;
	//	if (yaw_correction < -PATH_YAW_MAX_SPEED)
	//		yaw_correction = -PATH_YAW_MAX_SPEED;

	//	ctrl.chassis.speed_z = (int32_t)yaw_correction;
	//	speedz = ctrl.chassis.speed_z;

	//	// d0~d3是本段四轮里程增量；dx、dy是本段底盘平移位移，单位mm
	//	float d0 = can1_motor[0].distance - dead_d0;
	//	float d1 = can1_motor[1].distance - dead_d1;
	//	float d2 = can1_motor[2].distance - dead_d2;
	//	float d3 = can1_motor[3].distance - dead_d3;
	//	float dx = (d0 + d1 - d2 - d3) * 0.707f / 2.0f;
	//	float dy = (d0 - d1 - d2 + d3) * 0.707f / 2.0f;

	//	// traveled_distance表示当前dead_part这一段已经走过的有效距离
	//	float traveled_distance = 0.0f;

	//	switch (dead_part)
	//	{
	//	case 0: // 向前
	//		traveled_distance = -dx;
	//		break;

	//	case 1: // 向右
	//		traveled_distance = -dy;
	//		break;

	//	case 2: // 向后
	//		traveled_distance = dx;
	//		break;

	//	case 3: // 向左
	//		traveled_distance = dy;
	//		break;

	//	default:
	//		traveled_distance = 1000.0f; // 当前DEAD段目标距离，单位：mm
	//		break;
	//	}

	//	// part_done为true时，当前段结束并切换到下一段
	//	bool part_done = false;

	//	switch (dead_part)
	//	{
	//	case 0: // 向前
	//		if (dx <= -1000.0f) // 当前DEAD段目标距离，单位：mm
	//			part_done = true;
	//		break;

	//	case 1: // 向右
	//		if (dy <= -1000.0f) // 当前DEAD段目标距离，单位：mm
	//			part_done = true;
	//		break;

	//	case 2: // 向后
	//		if (dx >= 1000.0f) // 当前DEAD段目标距离，单位：mm
	//			part_done = true;
	//		break;

	//	case 3: // 向左
	//		if (dy >= 1000.0f) // 当前DEAD段目标距离，单位：mm
	//			part_done = true;
	//		break;

	//	default:
	//		break;
	//	}

	//	if (part_done)
	//	{
	//		if (dead_part < 3)
	//		{
	//			dead_part++;

	//			dead_d0 = can1_motor[0].distance;
	//			dead_d1 = can1_motor[1].distance;
	//			dead_d2 = can1_motor[2].distance;
	//			dead_d3 = can1_motor[3].distance;

	//			// 阶段切换时先给一个零速度目标
	//			ctrl.chassis.speed_x = 0;
	//			ctrl.chassis.speed_y = 0;
	//		}
	//		else
	//		{
	//			// 第四段完成，永久停止
	//			dead_part = 4;
	//			ctrl.chassis.speed_x = 0;
	//			ctrl.chassis.speed_y = 0;
	//		}
	//	}
	//	else
	//	{
	//		float measured_speed_mmps =
	//			DeadMeasuredSpeed(dead_part);

	//		float dead_speed = DeadSpeedProfile(
	//			traveled_distance,
	//			1000.0f, // 当前DEAD段目标距离，单位：mm
	//			PATH_FORWARD_SPEED,
	//			measured_speed_mmps
	//		);

	//		switch (dead_part)
	//		{
	//		case 0: // 向前
	//			ctrl.chassis.speed_x = dead_speed;
	//			ctrl.chassis.speed_y = 0;
	//			break;

	//		case 1: // 向右
	//			ctrl.chassis.speed_x = 0;
	//			ctrl.chassis.speed_y = dead_speed;
	//			break;

	//		case 2: // 向后
	//			ctrl.chassis.speed_x = -dead_speed;
	//			ctrl.chassis.speed_y = 0;
	//			break;

	//		case 3: // 向左
	//			ctrl.chassis.speed_x = 0;
	//			ctrl.chassis.speed_y = -dead_speed;
	//			break;

	//		case 4: // 全部完成
	//		default:
	//			ctrl.chassis.speed_x = 0;
	//			ctrl.chassis.speed_y = 0;
	//			break;
	//		}
	//	}

	//	// 保留Keep_Direction()，根据云台yaw电机编码器修正底盘平移坐标
	//	Keep_Direction();

	//	uint32_t ramp_slope = (fabsf(speedz) > (fabsf(speedx) + fabsf(speedy))) ? 190 * 5 : 150 *
	//		5;
	//	float target2 = -clamp_speed(-speedx * 0.707f - speedy * 0.707f + speedz);
	//	float target3 = -clamp_speed(-speedx * 0.707f + speedy * 0.707f + speedz);
	//	float target0 = -clamp_speed(speedx * 0.707f + speedy * 0.707f + speedz);
	//	float target1 = -clamp_speed(speedx * 0.707f - speedy * 0.707f + speedz);

	//	can1_motor[0].setspeed = (int32_t)Ramp(target0, can1_motor[0].setspeed, ramp_slope);
	//	can1_motor[1].setspeed = (int32_t)Ramp(target1, can1_motor[1].setspeed, ramp_slope);
	//	can1_motor[2].setspeed = (int32_t)Ramp(target2, can1_motor[2].setspeed, ramp_slope);
	//	can1_motor[3].setspeed = (int32_t)Ramp(target3, can1_motor[3].setspeed, ramp_slope);
	//
	//}
	else if (ctrl.mode == CONTROL::REAL)
	{
		// REAL一次完整流程：
		// 前1150 → 右1300 → 前3100 → 左1300 → 前1300
		// → 右1100 → 前1100 → 左1100 → 前900 → 左1750
		if (!real_started)//初始化
		{
			real_d0 = can1_motor[0].distance;
			real_d1 = can1_motor[1].distance;
			real_d2 = can1_motor[2].distance;
			real_d3 = can1_motor[3].distance;
			real_yaw_target = imu_pantile.GetAngleYaw();
			real_part = 0;
			real_returning = false;
			real_return_part = 0;
			real_auto_active = true;
			real_start_tick = xTaskGetTickCount();
			real_started = true;
		}
		// real_yaw_target是进入REAL时的目标航向；当前yaw偏离后用speedz纠正底盘
		if (real_returning || real_part < 20)
		{
			real_yaw_error = ctrl.GetDelta(
				real_yaw_target - imu_pantile.GetAngleYaw());
		}
		else
		{
			// 发射区底盘保持停止，云台扫描和自瞄时不再沿用行驶航向修正。
			real_yaw_error = 0.0f;
		}
		if (fabsf(real_yaw_error) < PATH_YAW_DEADBAND)
			real_yaw_error = 0.0f;
		float real_yaw_correction = PATH_YAW_KP * real_yaw_error;
		if (real_yaw_correction > PATH_YAW_MAX_SPEED)
			real_yaw_correction = PATH_YAW_MAX_SPEED;
		if (real_yaw_correction < -PATH_YAW_MAX_SPEED)
			real_yaw_correction = -PATH_YAW_MAX_SPEED;
		speedz = (int32_t)real_yaw_correction;

		// real_delta_d0~d3是本段四轮里程增量；real_dx、real_dy是本段位移
		float real_delta_d0 = can1_motor[0].distance - real_d0;
		float real_delta_d1 = can1_motor[1].distance - real_d1;
		float real_delta_d2 = can1_motor[2].distance - real_d2;
		float real_delta_d3 = can1_motor[3].distance - real_d3;
		float real_dx = (real_delta_d0 + real_delta_d1 - real_delta_d2 - real_delta_d3) * 0.707f / 2.0f;
		float real_dy = (real_delta_d0 - real_delta_d1 - real_delta_d2 + real_delta_d3) * 0.707f / 2.0f;

		// real_traveled_distance表示当前REAL段已经走过的有效距离
		float real_traveled_distance = 0.0f;
		// real_target_distance表示当前REAL段的目标距离，单位：mm
		float real_target_distance = 0.0f;
		// 去程使用real_part，返程使用10~19映射到real_return_part
		uint8_t real_motion_part = real_returning
			? (uint8_t)(10 + real_return_part)
			: real_part;
		switch (real_motion_part)
		{
		case 0: // 第1段：向前1150 mm
			real_traveled_distance = real_dx;
			real_target_distance = 1150.0f;
			break;
		case 1: // 第2段：向右1300 mm
			real_traveled_distance = real_dy;
			real_target_distance = 1300.0f;
			break;
		case 2: // 第3段：向前3100 mm
			real_traveled_distance = real_dx;
			real_target_distance = 3100.0f;
			break;
		case 3: // 第4段：向左1300 mm
			real_traveled_distance = -real_dy;
			real_target_distance = 1300.0f;
			break;
		case 4: // 第5段：向前1300 mm
			real_traveled_distance = real_dx;
			real_target_distance = 1300.0f;
			break;
		case 5: // 第6段：向右1100 mm
			real_traveled_distance = real_dy;
			real_target_distance = 1100.0f;
			break;
		case 6: // 第7段：向前1100 mm
			real_traveled_distance = real_dx;
			real_target_distance = 1100.0f;
			break;
		case 7: // 第8段：向左1100 mm
			real_traveled_distance = -real_dy;
			real_target_distance = 1100.0f;
			break;
		case 8: // 第9段：向前900 mm
			real_traveled_distance = real_dx;
			real_target_distance = 900.0f;
			break;
		case 9: // 第10段：向左1750 mm
			real_traveled_distance = -real_dy;
			real_target_distance = 1750.0f;
			break;
		case 10: // 返程第1段：向右1750 mm
			real_traveled_distance = real_dy;
			real_target_distance = 1750.0f;
			break;
		case 11: // 返程第2段：向后900 mm
			real_traveled_distance = -real_dx;
			real_target_distance = 900.0f;
			break;
		case 12: // 返程第3段：向右1100 mm
			real_traveled_distance = real_dy;
			real_target_distance = 1100.0f;
			break;
		case 13: // 返程第4段：向后1100 mm
			real_traveled_distance = -real_dx;
			real_target_distance = 1100.0f;
			break;
		case 14: // 返程第5段：向左1100 mm
			real_traveled_distance = -real_dy;
			real_target_distance = 1100.0f;
			break;
		case 15: // 返程第6段：向后1300 mm
			real_traveled_distance = -real_dx;
			real_target_distance = 1300.0f;
			break;
		case 16: // 返程第7段：向右1300 mm
			real_traveled_distance = real_dy;
			real_target_distance = 1300.0f;
			break;
		case 17: // 返程第8段：向后3100 mm
			real_traveled_distance = -real_dx;
			real_target_distance = 3100.0f;
			break;
		case 18: // 返程第9段：向左1300 mm
			real_traveled_distance = -real_dy;
			real_target_distance = 1300.0f;
			break;
		case 19: // 返程第10段：向后1150 mm
			real_traveled_distance = -real_dx;
			real_target_distance = 1150.0f;
			break;
		default:
			real_traveled_distance = 0.0f;
			real_target_distance = 0.0f;
			break;
		}
		// 所有REAL段都统一按“本段有效里程 >= 本段目标距离”判断完成
		bool real_part_done =
			(real_motion_part >= 20) ||(real_traveled_distance >= real_target_distance);

		if (real_part_done)
		{
			if (!real_returning && real_part < 9)
			{
				real_part++;

				real_d0 = can1_motor[0].distance;
				real_d1 = can1_motor[1].distance;
				real_d2 = can1_motor[2].distance;
				real_d3 = can1_motor[3].distance;

				// REAL阶段切换时先给一个零速度目标
				speedx = 0;
				speedy = 0;
			}
			else if (!real_returning)
			{
				// REAL去程第十段完成，进入AUTOAIM等待射击。
				ctrl.chassis.real_d0 = can1_motor[0].distance;
				ctrl.chassis.real_d1 = can1_motor[1].distance;
				ctrl.chassis.real_d2 = can1_motor[2].distance;
				ctrl.chassis.real_d3 = can1_motor[3].distance;
				ctrl.chassis.real_part = 20;
				speedx = 0;
				speedy = 0;
				speedz = 0;
				real_yaw_error = 0.0f;
				xuc.Tx_TJ.mode_TJ = 1;
			}
			else if (real_return_part < 9)
			{
				real_return_part++;
				real_d0 = can1_motor[0].distance;
				real_d1 = can1_motor[1].distance;
				real_d2 = can1_motor[2].distance;
				real_d3 = can1_motor[3].distance;
				// REAL返程阶段切换时先给一个零速度目标
				speedx = 0;
				speedy = 0;
			}
			else
			{
				// REAL返程第十段完成，回到启动区并停止。
				ctrl.chassis.real_return_part = 10;
				ctrl.chassis.real_returning = false;
				ctrl.chassis.real_auto_active = false;
				ctrl.chassis.real_completed = true;
				ctrl.mode = CONTROL::RESET;
				speedx = 0;
				speedy = 0;
			}
		}
		else
		{
			float real_measured_speed_mmps =
				RealMeasuredSpeed(real_motion_part);

			float real_speed = RealSpeedProfile
			(
				real_traveled_distance,
				real_target_distance,
				PATH_FORWARD_SPEED,
				real_measured_speed_mmps
			);

			switch (real_motion_part)
			{
			case 0: // 第1段：向前
			case 2: // 第3段：向前
			case 4: // 第5段：向前
			case 6: // 第7段：向前
			case 8: // 第9段：向前
				speedx = real_speed;
				speedy = 0;
				break;
			case 1: // 第2段：向右
			case 5: // 第6段：向右
				speedx = 0;
				speedy = real_speed;
				break;
			case 3: // 第4段：向左
			case 7: // 第8段：向左
			case 9: // 第10段：向左
				speedx = 0;
				speedy = -real_speed;
				break;
			case 10: // 返程第1段：向右875
			case 12: // 返程第3段：向右550
			case 16: // 返程第7段：向右650
				speedx = 0;
				speedy = real_speed;
				break;
			case 11: // 返程第2段：向后450
			case 13: // 返程第4段：向后550
			case 15: // 返程第6段：向后650
			case 17: // 返程第8段：向后1550
			case 19: // 返程第10段：向后575
				speedx = -real_speed;
				speedy = 0;
				break;
			case 14: // 返程第5段：向左550
			case 18: // 返程第9段：向左650
				speedx = 0;
				speedy = -real_speed;
				break;
			case 20: // 全部完成
			default:
				speedx = 0;
				speedy = 0;
				break;
			}
		}
		uint32_t ramp_slope;//加速限制，防止加速过快，电流太大烧坏电机
		{
			ramp_slope = (fabsf(speedz) > (fabsf(speedx) + fabsf(speedy)))
				? 190 * 5
				: 150 * 5;
		}
		float target2 = -clamp_speed(-speedx * 0.707f - speedy * 0.707f + speedz);
		float target3 = -clamp_speed(-speedx * 0.707f + speedy * 0.707f + speedz);
		float target0 = -clamp_speed(speedx * 0.707f + speedy * 0.707f + speedz);
		float target1 = -clamp_speed(speedx * 0.707f - speedy * 0.707f + speedz);

		can1_motor[0].setspeed = (int32_t)Ramp(target0, can1_motor[0].setspeed, ramp_slope);
		can1_motor[1].setspeed = (int32_t)Ramp(target1, can1_motor[1].setspeed, ramp_slope);
		can1_motor[2].setspeed = (int32_t)Ramp(target2, can1_motor[2].setspeed, ramp_slope);
		can1_motor[3].setspeed = (int32_t)Ramp(target3, can1_motor[3].setspeed, ramp_slope);
	}
}

void CONTROL::PANTILE::Update()
{
	if (ctrl.mode != CONTROL::REAL)
	{
		real_search_initialized = false;
		real_tracking_active = false;
	}
	if (ctrl.mode == RESET)
	{
		can1_motor[4].setspeed = 0;
	}
	else if (ctrl.mode == CONTROL::TEST)
	{
		if (mark_yaw > 8192.0) mark_yaw -= 8192.0;
		if (mark_yaw < 0.0) mark_yaw += 8192.0;
		can1_motor[4].setangle = mark_yaw;
		DMmotor[0].setPos = mark_pitch;
	}
	else if (ctrl.mode == CONTROL::FIRE)
	{
		if (mark_yaw > 8192.0) mark_yaw -= 8192.0;
		if (mark_yaw < 0.0) mark_yaw += 8192.0;
		can1_motor[4].setangle = mark_yaw;
		DMmotor[0].setPos = mark_pitch;
	}
	else if (ctrl.mode == CONTROL::AUTOAIM)
	{
		if (xuc.RxFresh() && xuc.Rx_TJ.control_TJ == XUC_CONTROL_AUTOAIM)
		{// 上位机给的是 rad，当前 Keep_Pantile 按 deg 计算
			const float target_yaw_deg = xuc.GetTargetYaw() * 57.2957795f;
			const float target_pitch_deg = xuc.GetTargetPitch() * 57.2957795f;
			Keep_Pantile(target_yaw_deg, PANTILE::YAW, imu_pantile);
			Keep_Pantile(target_pitch_deg, PANTILE::PITCH, imu_pantile);
		}
		if (ctrl.pantile.mark_pitch >= 0.35f)  ctrl.pantile.mark_pitch = 0.35f;
		if (ctrl.pantile.mark_pitch <= -0.324f) ctrl.pantile.mark_pitch = -0.324f;
		if (mark_yaw >= 8192.0f) mark_yaw -= 8192.0f;
		if (mark_yaw < 0.0f)    mark_yaw += 8192.0f;
		can1_motor[4].setangle = mark_yaw;
		DMmotor[0].setPos = mark_pitch;
	}
	else if (ctrl.mode == CONTROL::ROTATION)
	{
		ctrl.pantile.Keep_Pantile(keep_angle1, PANTILE::YAW, imu_pantile);
		if (mark_yaw > 8192.0f) mark_yaw -= 8192.0f;
		if (mark_yaw < 0.0f) mark_yaw += 8192.0f;
		can1_motor[4].setangle = mark_yaw;

	}
	else if (ctrl.mode == CONTROL::SEPARATE)
	{
		if (mark_yaw > 8192.0f) mark_yaw -= 8192.0f;
		if (mark_yaw < 0.0f) mark_yaw += 8192.0f;
		can1_motor[4].setangle = mark_yaw;
		DMmotor[0].setPos = mark_pitch;
	}
	else if (ctrl.mode == CONTROL::REAL)
	{
		if (xuc.Tx_TJ.mode_TJ == 1)
		{
			if (!real_search_initialized)
			{
				// 只在进入发射区后的第一次搜索时锁存真实云台反馈角度。
				real_search_center_yaw = can1_motor[4].angle[now];
				real_search_offset = 0;
				real_search_direction = -1;
				real_search_dwell = 0;
				real_tracking_active = false;
				real_search_initialized = true;
			}

			if (xuc.RxFresh() && xuc.Rx_TJ.control_TJ == XUC_CONTROL_AUTOAIM)
			{
				// 有目标：停止搜索，继续使用原来的自瞄跟踪。
				real_tracking_active = true;
				const float target_yaw_deg = xuc.GetTargetYaw() * 57.2957795f;
				const float target_pitch_deg = xuc.GetTargetPitch() * 57.2957795f;
				Keep_Pantile(target_yaw_deg, PANTILE::YAW, imu_pantile);
				Keep_Pantile(target_pitch_deg, PANTILE::PITCH, imu_pantile);
			}
			else
			{
				// 无目标：以到位时的实际云台角为中心，在相对±600范围内往复扫描。
				if (real_tracking_active)
				{
					// 丢失目标时从实际角度接续扫描，中心仍保持到位时的角度。
					real_search_offset = can1_motor[4].angle[now] - real_search_center_yaw;
					while (real_search_offset > 4096.0f) real_search_offset -= 8192.0f;
					while (real_search_offset < -4096.0f) real_search_offset += 8192.0f;
					// 自瞄越过搜索边界后，也只按扫描步长逐步回到范围内。
					if (real_search_offset <= -REAL_SEARCH_YAW_RANGE)
						real_search_direction = 1;
					else if (real_search_offset >= REAL_SEARCH_YAW_RANGE)
						real_search_direction = -1;
					real_search_dwell = 0;
					real_tracking_active = false;
				}
				if (real_search_dwell > 0)
				{
					real_search_dwell--;
				}
				else
				{
					real_search_offset += real_search_direction * REAL_SEARCH_YAW_STEP;
					if (real_search_direction < 0 && real_search_offset <= -REAL_SEARCH_YAW_RANGE)
					{
						real_search_offset = -REAL_SEARCH_YAW_RANGE;
						real_search_direction = 1;
						real_search_dwell = REAL_SEARCH_DWELL_CYCLES;
					}
					else if (real_search_direction > 0 && real_search_offset >= REAL_SEARCH_YAW_RANGE)
					{
						real_search_offset = REAL_SEARCH_YAW_RANGE;
						real_search_direction = -1;
						real_search_dwell = REAL_SEARCH_DWELL_CYCLES;
					}
				}

				float search_yaw = real_search_center_yaw + real_search_offset;
				while (search_yaw >= 8192.0f) search_yaw -= 8192.0f;
				while (search_yaw < 0.0f) search_yaw += 8192.0f;
				mark_yaw = search_yaw;
			}
		}
		else
		{
			// REAL去程尚未到发射区，保持当前云台目标角，不启动搜索。
			real_search_initialized = false;
			real_tracking_active = false;
		}
		if (ctrl.pantile.mark_pitch >= 0.35f)  ctrl.pantile.mark_pitch = 0.35f;
		if (ctrl.pantile.mark_pitch <= -0.324f) ctrl.pantile.mark_pitch = -0.324f;
		if (mark_yaw >= 8192.0f) mark_yaw -= 8192.0f;
		if (mark_yaw < 0.0f)    mark_yaw += 8192.0f;
		can1_motor[4].setangle = mark_yaw;
		DMmotor[0].setPos = mark_pitch;
	}
}

void CONTROL::SHOOTER::Update()
{

	if (ctrl.mode == CONTROL::AUTOAIM)
	{
		can2_motor[0].setspeed = -7000;
		can2_motor[1].setspeed = 7000;
		fire_now_single = (xuc.Rx_TJ.shoot_TJ == 1 && xuc.RxFresh() &&xuc.Rx_TJ.control_TJ == XUC_CONTROL_AUTOAIM);
		if(fire_now_single&&!(fire_last_single)&& can2_motor[2].shoot_mode_now == Motor::stop)
		{
			can2_motor[2].shoot_mode_now = Motor::single;
			can2_motor[2].need_curcircle = 5;
		}
		fire_last_single = fire_now_single;
	}
	 else if (ctrl.mode == CONTROL::FIRE)
	{
		can2_motor[0].setspeed = -7000;
		can2_motor[1].setspeed = 7000;

	}
	else if (ctrl.mode == CONTROL::REAL)
	{
		if (xuc.Tx_TJ.mode_TJ == 1)
		{
			can2_motor[0].setspeed = -7000;
			can2_motor[1].setspeed = 7000;
		}
		else
		{
			can2_motor[0].setspeed = 0;
			can2_motor[1].setspeed = 0;
		}
		fire_now_single = (xuc.Rx_TJ.shoot_TJ == 1 && xuc.RxFresh() && xuc.Rx_TJ.control_TJ == XUC_CONTROL_AUTOAIM);
		if (fire_now_single && !(fire_last_single) && can2_motor[2].shoot_mode_now == Motor::stop)
		{
			can2_motor[2].shoot_mode_now = Motor::single;
			can2_motor[2].need_curcircle = 5;
			can2_motor[2].shoot_num += 1;
		}
		fire_last_single = fire_now_single;
		if(can2_motor[2].shoot_num >=50 && !ctrl.chassis.real_returning)
		{
			xuc.Tx_TJ.mode_TJ = 0;
			if (!return_started)
			{
				ctrl.chassis.real_d0 = can1_motor[0].distance;
				ctrl.chassis.real_d1 = can1_motor[1].distance;
				ctrl.chassis.real_d2 = can1_motor[2].distance;
				ctrl.chassis.real_d3 = can1_motor[3].distance;
				ctrl.chassis.real_yaw_target = imu_pantile.GetAngleYaw();
				ctrl.chassis.real_returning = true;
				return_started = true;
			}
		}
	}
	 else
	 {
		 can2_motor[0].setspeed = 0.0f;
		 can2_motor[1].setspeed = 0.0f;
		 
	 }
	if (ctrl.mode == CONTROL::RESET)
	{
		can2_motor[2].setspeed = 0.0f;
	}
}
float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)//防止电机速度变化过快，导致电流过大，电机烧毁
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}
