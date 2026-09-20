#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include <math.h>
#include "RC.h"
#include "motor.h"
#include "xuc.h"

#define MAX_SPEED 3000.0f

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
	ctrl.pantile.mark_yaw = para.initial_yaw;
	ctrl.pantile.mark_pitch = para.initial_pitch;
	DMmotor[0].setSpeed = 5.0f;	
	DMmotor[0].setPos = para.initial_pitch;
	can1_motor[4].setangle = para.initial_yaw;
}

void CONTROL::Control_Pantile(float ch_yaw, float ch_pitch)//手动控制
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float adjangle = this->pantile.sensitivity * 3.f;

	ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);

	const float pitch_adjangle = 0.003f;   // 每周期 pitch 增量，按手感调大调小
	ctrl.pantile.mark_pitch -= (float)(pitch_adjangle * ch_pitch);
	if (ctrl.pantile.mark_pitch >= 0.35f)  ctrl.pantile.mark_pitch = 0.35f;
	if (ctrl.pantile.mark_pitch <= -0.324f) ctrl.pantile.mark_pitch = -0.324f;
	//ctrl.pantile.mark_pitch -= (float)(adjangle * ch_pitch);
	
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	float delta = 0, adjust = sensitivity;
	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;
		if (abs(delta) >= 3.0f)
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta);

	}
	else if (type == PITCH)
	{
		delta = ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch());
		if (abs(delta) >= 3.0f)
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta) / 57.2957795f;//deg->rad
	}
}

void CONTROL::CHASSIS::Keep_Direction()
{
	Motor* yaw = &can1_motor[4];

	if (!ctrl.pantile.pass_yaw_base)
	{
		ctrl.pantile.yaw_base = yaw->sum_angle;
		ctrl.pantile.pass_yaw_base = true;
	}

	//float deg = -(float)(yaw->sum_angle - ctrl.pantile.yaw_base) / 8192.0f * 360.0f;
	//float rad = deg * (PI / 180.0f);   // 换成弧度，给 sin/cos 用
	float rad = -(float)(yaw->sum_angle - ctrl.pantile.yaw_base) * (PI / 4096.0f);   // 换成弧度，给 sin/cos 用

	float vx = (float)speedx;   // 场地前进
	float vy = (float)speedy;   // 场地横移

	float c = cosf(rad);
	float s = sinf(rad);

	speedx = (int32_t)(vx * c + vy * s);
	speedy = (int32_t)(-vx * s + vy * c);

}

void CONTROL::CHASSIS::Update()
{
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
		float target2 = clamp_speed(-speedy * 0.707f - speedx * 0.707f + speedz);
		float target3 = clamp_speed(-speedy * 0.707f + speedx * 0.707f + speedz);
		float target0 = clamp_speed( speedy * 0.707f + speedx * 0.707f + speedz);
		float target1 = clamp_speed( speedy * 0.707f - speedx * 0.707f + speedz);

		can1_motor[0].setspeed = (int32_t)Ramp(target0, can1_motor[0].setspeed, ramp_slope);
		can1_motor[1].setspeed = (int32_t)Ramp(target1, can1_motor[1].setspeed, ramp_slope);
		can1_motor[2].setspeed = (int32_t)Ramp(target2, can1_motor[2].setspeed, ramp_slope);
		can1_motor[3].setspeed = (int32_t)Ramp(target3, can1_motor[3].setspeed, ramp_slope);

	}
	else if (ctrl.mode == CONTROL::SEPARATE)
	{
		speedx = ctrl.chassis.speedx;
		speedy = ctrl.chassis.speedy;
		speedz = ctrl.chassis.speedz;
		Keep_Direction();
		uint32_t ramp_slope;
		{
			ramp_slope = (fabsf(speedz) > (fabsf(speedx) + fabsf(speedy)))
				? 190 * 5
				: 150 * 5;
		}
		float target2 = clamp_speed(-speedy * 0.707f - speedx * 0.707f + speedz);
		float target3 = clamp_speed(-speedy * 0.707f + speedx * 0.707f + speedz);
		float target0 = clamp_speed(speedy * 0.707f + speedx * 0.707f + speedz);
		float target1 = clamp_speed(speedy * 0.707f - speedx * 0.707f + speedz);

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
		float target2 = clamp_speed(-speedy * 0.707f - speedx * 0.707f + speedz);
		float target3 = clamp_speed(-speedy * 0.707f + speedx * 0.707f + speedz);
		float target0 = clamp_speed(speedy * 0.707f + speedx * 0.707f + speedz);
		float target1 = clamp_speed(speedy * 0.707f - speedx * 0.707f + speedz);

		can1_motor[0].setspeed = (int32_t)Ramp(target0, can1_motor[0].setspeed, ramp_slope);
		can1_motor[1].setspeed = (int32_t)Ramp(target1, can1_motor[1].setspeed, ramp_slope);
		can1_motor[2].setspeed = (int32_t)Ramp(target2, can1_motor[2].setspeed, ramp_slope);
		can1_motor[3].setspeed = (int32_t)Ramp(target3, can1_motor[3].setspeed, ramp_slope);
	}
	
}

void CONTROL::PANTILE::Update()
{
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
		if (const bool xuc_active = xuc.RxFresh() && xuc.Rx_TJ.control_TJ == 1)
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
		while (mark_yaw > 8192.0f) mark_yaw -= 8192.0f;
		while (mark_yaw < 0.0f) mark_yaw += 8192.0f;
		can1_motor[4].setangle = mark_yaw;
		//mark_yaw -= 0.1f * ctrl.chassis.speedz;
	}
	else if (ctrl.mode == CONTROL::SEPARATE)
	{
		while (mark_yaw > 8192.0f) mark_yaw -= 8192.0f;
		while (mark_yaw < 0.0f) mark_yaw += 8192.0f;
		can1_motor[4].setangle = mark_yaw;
		DMmotor[0].setPos = mark_pitch;
	}
}

void CONTROL::SHOOTER::Update()
{
	
	if (ctrl.mode == CONTROL::AUTOAIM)
	{
		can2_motor[0].setspeed = -6000;
		can2_motor[1].setspeed = 6000;
		fire_now_single = (xuc.Rx_TJ.shoot_TJ == 2 && xuc.RxFresh());
		if (xuc.Rx_TJ.shoot_TJ == 1 && xuc.RxFresh() && xuc.Rx_TJ.control_TJ == 1)
		{
			can2_motor[2].shoot_mode_now = Motor::running;
		}
		/*else if(fire_now_single&&!(fire_last_single) )
		{
			can2_motor[2].shoot_mode_now = Motor::single;
			can2_motor[2].need_curcircle = 4;
		}*/
		else
		{
			can2_motor[2].shoot_mode_now = Motor::stop;
		}
		fire_last_single = fire_now_single;
	}
	 else if (ctrl.mode == CONTROL::FIRE)
	{
		can2_motor[0].setspeed = -2000;
		can2_motor[1].setspeed = 2000;
		
	}
	 else
	 {
		 can2_motor[0].setspeed = 0.0f;
		 can2_motor[1].setspeed = 0.0f;
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

