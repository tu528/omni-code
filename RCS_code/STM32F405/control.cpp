#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include <math.h>
#include "RC.h"
#include "motor.h"

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
		if (abs(delta) >= 10.f)
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta);

	}
	/*else if (type == PITCH)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;
		if (abs(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);
		}
	}*/
}

void CONTROL::CHASSIS::Keep_Direction()
{
	

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
		DMmotor[0].setSpeed = 0;
	}
	else if (ctrl.mode == CONTROL::TEST)
	{
		if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
		if (mark_yaw < 0.0)mark_yaw += 8192.0;
		can1_motor[4].setangle = mark_yaw;
		DMmotor[0].setPos = mark_pitch;

	}
	else if (ctrl.mode == CONTROL::FIRE)
	{
		if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
		if (mark_yaw < 0.0)mark_yaw += 8192.0;
		can1_motor[4].setangle = mark_yaw;
		DMmotor[0].setPos = mark_pitch;
	}
}

void CONTROL::SHOOTER::Update()
{
	

	 if (ctrl.mode == CONTROL::FIRE)
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

