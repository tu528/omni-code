#include "motor.h"
#include "gpio.h"
#include "HTmotor.h"
#include "imu.h"
#include "control.h"
#include "RC.h"
#include "xuc.h"
#include "task.h"
#define DEG_TO_RAD 0.017453292f  // �� / 180
Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _speed2)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[speed2], &_speed2, sizeof(PID));
	this->function = function;
}


Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	this->function = function;
}

void Motor::StatusIdentifier(int32_t torque_current)
{
	if (torque_current == old_torque_current)
		disconnectCount++;
	else
		disconnectCount = 0;

	if (disconnectCount >= disconnectMax)
	{
		disconnectCount = disconnectMax;
		if (old_torque_current == 0)
			m_status = UNCONNECTED;
		else
			m_status = DISCONNECTED;
	}
	else
		m_status = FINE;

	old_torque_current = torque_current;
}
uint8_t Motor::getStatus()const
{
	return (uint8_t)m_status;
}
void Motor::Ontimer(uint8_t idata[][8], uint8_t* odata)//idate: receive;odate: trainsmit;RC
{
	uint32_t trainsmit_or_receive_ID = this->ID - ID1;

	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID += 4;
	}*/
	//----------------------------------------------------------------
	this->torque_current = getword(idata[trainsmit_or_receive_ID][4], idata[trainsmit_or_receive_ID][5]);
	this->StatusIdentifier(this->torque_current);
	this->angle[now] = getword(idata[trainsmit_or_receive_ID][0], idata[trainsmit_or_receive_ID][1]);
	this->temperature = idata[trainsmit_or_receive_ID][6];
	//Get currrent speed

	motor_status = 0;
	if (temperature > 70) {
		setspeed = 0;
	}

	if (type == EC60)
	{
		curspeed = static_cast<float>(getdeltaa(angle[now] - angle[pre])) / T / 8192.f * 60.f;
	}
	else {
		curspeed = getword(idata[trainsmit_or_receive_ID][2], idata[trainsmit_or_receive_ID][3]);
	}
	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID -= 4;
	}*/
	/*----------------------------------------------------------------
	20220121--hz*/
    if (mode == ACE)
    {
        UpdateFeeder();
    }

	else if (mode == POS)
	{
		setspeed = pid[position].Position(getdeltaa((int16_t)(setangle - angle[0])),10000.f);//最短路径
		current = pid[speed].Position(setspeed - curspeed, 10000.f);
		setcurrent = current;
	}
	else if (mode == SPD)
	{

		current = pid[speed].Position(setspeed - curspeed, 10000.f);
		setcurrent = current;
	}
	recorded_the_Laps();
	GetDistanceFromMechanicalAngle();
	angle[pre] = angle[now];
	current = setrange(current, maxcurrent);
	odata[trainsmit_or_receive_ID * 2] = (current & 0xff00) >> 8;
	odata[trainsmit_or_receive_ID * 2 + 1] = current & 0x00ff;
}
void Motor::recorded_the_Laps() {
	int16_t delta = angle[now] - angle[pre];
	// �������ƣ�˳ʱ��
	if (delta > 8192 / 2)
		delta -= 8192;
	// �������ƣ���ʱ��
	else if (delta < -8192 / 2)
		delta += 8192;

	sum_angle+= delta;
//	round_count = total_count / encoder_resolution;
}

uint8_t initial_cnt=0;
void Motor::GetDistanceFromMechanicalAngle() {
	if (initial_cnt<5)
	initial_cnt++;
	distance=-(6.2831853f/ 8192.0f)*sum_angle * (WHEEL_RADIUS_MM / GEAR_RATIO)-initial_x;

	if(initial_cnt<3)
	initial_x = distance;
}

void Motor::getmax(const type_t type)
{
	adjspeed = 3000;
	switch (type)
	{
	case M3508:
		maxcurrent = 16384;
		maxspeed = 3800;
		break;
	case M3510:
		maxcurrent = 13000;
		maxspeed = 9000;
		break;
	case M2310:
		maxcurrent = 13000;
		maxspeed = 9000;
		adjspeed = 1000;
		break;
	case EC60:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6623:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6020:
		maxcurrent = 30000;
		maxspeed = 200;
		adjspeed = 80;
		break;
	case M2006:
		maxcurrent = 10000;
		adjspeed = 1000;
		maxspeed = 3000;
		break;
	default:;
	}
}

int16_t Motor::getdeltaa(int16_t diff)
{
	if (diff <= -4096)
		diff += 8192;
	else if (diff > 4096)
		diff -= 8192;
	return diff;
}

int16_t Motor::getword(const uint8_t high, const uint8_t low)
{
	const int16_t word = high;
	return (word << 8) + low;
}

int32_t Motor::setrange(const int32_t original, const int32_t range)
{
	return std::max(std::min(range, original), -range);
}


// All firing modes share one fixed-point single-shot controller.
void Motor::UpdateFeeder()
{
    FeederController::Input input;
    uint8_t frame[8]{};
    input.feedback_available = can2.MotorFeedback(ID - ID1, frame);
    input.angle = static_cast<uint16_t>(getword(frame[0], frame[1]));
    input.feedback_available = input.feedback_available && input.angle < 8192;
    input.rpm = getword(frame[2], frame[3]);
    input.now = HAL_GetTick();
    taskENTER_CRITICAL();
    input.session = static_cast<uint8_t>(ctrl.mode) + 1;
    input.enabled = (ctrl.mode == CONTROL::FIRE || ctrl.mode == CONTROL::AUTOAIM ||
        (ctrl.mode == CONTROL::REAL && xuc.Tx_TJ.mode_TJ == 1 && !ctrl.chassis.real_returning)) &&
        can2_motor[0].setspeed == -7000 && can2_motor[1].setspeed == 7000;
    input.request = single_requested;
    single_requested = false; // Requests while busy are dropped, not replayed.
    taskEXIT_CRITICAL();
    feeder.Update(input);
    setspeed = feeder.speed;
    shoot_mode_now = feeder.Busy() ? Motor::single : Motor::stop;
    if (feeder.started_count != feeder_last_start || feeder.state != FeederController::RUN ||
        abs(setspeed - input.rpm) > 150) pid[speed].m_error[INTEGRATE] = 0.0f;
    feeder_last_start = feeder.started_count;
    if (feeder.state == FeederController::RUN) {
        current = setrange(pid[speed].Position(setspeed - input.rpm, 4000.f), 4000);
    } else if (feeder.state == FeederController::SETTLE) {
        current = (input.rpm >= -30 && input.rpm <= 30) ? 0 :
            setrange(pid[speed].Position(-input.rpm, 2000.f), 2000);
    } else {
        current = 0;
    }
    setcurrent = current;
}
