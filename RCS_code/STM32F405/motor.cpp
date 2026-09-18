#include "motor.h"
#include "gpio.h"
#include "HTmotor.h"
#include "imu.h"
#include "control.h"
#include "RC.h"
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
        // 常调参数
        const int32_t DIRECTION = 1;       // 反了改为 -1
        const double STEP = 8192.0 * 36.0 / 7.0;
        const int32_t SPEED_LIMIT = 600;   // 转子rpm，先低速测试
        const int32_t CURRENT_LIMIT = 3000;
        const double POSITION_TOLERANCE = 200.0;
        const int32_t STOP_SPEED = 30;

        // 当前工程只有一台ACE电机
        static bool target_valid = false;
        static double target = 0.0;
        static uint8_t arrival_count = 0;
        static uint32_t step_started_ms = 0;
       
        static bool fault = false;          // 原来的保留
        static bool retry_armed = false;    // 新增：超时后是否已经回中

        uint32_t time_ms = HAL_GetTick();

        // sum_angle在函数末尾才更新：
        // 加本次增量得到当前累计角度，不重复写sum_angle
        double actual =
            static_cast<double>(sum_angle) +
            getdeltaa(static_cast<int16_t>(
                angle[now] - angle[pre]));

        auto stop_output = [&]()
            {
                setspeed = 0;
                current = 0;
                setcurrent = 0;

                for (int i = 0; i < 3; ++i)
                {
                    pid[speed].m_error[i] = 0.f;
                    pid[position].m_error[i] = 0.f;
                }
            };

        auto cancel_requests = [&]()
            {
                taskENTER_CRITICAL();
                spinning = false;
                need_curcircle = 0;
                taskEXIT_CRITICAL();
            };

        bool fire_mode = ctrl.mode == CONTROL::FIRE;
        bool allowed =
            fire_mode &&
            (ctrl.shooter.fraction || pd || fault) &&
            temperature <= 70;

        if (!allowed)
        {
            if (!fire_mode || temperature > 70)
            {
                fault = false;
                retry_armed = false;
            }

            pd = false;

            // 只有退出发射模式或过温，才重新建立拨弹基准
            if (!fire_mode || temperature > 70)
            {
                target_valid = false;
            }

            arrival_count = 0;

            cancel_requests();
            stop_output();
        }
        else if (std::abs(rc.rc.ch[0]) <= 300)
        {
            // 松杆：清除新请求，但保留pd和target
            // 下一次推杆先完成原来未完成的这一格
            cancel_requests();
            arrival_count = 0;

            // 暂停时间不计入这一格的超时
            step_started_ms = time_ms;

            // 故障后回中，允许下一次推杆重试
            if (fault && std::abs(rc.rc.ch[0]) <= 100)
            {
                retry_armed = true;
            }

            // 清除原来的驱动及PID积累
            stop_output();

            // 仍在转动时，给反向速度误差进行制动
            if (std::abs(curspeed) > STOP_SPEED)
            {
                float braking = pid[speed].Position(
                    -curspeed, 10000.f);

                current = setrange(
                    static_cast<int32_t>(braking),
                    std::min<int32_t>(
                        CURRENT_LIMIT, maxcurrent));

                setcurrent = current;
            }
        }
        else if (fault)
        {
            pd = false;
            stop_output();

            bool retry = false;

            taskENTER_CRITICAL();

            if (std::abs(rc.rc.ch[0]) <= 100)
            {
                // 超时后必须先回中
                retry_armed = true;
                spinning = false;
                need_curcircle = 0;
            }
            else if (retry_armed && ctrl.shooter.fraction)
            {
                // 回中后再次单发或连发，允许重试
                retry = need_curcircle > 0 || spinning;

                if (need_curcircle > 0)
                {
                    need_curcircle--;
                }
            }
            else
            {
                // 一直推住时，不自动反复重试
                spinning = false;
                need_curcircle = 0;
            }

            taskEXIT_CRITICAL();

            if (retry)
            {
                fault = false;
                retry_armed = false;
                pd = true;
                arrival_count = 0;
                step_started_ms = time_ms;

                // 保留原来的target，继续完成这一格
                // 这里不能再执行 target += DIRECTION * STEP
            }
        }
        else
        {
            if (!target_valid)
            {
                target = actual;
                target_valid = true;
            }

            // 只有上一格完成后，才开始新的一格
            if (!pd)
            {
                bool start_step = false;

                taskENTER_CRITICAL();

                if (need_curcircle > 0)
                {
                    need_curcircle--;
                    start_step = true;
                }
                else if (spinning)
                {
                    start_step = true;
                }

                if (start_step)
                    pd = true;

                taskEXIT_CRITICAL();

                if (start_step)
                {
                    // 从上一次目标累加，保留小数精度
                    target += DIRECTION * STEP;
                    arrival_count = 0;
                    step_started_ms = time_ms;

                    for (int i = 0; i < 3; ++i)
                        pid[position].m_error[i] = 0.f;
                }
            }

            if (pd &&
                static_cast<uint32_t>(
                    time_ms - step_started_ms) >= 5000)
            {
                // 一格5秒未完成：停止，不无限重试
                fault = true;
                retry_armed = false;
                pd = false;

                cancel_requests();
                stop_output();
            }
            else
            {
                double error = target - actual;

                if (std::fabs(error) <= POSITION_TOLERANCE)
                {
                    setspeed = 0;
                }
                else
                {
                    float output = pid[position].Position(
                        static_cast<float>(error), 10000.f);

                    // 必须先把位置环输出赋给目标速度
                    setspeed = setrange(
                        static_cast<int32_t>(output),
                        SPEED_LIMIT);

                    // 再限制最低接近速度
                    const int32_t MIN_SPEED = 60;

                    if (std::abs(setspeed) < MIN_SPEED)
                    {
                        setspeed = error > 0.0 ? MIN_SPEED : -MIN_SPEED;
                    }
                }

                // 共用现有速度PID函数
                float output = pid[speed].Position(
                    setspeed - curspeed, 10000.f);

                current = setrange(
                    static_cast<int32_t>(output),
                    std::min<int32_t>(
                        CURRENT_LIMIT, maxcurrent));
                setcurrent = current;

                bool arrived =
                    std::fabs(error) <= POSITION_TOLERANCE &&
                    std::abs(curspeed) <= STOP_SPEED;

                if (pd && arrived)
                {
                    if (arrival_count < 10)
                        arrival_count++;

                    if (arrival_count >= 10)
                    {
                        pd = false;
                        arrival_count = 0;
                    }
                }
                else
                {
                    arrival_count = 0;
                }
            }
        }
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
	distance=(6.2831853f/ 8192.0f)*sum_angle * (WHEEL_RADIUS_MM / GEAR_RATIO)-initial_x;  // ��λ��mm

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

