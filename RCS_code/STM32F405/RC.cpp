#include "label.h"
#include "RC.h"
#include "control.h"
#include "HTmotor.h"
#include "xuc.h"

void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void RC::OnRC()
{
	RC_CheckState();
	RC_Control();

	if (Shift_mode())
	{
		ctrl.pantile.keep_angle1 = imu_pantile.GetAngleYaw();
	}
	
}

void RC::OnPC()
{
	
}

void RC::Update()
{
	OnRC();
	OnPC();
}

void RC::RC_CheckState() {

	switch (RC_STATE(rc.s[0], rc.s[1]))
	{
	case RC_STATE(UP, UP):
		ctrl.mode = CONTROL::FIRE;
		break;

	case RC_STATE(UP, MID):
		ctrl.mode = CONTROL::AUTOAIM;
		break;

	case RC_STATE(UP, DOWN):
		ctrl.mode = CONTROL::TEST;
		break;

	case RC_STATE(MID, UP):
		ctrl.mode = CONTROL::SEPARATE;
		break;

	case RC_STATE(MID, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(MID, DOWN):
		ctrl.mode = CONTROL::TEST;
		break;

	case RC_STATE(DOWN, UP):
		ctrl.mode = CONTROL::TEST;
		break;

	case RC_STATE(DOWN, MID):
		ctrl.mode = CONTROL::TEST;
		break;

	case RC_STATE(DOWN, DOWN):
		ctrl.mode = CONTROL::TEST;
		break;

	default:
		break;
	}

}

void RC::RC_Control() 
{

	if (ctrl.mode != CONTROL::RESET)
	{
		if (ctrl.mode != CONTROL::FIRE&& ctrl.mode != CONTROL::AUTOAIM)
		{
			can2_motor[2].shoot_mode_now = Motor::stop;//强制关闭拨弹盘
		}
		if ( ctrl.mode != CONTROL::AUTOAIM)
		{
			xuc.Tx_TJ.mode_TJ = 0;
		}
		switch (ctrl.mode)
		{
		case CONTROL::ROTATION:
		{

		}
		break;

		case CONTROL::FOLLOW:
		{

		}
		break;

		case CONTROL::SEPARATE:
		{
			if (rc.ch[0] >= 30 || rc.ch[0] <= -30)
			{
				ctrl.chassis.speedz = (-1) * rc.ch[0] * para.max_speed / 660.f;
			}
			else
			{
				ctrl.chassis.speedz = 0;
			}
			break;

		case CONTROL::AUTOAIM:
		{
			xuc.Tx_TJ.mode_TJ = 1;
		}
		break;

		case CONTROL::FIRE:
		{
			if ((rc.ch[2] >= 20 || rc.ch[2] <= -20) || (rc.ch[3] >= 20 || rc.ch[3] <= -20))
			{
				ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, rc.ch[3] * para.pitch_speed / -660.f);
			}
			fire_now = (rc.ch[1] >= 100 || rc.ch[1] <= -100);//单发

			/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
			if (rc.ch[0] >= 30 || rc.ch[0] <= -30)//连发
			{
				can2_motor[2].shoot_mode_now = Motor::running;
			}

			else if (fire_now && !fire_last)
			{
				can2_motor[2].need_curcircle = 4;
				can2_motor[2].shoot_mode_now = Motor::single;
			}
			else if (can2_motor[2].shoot_mode_now != Motor::single)
			{
				can2_motor[2].shoot_mode_now = Motor::stop;

			}
			fire_last = fire_now;
		}

		break;

		case CONTROL::TEST:
		{
			if (rc.ch[0] >= 30 || rc.ch[0] <= -30)
			{
				ctrl.chassis.speedx = (-1) * rc.ch[0] * para.max_speed / 660.f;
			}
			else
			{
				ctrl.chassis.speedx = 0;
			}
			if (rc.ch[1] >= 30 || rc.ch[1] <= -30)
			{
				ctrl.chassis.speedy = (-1) * rc.ch[1] * para.max_speed / 660.f;
			}
			else
			{
				ctrl.chassis.speedy = 0;
			}
			if ((rc.ch[2] >= 20 || rc.ch[2] <= -20) || (rc.ch[3] >= 20 || rc.ch[3] <= -20))
			{
				ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, rc.ch[3] * para.pitch_speed / -660.f);
			}
		}
		break;

		case CONTROL::SPINNING:
		{

		}
		break;

		default:
		{
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
		}
		break;
		}
		}
	}else
	xuc.Tx_TJ.mode_TJ = 0;
	
}

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 或者报错
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	}

	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C 
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E

}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}