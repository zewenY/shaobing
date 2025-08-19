#include "kb.h"
#include "mytype.h"
#include "cmsis_os.h"
#include "bt.h"
#include "chassis.h"
#include "judge_sys.h"
#include "bsp_can.h"
u8  mainflod_use_date[2];
int beida=0,beidaflag=0,beida1=0,beidaflag1=0;
int huanxiang,huanxiangflag;
u8 huanchong=180;//缓冲能量
u8 armor_damage=0;//装甲伤害
u16 armor_damage_time=0;//装甲伤害次数
u8 fangxiang;//移动方向的标识，左为1，右为2，停为0
u8 swing_r,swing_l;//被打后移到边上后的标志 R右 L左;
extern u16 MAX_WHEEL_SPEED				;  //
extern u16 MAX_CHASSIS_VX_SPEED	;  // 平移
extern u16 MAX_CHASSIS_VY_SPEED	;  //	前后
//键盘操作时 各方向最小的移动速度
//不再是从0开始往上加
s16 ForwardSpeedMin		=	500	;		
s16 BackwardSpeedMin	=	-500;
s16 RightSpeedMin			=	500	;
s16 LiftSpeedMin			=	-500;
u8 a4,a5;
/************************* 全局变量 *******************************/
km_control_t  km;
u16 key_code=0;
vu8 kb_move_mode = 0;
u16 max_kb_move_speed = 1000;//200
AntiMode_t AntiMode;
u8 rc_Q_sta;
u8 rc_R_sta;
int key_use_flag = 0 ;

/************************* 全局变量 *******************************/
/**************************chassis_patrol_variable********************************/
int chassis_huanchong_power ;
int light_sensor_left,light_sensor_right;
int turning ;
/**************************chassis_patrol_variable********************************/
u16 MagazinePwm		=		1500;
u16 MagazineOpen	=		600;
u16 MagazineClose	=		1450;

extern u8 HurtAnti;

u8 cap_flag;
u16 max_power=6000;
enum
{
    KEY_IDLE = 0,					//没有按键按下的空闲状态
    KEY_DOWN_WAIT_CONFIRM,		//表示可能处于抖动状态，等待第二次扫描确认
    KEY_DOWN_WAIT_RELEASE,		//按键确认按下，等待释放进入空闲
}KeyStateTypeDef;
void swing_right()//右
{
	if(beidaflag/100%2==1)//奇数
		huanxiang=2;       
	if(beidaflag/100%2==0)//偶数
	{
		if(a5==0)	huanxiang=2;			//防止撞击
		else 			huanxiang=1;
	}
}
void swing_left()//左
{
	if(beidaflag1/100%2==1)
		huanxiang=1;
	if(beidaflag1/100%2==0)
	{
		if(a4==0)	huanxiang=1;			//防止撞击
		else 			huanxiang=2;
	}
}
//void chassis_patrol()					//底盘巡逻
//{	
//	light_sensor_left = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);//从屁股后面看头右限位
//	light_sensor_right = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);//从屁股后面看头左限位
//	chassis_huanchong_power = JudgeeverydataE.power_heat_data.chassis_power_buffer ;
//	if(light_sensor_left == 0)
//	{
//		turning = 0 ;	
//	}	
//	if(light_sensor_right == 0)
//	{
//		turning = 1 ;	
//	}
//	
//}

//used for avoid mouse  press shake 
u8 KeyStateMachine(u8* psta, u8 condition)    //仅按下瞬间有信号 给右键用 单发
{
    u8 ret=0;	//temp var must init! or it is a random 
    switch(*psta)
		{
        case KEY_IDLE:
            if(condition)
                *psta = KEY_DOWN_WAIT_CONFIRM;
            break;
        case KEY_DOWN_WAIT_CONFIRM:
            if(condition)
						{
                *psta = KEY_DOWN_WAIT_RELEASE;
                ret = 1;	//这句放在这里的话 就是按下立即触发。如果放下面就是释放才触发
            }
            else
                *psta = KEY_IDLE;
            break;
        case KEY_DOWN_WAIT_RELEASE:
            if(!condition)
                *psta = KEY_IDLE;
            break;
    }
    return ret;
}

u8 KeyStateMachine_hold(u8* psta, u8 condition)  //按下保持 给左键用的  点下连发
{
    u8 ret=0;	//temp var must init! or it is a random 
    switch(*psta)
		{
        case KEY_IDLE:
            if(condition)
                *psta = KEY_DOWN_WAIT_CONFIRM;
            break;
        case KEY_DOWN_WAIT_CONFIRM:
            if(condition)
						{
                *psta = KEY_DOWN_WAIT_RELEASE;
                ret = 1;	//这句放在这里的话 就是按下立即触发。如果放下面就是释放才触发
            }
            else
                *psta = KEY_IDLE;
            break;
        case KEY_DOWN_WAIT_RELEASE:
            if(!condition)
                *psta = KEY_IDLE;
						else
							ret = 1;	//按下保持
            break;
    }
    return ret;
}

//void set_manifold_vision_mode(u8 cmd)
//{
//		HAL_UART_Transmit(&huart6, mainflod_use_date, 2, 10);
//}
void can_send_cap_iq(s16 max_power)//超级电容
{
	
	hcan1.pTxMsg->StdId = 0x210;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 8;
	hcan1.pTxMsg->Data[0] = max_power>>8;
	hcan1.pTxMsg->Data[1] = max_power;

	HAL_CAN_Transmit(&hcan1, 1000);
	
}

/**
*@public:  in the future, delete kb.c this file/task, combine to chassis 
**/
void key_mouse_task(void const * argument)
{
  /* USER CODE BEGIN KeyDealTask */
  osDelay(1000);//等待稳定，不允许操作(主要是要等待陀螺仪二次复位稳定)
  /* Infinite loop */
  for(;;)
  {
    key_code = rc.kb.key_code;
    //if(rc.kb.bit.R)
     // chassis.is_snipe_mode = 1;
    //else
      //chassis.is_snipe_mode = 0;
    
    /*根据shitf或者ctrl键的情况来选择底盘前后左右加速减速模式*/
//    if(key_code & SHIFT)
//      kb_move_mode = ShiftQuicklyMode;
//    else if(key_code & CTRL)
//      kb_move_mode = CtrlSlowlyMode;
//    else
//      kb_move_mode = NormalMode;
		if(rc.kb.bit.B)  
		{
	//	 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		}
//		if(rc.kb.bit.W)  
//		{
//			km.vy += 6;
//			speed_max_use.max_wheel_speed=3600;

//			if (rc.kb.bit.SHIFT)
//			{
//				if(km.vy==0)
//				{
//					osDelay(40); 
//				}
//			 speed_max_use.max_wheel_speed=4500; 
//			 km.vy+=4; 
//			}
//	
//		}
//		else if(rc.kb.bit.S)
//		{
//			km.vy += -6;
//			speed_max_use.max_wheel_speed=3600;
//			if(rc.kb.bit.SHIFT)
//			{
//				if(km.vy==0)
//				{
//					osDelay(40);

//				}
//				speed_max_use.max_wheel_speed=4500; 
//				km.vy -=4; 
//			}
//		}
//	
//		else
//		{
//			km.vy = 0;
//			speed_max_use.max_wheel_speed=3600;
//		}
//		
		if(rc.kb.bit.A)
			km.vx += -5;
		else if(rc.kb.bit.D)
			km.vx += 5;
		else
			km.vx = 0;


		if(km.vx > max_kb_move_speed)
      km.vx = max_kb_move_speed;
    if(km.vx < -max_kb_move_speed)
      km.vx = -max_kb_move_speed;
    
    if(km.vy > max_kb_move_speed)
      km.vy = max_kb_move_speed;
    if(km.vy < -max_kb_move_speed)
      km.vy = -max_kb_move_speed;
		


		if( 0 == HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) )
		{
			 TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=7050;
		}
		else 
		{				
			TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=5200;
		}
		//chassis_patrol();
			
    osDelay(10);
  }
  /* USER CODE END KeyDealTask */
}
//void key_mouse_task(void const * argument)
//{
//  /* USER CODE BEGIN KeyDealTask */
//  osDelay(1000);//等待稳定，不允许操作(主要是要等待陀螺仪二次复位稳定)
//  /* Infinite loop */
//  for(;;)
//  {
//    key_code = rc.kb.key_code;

//		if(rc.kb.bit.W)  
//		{
//	  	if(km.vy < 0)	 km.vy=0;
//			
//			if(km.vy<ForwardSpeedMin)		km.vy=ForwardSpeedMin;
//			else 												km.vy += 6;
//		}
//		else if(rc.kb.bit.S)
//		{
//	  	if( km.vy > 0)	 km.vy=0;
//			
//			if(km.vy>BackwardSpeedMin)	km.vy=BackwardSpeedMin;
//			else 												km.vy -= 6;
//		}
//		else
//		{
//			km.vy = 0;
//		}
//		
//	  if(rc.kb.bit.A)
//	  {
//	  	if( km.vx > 0)	 km.vx=0;
//			
//			if(km.vx>LiftSpeedMin)		km.vx=LiftSpeedMin;
//			else 											km.vx -=5;

//  	}
//		else if(rc.kb.bit.D)
//		{
//			if( km.vx < 0)	 km.vx=0;
//			if(km.vx<RightSpeedMin)		km.vx=RightSpeedMin;
//			else 											km.vx +=5;
//    }
//		else
//      km.vx = 0;
//	
//		if(km.vx > max_kb_move_speed)
//      km.vx = max_kb_move_speed;
//    if(km.vx < -max_kb_move_speed)
//      km.vx = -max_kb_move_speed;
//    
//    if(km.vy > max_kb_move_speed)
//      km.vy = max_kb_move_speed;
//    if(km.vy < -max_kb_move_speed)
//      km.vy = -max_kb_move_speed;
//		
//    /*根据shitf或者ctrl键的情况来选择底盘前后左右加速减速模式*/
//    if( SpeedQuickly)
//      kb_move_mode = ShiftQuicklyMode;
//    else if(SpeedSlowly)
//      kb_move_mode = CtrlSlowlyMode;
//    else
//      kb_move_mode = NormalMode;

//		switch(kb_move_mode)
//		{
//			case ShiftQuicklyMode:
//			{
//				MAX_WHEEL_SPEED					= 7000;  //300
//				MAX_CHASSIS_VX_SPEED		=	4000;  //150 平移
//				MAX_CHASSIS_VY_SPEED		=	8000;  //150	前后
//				break;
//			}
//			case CtrlSlowlyMode:
//			{
//				MAX_WHEEL_SPEED					= 2500;  //300
//				MAX_CHASSIS_VX_SPEED		=	1500;  //150 平移
//				MAX_CHASSIS_VY_SPEED		=	1500;  //150	前后
//				break;
//			}
//			case NormalMode:
//			{
//				MAX_WHEEL_SPEED					= 5000;  //300
//				MAX_CHASSIS_VX_SPEED		=	3500;  //150 平移
//				MAX_CHASSIS_VY_SPEED		=	4000;  //150	前后
//				break;
//			}
//		}
//		if(KeyStateMachine(&rc_Q_sta, rc.kb.bit.Q)||HurtAnti==1)
//		{
//			switch(AntiMode)
//			{
//				case AntiForward1:		
//					AntiMode=AntiRight;
//					SendLed(1,1);		//
//					break;
//				case AntiRight:
//					SendLed(1,0);		
//					AntiMode=AntiForward2;
//					break;
//				case AntiForward2:
//					SendLed(2,1);		//
//					AntiMode=AntiLeft;
//					break;
//				case AntiLeft:
//					SendLed(2,0);		//
//					AntiMode=AntiForward1;
//					break;
//			}
//			HurtAnti=0;
//		}

//		if(KeyStateMachine(&rc_R_sta, rc.kb.bit.R))
//		{
//			if(MagazinePwm==MagazineOpen)
//			{
//				MagazinePwm=MagazineClose;
//			}
//			else
//			{
//				MagazinePwm=MagazineOpen;
//			}
//		}
//		TIM2->CCR1=MagazinePwm;
//    osDelay(10);
//		if(cap_flag)
//		{
//			can_send_cap_iq(max_power);
//			cap_flag=0;
//			osDelay(100);
//		}
//		a4=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);//左限位
//		a5=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);//右限位
//  }
//  chassis_patrol();
//	osDelay(10);
//  /* USER CODE END KeyDealTask */
//}
