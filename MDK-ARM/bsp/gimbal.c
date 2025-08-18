/********************************************************************************************************\
*                                     DJI FreeRTOS TASK PART 
*
*                                   (c) Copyright 2015; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
\********************************************************************************************************/
/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  langgo
  * @version V1.0
  * @date    2016-11-30
  * @brief   
	* @update		
		@update 2   After this version, gimbal's encoder feedback position range [-xxxx , +xxxx], symmetric by 0，
								after call @function s16 get_relative_pos(s16 raw_ecd, s16 center_offset) will
								get a position relative to the center is center_encoder/offset= GimbalCaliData.GimbalPit/YawOffset;
								all input refence range also = [-xxxx, xxxx], Symmetric by 0
		@update 3   when rc sw2 = RC_DN, stop everything
	*	@attention			
  * @verbatim you should comment with english avoid diff text encoder cause ?!$@#%^@$%&$*
	****************************************************************************
	*/

#include "mytype.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "cmsis_os.h"
#include "bt.h"
#include "bsp_can.h"
#include "mpu.h"
#include "error_task.h"
#include <math.h>
#include "chassis.h"
#include "gimbal.h"
#include "calibrate.h"
#include "kb.h"
#include "judge_sys.h"
#include "sys.h"

#define VAL_LIMIT(x,min,max)	\
do{									\
	if( (x) < (min) )	\
		(x) = (min);		\
	if( (x) > (max) )	\
		(x) = (max);		\
}while(0)						\

s16 shoot_CAN_Set_Current;
int iiii;
int shot_count=0;
float yaw_pos_rc_int = 0;		//rc_integral 
float yaw_pos_mouse_int;		//mouse_integral
float yaw_pos_ref = 0;			//rc_integral + mouse_integral
int kadanjishu=0;
float pit_pos_rc_int = 0;		//rc_integral 
float pit_pos_mouse_int=0;	//mouse_integral 
float pit_pos_ref =  0;			//rc_integral + mouse_integral 
static uint16_t TRIGGER_SPEED_REALLY=28;
int YawMouseMax=70;				//Yaw最大鼠标移动速度 防止鼠标动一下转很多圈
extern char TIM7_FLAG;
extern char USART6_FLAG;

int pit_center_offset = 6862;	//read from flash. == ecd value when gimbal pit in center position
int PitZenith	=7380; 								//pit想达到的最高点对应的值6461
int PitNadir	=6500;								//pit想达到的最低点对应的值7435
int PIT_MOVE_UP=0,PIT_MOVE_DOWN=0;	//上下移动的范围

float yaw_center_offset = 3440;	//read from motor esc when yaw axis in center.		//this will read from flash in the future. 
int YawZenith	=4393;								//pit想达到的最高点对应的值
int YawNadir	=2820;								//pit想达到的最低点对应的值
int YAW_MOVE_UP=3500,YAW_MOVE_DOWN=400;	//左右移动的范围//YAW_MOVE_UP=3500,YAW_MOVE_DOWN=-7500
 
/*gimbal move delta position, init value = 0, range (-MOVE_RANGE, MOVE_RANGE)*/
s16 pit_relative_pos;	//unit : encoder 
vs16 yaw_relative_pos;
s16 yaw_speed,yaw_angle1,yaw_angle2;
float watch_param[4];	//for jscope watch
float watch_pit[4];
int last_sw1;
int fric_wheel_run=0;	//run or not 

int poke_spd_ref,poke_pos_ref;	//poke bullet speed or 
//int big_poke_spd_ref,big_poke_pos_ref;	//poke bullet speed or 
int enter_stanby_mode_cnt;
u8  mouse_left_sta=0;
u8  mouse_right_sta=0;
float imu_tmp,imu_tmp_ref=50;
gimbal_yaw_t gYaw;
extern float euler[];
int reset_sequence=0;
int poke_stuck_cnt=0;									 //拨弹轮堵转时计数（计时）
int big_poke_stuck_cnt=0;							 //大
int poke_dir=-1;											 //拨弹轮转动方向

SHOT_MODE Shot_Mode;                   //射击模式，单点、三连发、连发，遥控器控制是使用
char SHOT_KEY=1;											 //射击允许位，用于实现遥控器控制单点
int Single_Data=36864;				     				 //小弹丸35
s16 ShootFTime=0;
u16 ShootFNum=80;//射频差不多15左右

u8 AgainStuck=0;
u8 AgainStuckData=60,StuckData=40;

ON_OFF MouseCtrl = off;

int MotoPokeSpeed,MotoPokeTotalAngle;

//不同等级下步兵连发的三个档位 储存射击周期  2021.5.6:这是哨兵，射速射频啥的不影响 所以这串代码没用
//u16 ShootPeriod[4][3]={200,200,40,			//0级步兵
//											 250,200,40,			//1级		最大射频设置为50
//											 190,150,40,			//2级
//											 130,100,40};			//3级

u8 FullSpeedShoot=0;
								 
u16 SHOT_WHEEL_SPEED_L = 1450;//tim4->ch3
u16 SHOT_WHEEL_SPEED_R = 1457;//tim4->ch3
u16 SHOT_WHEEL_SPEED_Single = 1330	;	//单发时射速
u16 SHOT_WHEEL_SPEED_Cont 	= 1500	;	//连发时射速
u8 shooter_single_heat=15;
											 
//u32 fric_delay=0;
direction PitDir={1,1,-1,-1,-1}; 
direction YawDir={1,1,1,-1,1};

u8 rc_B_sta;
extern s16 YawAntiOffset;

uint16_t Pock_Max=5500;
int huan_suan_p;
int huan_suan_y;
u16 yaw_angle;
u16 pitch_angle;
u16 distance;
//临时使用的变量// 
u8 auto_falg=0;
u8 shot_falg=0;
u16 shot_falg_count=0;
u8 pit_patrol_dir;//P轴巡逻方向
u8 yaw_patrol_dir;//Y轴巡逻方向
float pit_spin_speed=0.6;
float yaw_spin_speed=0.6;

u16 cv_yaw_target=320,cv_pit_target=240;

u8 shot_poke_falg=0;//拨弹标志位

void Vision_angle_huansuan()
 {
	 yaw_angle=(VisionRecvData_te.yaw_hangle<<8)|VisionRecvData_te.yaw_langle;
	 pitch_angle=(VisionRecvData_te.pitch_hangle<<8)|VisionRecvData_te.pitch_langle;
	 distance=(VisionRecvData_te.ldistance<<8)|VisionRecvData_te.hdistance;
	 
//	 huan_suan_y=
	 
//  huan_suan_p=(int32_t)(VisionRecvData_te.pitch_angle*100);
//  huan_suan_y=(int32_t)(VisionRecvData_te.yaw_angle*100);	
//	 
//  VisionRecvData_te.pitch_angle=(float)(huan_suan_p/100);
//  VisionRecvData_te.yaw_angle=(float)(huan_suan_y/100);
 }	
//void ShootFreqDis(void)			//射频处理
//{
//	if(buff_musk.power_rune_buff	&	0x02)
//	{
//			SHOT_WHEEL_SPEED_Cont=1300;
//			shooter_single_heat=25;
//	}
//	else
//	{
//		switch(game_robot_state.robot_level)
//		{
//			case 1:
//				SHOT_WHEEL_SPEED_Cont=1230;
//				shooter_single_heat=15;
//				break;
//			case 2:
//				SHOT_WHEEL_SPEED_Cont=1240;
//				shooter_single_heat=17;
//				break;
//			case 3:
//				SHOT_WHEEL_SPEED_Cont=1250;
//				shooter_single_heat=19;
//				break;
//		}
//	}
//	if(KeyStateMachine_hold(&mouse_left_sta, rc.mouse.l)	&&	KeyStateMachine_hold(&mouse_right_sta, rc.mouse.r) )			//全速射击 最高射频
//	{
//		ShootFNum=ShootPeriod[game_robot_state.robot_level][2];
//	}
//	//当前热量小于热量上限-100 中等射频
//	else if(power_heat_data.shooter_heat0<game_robot_state.shooter_heat0_cooling_limit-200)
//	{
//		ShootFNum=ShootPeriod[game_robot_state.robot_level][1];
//	}
//	//当前热量大于热量上限-100 低等射频
//	else
//	{
//		ShootFNum=ShootPeriod[game_robot_state.robot_level][0];
//	}
//}



void PockText()   //射击处理程序
{
	 rc.sw1 = date_reciev.s[0];
	 rc.sw2 = date_reciev.s[1];
		if(1 ) //rc.sw2 == RC_UP)||rc.sw2 == RC_DN{
		{ 

			fric_wheel_run = !fric_wheel_run;  //摩擦轮拨弹轮运动使能
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET); //open laser  开启激光
		}
		last_sw1 = rc.sw1;
		if (1) 
		{ 
			SendLed(4,1);			
			if(TIM4->CCR4<SHOT_WHEEL_SPEED_L)			TIM4->CCR4++;
			else																TIM4->CCR4 = SHOT_WHEEL_SPEED_L;
			
			if(TIM4->CCR3<SHOT_WHEEL_SPEED_R)			TIM4->CCR3++;
			else																TIM4->CCR3 = SHOT_WHEEL_SPEED_R;
		}
		else
		{
			SendLed(4,0);			//
			TIM4->CCR3 = TIM4->CCR4  = 1000;
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); 	//close laser
		}

		
//		//遥控器控制射击
//		if(Shot_Mode==ONE_MODE)   //SLL:single
//		{
			/*if(SHOT_KEY && fric_wheel_run && rc.sw1 == RC_DN&&( rc.sw2 == RC_UP ||rc.sw2 == RC_DN   ))
			{
				poke_pos_ref -= Single_Data*poke_dir;	
				power_heat_data.shooter_heat0+=25;
				SHOT_KEY=0;
			}	
			else if( fric_wheel_run && ((rc.sw2 == RC_UP && rc.sw1 == RC_DN) || (rc.sw2 == RC_DN && rc.sw1 == RC_DN) ))
			{
			  //SLL:空语句 保证单点功能的实现  不能删
			}
			else
			{
				SHOT_KEY=1;
			}*/
//		}
//		else if(Shot_Mode==THEER_MODE)   //SLL:three shot 三连发模式不能在整车上使用  危险  仅供调试发射结构
//		{
		/*	if(SHOT_KEY && fric_wheel_run &&( rc.sw1 == RC_DN ||  KeyStateMachine(&mouse_left_sta, rc.mouse.l)) )
			{				
				poke_pos_ref -= 3*Single_Data*poke_dir;	
				SHOT_KEY=0;
			}	
			else if(rc.sw1 == RC_DN || KeyStateMachine(&mouse_left_sta, rc.mouse.l) )
			{
			  //SLL:空语句 保证功能的实现  不能删
			}
			else
			{
				SHOT_KEY=1;
			}*/			
//	  }
	  if(fric_wheel_run&& ((rc.sw1 == RC_DN) ||shot_poke_falg) )   //SLL:continuous
		{
			ShootFTime++;		//射频计数
			if(ShootFTime>ShootFNum)
			{
				poke_pos_ref += Single_Data*poke_dir;	
				//power_heat_data.shooter_heat0+=shooter_single_heat;
				ShootFTime=0;
			}
	  }
	}

		//鼠标控制射击
//		if(KeyStateMachine_hold(&mouse_left_sta, rc.mouse.l) && fric_wheel_run)
//		{
//			SHOT_WHEEL_SPEED  = SHOT_WHEEL_SPEED_Cont;
//			ShootFTime++;		//射频计数
//			if(ShootFTime>ShootFNum)
//			{
//				poke_pos_ref -= Single_Data*poke_dir;	
//				power_heat_data.shooter_heat0+=shooter_single_heat;
//				ShootFTime=0;
//			}
//		}
//		else if(KeyStateMachine(&mouse_right_sta, rc.mouse.r) && fric_wheel_run) //rc.sw2 == RC_UP)||rc.sw2 == RC_DN{
//		{ 
//			SHOT_WHEEL_SPEED = SHOT_WHEEL_SPEED_Single;
//			power_heat_data.shooter_heat0+=25;
//			poke_pos_ref -= Single_Data*poke_dir;	
//		}
////////////***********热量限制************//
//////////		if(power_heat_data.shooter_heat0>game_robot_state.shooter_heat0_cooling_limit-25)
//////////		{
//////////			poke_pos_ref=moto_poke.total_angle/1000;
//////////		}
/**********防卡死**************/
//int p;
void trigger_motor_turn_back()
 {
   if(poke_pos_ref<moto_poke.total_angle-5000)
	 {
	    shot_count++;	 
	 }
	 else
	 {
	    shot_count=0 ;
	 }
	 if(shot_count>500)//200
	 {
	    //p=11;
	    kadanjishu=(moto_poke.total_angle-poke_spd_ref)/Single_Data;
		  poke_pos_ref=kadanjishu*Single_Data+poke_spd_ref;
		  shot_count=0;
	 }
 }


void PID_Init(void)
{

//	PID_struct_init(&pid_pit, POSITION_PID, 20000, 0,
//							40.0f,	0.0f,	10.0f);	
//	PID_struct_init(&pid_pit_omg, POSITION_PID, 20000, 1000,
//							60.0f,	0.1f,	20.0f	);
		PID_struct_init(&pid_pit, POSITION_PID, 20000, 0,
							20.5f,	0.0f,	5.0f);	//. 55，5，10  ，30000
  	PID_struct_init(&pid_pit_omg, POSITION_PID, 20000, 1000,
							41.0f,	0.0f,	10.0f	);
	pid_yaw.changing_i=0.0f;
	pid_yaw_omg.changing_i=0.05f;
//	PID_struct_init(&pid_yaw, POSITION_PID, 6000, 1000,
//							25.0f,	0.0f,	0.0f );
//	PID_struct_init(&pid_yaw_omg, POSITION_PID, 20000, 1000,
//							70.0f, 0.01f,	12.0f	);
	PID_struct_init(&pid_yaw, POSITION_PID, 6000, 1000,
							14.0f,	0.0f,	0.0f );
	PID_struct_init(&pid_yaw_omg, POSITION_PID, 20000, 1000,
							58.0f, 0.0f,	12.0f	);
	
	//移植这一套拨弹轮程序一定要改子弹对应的值（比如Single_Data，连发也要改）！！！不改烧电机！！
	PID_struct_init(&pid_poke, POSITION_PID, 10000,  0 ,	 0.3,	0,	0.9);   
    PID_struct_init(&pid_poke_omg, POSITION_PID, 10000 , 1000,  15,	 0.001, 7);		
	
	
	
	PID_struct_init(&pid_imu_tmp, POSITION_PID, 300, 150,
									10,	1,	0);	
									
	PID_struct_init(&pid_cv_y, POSITION_PID, 50, 30,
									0.01,		0.0f,	0);	
	PID_struct_init(&pid_cv_p, POSITION_PID,1200, 1200,
									0.01,		0.0f,	0);	

}
void ConsoleCtrl(void)
{
	if(rc.sw2 == RC_MI)//手动遥控
	{
		pit_pos_ref += -rc.ch4 *0.05f* GIMBAL_RC_MOVE_RATIO_PIT;  // P轴遥控器

		yaw_pos_ref += rc.ch3 *0.02f* GIMBAL_RC_MOVE_RATIO_YAW;  // Y轴遥控器
//		VAL_LIMIT(yaw_pos_ref, -500,500);   		//need change
//		if(shot_falg==1&&shot_falg_count<100)
//		{
//			yaw_center_offset += pid_calc(&pid_cv_y, cv_yaw_target, cv_yaw); 
//			pit_pos_ref += pid_calc(&pid_cv_p, cv_pit_target, cv_pit);
//		}		
	}
	
	if(rc.sw2 == RC_UP )//启动云台和自瞄
		auto_falg=1;
	else 
		auto_falg=0;
	if(auto_falg==1&&(VisionRecvData_te.shot_flag&&receive_vision)==0)//启动，但没识别到目标，自旋
	{
		if(pit_patrol_dir)//p轴自旋 
		{
			pit_pos_ref += pit_spin_speed;
			if(pit_pos_ref>=PIT_MOVE_UP)
				pit_patrol_dir = 0;
		}
		else 
		{
			pit_pos_ref -= pit_spin_speed;
			if(pit_pos_ref<=PIT_MOVE_DOWN)
				pit_patrol_dir = 1;
	  }
		
		if(yaw_patrol_dir)//y轴自旋//
		{
			yaw_pos_ref +=yaw_spin_speed;//
			if(yaw_pos_ref>=YAW_MOVE_UP)
				yaw_patrol_dir = 0;
		}
		else 
		{
			yaw_pos_ref -=yaw_spin_speed;
			if(yaw_pos_ref<=YAW_MOVE_DOWN)
				yaw_patrol_dir = 1;
		}		
	}
	if(auto_falg&&(VisionRecvData_te.shot_flag&&receive_vision)==1)//自瞄打单
	{
		  //0Vision_angle_huansuan();
//    pit_pos_ref =- (VisionRecvData_te.pitch_angle) * 22.75f;  // P轴遥控器

//		yaw_pos_ref =   (VisionRecvData_te.yaw_angle)  * 22.75f;  // Y轴遥控器
//		 pit_pos_ref = pit_relative_pos - (VisionRecvData_te.pitch_angle) * 22.75f;  // P轴遥控器

//		yaw_pos_ref =  yaw_relative_pos + (VisionRecvData_te.yaw_angle)  * 22.75f;  // Y轴遥控器
	
		shot_poke_falg=1;
		
	}
	else
	{
		shot_poke_falg=0;
  }
	
//   if(TIM7_FLAG==1)
//	{
//		 TIM7_FLAG=0;
//	   if(USART6_FLAG==1)
//      {
//		   USART6_FLAG=0;
//	    }
//	     else
//	    {
//  	   VisionRecvData_te.shot_flag=0;
//		   VisionRecvData_te.pitch_angle=0;
//		   VisionRecvData_te.yaw_angle=0;
//      }
//  }
	VAL_LIMIT(yaw_pos_ref, YAW_MOVE_DOWN,YAW_MOVE_UP);   		//need change
	VAL_LIMIT(pit_pos_ref, PIT_MOVE_DOWN,PIT_MOVE_UP);   		//need change
}

/**
	*@aim 		get position relative to center,center = GimbalCaliData.GimbalPit/Yaw Offset;
	*@range  	gimbal move range = [center-MOVE_RANGE, center+MOVE_RANGE]
	*@note	 	process when angle cross 0 !
	*/
s16 get_relative_pos(s16 raw_ecd, s16 center_offset)
{
	s16 tmp=0;
	if(center_offset >= 4096)
	{
		if(raw_ecd > center_offset - 4096)
			tmp = raw_ecd - center_offset; 
		else
			tmp = raw_ecd + 8192 - center_offset;
		
	}
	else
	{
		if(raw_ecd > center_offset + 4096)
			tmp = raw_ecd - 8192 - center_offset ; 
		else
			tmp = raw_ecd - center_offset;
	}
	return tmp;
}



void can_send_gimbal_iq(s16 yaw_iq, s16 pit_iq, s16 poke_iq,s16 poke2_iq)
{
	
	GIMBAL_CAN.pTxMsg->StdId = 0x1ff;
	GIMBAL_CAN.pTxMsg->IDE = CAN_ID_STD;
	GIMBAL_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	GIMBAL_CAN.pTxMsg->DLC = 8;
	GIMBAL_CAN.pTxMsg->Data[0] = yaw_iq>>8;
	GIMBAL_CAN.pTxMsg->Data[1] = yaw_iq;
	GIMBAL_CAN.pTxMsg->Data[2] = pit_iq>>8;
	GIMBAL_CAN.pTxMsg->Data[3] = pit_iq;
	GIMBAL_CAN.pTxMsg->Data[4] = poke_iq>>8;
	GIMBAL_CAN.pTxMsg->Data[5] = poke_iq;
	GIMBAL_CAN.pTxMsg->Data[6] = poke2_iq>>8;
	GIMBAL_CAN.pTxMsg->Data[7] = poke2_iq;
	HAL_CAN_Transmit(&GIMBAL_CAN, 1000);
	
}
static void ShootFreqDis(void)
{    if(/*(JudgeeverydataS.power_heat_data_t.shooter_heat0>=JudgeeverydataS.game_robot_status_t.shooter_heat0_cooling_limit)
			||*/(((JudgeeverydataE.game_robot_state.shooter_id1_17mm_cooling_limit-JudgeeverydataE.power_heat_data.shooter_id1_17mm_cooling_heat)/TRIGGER_SPEED_REALLY)<2)&&(JudgeeverydataE.game_robot_state.shooter_id1_17mm_cooling_limit!=0))
		{
			Single_Data=0;
		}
		else  /*if(JudgeeverydataS.power_heat_data_t.shooter_heat0<JudgeeverydataS.game_robot_status_t.shooter_heat0_cooling_limit)*/
		{
		   	Single_Data=36864;

					if((JudgeeverydataE.power_heat_data.shooter_id1_17mm_cooling_heat)<(JudgeeverydataE.game_robot_state.shooter_id1_17mm_cooling_limit/2))
					{
							ShootFNum=80;
			
					}
					else
					{
							ShootFNum=120;
					}

	  }		
}	
/**
	*@bref - pitch + yaw gimbal double loop control
	*@TODO : ADD speed pid loop, done!
	*/
void gimbal_task(const void *argu)
{
	poke_pos_ref=moto_poke.total_angle;
	poke_spd_ref=moto_poke.total_angle;
	PID_Init();		
	/* mpu6500 & istxxx magnet meter init ,for angular rate measure*/
	mpu_device_init();

	//gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ENCODER;//hide enco2der mode!!!
	PIT_MOVE_UP		=	PitZenith	-	pit_center_offset;
	PIT_MOVE_DOWN	=	PitNadir	-	pit_center_offset;
	YAW_MOVE_UP		=	YawZenith	-	yaw_center_offset;
	YAW_MOVE_DOWN	=	YawNadir	-	yaw_center_offset;

	gYaw.zgyro_offset= get_relative_pos(moto_yaw.angle, yaw_center_offset)/22.75f;	//get offset修改陀螺仪中间值、moto_yaw.angle观测
	//game_robot_state.shooter_heat0_cooling_limit=240;
	osDelay(2000);
	while(1)
	{
		chassis.angle_from_gyro = imu.wy;
		manifold_uart_init(); 

		//keep imu in a constant temperature
		imu_tmp = 21 + mpu_data.temp/333.87f;
		pid_calc(&pid_imu_tmp, imu_tmp, imu_tmp_ref);		//keep in 40 degree
		TIM3->CCR2 = pid_imu_tmp.pos_out;
		
		/* get angular speed, for gimbal speed loop */
		mpu_get_data();
		CalibrateHook(CALI_GIMBAL);
	
		pit_relative_pos = get_relative_pos(moto_pit.angle,pit_center_offset);//2700  5600,3号车是750 大于3460	
		
		yaw_relative_pos = get_relative_pos(moto_yaw.angle, yaw_center_offset);//修改800，此值为云台在中间位置时yaw电机编码器反馈值		

//		VAL_LIMIT(yaw_relative_pos, YAW_MOVE_DOWN,YAW_MOVE_UP);   		//need change
//	  VAL_LIMIT(pit_relative_pos, PIT_MOVE_DOWN,PIT_MOVE_UP);   		//need change
		
		ConsoleCtrl();//云台控制
		ShootFreqDis();
		PockText();  //shoot task
    trigger_motor_turn_back();
		pid_calc_changing_i(&pid_yaw, yaw_relative_pos, yaw_pos_ref); 
		pid_calc_changing_i(&pid_yaw_omg, YawDir.omg_pos*(-mpu_data.gz1)/10.0f,YawDir.omg_ref*pid_yaw.pos_out /10.0f); //YawDir.omg_ref*pid_yaw.pos_out/10.0f
		//
		pid_calc(&pid_pit, pit_relative_pos, pit_pos_ref); 
		pid_calc(&pid_pit_omg, PitDir.omg_pos*mpu_data.gx1/10.0f, PitDir.omg_ref*pid_pit.pos_out/10.0f); //*mpu_data.gx1
    //
		MotoPokeTotalAngle=moto_poke.total_angle;
		MotoPokeSpeed = moto_poke.speed_rpm;
		pid_calc(&pid_poke, MotoPokeTotalAngle, poke_pos_ref);
		pid_calc(&pid_poke_omg, MotoPokeSpeed, pid_poke.pos_out); //pid_poke.pos_out
		if(rc.sw2==RC_DN)
			{
        shoot_CAN_Set_Current =0;			
        poke_pos_ref= moto_poke.total_angle;
				poke_spd_ref=moto_poke.total_angle%36846;
			}
			else
			{
			shoot_CAN_Set_Current =(int16_t)(pid_poke_omg.pos_out);	
			}
		if (0)
		{		
			can_send_gimbal_iq(YawDir.out*pid_yaw_omg.pos_out, PitDir.out*pid_pit_omg.pos_out, shoot_CAN_Set_Current,0);  //pid_yaw_omg.pos_out, -pid_pit_omg.pos_out, pid_poke_omg.pos_out
//			can_send_gimbal_iq(0,  PitDir.out*pid_pit_omg.pos_out, 0, 0);
		}
		else	
		{ 		
			pid_poke.iout = 0;
			can_send_gimbal_iq(0, 0, 0, 0);	//relax state
		}
	}
}



