/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h" 
#include "bsp_can.h"
#include "can.h"
#include "pid.h"
#include "chassis.h"
#include "bt.h"
#include "error_task.h"
#include "gimbal.h"
#include "kb.h"
#include <math.h>
#include "judge_sys.h"
#include "sys.h"
#include "forbidden.h"

#define MyAbs(x) 	( (x>0) ? (x) : (-x) )
int Brake_Max_Op = 5000 ;
u16 MAX_WHEEL_SPEED					= 4500;  //300
u16 MAX_CHASSIS_VX_SPEED		=	3000;  //150 平移
u16 MAX_CHASSIS_VY_SPEED		=	4500;  //150	前后
u16 MAX_CHASSIS_OMEGA_SPEED	=	3600;  //300
int ceshi;
#define CHASSIS_OLD
#define MAX_FOLLOW_SPEED           6000
//#define DEBUG_REVERSE		//2 diff chassis
/************************Brake first virsion****************************/
int brake_calibration_angle = 49152   ;
int brake_total_angle                 ;
int brake_target_angle                ;
int brake_NOW_spd                     ;
int brake_1_total,brake_2_total       ;
int angle_difference                  ;
int flag_left=1,flag_right=0 ,switch_1,switch_2;
int system_fre = 50              ;
int brake_Max_op = 2000          ;
int brake_Mid_angle              ;
int Brake_single_angle = 13620   ;
int speed_array[50]              ;
int market_brake_1 =  0, market_brake_2 = 0;
int market_brake_3 = 1 , market_brake_4 = 1;
/************************Brake first virsion****************************/
typedef enum{
	MOVE_NONE=0,	//no move, just move accoding by rc.
	MOVE_Y_LINE,//move a straight line, then stop,
	MOVE_SQUARE,
	MOVE_CIRCLE,
}MOVE_STA_e;
MOVE_STA_e	chassis_move_sta = MOVE_NONE;
extern int last_sw1;
chassis_velocity chassis_v;
chassis_t chassis;	//main chassis object
extern SHOT_MODE Shot_Mode;
Judgeeverydata JudgeeverydataE;
RC_UnionDef tu_rc;
s16 buff_3510iq[4];
volatile double V_previous,V_Current;
volatile double dV;
volatile double iV;
volatile double V1;	
volatile double round_cnt;
volatile double V2_previous,V2_Current;
volatile double dV2;
volatile double iV2;
volatile double V2;	
volatile double round_cnt2;
volatile double V3_previous,V3_Current;
volatile double dV3;
volatile double iV3;
volatile double V3;	
volatile double round_cnt3;
volatile double V4_previous,V4_Current;
volatile double dV4;
volatile double iV4;
volatile double V4;	
volatile double round_cnt4;
float power_date;
int qqq,k1,j,oj,k2,k3,k4,oj2,oj3,oj4,cm1,cm2,cm3,cm4;
float pidd(float px1,float px2)
{

   const float v_p =2;//5
    const float v_d =4;//7
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    
    error_v[0] = error_v[1];
    error_v[1] = px1-px2;
   

    output = error_v[1] * v_p
   
         		+ (error_v[1] - error_v[0]) * v_d;
     
    if(output > 6000)
    {
        output = 6000;
    }
    
    if(output < -6000)
    {
        output = -6000;
    }
		 qqq=-output;
		return -output;
}
float pidd22(float px3,float px4)
{

   const float v2_p = 2;
    const float v2_d = 4;
    
    static float error2_v[2] = {0.0,0.0};
    static float output2 = 0;
		
 
    error2_v[0] = error2_v[1];//v0 存上一次的误差，v1 存本次的误差
    error2_v[1] = px3-px4;
    
    output2 = error2_v[1] * v2_p
             + (error2_v[1] - error2_v[0]) * v2_d;
     
    if(output2 > 6000)
    {
        output2 = 6000;
    }
    
    if(output2 < -6000)
    {
        output2 = -6000;
    }
		 
		return -output2;
}

float pidd33(float px5,float px6)
{

    const float v3_p =2;
    const float v3_d =4;
    
    static float error3_v[2] = {0.0,0.0};
    static float output3 = 0;
		
 
    error3_v[0] = error3_v[1];//v0 存上一次的误差，v1 存本次的误差
    error3_v[1] = px5-px6;
    
    output3 = error3_v[1] * v3_p
             + (error3_v[1] - error3_v[0]) * v3_d;
     
    if(output3 > 6000)
    {
        output3 = 6000;
    }
    
    if(output3 < -6000)
    {
        output3 = -6000;
    }
		 
		return -output3;
}
float pidd44(float px7,float px8)
{

   const float v4_p = 2;//45,0.2
    const float v4_d = 4;
    
    static float error4_v[2] = {0.0,0.0};
    static float output4 = 0;
		
 
    error4_v[0] = error4_v[1];//v0 存上一次的误差，v1 存本次的误差
    error4_v[1] = px7-px8;
    
    output4 = error4_v[1] * v4_p
             + (error4_v[1] - error4_v[0]) * v4_d;
     
    if(output4 > 6000)
    {
        output4 = 6000;
    }
    
    if(output4 < -6000)
    {
        output4 = -6000;
    }
		 
		return -output4;
}
float pidd55(float px9,float px10)
{

   const float v5_p = 0.8;//45,0.2
    const float v5_d = 0;
    
    static float error5_v[2] = {0.0,0.0};
    static float output5 = 0;
		
 
    error5_v[0] = error5_v[1];//v0 存上一次的误差，v1 存本次的误差
    error5_v[1] = px9-px10;
    
    output5 = error5_v[1] * v5_p
             + (error5_v[1] - error5_v[0]) * v5_d;
     
    if(output5 > 15)
    {
        output5 = 15;
    }
    
    if(output5 < -15)
    {
        output5 = -15;
    }
		 
		return -output5;
}
//chassis move measure
int ysigma,xsigma,asigma,last_ysigma,last_xsigma;
int ydelta,xdelta,adelta;
int xpos,ypos,apos;
int xx,yy,xtotal,ytotal;
float fai,theta;
int ma_total[4];	//forward = +, back = -\
extern float ZGyroModuleAngle;

extern AntiMode_t AntiMode; //躲避模式
s16 YawAntiOffset=0;

u8 rc_E_sta;

//void move_measure_hook()这段代码自己填
//{
//	
//	
//}


//actually this belong to gimbal will better
void reset_zgyro()
{
	while(ZGYRO_CAN.State == HAL_CAN_STATE_BUSY_TX);
	ZGYRO_CAN.pTxMsg->StdId = 0x404;
	ZGYRO_CAN.pTxMsg->IDE = CAN_ID_STD;
	ZGYRO_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	ZGYRO_CAN.pTxMsg->DLC = 0x08;
	ZGYRO_CAN.pTxMsg->Data[0] = 0;
	ZGYRO_CAN.pTxMsg->Data[1] = 1;
	ZGYRO_CAN.pTxMsg->Data[2] = 2;
	ZGYRO_CAN.pTxMsg->Data[3] = 3;
	ZGYRO_CAN.pTxMsg->Data[4] = 4;
	ZGYRO_CAN.pTxMsg->Data[5] = 5;
	ZGYRO_CAN.pTxMsg->Data[6] = 6;
	ZGYRO_CAN.pTxMsg->Data[7] = 7;
	HAL_CAN_Transmit(&ZGYRO_CAN, 1000);
}


/**
	* @function 麦轮解算函数， 输入x和y方向速度， 旋转角度， 输出到4个轮子
	*	@bref omega: + = cw, - = ccw
	* @output wheel_speed[4];
	* @note : 1:FR, 2=FL 3=BL, 4=BR, ↑↓=Vy， ←→=Vx
		@map TYPE=1
			2	%++++++%	1
					++++
					++++
			3	%++++++%	4    ↑=+Vy  →=+Vx
**/
void mecanum_calc(float vx, float vy, float omega, const int each_max_spd, s16 speed[]){
	s16 buf[4];
	int i;
	float max=0, rate;
	
	vx = vx > MAX_CHASSIS_VX_SPEED ? MAX_CHASSIS_VX_SPEED : vx;
	vx = vx < -MAX_CHASSIS_VX_SPEED ? -MAX_CHASSIS_VX_SPEED : vx;	
	vy = vy > MAX_CHASSIS_VY_SPEED ? MAX_CHASSIS_VY_SPEED : vy;
	vy = vy < -MAX_CHASSIS_VY_SPEED ? -MAX_CHASSIS_VY_SPEED : vy;
	omega = omega > MAX_CHASSIS_OMEGA_SPEED ? MAX_CHASSIS_OMEGA_SPEED : omega;
	omega = omega < -MAX_CHASSIS_OMEGA_SPEED ? -MAX_CHASSIS_OMEGA_SPEED : omega;
	
#ifdef DEBUG_REVERSE
	buf[0] = ( -vx + vy + omega );
	buf[1] = ( -vx - vy + omega );
	buf[2] = ( +vx + vy + omega );
	buf[3] = ( +vx - vy + omega );  //因为轮子方向的关系
#endif	
	
	
#ifdef CHASSIS_OLD
	buf[0] = ( -vx + vy - omega );
	buf[1] = ( -vx - vy - omega );
	buf[2] = ( +vx - vy - omega );
	buf[3] = ( +vx + vy - omega );  //因为轮子方向的关系
#else //这个不一定是正确的  等待修改
	buf[0] = ( vx + vy + omega );
	buf[1] = -( vx - vy - omega );
	buf[2] = ( vx - vy + omega );
	buf[3] = -( vx + vy - omega );  //因为轮子方向的关系
#endif
	//find max item
	for(i=0; i<4; i++){
		if ( MyAbs(buf[i]) > max )
			max = MyAbs(buf[i]) ;
	}
	//等比例缩放， 都是对于绝对值。 不允许单个轮子速度超过最大值，否则合成不正常
	if (max > each_max_spd){
		rate = each_max_spd / max;
		for(i=0; i<4; i++)
			buf[i] *= rate;
	}
	//output
	memcpy(speed, buf, sizeof(s16)*4); 
}
/**
	* @bref   send 4 calculated current to driver
	* @assume 3510 driver in current mode
	* @attention Do NOT change this function.for safety purpose.	//btw, i think you can't ~ ~
*/

//void Brake_calibration_Init()
//{	
//}	

void Brake_calibration()
{ 
	static int Brake_force_fre_1 = 0 ;	
	if(flag_left == 1)
	{
		brake_target_angle += brake_calibration_angle ;
		if(abs(pid_Brake_omg.pos_out)>brake_Max_op)
		{
			Brake_force_fre_1++ ;
		}
		if(Brake_force_fre_1 >= system_fre)
		{	
			brake_target_angle = moto_brake.total_angle ;
			brake_1_total = moto_brake.total_angle ;
			flag_left = 0;
			flag_right = 1;
			switch_1 =1 ;
			Brake_force_fre_1 = 0 ;
		}	
	}
	if(flag_right == 1)
	{
		static int brake_target_fre_2 = 0 ;
		brake_target_angle -= brake_calibration_angle ;
		if(abs(pid_Brake_omg.pos_out)>brake_Max_op)
		{
			brake_target_fre_2++;
		}
		if(brake_target_fre_2 >= system_fre)
		{
			brake_2_total = moto_brake.total_angle ;
			flag_right = 0	;
			switch_2 = 1 ;
			brake_target_fre_2 = 0 ;
		}		
	}
	if((switch_1 == 1) && (switch_2 == 1))
	{
		brake_target_angle = (brake_1_total + brake_2_total)/2 - 6780 ;
		brake_Mid_angle = (brake_1_total + brake_2_total)/2 - 6780 ;
		switch_1 = 0 ;
		switch_2 = 0 ;
	}	
}

//void Brake_calibration_turn_back()
//{
//	if(abs(pid_Brake.pos_out)>Brake_Max_Op)
//	brake_target_angle -= brake_calibration_offset ;  
//}	
void set_cm_current(s16 chassis_iq1,s16 Brake_iq)
{
	CHASSIS_CAN.pTxMsg->StdId = 0x200;
	CHASSIS_CAN.pTxMsg->IDE = CAN_ID_STD;
	CHASSIS_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	CHASSIS_CAN.pTxMsg->DLC = 8;
	CHASSIS_CAN.pTxMsg->Data[0] = chassis_iq1 >> 8 ;
	CHASSIS_CAN.pTxMsg->Data[1]	= chassis_iq1      ;
	CHASSIS_CAN.pTxMsg->Data[2] = Brake_iq >> 8    ;
	CHASSIS_CAN.pTxMsg->Data[3]	= Brake_iq         ;
	HAL_CAN_Transmit(&CHASSIS_CAN, 1000);
}	

//void set_cm_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4)
//{
//	
//	memcpy((hcan->pTxMsg), pack_can_msg(iq1,iq2,iq3,iq4), sizeof(CanTxMsgTypeDef));
//		
//	HAL_CAN_Transmit(hcan, 1000);
//	
//}	

void get_chassis_mode_set_ref(RC_Type* rc)
{
	chassis.last_mode = chassis.mode;
	
	switch(rc->sw2)
	{
		case RC_UP:
//			Shot_Mode=ONE_MODE;//
			Shot_Mode=CONT_MODE;
			chassis.mode =chassis_auto ;	
			break;
		case RC_MI:
			Shot_Mode=THEER_MODE;
			chassis.mode = CHASSIS_FOLLOW_GIMBAL_ENCODER;	//遥控可操控，做测试用
			break;
		case RC_DN:
			Shot_Mode=CONT_MODE;
			chassis.mode = CHASSIS_CLOSE_GYRO_LOOP;	//进入自动chassis_auto
			break;
		default:
			break;
	}
#ifdef SAFE_CHASSIS
		chassis.mode = CHASSIS_RELAX;	
#endif
	
	
//	switch(chassis.mode)
//	{
//		case CHASSIS_CLOSE_GYRO_LOOP:
//		{
//			 chassis.vx = 0;
//			 chassis.vy = 0;
//		}
//		break;
//		case CHASSIS_RELAX:
//		{
//			chassis.vx = 0;
//			chassis.vy = 0;
//		}
//		break;
//		case CHASSIS_FOLLOW_GIMBAL_ENCODER:
//		{
//			chassis.vy = rc->ch2*0.5f*CHASSIS_RC_MOVE_RATIO_Y + km.vy*CHASSIS_PC_MOVE_RATIO_Y + tu_rc.bits.ch2;	//last item is or
//			chassis.vx = rc->ch1*0.5f*CHASSIS_RC_MOVE_RATIO_X + km.vx*CHASSIS_PC_MOVE_RATIO_X + tu_rc.bits.ch1;
//		}
//		break;	
//	}			
}

float scope_param[2];

void chassis_task(void const * argu)
{	
	int flag = 1   ;
//	int ceshi = 1;
//	ceshi = 0    ;
	brake_target_angle = moto_brake.total_angle;
	for(int k=0; k<4; k++)
	{
		/*	max current = 20000, it may cause deadly injury !!! just like me today*/
		PID_struct_init(&pid_spd[k], POSITION_PID, 20000, 20000,		//origin = 20000
								4,	0.05f,	0.0f	);  //4,	0.05f,	0.0f
	}
	PID_struct_init(&pid_chassis_angle, POSITION_PID, 400, 0,
									0.18f,	0.0005f,	0.0f	);  
	PID_struct_init(&pid_vy, POSITION_PID, 200, 200,
									0.5f,	0.0f,	0.0f	);  
	PID_struct_init(&pid_Brake, POSITION_PID, 10000,  0 ,	 0.27,	0.02,	0);   
  PID_struct_init(&pid_Brake_omg, POSITION_PID, 10000 , 0,  3.8,	 0.1, 1);
	HAL_Delay(1000);
	//TODO : chassis follows gimbal. chassis follow gimbal encoder.
	while(1)
	{		
			brake_NOW_spd = moto_brake.speed_rpm      ;
			brake_total_angle = moto_brake.total_angle;
		//Brake_calibration_Init();
		if(rc.sw1 == RC_DN)
		{	
			Brake_calibration();
		}
		u16 MAX_WHEEL_SPEED					= 4500;
		get_chassis_mode_set_ref(&rc);//获取遥控器的值对应的模式		
		chassis_v.chongneng=200;									//三种速度//充能
		chassis_v.duobi=300;										//躲避
		chassis_v.xunluo=200;		//巡逻
		switch(chassis.mode)
	  {
		case CHASSIS_FOLLOW_GIMBAL_ENCODER:
			{
				if(rc.sw1 == RC_UP)
				{
					brake_target_angle += Brake_single_angle;
				}
        if(rc.sw1 == RC_MI)
				{
					brake_target_angle = brake_Mid_angle ;
				}
//				chassis.vy = rc.ch2*0.5f*CHASSIS_RC_MOVE_RATIO_Y + km.vy*CHASSIS_PC_MOVE_RATIO_Y + tu_rc.bits.ch2;	//last item is or
//				chassis.vx = rc.ch1*0.5f*CHASSIS_RC_MOVE_RATIO_X + km.vx*CHASSIS_PC_MOVE_RATIO_X + tu_rc.bits.ch1;
			}break;
			
		case CHASSIS_CLOSE_GYRO_LOOP:
		  {
				chassis.vx = 0;
				chassis.vy = 0;
			}break;
			
		case chassis_auto:
		  { 
				if(flag == 1)
				{ 
					chassis.vy = -chassis_v.xunluo ;
					flag = 0 ;	
				}
				if(market_brake_3 == 1)
				{	
					if(moto_chassis[0].total_angle<-556390)
					{
						brake_target_angle -= Brake_single_angle ;
						market_brake_1++                         ;
						if(market_brake_1>5)
						{ 					
							chassis.vy = chassis_v.xunluo          ;
							if(moto_chassis[0].real_current>=0)
							{
								brake_target_angle = brake_Mid_angle   ;
							}								
							market_brake_1 = 0 ;
							market_brake_3 = 0 ;
							market_brake_4 = 1 ;
							moto_chassis[0].total_angle += 20000;
						}											
					}	
				}	
			  else if(market_brake_4 == 1)
				{
					if(moto_chassis[0].total_angle>556390)
					{
						brake_target_angle += Brake_single_angle   ;
						market_brake_2++                           ;
						if(market_brake_2>5)
						{
							chassis.vy = -chassis_v.xunluo           ;
							if(moto_chassis[0].real_current<=0)
							{
								brake_target_angle = brake_Mid_angle   ;
							}								
							market_brake_2 = 0                       ;
							market_brake_3 = 1                       ;
							market_brake_4 = 0                       ;
						}											
					}	
				}
		}
	 }		
//		if(chassis.mode == CHASSIS_FOLLOW_GIMBAL_ENCODER&& rc.sw2 != RC_MI)  //need to change range(-xxxx -> 0 -> +xxxx)//done
//		{	
//			switch(AntiMode)
//			{
//				case AntiForward1:
//					YawAntiOffset = 0;					
//					chassis.omega = pid_calc(&pid_chassis_angle, yaw_relative_pos, 0);
//					break;
//				case AntiRight:
//					YawAntiOffset = -1000;
//					chassis.omega = pid_calc(&pid_chassis_angle, yaw_relative_pos, YawAntiOffset);
//					break;
//				case AntiForward2:
//					YawAntiOffset = 0;		
//					chassis.omega = pid_calc(&pid_chassis_angle, yaw_relative_pos, 0);
//					break;
//				case AntiLeft:
//					YawAntiOffset = 1000;
//					chassis.omega = pid_calc(&pid_chassis_angle, yaw_relative_pos, YawAntiOffset);
//					break;
//			}
//		}
//		else
//		{
//	    chassis.omega =0;      // pid_calc(&pid_chassis_angle, yaw_relative_pos, 0);
//		}
//		
//		if ( fabs(chassis.vx) < 10) chassis.vx = 0;	//avoid rc stick have little offset
//		if ( fabs(chassis.vy) < 10) chassis.vy = 0;

			pid_calc(&pid_Brake, brake_total_angle, brake_target_angle);
			pid_calc(&pid_Brake_omg, brake_NOW_spd, pid_Brake.pos_out);
			mecanum_calc(chassis.vx*10,chassis.vy*15,chassis.omega*(-20), MAX_WHEEL_SPEED, chassis.wheel_speed.s16_fmt);//chassis.omega*(-30)

		  buff_3510iq[0] =chassis.wheel_speed.s16_fmt[0]+pidd(moto_chassis[0].speed_rpm,chassis.wheel_speed.s16_fmt[0]);
//		buff_3510iq[1] =chassis.wheel_speed.s16_fmt[1]+pidd22(moto_chassis[1].speed_rpm,chassis.wheel_speed.s16_fmt[1]);
//		buff_3510iq[2] =chassis.wheel_speed.s16_fmt[2]+pidd33(moto_chassis[2].speed_rpm,chassis.wheel_speed.s16_fmt[2]);
//		buff_3510iq[3] =chassis.wheel_speed.s16_fmt[3]+pidd44(moto_chassis[3].speed_rpm,chassis.wheel_speed.s16_fmt[3]);
		
#if 1   //是否启动底盘
		if(rc.sw2!=RC_DN)
		set_cm_current(0,0);
#else 
		set_cm_current(0,0);
			
#endif
		osDelay(10);
	}
}

/* 初始化底盘PID */
//void chassis_pid_init(void)
//{

//}

