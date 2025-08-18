/**
  ******************************************************************************
  * @file		pid.h
  * @author		Langgo
  * @version		V1.1.0
  * @date		2015/11/30
  * @brief   
  * @update   2016年8月2日11:09:36
  * @by				langgo
  * @update2  2016年9月19日21:04:16 langgo
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
*/
#ifndef __pid_H
#define __pid_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
//typedef enum
//{
//    PID_Pitch_Pos		= 0,
//    PID_Pitch_Spd 	= 1,
//    PID_Yaw_Pos			= 2,
//    PID_Yaw_Spd			= 3,
//    PID_Pitch_NO_MPU= 4,
//    PID_Yaw_NO_MPU	= 5,
//    PID_Gryo 				= 6,
//    PID_Mapan 			= 7,
//    PID_3510_1,
//    PID_3510_Current1,
//}PID_ID;
//typedef struct _PID_TypeDef
//{
//	PID_ID id;
//	float last_target;
//	float target;							//目标值
//	
//	float kp;
//	float ki;
//	float kd;
//	
//	float   measure;					//测量值
//	float   err;							//误差
//	float   last_err;      		//上次误差
//	
//	int32_t pout;							//p输出
//	int32_t iout;							//i输出
//	int32_t dout;							//d输出
//	
//	int32_t output;						//本次输出
//	int32_t last_output;			//上次输出
//	
//	uint16_t max_acc;				//这次输出与上一次的最大差值，限制加速度
//	uint16_t MaxOutput;				//输出限幅
//	uint16_t IntegralLimit;		//积分限幅
//	uint16_t DeadBand;			  //死区（绝对值）
//	uint16_t ControlPeriod;		//控制周期
//	int16_t  Max_Err;					//最大误差
//	
//	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
//					PID_ID id,
//					uint16_t maxOutput,
//				  uint16_t integralLimit,
//					uint16_t deadband,
//					uint16_t controlPeriod,
//					int16_t	 max_err,     
//					float  target,
//				   
//					float kp,
//					float ki,
//					float kd);
//				   
//	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid三个参数修改
//	int32_t (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //pid计算
//}PID_TypeDef;
enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];				//目标值,包含NOW， LAST， LLAST上上次
    float get[3];				//测量值
    float err[3];				//误差
	
    
    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
    
    float pos_out;						//本次位置式输出
    float last_pos_out;				//上次输出
    float delta_u;						//本次增量值
    float delta_out;					//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;
    
		float max_err;
		float deadband;						//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//输出限幅
    uint32_t IntegralLimit;		//积分限幅
    float changing_i;
    void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                  uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid三个参数修改
//    float (*f_cal_pid)(struct __pid_t *pid, float get, float set);   			//pid计算
//		float (*f_cal_sp_pid)(struct __pid_t* pid, float get, float set, float gyro);
}pid_t;

void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd);
    
float pid_calc(pid_t* pid, float fdb, float ref);
float pid_calc_changing_i(pid_t* pid,float get, float set);
    
extern pid_t pid_rol;
extern pid_t pid_pit;
extern pid_t pid_yaw;
extern pid_t pid_vy;
extern pid_t pid_pit_omg;
extern pid_t pid_yaw_omg;	
extern pid_t pid_spd[4];
extern pid_t pid_yaw_alfa;
extern pid_t pid_chassis_angle;
extern pid_t pid_poke;
extern pid_t pid_poke_omg;
extern pid_t pid_big_poke;
extern pid_t pid_big_poke_omg;
extern pid_t pid_Brake;
extern pid_t pid_Brake_omg;		
extern pid_t pid_imu_tmp;		//imu_temperature
extern pid_t pid_cv_y;	//big buff yaw
extern pid_t pid_cv_p;
		
#endif

