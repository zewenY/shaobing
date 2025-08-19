
#pragma once



void gimbal_task(const void *argu);
void AHRS_GetAttitude(const void *argu);
void deal_info_date(const void *argu);
typedef enum{

	GIMBAL_INIT=0,
	GIMBAL_RELAX,
	GIMBAL_CLOSE_LOOP_ENCODER,
	GIMBAL_CLOSE_LOOP_ZGYRO,

}eGimbalMode;

typedef struct {
	eGimbalMode ctrl_mode;	//yaw 闭环模式
	eGimbalMode last_mode;	//
	int is_standby_mode;	//rc no action
	int over_range_err_cnt;	//for safety purpose
	int gimbal_init_done;
	int is_zgyro_ready;
	float zgyro_target;
	float last_zgyro_target;
	float zgyro_rc;
	float zgyro_mouse;
	float zgyro_angle;
	float zgyro_offset;
	float final_position_get;
	float final_position_set;
}gimbal_yaw_t;

//sll:shot mode
typedef enum
{
	ONE_MODE,
	
	THEER_MODE,
	
	CONT_MODE,  //continuous
	
}SHOT_MODE;

//typedef struct
//{
//	unsigned char Period;
//	unsigned char  
//	
//}ShootF;

//sll: poke work pattern

typedef struct
{
	int ref;
	int pos;
	int omg_ref;
	int omg_pos;
	int out;
}direction;

//direction PitDir={1,1,1,1,1,};
//direction YawDir={1,1,1,1,1,};
typedef enum
{
  CLOSE_ALL,
	
	OPEN_SMALL,
	
	OPEN_BIG,
	
	OPEN_ALL,  //continuous
	
}POKE_WORK;

extern gimbal_yaw_t gYaw;
extern float yaw_center_offset;
extern vs16 yaw_relative_pos;
extern u8 is_in_big_buff_mode;
extern u8 shot_falg;
extern u16 shot_falg_count;

