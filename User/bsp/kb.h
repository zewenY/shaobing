#ifndef __KB__
#define __KB__


#include "stm32f4xx_hal.h"
#include "mytype.h"
/************************ 按键地址 *************************/
/**********************************************************************************
   * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/
//#define W 			0x0001		//bit 0
//#define S 			0x0002
//#define A 			0x0004
//#define D 			0x0008
//#define SHIFT 	0x0010
//#define CTRL 		0x0020
//#define Q 			0x0040
//#define E				0x0080
//#define R 			0x0100
//#define F 			0x0200
//#define G 			0x0400
//#define Z 			0x0800
//#define X 			0x1000
//#define C 			0x2000
//#define V 			0x4000		//bit 15
//#define B				0x8000
/************************ 按键地址 *************************/
#define SpeedQuickly rc.kb.bit.SHIFT
#define SpeedSlowly rc.kb.bit.CTRL

typedef enum{
  NormalMode = 0, 
  ShiftQuicklyMode,
  CtrlSlowlyMode,
}KB_MoveMode;

typedef enum{
  AntiForward1 = 0, 		//向前
	AntiForward2,					//向前
  AntiRight,						//向右
  AntiLeft,							//向左
}AntiMode_t;

typedef struct{	
  float vx;								//chassis move x speed
  float vy;
	float vz;
	int acc;
  float qe_spin_angle;	//Q/E spin not used yet
  u8 		is_fire;				//not used yet
	u8 		fire_sta;				//not used
}km_control_t;

typedef enum
{
	off = 0,
	on  = 1,
}ON_OFF;

//typedef struct{
//  float VX_max_speed;								//chassis move x speed
//  float VY_max_speed;
//	float max_wheel_speed;

//}SPEED_USE;
extern int turning ;
extern km_control_t km;
//extern SPEED_USE speed_max_use ;
void key_mouse_task(const void*);
u8 KeyStateMachine(u8* psta, u8 condition);
u8 KeyStateMachine_hold(u8* psta, u8 condition);
extern int huanxiang;
//void chassis_patrol(void) ;
#endif
