#ifndef __JUDGEMENT_H
#define __JUDGEMENT_H

#include "stm32f4xx_hal.h"
#include "mytype.h"
#include "bt.h"
#define FRAME_BUFLEN 255

/** 
  * @brief  命令码ID
  */
typedef enum{
	game_state_ID									     =0x0001, //比赛状态数据
	event_data_ID                      =0x0101, //场地事件数据
	ID_game_robot_survivors            =0x0003, //比赛机器人血量数据
	supply_projectile_action_ID        =0x0102, //补给站动作标识
	//supply_projectile_booking_ID       =0x0103, //请求补给站补弹子弹
	game_robot_state_ID                =0x0201, //比赛机器人状态
	power_heat_data_ID                 =0x0202, //实时功率热量数据
	game_robot_pos_ID						       =0x0203, //机器人位置
	buff_musk_ID								       =0x0204, //机器人增益
	aerial_robot_energy_ID             =0x0205, //空中机器人能量状态
	robot_hurt_ID                      =0x0206, //伤害状态
	shoot_data_ID                      =0x0207, //实时射击信息
	student_interactive_header_data_ID =0x0301, //交互数据接收信息
	
//	client_custom_data_ID              =0x0301, //客户端 客户端自定义数据
//	robot_interactive_data_ID          =0x0301, //交互数据 机器人间通信
	RealRemoteControlId,
}comIdType;


/** 
  * @brief  FrameHeader structure definition,帧头结构体
  */
typedef __packed struct  //自己这端的刚开始检测
{
  uint8_t   sOF;
  uint16_t  dataLenth;
	uint8_t		seq;
  uint8_t   cRC8;
}tFrameHeader;


extern uint8_t judgementBuf[];
void judgementDataHandler(uint8_t *READ_FROM_USART);
//2019

typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;


typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;
 
  

typedef __packed struct
{
 uint8_t supply_projectile_id;                //补给站补弹口 ID
 uint8_t supply_robot_id;                     //补弹机器人 ID
uint8_t supply_num;                           //补弹数目
} ext_supply_projectile_booking_t;

typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_id1_17mm_cooling_rate;
 uint16_t shooter_id1_17mm_cooling_limit;
 uint16_t shooter_id1_17mm_speed_limit;
 uint16_t shooter_id2_17mm_cooling_rate;
 uint16_t shooter_id2_17mm_cooling_limit;
 uint16_t shooter_id2_17mm_speed_limit;
 uint16_t shooter_id1_42mm_cooling_rate;
 uint16_t shooter_id1_42mm_cooling_limit;
 uint16_t shooter_id1_42mm_speed_limit;
 uint16_t chassis_power_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;


typedef __packed struct
{
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_id1_17mm_cooling_heat;
uint16_t shooter_id2_17mm_cooling_heat;
uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;

typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_t;


typedef __packed struct
{
 uint8_t energy_point;                  //积累的能量点
 uint8_t attack_time;                   //可攻击时间（单位s）50s递减至0
} ext_aerial_robot_energy_t;

typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;


typedef __packed struct
{
float data1;                                  //自定义浮点数据 1
float data2;                                  //自定义浮点数据 2
float data3;                                  //自定义浮点数据 3
uint8_t masks;                                //自定义8位数据 4
} ext_client_custom_data_t;

typedef __packed struct
{
	u16 DataID;
	u16 SenderID;
	u16 ClientID;
	
	ext_client_custom_data_t client_custom_data;

}ext_TransmitData_t;

typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;



extern uint8_t judgementBuf[];
void judgementDataHandler(uint8_t *READ_FROM_USART);
extern RC_UnionDef			tu_rc;

//2021新裁判系统
typedef __packed struct
{	
	tFrameHeader                          testFrameHeader;
 ext_game_status_t 							        game_state;
 ext_event_data_t                       event_data;
 ext_supply_projectile_action_t         supply_projectile_action;        //补给站动作标识
 ext_supply_projectile_booking_t        supply_projectile_booking;
 ext_game_robot_status_t                 game_robot_state;                //比赛机器人状态
 ext_power_heat_data_t                  power_heat_data;                 //实时功率热量数据
 ext_game_robot_pos_t                   game_robot_pos;
 ext_buff_t                             buff_t;
 ext_aerial_robot_energy_t              aerial_robot_energy;
 ext_robot_hurt_t                       robot_hurt;                      //机器人伤害状态
 ext_shoot_data_t                       shoot_data;                      //实时射击信息
 ext_student_interactive_header_data_t  student_interactive_header_data;
 ext_client_custom_data_t               client_custom_data;
 ext_game_robot_HP_t                    game_robot_HP_t;
//extern ext_robot_interactive_data_t           robot_interactive_data;
}Judgeeverydata;

extern  Judgeeverydata JudgeeverydataE;
extern int16_t receive_vision;
void SendLed(u8 LED,u8 LEDState);		 // Send LED state to the client
void SendData(u8 FNum,float FData);  //Send data to the client
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
#endif
