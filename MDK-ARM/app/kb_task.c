




#include "dogconfig.h"

#define MAX_KB_SLOWLY_SPEED     150
#define MAX_KB_NORMAL_SPEED     350
#define MAX_KB_QUICKLY_SPEED    450

//#define MAX_

enum{
  NormalMode = 0, 
  ShiftQuicklyMode,
  CtrlSlowlyMode,
}KB_MoveMode;

/************************* ȫ�ֱ��� *******************************/
//extern RC_Type rc; 
kb_control_t  kb;
int hello, look, fuck;
u16 key_code=0;
u8 openKeyenceDetect = 0;
vu8 kb_move_mode = 0;
u16 max_kb_move_speed;
u16 open_bullet_door_cnt=0;
/************************* ȫ�ֱ��� *******************************/

/**
*@public:
**/
void KeyDealTask_Func(void const * argument)
{
  /* USER CODE BEGIN KeyDealTask */
	//RemoteControl_Param_Init();
	//Cloud_PID_Param2Hand_Mode();//�ϵ��ֶ�ģʽ
	osDelay(1000);//�ȴ��ȶ������������(��Ҫ��Ҫ�ȴ������Ƕ��θ�λ�ȶ�)
  /* Infinite loop */
  for(;;)
  {
    key_code = rc.keyBoard.key_code;
    if(key_code & ClickE)
      myUnderpan.is_snipe_mode = 1;
    else
      myUnderpan.is_snipe_mode = 0;
    
    /*����shitf����ctrl���������ѡ�����ǰ�����Ҽ��ټ���ģʽ*/
    if(key_code & ClickShift)
      kb_move_mode = ShiftQuicklyMode;
    else if(key_code & ClickCtrl)
      kb_move_mode = CtrlSlowlyMode;
    else
      kb_move_mode = NormalMode;
    /*�ϵ�֮���ٶȼ���*/
    if(   eLandingState == SeatOnIslandFirst
        || eLandingState == StandOnIslandToCatchEggs
        || eLandingState == CatchEggsDoneSeatOnIsland)
      kb_move_mode = CtrlSlowlyMode;
    /*�����ٶ�ģʽѡ�񰴼����ֵ�����ٶ�*/
    switch(kb_move_mode){
        case NormalMode:
          max_kb_move_speed = MAX_KB_NORMAL_SPEED;break;
        case ShiftQuicklyMode:
          max_kb_move_speed = MAX_KB_QUICKLY_SPEED;break;
        case CtrlSlowlyMode:
          max_kb_move_speed = MAX_KB_SLOWLY_SPEED;break;
        default:
          max_kb_move_speed = MAX_KB_NORMAL_SPEED;break;
    }
    
    
    
    if(key_code & ClickW)   //�������ٵ�400
      kb.speed_x += 2;
    else if(key_code & ClickS)
      kb.speed_x += -2;
    else
      kb.speed_x = 0;
    
    if(key_code & ClickA)
      kb.speed_y += -2;
    else if(key_code & ClickD)
      kb.speed_y += 2;
    else
      kb.speed_y = 0;
    
    
    
    
    if(kb.speed_x > max_kb_move_speed)
      kb.speed_x = max_kb_move_speed;
    if(kb.speed_x < -max_kb_move_speed)
      kb.speed_x = -max_kb_move_speed;
    
    if(kb.speed_y > max_kb_move_speed)
      kb.speed_y = max_kb_move_speed;
    if(kb.speed_y < -max_kb_move_speed)
      kb.speed_y = -max_kb_move_speed;
    
		switch(key_code)
		{
      //switch-case �൱��if else if �ṹ������wsad�����γ���ϼ���Ӧ��ʹ��ifͬ���ж�
//			case(ClickW):// ���൱���ĸ�Ҫһ�𰴡�����

			case(ClickE):	//enter snipe mode 
//myUnderpan.is_snipe_mode = 1;
				break;
			case(ClickR):
        PULL_CATCH_EGG();
				break;
			case(ClickR|ClickCtrl):
        PUSH_CATCH_EGG();
				break;
      
      case(ClickF):
        openKeyenceDetect = 1;
        PUSH_CATCH_EGG();
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigStart);
				break;
      case (ClickF | ClickShift):
        AIR_FRONT_LONG;
        break;
      case (ClickF | ClickCtrl):
        AIR_FRONT_SHORT;
        break;
      case (ClickB | ClickShift):
        AIR_BACK_LONG;
        break;
      case (ClickB | ClickCtrl):
        AIR_BACK_SHORT;
        break;
      
			case(ClickZ):
				
					//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigGoUp);
				break;
			case(ClickX):
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigGoOut);
				break;
			/*���C ����50ms �൱�ڷ���� ���ȡ��������������ȥ*/
			case(ClickC):
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigGoDown);
				break;  
			case(ClickV):
        
				break;
			case(ClickB):	//just like break-
        open_bullet_door_cnt = 200;
        osDelay(1000);
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigStop);
				break;
			case(ClickShift):
				break;
			case(ClickCtrl):
				break;
			
			/*combine keys */
			case(ClickZ|ClickCtrl):
				/*������סCtrl��������ץ�������������¶���ֱ�����ʵ�ʱ��ſ�*/
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigGoDown);
				break;
			
			case(ClickQ|ClickShift):
        kb.angle -= 0.5f;  //1s = 20'
				break;
			case(ClickE|ClickShift):
        kb.angle += 0.5f;  //1s = 20'
				break;
			case(ClickShift|ClickCtrl):
				break;
			
			default :
				break;
		}
		
    if(open_bullet_door_cnt){
      OPEN_BULLET_DOOR();
      open_bullet_door_cnt--;
    }
    else
      CLOSE_BULLET_DOOR();
    
		osDelay(10);
  }
  /* USER CODE END KeyDealTask */
}



