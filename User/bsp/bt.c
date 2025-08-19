
/**
  * @update 2016年5月24日22:50:28
  * @note  更新了串口接收的方式。 
	*				//使用DMA来接收，每当空闲中断的时候，也就是不收数据了的时候处理接收到的数据。
	*				//DMA中断正常情况下是不触发的， 要保证MAX-RX-SIZE大于每次的一帧数据长度
	*				//所以DMA的作用就是加快收数据的速率，然后其他没了
	*				//################### 状态机： ########################
	*				进入空闲中断--处理收到的数据--清除中断标记
	*				//--重新初始化DMA--等待接收数据直到空闲--进入空闲中断
  *       //################### 状态机： ########################	
  * @after	测试过一次，成功！ 后面可能还要测一下 
  */



#include "mytype.h"
//#include "mpu.h"
#include "bt.h"
//#include "led.h"
#include "usart.h"
#include "error_task.h"
#include "pid.h"
#include "judge_sys.h"
#include "sys.h"
#include "mahonyAHRS.h"
#include "math.h"
#include "mpu.h"
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
#include "bt.h"
#if defined STM32F4
u32 JumpIapFlag __attribute__((at(0x40024000)));
#endif

#define MAX_DMA_COUNT						100
#define DBUS_RX_MAX_BUFLEN			20
#define BT_RX_MAX_BUFLEN				100	//这个只要比一帧大就行拉

#define ARMAPI		extern "C"	//add this before hal callback
 VisionRecvData_t  VisionRecvData_te;
float wwww;

FormatTrans   info_dateTran;
u8 cv_num=0;

RC_Type rc;

uint32_t Vision_Time_Test[2] = {0};//Ç°ºóÁ½´ÎÊÂ¼þ
uint16_t Vision_Ping = 0;//²âÊÔÊ±¼ä¼ä¸ô
char USART6_FLAG=0;
u8 	bt_rx_buff[BT_RX_MAX_BUFLEN]; 
u8	dbus_buff[DBUS_RX_MAX_BUFLEN];
u8  cv_buff[50];
u16 cv_yaw=0,cv_pit=0;//装甲板坐标
u8  info_judge[46];
FloatConvertType   rcDataTable[RcTableLength]; 
float easyTable[RcTableLength];
uint8_t transmit_frame[FrameLength];
u8 frame_cnt,err_cnt;
u32 fps_total=0;
u32 last_cv;
u32 delta_cv;
u8 fps;
GameInfo_Struct game_info_use;

u8 DBUS_RXD_Count=0;
u8 Last_DBUS_RXD_Count=0;
//enable global uart it, do not use DMA transfer done it!!!
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, u8* pData, u32 Size){
  uint32_t tmp1 = 0;
  
  tmp1 = huart->RxState;    
  if(tmp1 == HAL_UART_STATE_READY)
  {
    if((pData == NULL ) || (Size == 0)) 
    {
      return HAL_ERROR;
    }
    
    /* Process Locked */
    __HAL_LOCK(huart);
    
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    
    /* Enable the DMA Stream */
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,  (uint32_t)pData, Size);
    
    /* Enable the DMA transfer for the receiver request by setting the DMAR bit 
    in the UART CR3 register */
    huart->Instance->CR3 |= USART_CR3_DMAR;
    
    /* Process Unlocked */
    __HAL_UNLOCK(huart);
    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY; 
  }
}

void judge_use_sys()
{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    UART_Receive_DMA_No_IT(&huart3, info_judge, FRAME_BUFLEN);
}


void deal_info_date(const void *argu)
{
	wwww=0;
	while(1)
	{
	//	judge_use_sys();
	if (info_judge[0]==0X53)
	{
		 //HAL_Delay(10);//等数据稳定
	   
  info_dateTran.U[0]=info_judge[13];
	info_dateTran.U[1]=info_judge[14];
	info_dateTran.U[2]=info_judge[15];
	info_dateTran.U[3]=info_judge[16];
	game_info_use.realChassisOutV=info_dateTran.F;
			 
	info_dateTran.U[0]=info_judge[17];
	info_dateTran.U[1]=info_judge[18];
	info_dateTran.U[2]=info_judge[19];
	info_dateTran.U[3]=info_judge[20];
	game_info_use.realChassisOutA=info_dateTran.F;
	chassis.WW=game_info_use.realChassisOutV*game_info_use.realChassisOutA;	

  if(chassis.WW>140){wwww=1;	}		 
	info_dateTran.U[0]=info_judge[38];
	info_dateTran.U[1]=info_judge[39];
	info_dateTran.U[2]=info_judge[40];
	info_dateTran.U[3]=info_judge[41];
	game_info_use.energy=info_dateTran.F;
			 
	game_info_use.blood= ((int16_t)info_judge[12] << 8) | info_judge[11];

		

	}
	 osDelay(5);
}	
}
void judge_sys_init()
{
	#ifdef JUDGE_HUART
    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    UART_Receive_DMA_No_IT(&JUDGE_HUART, judgementBuf, FRAME_BUFLEN);
	#endif
}

void manifold_uart_init()
{
	#ifdef CV_HUART
		__HAL_UART_CLEAR_IDLEFLAG(&CV_HUART);
		__HAL_UART_ENABLE_IT(&CV_HUART, UART_IT_IDLE);
		UART_Receive_DMA_No_IT(&CV_HUART, cv_buff, 50);
	#endif

}
void dbus_init(){
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
	//clear idle it flag & open idle it
	UART_Receive_DMA_No_IT(&DBUS_HUART, dbus_buff, DBUS_RX_MAX_BUFLEN);
	//下面一句使用联合体来代替解码，效率高很多
}

void bt_init(){
	__HAL_UART_CLEAR_IDLEFLAG(&BT_HUART);
	__HAL_UART_ENABLE_IT(&BT_HUART, UART_IT_IDLE);
	UART_Receive_DMA_No_IT(&BT_HUART, bt_rx_buff, BT_RX_MAX_BUFLEN);
	//状态机：进入空闲中断--处理收到的数据--清除中断标记
	//--重新初始化DMA--等待接收数据直到空闲--进入空闲中断
}

/**
	*@bref use uart idle it + dma(no it) to receive a frame data.
	*/
void uart_idle_dma_rx_init(UART_HandleTypeDef *huart, u8* rx_buff, u16 buff_len){
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	UART_Receive_DMA_No_IT(huart, rx_buff, buff_len);
}


void uart_reset_idle_rx_callback(UART_HandleTypeDef *huart){
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)){
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		//clear idle it flag
		//重启DMA
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);//根据串口的不同来选择清除不同的DMA标志位
		
		__HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,	DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx,	MAX_DMA_COUNT);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}



void Dma_Callback_RC_Handle(RC_Type* rc, uint8_t* buff);
//这个函数放在BT对应的 Usart-IRQHandler 里面
void	LanggoUartFrameIRQHandler(UART_HandleTypeDef *huart)
{
	


	if(huart == &DBUS_HUART)
	{
	//HAL_UART_Receive_DMA(&huart4, rcUnion.buff, RC_Frame_Lentgh);
		Dma_Callback_RC_Handle(&rc, dbus_buff);
		err_detector_callback(DbusTOE);
	//使用DMA来接收，每当空闲中断的时候，也就是不收数据了的时候处理接收到的数据。
//		DBUS_RXD_Count++;           //丢控保护 未成功
//		if(DBUS_RXD_Count>200)
//		{
//			DBUS_RXD_Count=0;
//		}
	}
	
	#ifdef JUDGE_HUART
	else if(huart == &JUDGE_HUART)
	{
			judgementDataHandler(judgementBuf);
	}
	#endif
	
	#ifdef CV_HUART
	else if(huart == &CV_HUART)
	{
//    if(cv_buff[0]==0xff)
//		{
//		    VisionRecvData_te.SOF=cv_buff[0];
		if(cv_buff[0] == 0x53)
		{
			 VisionRecvData_te.date_type=cv_buff[1];
			 VisionRecvData_te.shot_flag=cv_buff[2];
			 VisionRecvData_te.yaw_head=cv_buff[3];
			
			 VisionRecvData_te.yaw_langle=cv_buff[4];
			 VisionRecvData_te.yaw_hangle=cv_buff[5];
			
			 VisionRecvData_te.pitch_head=cv_buff[6];
			VisionRecvData_te.pitch_langle=cv_buff[7];
			 VisionRecvData_te.pitch_hangle=cv_buff[8];
			
			VisionRecvData_te.ldistance=cv_buff[9];
			VisionRecvData_te.hdistance=cv_buff[10];
			
			VisionRecvData_te.Crc=cv_buff[11];
			VisionRecvData_te.frame_tail=cv_buff[12];
			
			
			
//			  info_dateTran.U[0]=cv_buff[0];
//			  info_dateTran.U[1]=cv_buff[1];
//			  info_dateTran.U[2]=cv_buff[2];
//			  info_dateTran.U[3]=cv_buff[3];
//			 VisionRecvData_te.SOF=info_dateTran.F;
//    if(VisionRecvData_te.SOF==520.13140000f)
//	      {
//			 VisionRecvData_te.pitch_angle_FLAG=cv_buff[5];
//			   
//			  info_dateTran.U[0]=cv_buff[4];
//			  info_dateTran.U[1]=cv_buff[5];
//			  info_dateTran.U[2]=cv_buff[6];
//			  info_dateTran.U[3]=cv_buff[7];
//			  VisionRecvData_te.pitch_angle=info_dateTran.F;
//	    VisionRecvData_te.yaw_angle_FLAG=cv_buff[10];
		    
//			  info_dateTran.U[0]=cv_buff[8];
//			  info_dateTran.U[1]=cv_buff[9];
//			  info_dateTran.U[2]=cv_buff[10];
//			  info_dateTran.U[3]=cv_buff[11];	
//		    VisionRecvData_te.yaw_angle=info_dateTran.F; 
			
//			  VisionRecvData_te.shot_flag=cv_buff[12];
		    memset(cv_buff,0,50);
			  }
      Vision_Time_Test[0]= xTaskGetTickCount();
			Vision_Ping=Vision_Time_Test[0]-Vision_Time_Test[1];
			Vision_Time_Test[1]=Vision_Time_Test[0];
	
	}
	#endif

	else if(huart == &BT_HUART){
		
	}

	uart_reset_idle_rx_callback(huart);
}
void	LanggoUartFrameIRQHandler1(UART_HandleTypeDef *huart)
{
	uart_reset_idle_rx_callback(huart);
}

//Warning！ ARMAPI shoule not be remove？！
//ARMAPI void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	//
void Dma_Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
//	int six_total = buff[0] + buff[1] + buff[2]
//								+ buff[3] + buff[4] + buff[5];
//	if(0 == six_total)return;	//可能出现偶然情况么 - -。。。
	if(	buff[0] == 0 && buff[1] == 0 && buff[2]==0	
		&& buff[3] == 0 && buff[4] == 0 && buff[5]==0)
		return;
	//err_detector_callback(DbusTOE);	//a hook of dbus rx monitor
	
	//如果数组前面表示通道的全部为0，说明是错误数据则return;
//	rc->ch1 = (*buff | *(buff+1) << 8) & 0x07FF;	offset  = 1024
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	
	rc->sw1 = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->sw2 =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.l = buff[12];	// is pressed?
	rc->mouse.r = buff[13];
	
	rc->kb.key_code = buff[14] | buff[15] << 8; //key borad code
}


void uart_send_frame(UART_HandleTypeDef *huart, RcTableType id){
    FloatConvertType	tmp;
    transmit_frame[Head1]   = 0x55;
    transmit_frame[Head2]   = 0xAA;
    transmit_frame[DataID]  = id;
   
    tmp.F32type = rcDataTable[id].F32type;

    transmit_frame[Byte0]   = tmp.U8type[0];
    transmit_frame[Byte1]   = tmp.U8type[1];
    transmit_frame[Byte2]   = tmp.U8type[2];
    transmit_frame[Byte3]   = tmp.U8type[3];
    
    transmit_frame[SumCheck]=   (uint8_t)(transmit_frame[DataID] 
                                        + transmit_frame[Byte0] 
                                        + transmit_frame[Byte1]  
                                        + transmit_frame[Byte2] 
                                        + transmit_frame[Byte3]); 
    transmit_frame[Tail]    = 0xFF;
    
	HAL_UART_Transmit(huart, transmit_frame, FrameLength,100);  
		//use blocking mode transmit
}

void uart_send2pc(UART_HandleTypeDef* huart, RcTableType id, float value){
	rcDataTable[id].F32type = value;
	uart_send_frame(huart, id);
}

void uart_send4byte(UART_HandleTypeDef* huart, RcTableType id, void* buff){
	
	u8* p = buff;
	rcDataTable[id].U8type[0] = *p;
	rcDataTable[id].U8type[1] = *(p+1);
	rcDataTable[id].U8type[2] = *(p+2);
	rcDataTable[id].U8type[3] = *(p+3);
	
	uart_send_frame(huart, id);
}

void bt_printf(const char* fmt, ...){
	char buff[128] = {0};
	char *p = buff;
	va_list ap;
//	__va_list ap;
	va_start(ap, fmt);
	vsprintf(buff, fmt, ap);
	
	int size=0;
	while(*p++){
		size++;
	}
	
	HAL_UART_Transmit(&BT_HUART, (u8*)buff, size, 1000);
	va_end(ap);

}

