#include "stm32f4xx_hal.h"
//#include "DogConfig.h"

#ifndef __ERROR_TASK
#define __ERROR_TASK

#define __DEBUG
#define 	BEEP_HaveSource		0		//0=��Դ�� 1=��Ե

#ifdef __DEBUG
#define 	DBG_PRINT(fmt, ...) \
						bt_printf("File:%s , Line:%4d: "fmt"\n", __FILE__ , __LINE__, ##__VA_ARGS__)
#else
#define		DBG_PRINT(fmt, ...)
#endif

#define LED_PORT					GPIOC



enum{

	Do1L = 0, ///*261.63Hz*/382
	Re2L, ///*293.66Hz*/    341
	Mi3L, ///*329.63Hz*/    303
	Fa4L, ///*349.23Hz*/    286
	So5L, ///*392.00Hz*/    255
	La6L, ///*440.00Hz*/    227
	Si7L, ///*493.88Hz*/    202

	Do1M, ///*523.25Hz*/    191
	Re2M, ///*587.33Hz*/    170
	Mi3M, ///*659.26Hz*/    152
	Fa4M, ///*698.46Hz*/    143
	So5M, ///*784.00Hz*/    128
	La6M, ///*880.00Hz*/    114
	Si7M, ///*987.77Hz*/    101

	Do1H, ///*1046.50Hz*/   96
	Re2H, ///*1174.66Hz*/   85
	Mi3H, ///*1318.51Hz*/   76
	Fa4H, ///*1396.91Hz*/   72
	So5H, ///*1567.98Hz*/   64
	La6H, ///*1760.00Hz*/   57
	Si7H, ///*1975.53Hz*/   51
	
	Silent,
};
	

//��������, 16b�����ݣ� 
//��λ��1����100ms��һ��һ����1.6s
enum{
	NOBEEP 		= 0x000000,	//��32λ��϶�һЩ
	Void_Bi 	= 0x000003,	//0b 0000 0000 || 0000 0000=0000 0000=0000 0011
	Void_Biii = 0x00000F,	
	Bi_Bi 		= 0x000303,
	Bi_Biii 	= 0x00010F,
	Biii_Biii = 0x000F0F, 
	//��LSB�쵽MSB			<-------
	Bi_Bi_Bi				= 0x010101,	//	###
	Bi_Bi_Biii 			= 0x01010F,	// 	##-
	Bi_Biii_Bi 			= 0x010F01,	//	#-#
	Bi_Biii_Biii		= 0x010F0F,
	Biii_Bi_Bi 			= 0x0F0101,
	Biii_Bi_Biii		= 0x0F010F,
	Biii_Biii_Bi 		= 0x0F0F01,
	Biii_Biii_Biii	=	0x0F0F0F,
	
	//����ĳ�int32λ���� ����Ը���������
};

//����ID	TimeOutError = TOE
typedef enum{
	NoTimeOutErr = 0,
	ChassisGyroTOE,
	ChassisMoto1TOE,
	BrakeMotoTOE,
//	ChassisMoto2TOE,
	ChassisMoto3TOE,
	ChassisMoto4TOE,
	CurrentSampleTOE,
	DbusTOE,
	JudgeTOE,
	GimbalYawTOE,
	GimbalPitTOE,
	PokeMotoTOE,	//С���貦�����
	ErrorListLength,
}TOE_ID_e;

extern void* err_str[];

//#pragma pack(push)
#pragma pack(4)
typedef struct{
	volatile uint32_t last_time;
	volatile uint32_t err_exist	:1;	//1 = err_exist, 0 = everything ok
	volatile uint32_t enable			:1;  //ʹ��λ
	volatile uint32_t warn_pri		:6;  //�����ͻ�Ļ���
	//uint32_t err_id			:8;	//��ʱû�õ�
	volatile uint32_t beep_mask	:24;
	volatile uint32_t delta_time	:16;	//���ھ�����һ�ν��յ�����ʱ��ʱ����
	volatile uint32_t set_timeout:16;
	void (*f_err_deal)(void);//���ڽ������ĺ���ָ��
}ErrorFlagTypedef;
#pragma pack()


typedef struct{
	volatile ErrorFlagTypedef* err_now;
	volatile ErrorFlagTypedef err_list[ErrorListLength];
	char *str;	//for oled display
	TOE_ID_e id;
}GlbRxErrTypeDef;

typedef enum{
	LED_OFF = 0,
	LED_R = (1<<1),				//1
	LED_G = (1<<2),
	LED_B = (1<<0),
}LedShowTypeDef;

//��Щ��������Ҫ��Ϊ�������handler�� �ɺ���ָ����ã����ڻ�ûд
//static void ErrorDealParamInit(void);
//static void UndPanGyroxyRxErrorDeal(void);
//static void minPCRxErrorDeal(void);
//static void CanYawRxErrorDeal(void);
//static void CanPitchRxErrorDeal(void);
//static void RCRxErrorDeal(void);
//static void scanErrorFlag(void);


void global_err_detector_init(void);
//������������뵽�жϻص����ߴ���ʽ�ж�ͬ��������
void err_detector_callback(int err_id);
void err_deadloop(void);
//void err_detector_callback(ErrorFlagTypedef* e);
void StartErrDecetorTask(void const * argument);
extern GlbRxErrTypeDef	gRxErr; 

/*  
*	can����ʧ�� �������취 ��######################��δ�뵽��
*	�ĸ����û�������� �������취 �� ͨ��������� �ͷŵ�� �ջ��߾�û��
*	�����ǹ��� �������취 �� �����߻�������̨�����ǻ��ֽǶ�
*	ң�������� �������취 �� 
1 ��ֱ����ǰײ 
2 ������debus ���̻�һ��ң���� 
3 �����Լ���ң����
*	������� ����or������
*/

#endif
