

#include "DogConfig.h"
//��2��3510���

#include "bullet_task.h"

//�������µ��ݵ���λ����
#define READ_UP_LIMIT				HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
#define READ_DN_LIMIT				HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
#define	READ_INSIDE_LIMIT		HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
#define READ_OUTSIDE_LIMIT	HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

enum{
	Supply = 0,
	Lift = 1, 
};

s16 speed[4] = {0};


void bullets_supply_init(){
	Moto3510Measure[0].total_angle = 0;
	Moto3510Supply.total_angle = 0;
	Moto3510Lift.total_angle = 0;
}

void self_check(){
	
	speed[Supply] = 1000;
//	while(
}

void StartBulletSupplyTask(const void *argu){
	bullets_supply_init();
	
	for(;;){
		
		
		//CAN_Send_Message(&hcan1, CAN_3510Moto_four_ID, speed);
		osDelay(100);
	}

}
