

#include "gpio.h"
//blue control by pwm!
#define LED_G_ON	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)	
#define LED_G_OFF	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_B_ON	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET)
#define LED_B_OFF	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET)
#define LED_R_ON	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET)
#define LED_R_OFF	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET)

//#define LED_ON_BOARD_ON		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
//#define LED_ON_BOARD_OFF	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)

//#define BELL_ON			HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_SET)	
//#define BELL_OFF		HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_RESET)	


void ledShowError(void);
//void set_led(char color, char isLight){
//	
//}

void led_fly_status(char is_ready);

