#ifndef _USER_BUTTON_H_
#define _USER_BUTTON_H_

#include "user_util.h"
#include "main.h"

#define BUT_TIME_DETECT         80

#define BUT_COUNT_HOLD          20
#define BUT_COUNT_FAST          50

#define BUTTON_ENTER      HAL_GPIO_ReadPin(BUTTON_PRESS_4_GPIO_Port, BUTTON_PRESS_4_Pin)
#define BUTTON_UP         HAL_GPIO_ReadPin(BUTTON_PRESS_3_GPIO_Port, BUTTON_PRESS_3_Pin)
#define BUTTON_DOWN       HAL_GPIO_ReadPin(BUTTON_PRESS_2_GPIO_Port, BUTTON_PRESS_2_Pin)
#define BUTTON_ESC        HAL_GPIO_ReadPin(BUTTON_PRESS_1_GPIO_Port, BUTTON_PRESS_1_Pin)

// Button
typedef struct
{
	uint8_t     Value;
	uint32_t    Count;
	uint8_t     Status;     // 0 - ko nhan, 1 - co nhan
	int32_t     Old_value;

	uint32_t    LandMarkPressButton_u32;
	uint32_t    Timeout_NextButton_u32;
}sButtonStruct;


extern sButtonStruct sButton;

/*===============struct, var================*/

typedef enum
{
    _ENTER = 1,
    _UP = 2,
    _DOWN = 4,
    _ESC = 8,
}eButton;



/*============ Function =============*/
uint8_t BUTTON_scan(void);
void BUTTON_Process(void);

void BUTTON_ESC_Process (void);
void BUTTON_Down_Process (void);
void BUTTON_Up_Process (void);
void BUTTON_Enter_Process (void);

#endif /* _USER_BUTTON_H_ */

