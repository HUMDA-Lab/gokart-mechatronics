#ifndef _APP_H
#define _APP_H

#include "spektrum_nucleo.h"

typedef struct app_state {

	float acc_percent;
	float steer_percent;

	int control_mode;
	int gokart_EM_status;
	//0 forward, 1 reverse
	int gokart_reverse;

	spektrum_nucleo_state_t rc_receiver;
	spektrum_state_t rc_receiver_state;

} app_state_t;

void app_run(app_state_t *app);

extern app_state_t *main_app;
extern UART_HandleTypeDef huart2;

#define SPEKTRUM_UART &huart2

#define CTRL_SATURATION_CTR 50
#define CTRL_SATURATION_THRESHOLD 40

#endif
