#include "stdio.h"
#include "app.h"


//Humda
float SPEKTRUM_THROTTLE_MIN = 157.0;
float SPEKTRUM_THROTTLE_MAX = 869.0;
float SPEKTRUM_THROTTLE_NEUTRAL = 511.0;

//Humda
float SPEKTRUM_STEER_MIN = 153.0;
float SPEKTRUM_STEER_MAX = 870.0;
float SPEKTRUM_STEER_NEUTRAL = 464.0;

//Humda
float SPEKTRUM_MODE_MIN = 153.0;
float SPEKTRUM_MODE_MAX = 870.0;
float SPEKTRUM_MODE_NEUTRAL = 507.0;
float SPEKTRUM_MODE_AUTONMOUS_ZONE = 650.0;

//Humda
float SPEKTRUM_REVERSE_MIN = 152.0;
float SPEKTRUM_REVERSE_MAX = 870.0;
float SPEKTRUM_REVERSE_NEUTRAL = 507.0;
float SPEKTRUM_REVERSE_ZONE = 200.0;


#define JOY_STEERING_INDEX 1
#define JOY_THROTTLE_INDEX 0

//Humda
#define JOY_MODE_INDEX 3
#define JOY_REVERSE_INDEX 2
#define JOY_EMK_INDEX 4


float SPEKTRUM_EMK_MIN = 153.0;
float SPEKTRUM_EMK_MAX = 869.0;
float SPEKTRUM_EMK = 869.0;

//#define JOY_MODE_INDEX 5

#define MEASURE_TIME_BETWEEN_PACKETS 1

extern volatile uint32_t ctrl_connected;
extern volatile uint32_t ctrl_connection_counter;
extern volatile uint32_t prev_time;

extern void send_diag_rc();

//int prev_time = 0;

static void joy_steer_to_steer(app_state_t *app)
{
	int steer_val = app->rc_receiver_state.channels[JOY_STEERING_INDEX].servo_position;

	if (steer_val > SPEKTRUM_STEER_NEUTRAL)
	{
		app->steer_percent = (steer_val - SPEKTRUM_STEER_NEUTRAL) / (SPEKTRUM_STEER_MAX - SPEKTRUM_STEER_NEUTRAL);
	}
	else
	{
		app->steer_percent = -(SPEKTRUM_STEER_NEUTRAL - steer_val) / (SPEKTRUM_STEER_NEUTRAL - SPEKTRUM_STEER_MIN);
	}
}

static void joy_acc_to_acc(app_state_t *app)
{
	int acc_val = app->rc_receiver_state.channels[JOY_THROTTLE_INDEX].servo_position;
	float acc_percent = (acc_val - SPEKTRUM_THROTTLE_NEUTRAL) / (SPEKTRUM_THROTTLE_MAX - SPEKTRUM_THROTTLE_NEUTRAL);

	app->acc_percent = acc_percent;

	//Toggle LD1 LED on board to indicate of acc is requested
	if (acc_percent < 0.0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	}
}

static void joy_control_to_control(app_state_t *app)
{
	int control_val = app->rc_receiver_state.channels[JOY_MODE_INDEX].servo_position;

//	if (control_val == 1706)
//	{
//		app->control_mode = 2;
//	}
//	else
	if (control_val > SPEKTRUM_MODE_AUTONMOUS_ZONE)
	{
		app->control_mode = 1;
	}
	else //if (control_val < SPEKTRUM_MODE_AUTONMOUS_ZONE)
	{
		app->control_mode = 0;
	}

	//emergency stop from remote Fmode switch is used
	int deadmanswitch = app->rc_receiver_state.channels[JOY_EMK_INDEX].servo_position;

	if (deadmanswitch == SPEKTRUM_EMK)
	{
		app->gokart_EM_status = 1;
	}
	else //if (deadmanswitch == 153)
	{
		app->gokart_EM_status = 0;
	}

	//reverse
	int reverse = app->rc_receiver_state.channels[JOY_REVERSE_INDEX].servo_position;

	if (reverse < SPEKTRUM_REVERSE_ZONE)
	{
		app->gokart_reverse = 1;
	}
	else
	{
		app->gokart_reverse = 0;
	}

}

static void convert_channels_to_commands(app_state_t *app)
{
	joy_steer_to_steer(app);
	joy_acc_to_acc(app);
	joy_control_to_control(app);
}

static void handle_spektrum_msg(const spektrum_internal_msg_t *msg, void *context)
{
	app_state_t *app = (app_state_t *)context;
	spektrum_msg_to_state(msg, &app->rc_receiver_state, (long)HAL_GetTick());
	convert_channels_to_commands(app);
	ctrl_connected = 1;
	if (ctrl_connection_counter < CTRL_SATURATION_CTR)
		ctrl_connection_counter++;
#if MEASURE_TIME_BETWEEN_PACKETS
	printf("\r\n Time: %d", HAL_GetTick()-prev_time);
	prev_time = HAL_GetTick();
	send_diag_rc(app->rc_receiver_state.channels[0].servo_position,
			app->rc_receiver_state.channels[1].servo_position,
			app->rc_receiver_state.channels[2].servo_position,
			app->rc_receiver_state.channels[3].servo_position);
#endif
}

void app_run(app_state_t *app)
{
	app->steer_percent = 0.0;
	app->acc_percent = 0.0;
	app->control_mode = 0;
	app->gokart_EM_status = 0;
	app->gokart_reverse = 0;

	spektrum_nucleo_state_t *rc_receiver = &app->rc_receiver;

	spektrum_nucleo_init(rc_receiver);
	spektrum_nucleo_start_receiving(SPEKTRUM_UART);

	rc_receiver->msg_handler = handle_spektrum_msg;
	rc_receiver->msg_handler_context = app;
}
