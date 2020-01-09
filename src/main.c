
#include <zephyr.h>
#include <kernel.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/can.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>

static struct k_fifo can_rx_fifo;
static bool rxWork = false;
struct fifo_can_frame {
	void *fifo_reserved;
	struct zcan_frame frame;
};

#include "motor.h"


#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
#define STATE_POLL_THREAD_STACK_SIZE 512
#define STATE_POLL_THREAD_PRIORITY 2
#define LED_MSG_ID 0x10
#define COUNTER_MSG_ID 0x12345
#define SET_LED 1
#define RESET_LED 0
#define SLEEP_TIME K_MSEC(100)
#define LED_PORT	"GPIOE"
#define LED		13

#define SILNIK1_MSG_ID 0x601
#define COUNTER_MSG_ID 0x581
#define SET_LED 1
#define RESET_LED 0

/* 1000 msec = 1 sec */
#define SLEEP_TIME	2000

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(poll_state_stack, STATE_POLL_THREAD_STACK_SIZE);

struct device *can_dev;



struct k_thread rx_thread_data;
struct k_thread poll_state_thread_data;
struct zcan_work rx_work;
struct k_work state_change_work;
enum can_state current_state;
struct can_bus_err_cnt current_err_cnt;

CAN_DEFINE_MSGQ(counter_msgq, 10);




int init_can(struct device *can_dev)
{
    can_dev = device_get_binding("CAN_0");
	if (!can_dev) {
		can_dev = device_get_binding("CAN_1");
	}
    else
        return 0;

	if (!can_dev) {
		printk("CAN: Device driver not found.\n");
		return -1;
	}

    return 1;
}

void set_speed(struct zcan_frame *frame, int speed)
{
    frame->data[4] = speed;
}

void tx_irq_callback(u32_t error_flags, void *arg)
{
	char *sender = (char *)arg;

	if (error_flags) {
		printk("Callback! error-code: %d\nSender: %s\n",
		       error_flags, sender);
	}
}

void rx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	const struct zcan_filter filter = {
			.id_type = CAN_STANDARD_IDENTIFIER,
			.rtr = CAN_DATAFRAME,
			.std_id = COUNTER_MSG_ID,
			.rtr_mask = 1,
			.std_id_mask = CAN_EXT_ID_MASK
	};

	struct zcan_frame msg;
	struct fifo_can_frame frame;
	int filter_id;

	filter_id = can_attach_msgq(can_dev, &counter_msgq, &filter);
	printk("Counter filter id: %d\n", filter_id);
	if(filter_id < 0){
		printk("Unable to attach isr [%d]", filter_id);
	}
	rxWork = true;
	while (rxWork) {
		k_msgq_get(&counter_msgq, &msg, K_FOREVER);
		frame.frame = msg;
		k_fifo_put(&can_rx_fifo, &frame);

	}
}

void change_led(struct zcan_frame *msg, void *led_dev_param)
{
	struct device *led_dev = (struct device *)led_dev_param;


	if (!led_dev_param) {
		printk("No LED GPIO device\n");
		return;
	}

	switch (msg->data[0]) {
	case SET_LED:
		gpio_pin_write(led_dev, 14, 1);
		break;
	case RESET_LED:
		gpio_pin_write(led_dev, 14, 0);
		break;
	}
}

char *state_to_str(enum can_state state)
{
	switch (state) {
	case CAN_ERROR_ACTIVE:
		return "error-active";
	case CAN_ERROR_PASSIVE:
		return "error-passive";
	case CAN_BUS_OFF:
		return "bus-off";
	default:
		return "unknown";
	}
}

void poll_state_thread(void *unused1, void *unused2, void *unused3)
{
	struct can_bus_err_cnt err_cnt = {0, 0};
	struct can_bus_err_cnt err_cnt_prev = {0, 0};
	enum can_state state_prev = CAN_ERROR_ACTIVE;
	enum can_state state;

	while (1) {
		state = can_get_state(can_dev, &err_cnt);
		if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
		    err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt ||
		    state_prev != state) {

			err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
			err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
			state_prev = state;
			printk("state: %s\n"
			       "rx error count: %d\n"
			       "tx error count: %d\n",
			       state_to_str(state),
			       err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);
		} else {
			k_sleep(K_MSEC(100));
		}
	}
}


void state_change_work_handler(struct k_work *work)
{
	printk("State Change ISR\nstate: %s\n"
	       "rx error count: %d\n"
	       "tx error count: %d\n",
		state_to_str(current_state),
		current_err_cnt.rx_err_cnt, current_err_cnt.tx_err_cnt);

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	if (current_state == CAN_BUS_OFF) {
		printk("Recover from bus-off\n");

		if (can_recover(can_dev, K_MSEC(100) != 0)) {
			printk("Recovery timed out\n");
		}
	}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */
}

void state_change_isr(enum can_state state, struct can_bus_err_cnt err_cnt)
{
	current_state = state;
	current_err_cnt = err_cnt;
	k_work_submit(&state_change_work);
}

void main(void)
{
	k_fifo_init(&can_rx_fifo);

	u32_t cnt = 0;
	struct device *dev;
	struct device *dev1;

	dev = device_get_binding(LED_PORT);
	dev1 = device_get_binding("GPIOE");
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED, GPIO_DIR_OUT);
	gpio_pin_configure(dev1, 14, GPIO_DIR_OUT);
	const struct zcan_filter change_led_filter = {
			.id_type = CAN_STANDARD_IDENTIFIER,
			.rtr = CAN_DATAFRAME,
			.std_id = LED_MSG_ID,
			.rtr_mask = 1,
			.std_id_mask = CAN_STD_ID_MASK
		};
		struct zcan_frame change_led_frame = {
			.id_type = CAN_STANDARD_IDENTIFIER,
			.rtr = CAN_DATAFRAME,
			.std_id = LED_MSG_ID,
			.dlc = 1
		};
		struct zcan_frame counter_frame = {
				.id_type = CAN_EXTENDED_IDENTIFIER,
					.rtr = CAN_DATAFRAME,
					.ext_id = COUNTER_MSG_ID,
					.dlc = 2
		};


		u8_t toggle = 1;
		u16_t counter = 0;
		struct device *led_gpio_dev = NULL;
		k_tid_t rx_tid, get_state_tid;
		int ret;

		/* Usually the CAN device is either called CAN_0 or CAN_1, depending
		 * on the SOC. Let's check both and take the first valid one.
		 */
		can_dev = device_get_binding("CAN_0");
		if (!can_dev) {
			can_dev = device_get_binding("CAN_1");
		}

		if (!can_dev) {
			printk("CAN: Device driver not found.\n");
			gpio_pin_write(dev1, 14, 0);
		} else {
			gpio_pin_write(dev1, 14, 1);
		}


/*#ifdef CONFIG_LOOPBACK_MODE
	can_configure(can_dev, CAN_LOOPBACK_MODE, 125000);
#endif*/
#ifdef CONFIG_NORMAL_MODE
	can_configure(can_dev, CAN_NORMAL_MODE, 125000);
#endif

	k_work_init(&state_change_work, state_change_work_handler);

	ret = can_attach_workq(can_dev, &k_sys_work_q, &rx_work, change_led,
			       led_gpio_dev, &change_led_filter);
	if (ret == CAN_NO_FREE_FILTER) {
		printk("Error, no filter available!\n");
		return;
	}

//	printk("Change LED filter ID: %d\n", ret);

	rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
				 K_THREAD_STACK_SIZEOF(rx_thread_stack),
				 rx_thread, NULL, NULL, NULL,
				 RX_THREAD_PRIORITY, 0, K_NO_WAIT);
	if (!rx_tid) {
		printk("ERROR spawning rx thread\n");
	}
	get_state_tid = k_thread_create(&poll_state_thread_data,
					poll_state_stack,
					K_THREAD_STACK_SIZEOF(poll_state_stack),
					poll_state_thread, NULL, NULL, NULL,
					STATE_POLL_THREAD_PRIORITY, 0,
					K_NO_WAIT);
	if (!get_state_tid) {
		printk("ERROR spawning poll_state_thread\n");
	}

	can_register_state_change_isr(can_dev, state_change_isr);

	/* This sending call is none blocking. */

	struct motor_dev motor1;
	motor_init(&motor1, 1, can_dev);
	motor_disable(&motor1, can_dev);
	motor_svel_feedback_encoder(&motor1, can_dev);
	motor_set_encoder_resolution(&motor1, can_dev, 2000);
	motor_mode(&motor1, can_dev, MOTOR_MODE_POS);
	motor_set_velocity(&motor1, can_dev, 500);
	motor_set_factor_group(&motor1, can_dev, 3);
	motor_enable(&motor1, can_dev);
	motor_set_actual_position(&motor1, can_dev, 0);
	motor_movr(&motor1, can_dev, 10);
	k_sleep(3000);
	motor_movr(&motor1, can_dev, -10);

	/*motor_mode(&motor1, can_dev, MOTOR_MODE_VEL);
	motor_enable(&motor1, can_dev);

	motor_set_velocity(&motor1, can_dev, 0);
	motor_set_svelocity(&motor1, can_dev, -100);
	motor_set_position(&motor1, can_dev, -100);
	motor_set_current(&motor1, can_dev, -100);
	motor_set_velocity(&motor1, can_dev, -100);
	int32_t i = 0;
	get_motor_setpoints(&motor1, can_dev);
	get_motor_actual_values(&motor1, can_dev);
	get_motor_actual_limits(&motor1, can_dev);*/

	while (1) {
		gpio_pin_write(dev, LED, cnt % 2);
		cnt++;
		k_sleep(500);
		/*motor_set_velocity(&motor1, can_dev, 100);
		k_sleep(3000);
		motor_set_velocity(&motor1, can_dev, 0);
		k_sleep(3000);
		motor_set_velocity(&motor1, can_dev, -100);
		k_sleep(3000);
		motor_set_velocity(&motor1, can_dev, 0);
		k_sleep(3000);*/
		//if(  cnt % 2) motor_set_velocity(&motor1, can_dev, -100);
		//else motor_set_velocity(&motor1, can_dev, 100);

	}
}
