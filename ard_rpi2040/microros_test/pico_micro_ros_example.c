#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "hardware/uart.h"
#include "hardware/irq.h"


/// \tag::uart_advanced[]

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY	UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define STEP_PIN 8
#define DIR_PIN 9
#define MM 16
#define STEPS_PER_REVOLUTION (200 * MM)
// diam in m
#define WDIAM 0.06
#define WCIRCUMFERENCE (WDIAM * M_PI)
#define STEPS_PER_METER (STEPS_PER_REVOLUTION / WCIRCUMFERENCE)

static int chars_rxed = 0;

// RX interrupt handler
void on_uart_rx() {
	while (uart_is_readable(UART_ID)) {
		uint8_t ch = uart_getc(UART_ID);
		// Can we send it back?
		if (uart_is_writable(UART_ID)) {
			// Change it slightly first!
			ch++;
			uart_putc(UART_ID, ch);
		}
		chars_rxed++;
	}
}

void custom_uart_init()
{
	// Set up our UART with a basic baud rate.
	uart_init(UART_ID, 115200);

	// Set the TX and RX pins by using the function select on the GPIO
	// Set datasheet for more information on function select
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

	// Actually, we want a different speed
	// The call will return the actual baud rate selected, which will be as close as
	// possible to that requested
	int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

	// Set UART flow control CTS/RTS, we don't want these, so turn them off
	uart_set_hw_flow(UART_ID, false, false);

	// Set our data format
	uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

	// Turn off FIFO's - we want to do this character by character
	uart_set_fifo_enabled(UART_ID, false);

	// Set up a RX interrupt
	// We need to set up the handler first
	// Select correct interrupt for the UART we are using
	int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

	// And set up and enable the interrupt handlers
	irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
	irq_set_enabled(UART_IRQ, true);

	// Now enable the UART to send interrupts - RX only
	uart_set_irq_enables(UART_ID, true, false);


}

enum states {
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
} state;

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
//rclc_executor_t executor_sub;
//rclc_executor_t executor_speed_sub;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg, omsg;
std_msgs__msg__Float64 smsg;
rcl_subscription_t subscriber, speed_subscriber;

const uint LED_PIN = 25;

static volatile long counter;

int64_t vel2freq(double v)
{
	return (int64_t)(v / WCIRCUMFERENCE * STEPS_PER_REVOLUTION);
}

void set_speed(double v)
{
	float freq = vel2freq(v);
	char buf[40];
	snprintf(buf, sizeof(buf), "\nFreq: %g\n", freq);
	uart_puts(UART_ID, buf);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
	msg.data++;
	gpio_put(LED_PIN, msg.data % 2);
	char buf[40];
	snprintf(buf, sizeof(buf), "\nUART interrupt, counter: %ld\n", counter);
	uart_puts(UART_ID, buf);
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
	char buf[40];
	snprintf(buf, sizeof(buf), "\nMessage received: %d\n", msg->data);
	uart_puts(UART_ID, buf);
}

void speed_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;
	char buf[40];
	snprintf(buf, sizeof(buf), "\nSet speed to: %g\n", msg->data);
	uart_puts(UART_ID, buf);
	set_speed(msg->data);
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)	do { \
	static volatile int64_t init = -1; \
	if (init == -1) { init = uxr_millis();} \
	if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)


bool create_entities()
{
	counter = 0;

	allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "pico_example", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_best_effort(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"pico_message"));

// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"pico_command"));

	RCCHECK(rclc_subscription_init_default(
		&speed_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
		"speed"));

	// create timer,
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &omsg, &subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &speed_subscriber, &smsg, &speed_subscription_callback, ON_NEW_DATA));

	msg.data = 0;

	return true;
}

void destroy_entities()
{
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	rcl_publisher_fini(&publisher, &node);
	rcl_timer_fini(&timer);
	rclc_executor_fini(&executor);
	rcl_subscription_fini(&subscriber, &node);
	rcl_subscription_fini(&speed_subscriber, &node);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}

bool counter_timer_callback(repeating_timer_t *rt) {
	counter++;
	gpio_put(STEP_PIN, counter % 2);
	return true; // keep repeating
}



int main()
{

	state = WAITING_AGENT;
	rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_set_dir(STEP_PIN, GPIO_OUT);
	gpio_set_dir(DIR_PIN, GPIO_OUT);

	custom_uart_init();

	gpio_put(LED_PIN, 1);

	gpio_put(DIR_PIN, 0);

	repeating_timer_t counter_timer;
	add_repeating_timer_us(-1000, counter_timer_callback, NULL, &counter_timer);

	while (true)
	{
		switch (state) {
			case WAITING_AGENT:
				EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
				break;
			case AGENT_AVAILABLE:
				state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
				if (state == WAITING_AGENT) {
					destroy_entities();
				};
				break;
			case AGENT_CONNECTED:
				EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
				if (state == AGENT_CONNECTED) {
					rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
				}
				break;
			case AGENT_DISCONNECTED:
				destroy_entities();
				state = WAITING_AGENT;
				break;
			default:
				break;
		}
	}
	return 0;
}

