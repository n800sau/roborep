#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
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


const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
	msg.data++;
	gpio_put(LED_PIN, msg.data % 2);
	uart_puts(UART_ID, "\nHello, uart interrupts\n");
}

int main()
{

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

	rcl_timer_t timer;
	rcl_node_t node;
	rcl_allocator_t allocator;
	rclc_support_t support;
	rclc_executor_t executor;

	allocator = rcl_get_default_allocator();

	// Wait for agent successful ping for 2 minutes.
	const int timeout_ms = 1000; 
	const uint8_t attempts = 120;

	rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

	if (ret != RCL_RET_OK)
	{
		// Unreachable agent, exiting program.
		return ret;
	}

	rclc_support_init(&support, 0, NULL, &allocator);

	rclc_node_init_default(&node, "pico_node", "", &support);
	rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"pico_publisher");

	rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(1000),
		timer_callback);

	rclc_executor_init(&executor, &support.context, 1, &allocator);
	rclc_executor_add_timer(&executor, &timer);

	custom_uart_init();

	gpio_put(LED_PIN, 1);

	msg.data = 0;
	while (true)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	}
	return 0;
}
