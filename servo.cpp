#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "servo.h"

#define MAX_SERVO 16

int8_t servo_running = 0;
int8_t counter = 0;

int16_t servo_values[MAX_SERVO];

void send_servo_update() {
	if ( (counter++) % 2 == 0) {
		gpioServo(SERVO_X, servo_values[SERVO_X]);
	} else {
		gpioServo(SERVO_Y, servo_values[SERVO_Y]);
	}
}

void interrupt_stop(int32_t sigNum) {
	servo_running = 0;
}

void check_bounds(int8_t pin) {
	if (servo_values[pin] > 1900) servo_values[pin] = 1900;
	if (servo_values[pin] < 1100) servo_values[pin] = 1100;
}

void servo_init(void) {
	if( gpioInitialise() < 0) {
		printf("GPIO init failed");
		exit(EXIT_FAILURE);
	}

	servo_running = 1;
	
	gpioSetSignalFunc(SIGINT, interrupt_stop);
}

void servo_destroy(void) {
	gpioTerminate();
}

void servo_set( int8_t pin, int16_t value ) {
	servo_values[pin] = value;
	check_bounds(pin);
}

int16_t servo_set_delta( int8_t pin, int16_t delta ) {
	servo_values[pin] += delta;
	check_bounds(pin);
	return servo_values[pin];
}

int8_t servo_update(void) {
	send_servo_update();
	return servo_running;
}

