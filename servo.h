#include <pigpio.h>

#define SERVO_X 14
#define SERVO_Y 15

void servo_set( int8_t pin, int16_t value );
int16_t servo_set_delta( int8_t pin, int16_t delta );
int8_t servo_update(void);
void servo_init(void);
void servo_destroy(void);
