#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include <usart.h>
#include <PID.h>

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/sfr_defs.h>
#include <math.h>

#define SPEED_OF_SOUND_MM_US 0.343f // Speed of sound (mm/uS)

#define WATER_TANK_HEIGHT_MM 347.0f // mm
#define WATER_LEVEL_MIN 10.0f
#define WATER_LEVEL_MAX 330.0f
// #define TARGET_WATER_LEVEL 80.0f // mm - Set target water level (setpoint)
#define NUM_MEASUREMENTS 7

#define INTERVAL_MS 60

// Controller parameters
#define PID_KP 1.0f	 // 0.576f
#define PID_KI 0.1f	 // 0.0576f //0.0005f // 0.0442368f
#define PID_KD 0.05f // 25f // 0.0288f
#define PID_TAU 0.2f
#define PID_LIM_MIN -20.0f
#define PID_LIM_MAX 20.0f
#define PID_LIM_MIN_INT -10.0f
#define PID_LIM_MAX_INT 10.0f
#define SAMPLE_TIME_S 0.48f // 1.0, 0.06, 0.12, 0.24

void show_menu(void);
void sensor_init(void);
void timer_init(void);
uint32_t get_water_level(void); // Was uint_32 before
void update_target_water_level(void);

volatile unsigned long millis_counter = 0;

static volatile uint32_t pulse_start = 0;
static volatile uint32_t pulse_end = 0;
static volatile uint32_t duration = 0;

uint32_t target_water_level = 110.0;
float measured_water_level, control_output, error_out;

const char fdata1[] PROGMEM = "Sonar Test\n\n\r";

void show_menu(void)
{
	usart_prints("-------------------------------------------------\n\r");
	usart_printf(fdata1);
	usart_prints("\n\r");
}

void sensor_init(void)
{
	cli();

	// Set PD6 as output - Trig pin
	DDRD |= (1 << 6);

	/*
	Timer0 generates pulse that is sent to Trig pin, initiating ultrasonic pulse from sensor
	Timer0 set to fast PWM configuration
	*/
	TCCR0A |= (1 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << CS01); // Prescaler 8 for Timer0

	//  Desired pulse width is 10uS => (10 uS / (16 MHz / 8)) - 1 ≈ 125 - 1 = 124
	OCR0A = 235;

	/*
	Timer1 used to measure pulse width of Echo pulse. When Echo pulse is detected it stores the time of the rising edge and time of falling edge
	Timer1 set to normal mode
	*/
	DDRB &= ~(1 << DDB0); // Set PB0 (ICP1) as input - Echo pin
	TCCR1B |= (1 << ICNC1) | (1 << ICES1) | (1 << CS11);
	sei();
	TIMSK1 |= (1 << ICIE1); // Enable Timer1 input capture interrupt
}

void timer_init()
{
	// Set up Timer2 for 1ms interrupts
	TCCR2A = 0; // Set entire TCCR2A register to 0
	TCCR2B = 0; // Same for TCCR2B
	TCNT2 = 0;	// Initialize counter value to 0
	// set compare match register for 1ms increments
	OCR2A = 249; // = (16e6 / (1000 * 64)) - 1 (must be <256)
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// Set CS22 and CS21 bits for 64 prescaler
	TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);
}

static uint32_t measurements[NUM_MEASUREMENTS];
static uint8_t measurement_index = 0;
float prev_measurement = 0.0;
float difference = 0;

uint32_t get_water_level() //  Was uint32 before
{
	static uint32_t echo_pulse_uS; // Both were uint32 before
	static uint32_t distance_mm;
	echo_pulse_uS = (float)duration * 32768.0 / 65536.0;	  // Convert pulse duration to microseconds
															  // 32768 uS = 65536 clock ticks for Timer1 with prescale 8
	distance_mm = echo_pulse_uS * SPEED_OF_SOUND_MM_US / 2.0; // Calculate distance using speed of sound

	if (prev_measurement != 0)
	{
		difference = fabs(distance_mm - prev_measurement);
	}

	int value = rand() % 500;
	if (difference > 2 | value == 225)
	{
		distance_mm = prev_measurement;
	}
	else
	{
		prev_measurement = distance_mm;
	}

	// Store the current measurement
	measurements[measurement_index] = distance_mm;
	measurement_index = (measurement_index + 1) % NUM_MEASUREMENTS;

	// Calculate the average of the last NUM_MEASUREMENTS
	uint32_t sum = 0;
	for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++)
	{
		sum += measurements[i];
	}
	uint32_t average_distance_mm = sum / NUM_MEASUREMENTS;

	// return average_distance_mm;
	return distance_mm;
}

void update_target_water_level(void)
{
	if (bit_is_clear(PINB, 4) && bit_is_clear(PINB, 5))
	{
		while (bit_is_clear(PINB, 4) && bit_is_clear(PINB, 5))
		{
		}

		// Clockwise
		if (bit_is_clear(PINB, 5))
		{
			if (target_water_level != 0)
			{
				target_water_level++;
			}
		}
		// Counter clockwise
		else if (bit_is_clear(PINB, 4))
		{
			target_water_level--;
		}
		if (target_water_level > WATER_LEVEL_MAX)
		{
			target_water_level = WATER_LEVEL_MAX;
		}
		if (target_water_level < WATER_LEVEL_MIN)
		{
			target_water_level = WATER_LEVEL_MIN;
		}
	}
}

void sendUSART(float measured_water_level, float error_out, float control_output, float target_water_level)
{
	char water_level_buffer[20];
	int water_level_int = (int)measured_water_level;
	itoa(water_level_int, water_level_buffer, 10);

	int error = (int)error_out;
	char error_buffer[20];
	itoa(error, error_buffer, 10);

	char control_output_buffer[20];
	int control_output_int = (int)control_output;
	itoa(control_output_int, control_output_buffer, 10);

	char target_water_level_buffer[20];
	int target_water_level_int = (int)target_water_level;
	itoa(target_water_level_int, target_water_level_buffer, 10);

	char output_buffer[50];
	sprintf(output_buffer, "%s,%s,%s,%s", water_level_buffer, control_output_buffer, error_buffer, target_water_level_buffer);

	usart_prints(output_buffer);
	usart_putc('\n');
}

ISR(TIMER1_CAPT_vect)
{
	if ((TCCR1B & (1 << ICES1)) == (1 << ICES1)) // Checks for rising edge
	{
		pulse_start = ICR1; // Store start time of pulse
	}
	else
	{
		pulse_end = ICR1; // Store end time of pulse
	}

	if (pulse_start != 0 && pulse_end != 0)
	{
		duration = pulse_end - pulse_start; // Get duration from pulse_start to pulse_end
		pulse_start = 0;					// Reset pulse_start
		pulse_end = 0;						// Reset pulse_end
	}

	TCCR1B ^= (1 << ICES1); // Toggle edge detection bit
	TIFR1 = (1 << ICF1);	// Clear Input Capture Flag
}

ISR(TIMER2_COMPA_vect)
{
	millis_counter++;
}

ISR(TIMER0_OVF_vect) {}

unsigned long millis()
{
	unsigned long ms;
	cli();
	ms = millis_counter;
	sei();
	return ms;
}

void delay(unsigned long ms)
{
	unsigned long start = millis();
	while (millis() - start < ms)
		;
}

int main(void)
{
	DDRC |= (1 << 3) | (1 << 4); // Set LEDs to be output
	DDRB |= (0 << 4) | (0 << 5); // Set PB4 and PB5 as input for RPG
	// PORTC |= (1 << PORT3);
	// PORTC |= (1 << PORT4);
	// PORTC |= (0 << PORT3);
	// PORTC |= (0 << PORT4);

	usart_init(); // Initialzie USART
	timer_init();
	sensor_init(); // Initialize ultrasonic sensor
	sei();
	// show_menu();

	// Initialize PID controller
	PIDController pid = {PID_KP, PID_KI, PID_KD,
						 PID_TAU,
						 PID_LIM_MIN, PID_LIM_MAX,
						 PID_LIM_MIN_INT, PID_LIM_MAX_INT,
						 SAMPLE_TIME_S};

	PIDController_Init(&pid);

	unsigned long previous_millis = 0.0;

	while (1)
	{
		unsigned long current_millis = millis();

		update_target_water_level();

		if (current_millis - previous_millis >= INTERVAL_MS)
		{
			previous_millis = current_millis;

			uint32_t measured_water_level = WATER_TANK_HEIGHT_MM - get_water_level();
			// float measured_water_level = 50.0;

			// Gets water level
			if (measured_water_level > 0.0 && measured_water_level < 130.0)
			{

				float error_out;

				// Update PID controller with setpoint and measured water level
				PIDController_Update(&pid, target_water_level, measured_water_level, &error_out);
				float control_output = pid.out;

				// When PID output is positive, there is not enough water
				if (control_output > 1.5)
				{
					// Turn on red LED - Turn off green LED - ADD WATER
					PORTC &= ~(1 << PORT4);
					PORTC |= (1 << PORT3);
				}
				// When PID output is negative, there is too much water
				else if (control_output < -1.5)
				{
					// Turn on green LED - Turn off red LED - REMOVE WATER
					PORTC &= ~(1 << PORT3);
					PORTC |= (1 << PORT4);
				}
				else
				{
					// Turn off both pumps (LEDs)
					PORTC &= ~(1 << PORT3);
					PORTC &= ~(1 << PORT4);
				}

				sendUSART(measured_water_level, error_out, control_output, target_water_level);
				usart_clear();
			}

			// _delay_ms(60);
			// delay(INTERVAL_MS);
		}
	}
}
