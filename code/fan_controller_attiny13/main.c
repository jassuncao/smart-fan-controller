/*
 * main.c
 *
 *  Created on: Jan 23, 2016
 *      Author: jassuncao
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "macro_helpers.h"
#include "dht11.h"

#define OUT1 B,0
#define ZERO_IN B,1
#define LED B,3
//#define DHT_PIN B,4

//The mains frequency
#define ONE_SECOND 50

#define INTERVAL_50HZ 50
#define INTERVAL_60HZ 42
#define TRIAC_PULSES 20

#define MS_TO_CYCLES(X) X/20

typedef enum  {
	LIGHT_OFF,
	LIGHT_COUNT_TIME_ON,
	LIGHT_ON,
} light_state_t;

typedef enum  {
	LIGHT_EVT_IDLE,
	LIGHT_EVT_ON,
	LIGHT_EVT_PULSE,
	LIGHT_EVT_OFF,
} light_event_t;

typedef enum  {
	MENU_IDLE,
	MENU_CFG_ACTIVATION,
	MENU_CFG_MODE,
} menu_state_t;

typedef enum  {
	LIGHT_MODE_STATE_OFF,
	LIGHT_MODE_STATE_DEBOUNCE,
	LIGHT_MODE_STATE_ON
} light_mode_state_t;

/*
 * Incremented when zero crossing (every half cycle) and reset when it reaches 100(hz).
 * We use it to count seconds
 */
register uint8_t subsec_counter asm("r2");

/*
 * Incremented when zero crossing and used to implement delays
 */
//static volatile uint8_t cycles_counter;

/*
 * Incremented when the timer compare A interrupt occurs.
 * Used to measure the number of timer ticks between zero crossings (one full cycle)
 */
register uint8_t zc_ticks asm("r4");
//static volatile uint8_t zc_ticks = 0;

//Incremented every second
static volatile uint8_t secs_counter = 0;

//static volatile uint8_t triac_edges = 0;
register uint8_t triac_edges asm("r5");

static volatile uint8_t triac_on = 0;

//If the light is kept on less than LDR_PULSE_MAX_TIME (seconds) we consider it a pulse
#define LDR_PULSE_MAX_TIME 5
//We enter CFG mode when the light switches CFG_MODE_PULSES times in less than CFG_MODE_ACTIVATION_MAX_TIME
#define CFG_MODE_PULSES 5
#define CFG_MODE_ACTIVATION_MAX_TIME 10

#define CFG_MODE_MAX_TIME 10

#define HUMIDITY_READ_PERIOD 30
//In light mode, we only turn the fan on if the light is kept on for at least LIGHT_DEBOUNCE_PERIOD seconds
#define LIGHT_DEBOUNCE_PERIOD 15

static light_state_t light_state = LIGHT_OFF;

static uint8_t light_timer = 0;

void timer1_init(){
	TCCR0B = 0;
	TCNT0  = 0;					// Initialize counter value to 0
	TCCR0A = _BV(WGM01); 		// Turn on CTC mode
	//TCCR0A |= _BV(COM0A0);		// Enable the Output compare A pin in toggle mode
	OCR0A = 240;				// Set compare Match Register -> Will interrupt every 0.0002 with the prescaler set to 0
	TIMSK0 |= _BV(OCIE0A);		// Enable timer compare A interrupt
	TIFR0  |= _BV(OCF0A);		// Clear the output compare A flag
	//TCCR0B |= _BV(FOC0A);		// Force output compare A ensuring the output pin is in high state (the triac is active low)
}

static inline void timer1_start(){
	TCNT0  = 0;
	TCCR0B = _BV(CS00); // Set prescaler to 0
}

static inline void timer1_stop(){
	TCCR0B = 0;
}

static inline void timer1_restart(){
	TCNT0 = 0;
	TIFR0 |= _BV(OCF0A);
}

static inline void adc_init()
{
	//Set the ADC prescaler to 64 making the ADC run at 150KHz (9.6MHz/64)
	ADCSRA = _BV(ADPS2) | _BV(ADPS1);
	ADMUX  = _BV(ADLAR) | _BV(MUX0); //Use VCC as reference, PB2 as input and Left align the ADC value
	DIDR0 = _BV(ADC1D); //Disable digital input PB2
}

static uint8_t adc_read(void)
{
	ADCSRA |= _BV(ADEN) | _BV(ADSC);  // Enable ADC and start convertion
	while(ADCSRA & _BV(ADSC))
		;
	ADCSRA &= ~(_BV(ADEN));
	return ADCH;
}

static inline uint8_t ldr_read()
{
	uint8_t ldr = adc_read();
	return ldr < 200;
}


#ifdef DEBUG2

#define DEBUG_LED_INIT() OUTPUT(LED)
#define DEBUG_LED_ON()	HIGH(LED)
#define DEBUG_LED_OFF()	 LOW(LED)

static inline void turnOn()
{
	HIGH(LED);
}

static inline void turnOff()
{
	LOW(LED);
}


#else

#define DEBUG_LED_INIT() OUTPUT(LED)
#define DEBUG_LED_ON()	HIGH(LED)
#define DEBUG_LED_OFF() LOW(LED)

static inline void turnOn(){
	triac_on = 1;
}

static inline void turnOff(){
	triac_on = 0;
}

#endif

static inline uint8_t secs_counter_read(void)
{
	return secs_counter;
}

/*
 * light_state_machine polls the LDR and produces the following events: light turned on, light turned off, light pulsed.
 * Returns LIGHT_EVT_IDLE when nothing happened
 */
static light_event_t light_state_machine(const uint8_t now)
{
	light_event_t event = LIGHT_EVT_IDLE;
	uint8_t elapsed;
	uint8_t ldr_on = ldr_read();
	switch(light_state){
	case LIGHT_OFF:
		if(ldr_on){
			//Switch state and start measuring time
			light_state = LIGHT_COUNT_TIME_ON;
			light_timer = now;
		}
		break;
	case LIGHT_COUNT_TIME_ON:
		if(ldr_on){
			elapsed = now - light_timer;
			if(elapsed > LDR_PULSE_MAX_TIME){
				//If the light is on for more that the max pulse time we switch state
				light_state = LIGHT_ON;
				event = LIGHT_EVT_ON;
			}
		}
		else{
			//We got a pulse
			event = LIGHT_EVT_PULSE;
			light_state = LIGHT_OFF;
		}
		break;
	case LIGHT_ON:
		//Keep in this state until the light turns off
		if(!ldr_on){
			light_state = LIGHT_OFF;
			event = LIGHT_EVT_OFF;
		}
		break;
	}
	return event;
}

static uint8_t mul10(uint8_t v)
{
	v = v << 1;
	return v+v+v+v+v;
}

static void writeConfiguration(uint8_t cfgByte)
{
	eeprom_update_byte((uint8_t*)0, 0xAA);//Write magic number
	eeprom_update_byte((uint8_t*)1, cfgByte);
}

static uint8_t readConfiguration()
{
	uint8_t magicNumber;
	uint8_t cfgByte;
	magicNumber = eeprom_read_byte((uint8_t*)0);//Read magic number
	if(magicNumber==0xAA){
		cfgByte = eeprom_read_byte((uint8_t*)1);
	}
	else{
		cfgByte = 50;//The default mode is humidity with 50 %
		//writeConfiguration(cfgByte);
	}
	return cfgByte;
}


static void delay_cycles(uint8_t cycles) {
	uint8_t ts = zc_ticks;
	uint8_t elapsed;
	do {
		sleep_mode();
		elapsed = zc_ticks - ts;
	} while(elapsed < cycles);
}


static void delay_secs(uint8_t secs) {
	uint8_t t0 = secs_counter_read();
	uint8_t elapsed;
	do {
		elapsed = secs_counter_read() - t0;
	} while(elapsed<secs);
}

int main(void)
{
	menu_state_t menu_state = MENU_IDLE;
	light_mode_state_t light_mode_state = LIGHT_MODE_STATE_OFF;
	uint8_t pulse_counter = 0;//Used to count pulses to activate the configuration mode and counting pulses to set the mode
	uint8_t main_timer; //used to measure time between humidity measures
	uint8_t menu_timer = 0; //Used to measure time in seconds, for example in the configuration mode
	/* keeps the coniguration mode where a value below 100 uses humidity as a trigger and the value is the humidity threshold.
	 * A value above 100 means the unit works as a timer activated by light
	 */
	uint8_t cfg_mode;

	cli();

	//cycles_counter = 0;
	zc_ticks = 0;
	subsec_counter = 0;

	//Setup outputs
	DEBUG_LED_INIT();
	OUTPUT(OUT1);
	HIGH(OUT1);

	//Setup inputs
	DIDR0 |= _BV(AIN1D); //Disable digital input buffer in the pin used to detect zero crossing
	ADCSRB &= ~(_BV(ACME)); //Disable the ANA multiplexer
	ACSR &= ~(_BV(ACD)); //Enable the analog comparator by clearing the disable bit
	ACSR |= _BV(ACBG);  // Use the 1.1v bandgap reference as the positive input of the ANA
	ACSR |= _BV(ACIE);  //Enable ANA Interrupt

	GIFR = GIFR;          //Clear interrupt flags

	WDTCR |= _BV(WDTIE); // Enable watchdog timer interrupts. Using the minimum timeout 16ms
	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();

	timer1_init();
	adc_init();
	init_dht11();

	HIGH(LED);
	sleep_mode();
	LOW(LED);
	sleep_mode();
	HIGH(LED);
	sleep_mode();
	LOW(LED);

	//clock_prescale_set(clock_div_16);  //slow down to 500kHz

	cfg_mode = readConfiguration();
	//cfg_mode = 110;
	//cfg_mode = 80;
	main_timer = secs_counter_read();
	while(1){
		sleep_mode();
		uint8_t elapsed;
		uint8_t now;
		now = secs_counter_read();
		light_event_t light_event = light_state_machine(now);

		if(menu_state == MENU_IDLE){
			if(cfg_mode <= 100){//Humidity mode
				elapsed = now - main_timer;
				if(elapsed > HUMIDITY_READ_PERIOD) {
					int8_t humidity = read_humidity();
					if(humidity > 0){
						//Successful read
						if(humidity >= cfg_mode){
							turnOn();
						}
						else{
							turnOff();
						}
					}
					main_timer = now;
				}
			}
			else{//timer mode
				switch(light_mode_state){
					case LIGHT_MODE_STATE_DEBOUNCE:
						if(light_event == LIGHT_EVT_OFF){
							light_mode_state = LIGHT_MODE_STATE_OFF;
						}
						else{
							elapsed = now - main_timer;
							if(elapsed > LIGHT_DEBOUNCE_PERIOD){
								light_mode_state = LIGHT_MODE_STATE_ON;
								turnOn();
							}
						}
						break;
					case LIGHT_MODE_STATE_ON:
						if(light_event == LIGHT_EVT_OFF){
							light_mode_state = LIGHT_MODE_STATE_OFF;
							turnOff();
						}
						break;
					case LIGHT_MODE_STATE_OFF:
					default:
						if(light_event == LIGHT_EVT_ON){
							main_timer = now;
							light_mode_state = LIGHT_MODE_STATE_DEBOUNCE;
						}
					break;
				}
			}
		}

		switch (menu_state) {
			case MENU_IDLE:
				if(light_event == LIGHT_EVT_PULSE){
					//Pulse ocurred
					pulse_counter = 1;
					menu_state = MENU_CFG_ACTIVATION;
					menu_timer = now;
				}
				break;
			case MENU_CFG_ACTIVATION://In this state we count the number of light pulses until we get at least CFG_MODE_PULSES or timeout occurs
				elapsed = now - menu_timer;
				if(elapsed > CFG_MODE_ACTIVATION_MAX_TIME){
					menu_state = MENU_IDLE;
					DEBUG_LED_OFF();
				}
				else{
					if(light_event == LIGHT_EVT_PULSE){
						menu_timer = now;
						pulse_counter++;
						if(pulse_counter >= CFG_MODE_PULSES){
							DEBUG_LED_ON();
							menu_state = MENU_CFG_MODE;
							pulse_counter = 0;
							turnOff();
						}
					}
				}
				break;
			case MENU_CFG_MODE:
				elapsed = now - menu_timer;
				if(elapsed > CFG_MODE_MAX_TIME){
					menu_state = MENU_IDLE;
					if(pulse_counter>0){
						cfg_mode = mul10(pulse_counter);
						DEBUG_LED_OFF();
						while(pulse_counter>0){
							DEBUG_LED_ON();
							_delay_ms(500);
							DEBUG_LED_OFF();
							_delay_ms(500);
							pulse_counter--;
						}
						//Save settings
						writeConfiguration(cfg_mode);

					}
				}
				if(light_event == LIGHT_EVT_PULSE){
					menu_timer = now;
					pulse_counter++;
				}
				break;
			default:
				menu_state = MENU_IDLE;
				break;
		}
	}
//
//	triac_on = 1;
//	while(1){
///*
//		if(ldr_read()){
//			powerOn = 1;
//		}
//		else{
//			powerOn = 0;
//		}
//		*/
//		delay_cycles(10);
//		HIGH(LED);
//		delay_cycles(10);
//		LOW(LED);
//		/*
//		_delay_ms(250);
//		HIGH(LED);
//		_delay_ms(250);
//		LOW(LED);
//		*/
//		/*
//		sleep_mode(); // light up LED for 0.25 seconds to indicate initialization
//		HIGH(LED);
//		sleep_mode(); // light up LED for 0.25 seconds to indicate initialization
//		LOW(LED);
//		*/
//	}

}

ISR(ANA_COMP_vect) {
	++zc_ticks;

	subsec_counter++;
	if(subsec_counter == 100){
		subsec_counter = 0;
		++secs_counter;
	}

	if(triac_on){
		triac_edges = TRIAC_PULSES;
		timer1_start();
	}
}

EMPTY_INTERRUPT(WDT_vect)

ISR(TIM0_COMPA_vect) {
	--triac_edges;
	FAST_TOGGLE(OUT1);
	if(triac_edges == 0){
		timer1_stop();
		HIGH(OUT1);
	}

}
