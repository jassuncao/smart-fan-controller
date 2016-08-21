/*
 * main.c
 *
 *  Created on: Jan 23, 2016
 *      Author: jassuncao
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "macro_helpers.h"
#include "dht11.h"


#define OUT_BIT 0
#define ZERO_IN_BIT 1
#define LED_BIT 4

#define OUT1 B,0
#define ZERO_IN B,1
//#define PUSH_IN B,2
//#define DHT_PIN B,3
#define LED B,4
//#define AUX_OUT B,4

//#define AUX_OUT B,0

//The mains frequency
#define ONE_SECOND 50

#define INTERVAL_50HZ 50
#define INTERVAL_60HZ 42
#define TRIAC_PULSES 10

#define MS_TO_CYCLES(X) X/20

/*
 * Incremented when zero crossing (every full cycle) and reset when it reaches 50(hz).
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

/*
 * Keeps the number of timer ticks in half a cycle
 */
static volatile uint8_t zc_interval = INTERVAL_50HZ;
//static const uint8_t zc_interval = INTERVAL_50HZ;

static volatile uint8_t fire_triac = 0;
//register uint8_t fire_triac asm("r5");


void timer1_init(){
	TCCR0B = 0;
	TCNT0  = 0;					// Initialize counter value to 0
	TCCR0A = _BV(WGM01); 		// Turn on CTC mode
	OCR0A = 30;					// Set compare Match Register -> Will interrupt every 0.0002 with the prescaler set to 64
	TIMSK0 |= _BV(OCIE0A);		// Enable timer compare A interrupt
	TIFR0  |= _BV(OCF0A);
}

static inline void timer1_start(){
	TCNT0  = 0;
	TCCR0B = _BV(CS01) | _BV(CS00); // Set prescaler to 64
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
	//DIDR0 = _BV(ADC1D); //Disable digital input PB2
}

static uint8_t adc_read(void)
{
	ADCSRA |= _BV(ADEN) | _BV(ADSC);  // Enable ADC and start convertion
	while(ADCSRA & _BV(ADSC))
		;
	ADCSRA &= ~(_BV(ADEN));
	return ADCH;
}


typedef enum {
	State_Idle,
	//State_TurnOn,
	State_Calibration,
	State_PowerOn,
	//State_TurnOff,
} ZeroDetectionState_t;


static volatile ZeroDetectionState_t state = State_Idle;

#ifdef DEBUG2

#define DEBUG_LED_ON()		HIGH(LED)
#define DEBUG_LED_OFF()		LOW(LED)

static inline void turnOn()
{
	HIGH(LED);
}

static inline void turnOff()
{
	LOW(LED);
}


#else

#define DEBUG_LED_ON() HIGH(LED)
#define DEBUG_LED_OFF() LOW(LED)

static inline void turnOn(){
	state = State_Calibration;
}

static inline void turnOff(){
	//state = State_TurnOff;
	state = State_Idle;
}

#endif


static inline uint8_t ldr_read()
{
	uint8_t ldr = adc_read();
	return ldr<200;
}

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


static inline uint8_t secs_counter_read(void)
{
	uint8_t aux;
	aux = secs_counter;
	return aux;
}

#define LDR_PULSE_MAX_TIME 5
static light_state_t light_state = LIGHT_OFF;
static uint8_t light_timer = 0;

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

#define CFG_MODE_ACTIVATION_MAX_TIME 10
#define CFG_MODE_MAX_TIME 10
#define CFG_MODE_PULSES 5
#define HUMIDITY_READ_PERIOD 30
#define LIGHT_DEBOUNCE_PERIOD 60


/*
static void main_state_machine(void)
{
	uint8_t aux;
	uint8_t elapsed;
	switch (main_state) {
		case MAIN_IDLE:
			LOW(LED);
			if(ldr_state_machine()){
				//Pulse ocurred
				pulse_counter = 1;
				main_state = MAIN_COUNTING;
				main_timer = secs_counter_read();
			}
			break;
		case MAIN_COUNTING:
			elapsed = secs_counter_read() - main_timer;
			if(elapsed>PGM_MODE_ACTIVATION_MAX_TIME){
				main_state = MAIN_IDLE;
			}
			else{
				if(ldr_state_machine()){
					pulse_counter++;
					if(pulse_counter>=PGM_MODE_PULSES){
						HIGH(LED);
						main_state = MAIN_PGM_MODE;
						main_timer = secs_counter_read();
						pulse_counter = 0;
					}
				}
			}
			break;
		case MAIN_PGM_MODE:
			elapsed = secs_counter_read() - main_timer;
			if(elapsed>PGM_MODE_MAX_TIME){
				main_state = MAIN_IDLE;
				if(pulse_counter>0){
					//Save settings
					for(char i=0; i<5;++i){
						TOGGLE(LED);
						_delay_ms(250);
					}
				}
			}
			if(ldr_state_machine()){
				pulse_counter++;
			}
			break;
		default:
			main_state = MAIN_IDLE;
			break;
	}

}
*/


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
		cfgByte = 70;//The default mode is humidity with 70 %
		//writeConfiguration(cfgByte);
	}
	return cfgByte;
}

/*
static void delay_cycles(uint8_t cycles){
	uint8_t ts = cycles_counter;
	uint8_t elapsed;
	do {
		elapsed = cycles_counter - ts;
	} while(elapsed<cycles);
}
*/

static void delay_secs(uint8_t secs){
	uint8_t t0 = secs_counter_read();
	uint8_t elapsed;
	do {
		elapsed = secs_counter_read() - t0;
	} while(elapsed<secs);
}


int main(void)
{
	menu_state_t menu_state = MENU_IDLE;
	uint8_t pulse_counter = 0;//Used to count pulses to activate the configuration mode and counting pulses to set the mode
	uint8_t main_timer; //used to measure time between humidity measures
	uint8_t menu_timer = 0; //Used to measure time in seconds, for example in the configuration mode
	/* keeps the coniguration mode where a value below 100 uses humidity as a trigger and the value is the humidity threshold.
	 * A value above 100 means the unit works as a timer activated by light
	 */
	uint8_t cfg_mode;
	light_mode_state_t light_mode_state = LIGHT_MODE_STATE_OFF;

	cli();
	subsec_counter = 0;
	//cycles_counter = 0;
	zc_ticks = 0;

	timer1_init();


	//DDRB = _BV(LED_BIT) | _BV(OUT_BIT);
	//PORTB = _BV(ZERO_IN_BIT) | _BV(OUT_BIT);
	//Setup outputs

	OUTPUT(LED);
	OUTPUT(OUT1);
	HIGH(OUT1);
	//OUTPUT(AUX_OUT);


	//Setup inputs
	INPUT(ZERO_IN);
	HIGH(ZERO_IN);

	GIFR = GIFR;          //Clear interrupt flags
	GIMSK |= _BV(INT0);		//Enable INT0
	MCUCR |= _BV(ISC01);    //INT0 in Falling edge
	sei();

//	while(1){
//		TOGGLE(OUT1);
//		delay_cycles(MS_TO_CYCLES(100));
//	}

//	while(1){
//		TOGGLE(OUT1);
//		delay_secs(5);
//	}
	init_dht11();
	adc_init();
/*
	for(char i=0; i<5; ++i){
		TOGGLE(LED);
		delay_cycles(MS_TO_CYCLES(250));
		//_delay_ms(250);
	}
	LOW(LED);
*/
//	turnOn();
//	while(1){
//
//	}
	//cfg_mode = readConfiguration();
	cfg_mode = 110;

	main_timer = secs_counter_read();
	while(1){
		uint8_t elapsed;
		uint8_t now;
		now = secs_counter_read();
		light_event_t light_event = light_state_machine(now);

		if(menu_state == MENU_IDLE){
			if(cfg_mode <= 100){//Humidity mode
				elapsed = now - main_timer;
				if(elapsed > HUMIDITY_READ_PERIOD){
					int8_t aux = read_humidity();
					if(aux>0){
						//Successful read
						if(aux >= cfg_mode){
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
				if(light_event==LIGHT_EVT_PULSE){
					//Pulse ocurred
					pulse_counter = 1;
					menu_state = MENU_CFG_ACTIVATION;
					menu_timer = now;
				}
				break;
			case MENU_CFG_ACTIVATION:
				elapsed = now - menu_timer;
				if(elapsed > CFG_MODE_ACTIVATION_MAX_TIME){
					menu_state = MENU_IDLE;
					DEBUG_LED_OFF();
				}
				else{
					if(light_event == LIGHT_EVT_PULSE){
						menu_timer = now;
						pulse_counter++;
						if(pulse_counter>=CFG_MODE_PULSES){
							DEBUG_LED_ON();
							menu_state = MENU_CFG_MODE;
							pulse_counter = 0;
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
						//Save settings
						writeConfiguration(cfg_mode);
						DEBUG_LED_OFF();
					}
				}
				if(light_event==LIGHT_EVT_PULSE){
					menu_timer = now;
					pulse_counter++;
				}
				break;
			default:
				menu_state = MENU_IDLE;
				break;
		}
	}
}

ISR(INT0_vect)
{
	//Ignore spurious interrupts
	if(zc_ticks>1 && zc_ticks<25){
		return;
	}

	//cycles_counter++;
	subsec_counter++;
	if(subsec_counter == 50){
		subsec_counter = 0;
		secs_counter++;
	}

	//TOGGLE(LED);
	switch(state){
	case State_PowerOn:
		fire_triac = TRIAC_PULSES;
		break;
	case State_Calibration:
		if(zc_ticks>75 && zc_ticks<110){
			state = State_PowerOn;
			zc_interval = (zc_ticks>>1);
		}
		break;
	default:
		break;
	}
	zc_ticks = 1;
	timer1_start();
}

ISR(TIM0_COMPA_vect) {

//	if(zc_ticks>110){
//		return;
//	}
	zc_ticks++;

	if(state == State_PowerOn){

		if(zc_ticks == zc_interval){
			fire_triac = TRIAC_PULSES;
		}

		if(fire_triac){
			fire_triac--;
			if(fire_triac & 0x1){
				LOW(OUT1);
			}
			else{
				HIGH(OUT1);
			}
		}
	}
}




