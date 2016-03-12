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


#define OUT1 B,0
#define ZERO_IN B,1
//#define PUSH_IN B,2
//#define DHT_PIN B,3
#define LED B,4

//#define AUX_OUT B,4

//The mains frequency
#define ONE_SECOND 50
//Incremented in zero crossing
static uint8_t subsec_counter = 0;
//Incremented every second
static volatile uint8_t secs_counter = 0;

static volatile uint8_t turn_on_flag = 0;

void timer1_init(){
	//cli();
	TCCR0B = 0;
	TCNT0  = 0;					// Initialize counter value to 0
	TCCR0A = _BV(WGM01); 		// Turn on CTC mode
	OCR0A = 30;					// Set compare Match Register -> Will interrupt every 0.0002 with the prescaler set to 64
	TIMSK0 |= _BV(OCIE0A);		// Enable timer compare A interrupt
	//sei();
}

static inline void timer1_start(){
	TCNT0  = 0;
	TCCR0B = _BV(CS01) | _BV(CS00); // Set prescaler to 64
}

static inline void timer1_stop(){
	TCCR0B = 0;
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

#define STATE_UNDEFINED 0
#define STATE_CALIBRATING 1
#define STATE_RUNNING 2
#define INTERVAL_50HZ 45
#define INTERVAL_60HZ 42

typedef enum {
	State_Uncalibrated,
	State_Calibration,
	State_Calibrated
} ZeroDetectionState_t;

static volatile uint8_t zc_interval = INTERVAL_50HZ;
static volatile uint8_t zc_cycles = 0;
static volatile uint8_t fire_triac = 0;
static volatile uint8_t zc_debounce = 0;
static volatile ZeroDetectionState_t state = State_Uncalibrated;

/*
static inline void turnOn(){
	GIMSK |= _BV(INT0);		//Enable INT0
}

static inline void turnOff(){
	state = State_Uncalibrated;
	GIMSK &= ~(_BV(INT0));		//Disable INT0
}
*/

static inline uint8_t ldr_read()
{
	uint8_t ldr = adc_read();
	return ldr<200;
}

typedef enum  {
	LIGHT_IDLE,
	LIGHT_COUNT_TIME_ON,
	LIGHT_ON,
} ldr_state_t;

static inline uint8_t secs_counter_read(void)
{
	uint8_t aux;
	//cli();
	aux = secs_counter;
	//sei();
	return aux;
}

#define LDR_PULSE_MAX_TIME 5
static ldr_state_t ldr_state = LIGHT_IDLE;
static uint8_t ldr_timer = 0;

static uint8_t ldr_state_machine(void)
{
	uint8_t res = 0;
	uint8_t elapsed;
	uint8_t secs = secs_counter_read();
	uint8_t ldr_on = ldr_read();
	switch(ldr_state){
	case LIGHT_IDLE:
		if(ldr_on){
			//Switch state and start measuring time
			ldr_state = LIGHT_COUNT_TIME_ON;
			ldr_timer = secs;
		}
		break;
	case LIGHT_COUNT_TIME_ON:
		if(ldr_on){
			elapsed = secs - ldr_timer;
			if(elapsed>LDR_PULSE_MAX_TIME){
				//If the light is on for more that the max pulse time we switch state
				ldr_state = LIGHT_ON;
			}
		}
		else{
			ldr_state = LIGHT_IDLE;
			//We got a pulse
			res = 1;
		}
		break;
	case LIGHT_ON:
		//Keep in this state until the light turns off
		if(!ldr_on){
			ldr_state = LIGHT_IDLE;
		}
		break;
	}
	return res;
}

typedef enum  {
	MAIN_IDLE,
	MAIN_COUNTING,
	MAIN_PGM_MODE,
} main_state_t;

#define PGM_MODE_ACTIVATION_MAX_TIME 10
#define PGM_MODE_MAX_TIME 10
#define PGM_MODE_PULSES 5

static main_state_t main_state = MAIN_IDLE;
static uint8_t pulse_counter = 0;
static uint8_t main_timer = 0;

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
/*
//Used to test zero detection as clock source
int main(void)
{
	OUTPUT(LED);
	OUTPUT(OUT1);
	//OUTPUT(AUX_OUT);
	HIGH(OUT1);

	//Setup inputs
	INPUT(ZERO_IN);
	HIGH(ZERO_IN);

	//INPUT(PUSH_IN);
	//HIGH(PUSH_IN);


	for(char i=0; i<5;++i){
		TOGGLE(LED);
		_delay_ms(250);
	}
	LOW(LED);

	int8_t secs = 0;
	int8_t pulse_occurred = 0;
	uint8_t t0;

	//GIFR = GIFR;             //Clear interrupt flags
	GIMSK |= _BV(INT0);		//Enable INT0
	MCUCR |= _BV(ISC01);     //INT0 in Falling edge
	sei();

	uint8_t now;
	t0 = secs_counter_read();
	while(1){
		do{
			now = secs_counter_read();
		}
		while((now-t0)<60);
		t0 = now;
		HIGH(LED);
		_delay_ms(250);
		LOW(LED);
	}
}
*/

static void writeConfiguration(uint8_t cfgByte){
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
		cfgByte = 8;//The default mode is humidity with 8 %
		writeConfiguration(cfgByte);
	}
	return cfgByte;
}

int main(void)
{
	timer1_init();

	//Setup outputs
	OUTPUT(LED);
	OUTPUT(OUT1);
	//OUTPUT(AUX_OUT);
	HIGH(OUT1);

	//Setup inputs
	INPUT(ZERO_IN);
	HIGH(ZERO_IN);

	//GIFR = GIFR;          //Clear interrupt flags
	GIMSK |= _BV(INT0);		//Enable INT0
	MCUCR |= _BV(ISC01);    //INT0 in Falling edge
	sei();

	char humidity_read_delay = 0;

	init_dht11();
	adc_init();

	for(char i=0; i<5;++i){
		TOGGLE(LED);
		_delay_ms(250);
	}
	LOW(LED);

	int8_t secs = 0;
	int8_t pulse_occurred = 0;
	uint8_t elapsed;


	while(1){
		pulse_occurred = ldr_state_machine();
		secs = secs_counter_read();
		switch (main_state) {
			case MAIN_IDLE:
				LOW(LED);
				if(pulse_occurred){
					//Pulse ocurred
					pulse_counter = 1;
					main_state = MAIN_COUNTING;
					main_timer = secs;
				}
				break;
			case MAIN_COUNTING:
				elapsed = secs - main_timer;
				if(elapsed>PGM_MODE_ACTIVATION_MAX_TIME){
					main_state = MAIN_IDLE;
				}
				else{
					if(pulse_occurred){
						main_timer = secs;
						pulse_counter++;
						if(pulse_counter>=PGM_MODE_PULSES){
							HIGH(LED);
							main_state = MAIN_PGM_MODE;
							pulse_counter = 0;
						}
					}
				}
				break;
			case MAIN_PGM_MODE:
				elapsed = secs - main_timer;
				if(elapsed>PGM_MODE_MAX_TIME){
					main_state = MAIN_IDLE;
					if(pulse_counter>0){
						//Save settings
						LOW(LED);
						while(pulse_counter>0){
							--pulse_counter;
							_delay_ms(250);
							HIGH(LED);
							_delay_ms(250);
							LOW(LED);
						}
					}
				}
				if(pulse_occurred){
					TOGGLE(LED);
					_delay_ms(100);
					TOGGLE(LED);
					main_timer = secs;
					pulse_counter++;
				}
				break;
			default:
				main_state = MAIN_IDLE;
				break;
		}
	}
}

//int main2(void)
//{
//	timer1_init();
//
//	//Setup outputs
//	OUTPUT(LED);
//	OUTPUT(OUT1);
//	//OUTPUT(AUX_OUT);
//	HIGH(OUT1);
//
//	//Setup inputs
//	INPUT(ZERO_IN);
//	HIGH(ZERO_IN);
//
//	INPUT(PUSH_IN);
//	HIGH(PUSH_IN);
//
//	GIFR = GIFR;             //Clear interrupt flags
//
//	MCUCR |= _BV(ISC01);     //INT0 in Falling edge
//
//	char oldState = READ(PUSH_IN);
//	char humidity_read_delay = 0;
//
//	init_dht11();
//	adc_init();
//
//	for(char i=0; i<5;++i){
//		TOGGLE(LED);
//		_delay_ms(250);
//	}
//	LOW(LED);
//	uint8_t fault = 0;
//	int8_t aux = 0;
//	while(1){
//		main_state_machine();
//		/*
//		//Humidity sensor test
//		if(fault){
//			if(aux==-1){
//				for(int i=0; i<10;++i){
//					HIGH(LED);
//					_delay_ms(50);
//					LOW(LED);
//					_delay_ms(50);
//				}
//			}
//			else if(aux==-4){
//				HIGH(LED);
//				_delay_ms(250);
//				LOW(LED);
//				_delay_ms(250);
//			}
//			else{
//				HIGH(LED);
//				_delay_ms(20);
//				LOW(LED);
//				_delay_ms(480);
//			}
//		}
//		else{
//			_delay_ms(500);
//		}
//		if(humidity_read_delay++ >= 10){
//			humidity_read_delay = 0;
//			aux = read_humidity();
//			fault = aux<0;
//			if(aux>50){
//				HIGH(LED);
//			}
//			else{
//				LOW(LED);
//			}
//		}
//		*/
//		/*
//		char aux = READ(PUSH_IN);
//		if(aux!=oldState){
//			if(!aux){
//				turnOn();
//			}
//			else{
//				turnOff();
//			}
//			oldState = aux;
//		}
//		*/
//		/*
//		uint8_t ldr = adc_read();
//		if(ldr<200){
//			HIGH(LED);
//		}
//		else{
//			LOW(LED);
//		}
//		*/
//		/*
//		_delay_ms(250);
//		LOW(LED);
//		_delay_ms(250);
//		*/
//	}
//}


#define INVALID 255


ISR(INT0_vect)
{
	char tmp;
	for(tmp=0; tmp<5;){
		if(!READ(ZERO_IN)){
			tmp++;
		}
		else{
			tmp = INVALID;
		}
	}
	if(tmp==INVALID)
		return;


	subsec_counter++;
	if(subsec_counter==50){
		subsec_counter = 0;
		secs_counter++;
	}

	if(!turn_on_flag)
		return;
/*
	//HIGH(AUX_OUT);
	if(state==State_Calibration){
		if(zc_cycles>75 && zc_cycles<110){
			state = State_Calibrated;
			zc_interval = (zc_cycles>>1);
			zc_cycles = 0;
			TCNT0 = 0;
		}
		else{
			state = State_Uncalibrated;
			timer1_stop();
		}
	}
	else if(state==State_Calibrated){
		if(zc_cycles>zc_interval){//Ignore possible oscilations
			fire_triac = 5;
			zc_cycles = 0;
			TCNT0 = 0;
			TIFR0|= _BV(OCF0A);
		}
	}
	else{
		zc_interval = INVALID;
		zc_cycles = 0;
		state = State_Calibration;
		timer1_start();
	}
*/
}

ISR(TIM0_COMPA_vect) {

	/*
	//LOW(AUX_OUT);
	if(state==State_Calibration){
		zc_cycles++;
	}
	else if(state==State_Calibrated){
		if(fire_triac>0){
			if(fire_triac & 0x1){
				LOW(OUT1);
			}
			else{
				HIGH(OUT1);
			}
			fire_triac--;
		}
		zc_cycles++;
		if(zc_cycles==zc_interval){
			fire_triac = 6;
			zc_cycles = 0;
		}
	}
	else{
		//timer1_stop();
		//HIGH(OUT1);
	}
	*/
}


//
//ISR(TIM0_COMPA_vect) {
//	//TOGGLE(AUX_OUT);
//	TOGGLE(OUT1);
//	stopTimer1();
//}



