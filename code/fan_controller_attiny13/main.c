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
#define PUSH_IN B,2
//#define DHT_PIN B,3
#define LED B,4

//#define AUX_OUT B,4

void timer1_init(){
	cli();
	TCCR0B = 0;
	TCNT0  = 0;					// Initialize counter value to 0
	TCCR0A = _BV(WGM01); 		// Turn on CTC mode
	OCR0A = 30;					// Set compare Match Register -> Will interrupt every 0.0002
	TIMSK0 |= _BV(OCIE0A);		// Enable timer compare A interrupt
	sei();
}

static inline void timer1_start(){
	TCNT0  = 0;
	TCCR0B = _BV(CS01); // Set prescaler to 8
}

static inline void timer1_stop(){
	TCCR0B = 0;
}

static inline void adc_init()
{
	//Set the ADC prescaler to 32 making the ADC run at 150KHz (4.8MHz/32)
	ADCSRA = _BV(ADPS2) | _BV(ADPS0);
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

static inline void turnOn(){
	GIMSK |= _BV(INT0);		//Enable INT0
}

static inline void turnOff(){
	state = State_Uncalibrated;
	GIMSK &= ~(_BV(INT0));		//Disable INT0
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

	INPUT(PUSH_IN);
	HIGH(PUSH_IN);

	GIFR = GIFR;             //Clear interrupt flags

	MCUCR |= _BV(ISC01);     //INT0 in Falling edge

	char oldState = READ(PUSH_IN);
	char humidity_read_delay = 0;

	init_dht11();
	adc_init();

	for(char i=0; i<5;++i){
		TOGGLE(LED);
		_delay_ms(250);
	}
	LOW(LED);
	uint8_t fault = 0;
	int8_t aux = 0;
	while(1){
		/*
		//Humidity sensor test
		if(fault){
			if(aux==-1){
				for(int i=0; i<10;++i){
					HIGH(LED);
					_delay_ms(50);
					LOW(LED);
					_delay_ms(50);
				}
			}
			else if(aux==-4){
				HIGH(LED);
				_delay_ms(250);
				LOW(LED);
				_delay_ms(250);
			}
			else{
				HIGH(LED);
				_delay_ms(20);
				LOW(LED);
				_delay_ms(480);
			}
		}
		else{
			_delay_ms(500);
		}
		if(humidity_read_delay++ >= 10){
			humidity_read_delay = 0;
			aux = read_humidity();
			fault = aux<0;
			if(aux>50){
				HIGH(LED);
			}
			else{
				LOW(LED);
			}
		}
		*/
		/*
		char aux = READ(PUSH_IN);
		if(aux!=oldState){
			if(!aux){
				turnOn();
			}
			else{
				turnOff();
			}
			oldState = aux;
		}
		*/
		uint8_t ldr = adc_read();
		if(ldr<200){
			HIGH(LED);
		}
		else{
			LOW(LED);
		}
		/*
		_delay_ms(250);
		LOW(LED);
		_delay_ms(250);
		*/
	}
}


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
}

ISR(TIM0_COMPA_vect) {
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
		timer1_stop();
		HIGH(OUT1);
	}

}


//
//ISR(TIM0_COMPA_vect) {
//	//TOGGLE(AUX_OUT);
//	TOGGLE(OUT1);
//	stopTimer1();
//}



