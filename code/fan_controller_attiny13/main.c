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


#define OUT1 B,0
#define ZERO_IN B,1
#define PUSH_IN B,2
#define LED B,3
#define AUX_OUT B,4

void initTimer1(){
	cli();
	TCCR0B = 0;
	TCNT0  = 0;					// Initialize counter value to 0
	TCCR0A = _BV(WGM01); 		// Turn on CTC mode
	OCR0A = 30;					// Set compare Match Register -> Will interrupt every 0.0002
	TIMSK0 |= _BV(OCIE0A);		// Enable timer compare A interrupt
	sei();
}

static inline void startTimer1(){
	TCNT0  = 0;
	TCCR0B = _BV(CS01); // Set prescaler to 8
}

static inline void stopTimer1(){
	TCCR0B = 0;
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
	initTimer1();

	//Setup outputs
	OUTPUT(LED);
	OUTPUT(OUT1);
	OUTPUT(AUX_OUT);
	HIGH(OUT1);

	//Setup inputs
	INPUT(ZERO_IN);
	HIGH(ZERO_IN);

	INPUT(PUSH_IN);
	HIGH(PUSH_IN);

	GIFR = GIFR;             //Clear interrupt flags

	MCUCR |= _BV(ISC01);     //INT0 in Falling edge

	char oldState = READ(PUSH_IN);
	while(1){
		HIGH(LED);
		_delay_ms(100);
		LOW(LED);
		_delay_ms(100);
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

	HIGH(AUX_OUT);
	if(state==State_Calibration){
		if(zc_cycles>75 && zc_cycles<110){
			state = State_Calibrated;
			zc_interval = (zc_cycles>>1);
			zc_cycles = 0;
			TCNT0 = 0;
		}
		else{
			state = State_Uncalibrated;
			stopTimer1();
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
		startTimer1();
	}
}

ISR(TIM0_COMPA_vect) {
	LOW(AUX_OUT);
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
		stopTimer1();
		HIGH(OUT1);
	}

}


//
//ISR(TIM0_COMPA_vect) {
//	//TOGGLE(AUX_OUT);
//	TOGGLE(OUT1);
//	stopTimer1();
//}



