/*
 * tests.c
 *
 *  Created on: Mar 14, 2016
 *      Author: jassuncao
 */


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


//
//ISR(TIM0_COMPA_vect) {
//	//TOGGLE(AUX_OUT);
//	TOGGLE(OUT1);
//	stopTimer1();
//}

