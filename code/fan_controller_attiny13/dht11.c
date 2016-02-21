/*
 * Synchronous DHT-11 temp & humidity sensor module reader
 *
 *
 * Copyright (c) 2012, Angus Gratton. Licensed under the Modified BSD License.
 *
 */
#include "config.h"
#include "macro_helpers.h"
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>

#include "dht11.h"

#ifndef DHT_PIN
#error You should define a pin named DHT_PIN in config.h, with form D,2
#endif

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

//static const uint8_t MAX_WAIT = microsecondsToClockCycles(100);
#define MAX_WAIT microsecondsToClockCycles(100)

void init_dht11() {
  INPUT(DHT_PIN);
  // use internal 20k pullup (lazy!)
  HIGH(DHT_PIN);
}

union _dht_output {
  uint8_t bytes[5];
  struct {
    dht11_data inner;
    uint8_t checksum;
  } data;
};


static int8_t read_preamble(void) {
	/*
	 * The DHT11 generates a preamble/ACK where the line is pulled down for 80us and pulled up for 80us
	 */
	uint16_t wait_time = 0;
	while(READ(DHT_PIN)){
	  if(wait_time++ == MAX_WAIT)
		  return -1; // bad edge
	}
	wait_time=0;
	while(!READ(DHT_PIN)){
	  if(wait_time++ == MAX_WAIT)
		  return -1; // bad edge
	}
	wait_time=0;
	while(READ(DHT_PIN)){
	  if(wait_time++ == MAX_WAIT)
		  return -1; // bad edge
	}
	return 0;
}

static int8_t read_bit(void) {
	/*
	 * A data bit starts with a 50us delay, where the data line is pulled down, followed
	 * by a pulse high where the duration is used to distinguish a zero from a one.
	 * A pulse with 26-28us is a zero and a pulse with 70us is a one.
	 * We measure the duration of the initial delay and the duration of the pulse and use
	 * the duration of the delay as a reference to distinguish the a zero from a one.
	 *
	 */
	uint16_t bit_time = 0;
	uint16_t bitstart_delay = 0;
  while(!READ(DHT_PIN)){
	  if(bitstart_delay++ == MAX_WAIT)
		  return -1; // bad edge
  }
  while(READ(DHT_PIN)){
  	  if(bit_time++ == MAX_WAIT)
  		  return -1; // bad edge
    }

  return bit_time>bitstart_delay;
}

int8_t read_humidity(void)
{
  OUTPUT(DHT_PIN);
  /*
   * Send a DHT wakeup/start signal.
   * This signal is made of a low pulse with a minimum duration of 18ms.
   * After 40us the DHT sends an preamble/ACK by pulling the line low.
   */
  LOW(DHT_PIN); // send DHT wakeup signal
  _delay_ms(18);
  HIGH(DHT_PIN);
  _delay_us(40);
  INPUT(DHT_PIN);

  if(read_preamble()<0)
	  return bad_ack;

  union _dht_output raw = { .bytes={0} };
  for(uint8_t byte = 0; byte < sizeof(raw); byte++) {
	for(int8_t bit = 7; bit >= 0; bit--) {
		int8_t b =read_bit();
		if(b<0){
			return bad_bit_data_pause; // invalid data pulse
		}
		if(b){
			raw.bytes[byte] |= _BV(bit);
		}
	}
  }

  if(raw.data.checksum != (uint8_t)(raw.bytes[0]+raw.bytes[1]+raw.bytes[2]+raw.bytes[3]))
	return bad_checksum;
  return raw.data.inner.humidity_integral;
}
