/*
 * Synchronous DHT-11 temp & humidity sensor module reader
 *
 *
 * Copyright (c) 2012, Angus Gratton. Licensed under the Modified BSD License.
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

/* Wait for a DHT signal edge, high or low. Edge must be at least min_length us and no longer than max_length us.
 *
 * Returns the edge duration, or 0 if the edge was outside of limits (too long or too short)
 */
static uint8_t listen_edge(bool high, uint8_t min_length, uint8_t max_length)
{
  uint8_t wait_time;
  for(wait_time = 0; wait_time < max_length && (READ(DHT_PIN) == high); wait_time++)
    _delay_us(1);
  if(wait_time == max_length || wait_time < min_length)
    return 0; // bad edge
  return wait_time;
}

dht11_status read_dht11(dht11_data *data) {
  OUTPUT(DHT_PIN);
  LOW(DHT_PIN); // send DHT wakeup signal
  _delay_ms(18);
  HIGH(DHT_PIN);
  _delay_us(40);
  INPUT(DHT_PIN);

  if(!listen_edge(false, 0, 100)) // hold for DHT ack signal, 80us
    return bad_ack;

  _delay_us(5);
  if(!listen_edge(true, 0, 100)) // pre-transmission pause from DHT
    return bad_post_ack;

  union _dht_output raw = { .bytes={0} };
  for(uint8_t byte = 0; byte < sizeof(raw); byte++) {
    for(int8_t bit = 7; bit >= 0; bit--) {
      if(!listen_edge(false, 20, 80)) // 50us bit prelude
        return bad_bit_prelude;
      uint8_t bit_length = listen_edge(true, 10, 90); // bit signal - 26-28us for 0 70us for 1
      if(!bit_length)
        return bad_bit_data_pause; // invalid data pulse
      if(bit_length > 30)
        raw.bytes[byte] |= _BV(bit);
    }
  }

  if(raw.data.checksum != (uint8_t)(raw.bytes[0]+raw.bytes[1]+raw.bytes[2]+raw.bytes[3]))
    return bad_checksum;

  memcpy(data, &raw.data.inner, sizeof(dht11_data));
  return success;
}
