/*
 * macro_helpers.h
 *
 *  Created on: Jan 25, 2016
 *      Author: jassuncao
 */

#ifndef MACRO_HELPERS_H_
#define MACRO_HELPERS_H_

/* Thanks to startlino for these quick avr macros
   http://www.starlino.com/port_macro.html
*/

// MACROS FOR EASY PIN HANDLING FOR ATMEL GCC-AVR
//these macros are used indirectly by other macros , mainly for string concatination
#define _SET(type,name,bit)            type ## name  |= _BV(bit)
#define _CLEAR(type,name,bit)        type ## name  &= ~ _BV(bit)
#define _TOGGLE(type,name,bit)        type ## name  ^= _BV(bit)
#define _GET(type,name,bit)            ((type ## name >> bit) &  1)
#define _PUT(type,name,bit,value)    type ## name = ( type ## name & ( ~ _BV(bit)) ) | ( ( 1 & (unsigned char)value ) << bit )

//these macros are used by end user
#define OUTPUT(pin)            _SET(DDR,pin)
#define INPUT(pin)            _CLEAR(DDR,pin)
#define HIGH(pin)            _SET(PORT,pin)
#define LOW(pin)            _CLEAR(PORT,pin)
#define TOGGLE(pin)            _TOGGLE(PORT,pin)
#define FAST_TOGGLE(pin)            _SET(PIN,pin)
#define READ(pin)            _GET(PIN,pin)

#endif /* MACRO_HELPERS_H_ */
