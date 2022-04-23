# smart-fan-controller
Device to control a bathroom fan offering two modes of operation: level of humidity or timed mode


# Introduction

## A bit of story.
This project originated from a request of my girlfriend. She needed a bathroom fan for her new house but she wanted to avoid the ones that run every time the light is turned on. 
We looked for fans with timer or humidity sensors in our local stores, but all them were big for the existing hole. The ones that fitted, were the basic ones we were trying to avoid. 

I decided for one of the basic ones, and then noticed it had a small compartment with a plastic cover. 
The cover featured a small window, suggesting it was designed for a sensor. 
This gave me the idea of making a fan controller that could fit inside the compartment, and make use of the small window for a humidity sensor.

[INSERT PHOTO OF THE FAN]

## Requirements

The initial requirements for the fan controller were the following:
* Fit inside the compartment provided by the fan.
* Feature a humidity sensor to activate the fan depending on the level of humidity.
* Low power consumption.
* Adjustable humidity threshold.

The ability to set the humidity threshold triggered a new requirement
I spent some time thinking of ways that could be used to set the humidity threshold when the idea of using the light of the bathroom come to my mind. 
This way there wouldn't be no need to use a ladder to fiddle with the fan or the need for a remote control.
After analyzing various options, I decided to use the following scheme to configure the humidity threshold.
* The user toggles the light switch a specific number of times.
* The device enter configuration mode.
* The user toggle the switch a number of times equivalent to the desired humidity threshold. For example, five times for a 50% humidity.
* After a period without light pulses the device leaves configuration mode

# The hardware

The hardware can be divided in the following major blocks:
* Power supply
* Humidity sensor module
* Light sensor
* Motor control
* Zero cross detection
* Microcontroller

The power supply reduces the mains voltage (220V AC) to a suitable voltage for use by the microcontroller and the humidity sensor module.

The humidity sensor module has built in all the electronics needed for relative humidity acquisition. This includes the actual humidity sensor element, temperature sensor, and interface logic. The module provides a one wire interface used for humidity and temperature readings.

A light-dependent resistor (LDR) is used as light sensor and feeds one ADC input of the microcontroller.

The fan motor is driven using a TRIAC. A TRIAC is a cheaper and smaller alternative to a relay, another common device used to drive AC loads.

Zero cross detection is used for triggering the TRIAC and as a time source.

[Block diagram]

## The microcontroller

This project uses Atmel's ATtiny13 microcontroller. The reasons behind this choice were physical size and availability in my home lab.
The ATtiny13 microcontroller, can run at up to 20 MHz and features 1 KByte of flash and six I/O pins in an eight pin package. 
The number of of I/O pins exceeds the requirements and leaves us with two spare pins. One of these spare pins is used to drive a feedback LED introduced later on.

## Power supply
The power for the microcontroller and sensors is provided by a transform-less power supply.
A voltage divider with a zener is used to reduce the mains voltage to approximately 5V. This provides a relatively stable output as long as the as the output load is kept below the zener current.

The voltage divider uses a dropper capacitor and a resistor as input impedance. The dropper capacitor provides most of the input impedance and the resistor limits the inrush current. If the input impedance was purely resistive we would have considerable losses as heat, a side effect less noticeable with this arrangement.
One disadvantage is that we need to use a capacitor with high capacitance and a high voltage rating. Capacitors with these two characteristics are not small and space is not a commodity in abundance in the fan enclosure.

Rectification is performed with a single diode, meaning we get half wave rectification. This is less efficient in terms of output current but allows direct driving of the triac, making the use of a optocoupler or optotriac unnecessary.
If a full wave rectifier was used, the microcontroller and the triac would not share a common reference. With the full wave rectifier, the microcontroller's ground would be provided by 
two diodes, with one conducting to neutral in one half cycle and the other conducting to live in the other half cycle. In this situation we would only be able to drive the triac in half of the cycles.
Finally two electrolytic capacitors are used to reduce ripple and stabilize the output voltage.

The values for the capacitor and the resistor were obtained using a spreadsheet provided by Designer Circuits LLC. This spreadsheet is a companion to the excelent design note "Transformerless Power Supply Design". Microchip Application Note AN954 was used initially but the equations presented in that document were not clear and the results from simulation didn't match the calculations. 

## Triac control 

Ideally,triacs should be triggered when the current is at its lowest value. For pure resistive loads, where the current is in phase with the voltage, the triac should be triggered when the voltage crosses zero.
In the case of a fan motor, or any other kind of device that involves coils, we have an inductive load. 
These kinds of loads introduce a phase shift in the current relative to the voltage. For practical purposes, the instant where current is zero will be delayed relatively to the instant the voltage is zero. This means that for inductive loads, instead of zero voltage detection we should use zero current detection.

A possible approach to zero current detection is the use of a shunt resistor and a operational amplifier to measure the current. This approach not only introduces a new IC with specific power requirements, but also additional components. Since PCB real estate is not abundant, the approach was discarded.
Another approach suggested in a NXP application note (AN467) is to monitor the triac gate voltage with respect to the T1 terminal. This can be achieved with a few resistors and a window comparator. Some microcontrollers include a window comparator, but that is not the case for the ATtiny13. Adding an external window comparator IC was also discarded due to space restrictions.

Exhausted the possibilities, I decided to use zero voltage crossing and hope the EMI noise and power losses would be minimal.

To reduce the power needed to trigger the triac a simple trick is used. Instead of driving the gate continuously, the gate is driven with a pulse train. With this we can achieve a reduction of the power required to driver the triac.

After some failed experiments with other types of triacs, I ended using the BTA204-800E. This triac is part of NXP 3Q Hi-Com Triac family and offers the following benefits:

* 3Q technology for improved noise immunity
* Direct triggering from low power drivers and logic ICs
* High blocking voltage capability
* High commutation capability
* Planar passivated for voltage ruggedness and reliability
* Sensitive gate for easy logic level triggering
* Triggering in three quadrants only

Thanks to this characteristics is possible to drive the triac directly with a microcontroller pin. 
It also eliminates the need for a snubber network. A snubber network reduces the side effects produced when driving inductive loads. It is made of a RC circuit in parallel with the load. 
Due to the size of a capacitor rated for 220V AC, it would be quite difficult to fit all inside.

One disadvantage of this Triac is that it can only be triggered in three quadrants. The 3+ quadrant (T2-, G+) can't be used, meaning we can't drive the gate with a circuit that sources current. Instead we need a circuit that sinks current. In practical terms, we can see the triac gate as a active low pin. This is not an issue because most digital output pins can sink more current that they can source current.

[Include figure from NXP three-quadrant_NXP Hi-Com triacs]

As mentioned before, other triacs were tested before the BTA204-800E was chosen. They worked fine during tests with 20V AC and a incandescent lamp as a load, but not so fine when using the actual fan. I experienced unstable behavior or a completely inability to rotate the fan, that I assume was caused by noise due to lack of a snubber network.


## Sensors

Two sensors are used. A humidity sensor module and a light dependent resistor (LDR).

For the humidity sensor is used the ubiquitous DHT11. This module has limited precision (+/-5%) and limited range (from 20% to 80%). These characteristics are enough for the requirements of a bathroom fan. If more precision is needed, replacing the DHT11 with is bigger brother DHT22, can be an option.
This module features a non-standard one wire interface used by the microcontroller to acquire humidity and temperature readings. 

A voltage divider made of a LDR and a fixed resistor provides a voltage that varies accordingly to the amount of light. This voltage is feed into a analog input of the microcontroller and converted into a digital value.

## Zero cross detection

# Software
* Zero cross detection
* Triac firing

* "User interface" 

## Sensors

### DHT111

The code used to read humidity values is based on a open source library developed by Angus Gratton (original source: https://github.com/projectgus/flatdrier).
Some minor changes were made to reduce the size of the compiled code. For example, the number of invocations of external functions was reduced, some portions of the code have been reorganized, and a few simplifications were used to make the job of the compiler easier.

When the compiler translates a C function invocation, the resulting assembly code is composed of three sections. The function prologue, the call of the routine implementing the function, and a function epilogue. Of these, the function prologue is the one that usually uses more instructions, with an amount depending on the number of arguments. By reducing the number of invocations we can save a few precious bytes. This was done in the function used to read bits sent by the DHT11. 

The original library uses the following portion of code to store the bits sent by the DHT11:

	...
	for(int8_t bit = 7; bit >= 0; bit--) {
	  if(!listen_edge(false, 20, 80)) // 50us bit prelude
		return bad_bit_prelude;
	  uint8_t bit_length = listen_edge(true, 10, 90); // bit signal - 26-28us for 0 70us for 1
	  if(!bit_length)
		return bad_bit_data_pause; // invalid data pulse
	  if(bit_length > 30)
		raw.bytes[byte] |= _BV(bit);
	}
	...

And this is the new code used for the same purpose.
	...
	uint8_t aux = 0;	
	for(uint8_t bit = 0; bit < 8; bit++) {
		int8_t b = read_bit();
		if(b < 0){
			return bad_bit_data_pause; // invalid data pulse
		}
		
		aux = aux << 1;
		aux |= b & 0x01;
	}
	raw.bytes[byte] = aux;
	...

The number of if statements inside the for loop have been reduced to one. This eliminates a couple of instructions used to test and branch.

By using an auxiliary variable we reduced the overhead of accessing the `raw.bytes` array. In the original code, a few additional instructions are needed to fetch the contents of the array position from RAM into a register. In the new code this is not needed. The compiler translates the auxiliary variable into a register that is used directly during logic operations. After the for finishes, the contents of this register is stored into RAM.
	
* Size optimizations

