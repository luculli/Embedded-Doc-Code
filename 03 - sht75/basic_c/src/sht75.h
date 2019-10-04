/* sht75 - Basic configuration
 *
 * Copyright (C) 2018 Gabriele LUCULLI
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/**
 * @file basic.c
 * @author Gabriele LUCULLI
 * @date 19 Jun 2018
 * @brief File containing a basic configuration from the temperature/humidity sensor on the RPI 3B+
 */


#ifndef _SHT75
#define _SHT75

#include <stdint.h>
#include <stdbool.h>

// Enable CRC checking
#define CRC_ENA

// Enable ('1') or disable ('0') internal pullup on DATA line
// Commenting out this #define saves code space but leaves internal pullup
//   state undefined (ie, depends on last bit transmitted)
//#define DATA_PU 1

// Clock pulse timing macro
// Lengthening these may assist communication over long wires
//#define PULSE_LONG  delayMicroseconds(3)
//#define PULSE_SHORT delayMicroseconds(1)
#define DATA_PIN 6
#define CLOCK_PIN 5
#define HIGH 1
#define LOW 0

#define PULSE_LONG 3
#define PULSE_SHORT 1

// Useful macros
#define measTemp(result) meas(TEMP, result, BLOCK)
#define measHumi(result) meas(HUMI, result, BLOCK)

#define MEAS_TEMP 	((uint8_t) 0x03)	// 000  0001   1
#define MEAS_HUMI 	((uint8_t) 0x05)	// 000  0010   1
#define STAT_REG_W 	((uint8_t) 0x06)	// 000  0011   0
#define STAT_REG_R 	((uint8_t) 0x07)	// 000  0011   1
#define SOFT_RESET 	((uint8_t) 0x1e)	// 000  1111   0

// Status register writable bits
#define SR_MASK	((uint8_t) 0x07)

// getByte flags
#define noACK 	((bool) false)
#define ACK		((bool) true)

// Temperature & humidity equation constants
#define D1		((float) -39.66)
#define D2h		((float) 0.01)
#define D2l		((float) 0.04)

#define C1		((float) -2.0468)		// for V4 sensors
#define C2h		((float) 0.0367)		// for V4 sensors, 12-bit precision
#define C3h		((float) -1.5955E-6)	// for V4 sensors, 12-bit precision
#define C2l		((float) 0.5872)		// for V4 sensors, 8-bit precision
#define C3l		((float) -4.0845E-4)	// for V4 sensors, 8-bit precision

#define T1	((float) 0.01)		// for V3 and V4 sensors
#define T2h	((float) 0.00008)	// for V3 and V4 sensors, 12-bit precision
#define T2l	((float) 0.00128)	// for V3 and V4 sensors, 8-bit precision

// User constants
#define TEMP		((uint8_t) 0)
#define HUMI		((uint8_t) 1)
#define BLOCK		((bool) true)
#define NONBLOCK	((bool) false)

// Status register bit definitions
#define LOW_RES		((uint8_t) 0x01)	// 12-bit Temp / 8-bit RH (vs. 14 / 12)
#define NORELOAD	((uint8_t) 0x02)	// No reload of calibrarion data
#define HEAT_ON		((uint8_t) 0x04)	// Built-in heater on
#define BATT_LOW	((uint8_t) 0x40)	// VDD < 2.47V


// Function return code definitions
#define S_Err_NoACK		((uint8_t) 1)	// ACK expected but not received
#define S_Err_CRC		((uint8_t) 2)	// CRC failure
#define S_Err_TO		((uint8_t) 3)	// Timeout
#define S_Meas_Rdy		((uint8_t) 4)	// Measurement ready

uint16_t *_presult;
uint8_t _stat_reg;
#ifdef CRC_ENA
uint8_t _crc;
#endif

// public interface
void sht75_Init();
float sht75_readTemperature();
float sht75_readHumidity(const float _temp);

uint8_t getResult(uint16_t *result);
uint8_t putByte(uint8_t value);
uint8_t getByte(bool ack);
void startTransmission(void);
void resetConnection(void);

#ifdef CRC_ENA
void calcCRC(uint8_t value, uint8_t *crc);
uint8_t bitrev(uint8_t value);
#endif

uint8_t measure(float *temp, float *humi, float *dew);
uint8_t meas(uint8_t cmd, uint16_t *result, bool block);
uint8_t measRdy(void);
uint8_t writeSR(uint8_t value);
uint8_t readSR(uint8_t *result);
uint8_t reset(void);
float calcTemp(uint16_t rawData);
float calcHumi(uint16_t rawData, float temp);
float calcDewpoint(float humi, float temp);

#endif
