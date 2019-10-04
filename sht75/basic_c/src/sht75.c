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


#include <pigpio.h>
#include <unistd.h>
#include <stdio.h>

#include "sht75.h"

bool init = false;

void sht75_Init()
{
	gpioInitialise();
	_presult = NULL;
	_stat_reg = 0x00;

	gpioSetMode(CLOCK_PIN, PI_OUTPUT);

	resetConnection();
	putByte(SOFT_RESET);

	printf("*** SHT75 Initialization finished\n");
}

float sht75_readTemperature()
{
	uint16_t raw = 0;
	meas(TEMP, &raw, BLOCK);
	return calcTemp(raw);
}

float sht75_readHumidity(const float _temp)
{
	uint16_t raw = 0;
	meas(HUMI, &raw, BLOCK);
	return calcHumi(raw, _temp);
}

// All-in-one (blocking): Returns temperature, humidity, & dewpoint
uint8_t measure(float *temp, float *humi, float *dew)
{
    uint16_t rawData;
    uint8_t error;
    if (error = measTemp(&rawData))
        return error;
    *temp = calcTemp(rawData);
    if (error = measHumi(&rawData))
        return error;
    *humi = calcHumi(rawData, *temp);
    *dew = calcDewpoint(*humi, *temp);
    return 0;
}

// Initiate measurement.  If blocking, wait for result
uint8_t meas(uint8_t cmd, uint16_t *result, bool block)
{
    uint8_t error, i;
#ifdef CRC_ENA
    _crc = bitrev(_stat_reg & SR_MASK); // Initialize CRC calculation
#endif
    startTransmission();
    if (cmd == TEMP)
        cmd = MEAS_TEMP;
    else
        cmd = MEAS_HUMI;
    if (error = putByte(cmd))
        return error;
#ifdef CRC_ENA
    calcCRC(cmd, &_crc); // Include command byte in CRC calculation
#endif
    // If non-blocking, save pointer to result and return
    if (!block) {
        _presult = result;
        return 0;
    }
    // Otherwise, wait for measurement to complete with 720ms timeout
    i = 240;
    while (gpioRead(DATA_PIN)) {
        i--;
        if (i == 0)
            return S_Err_TO; // Error: Timeout
        sleep(3);  // original code was sleep(3)
    }
    error = getResult(result);
    return error;
}

// Check if non-blocking measurement has completed
// Non-zero return indicates complete (with or without error)
uint8_t measRdy(void)
{
    uint8_t error = 0;
    if (_presult == NULL) // Already done?
        return S_Meas_Rdy;
    if (gpioRead(DATA_PIN) != 0) // Measurement ready yet?
        return 0;
    error = getResult(_presult);
    _presult = NULL;
    if (error)
        return error; // Only possible error is S_Err_CRC
    return S_Meas_Rdy;
}

// Get measurement result from sensor (plus CRC, if enabled)
uint8_t getResult(uint16_t *result)
{
    uint8_t val;
#ifdef CRC_ENA
    val = getByte(ACK);
    calcCRC(val, &_crc);
    *result = val;
    val = getByte(ACK);
    calcCRC(val, &_crc);
    *result = (*result << 8) | val;
    val = getByte(noACK);
    val = bitrev(val);
    if (val != _crc) {
        *result = 0xFFFF;
        return S_Err_CRC;
    }
#else
    *result = getByte(ACK);
    *result = (*result << 8) | getByte(noACK);
#endif
    return 0;
}

// Write status register
uint8_t writeSR(uint8_t value)
{
    uint8_t error;
    value &= SR_MASK;  // Mask off unwritable bits
    _stat_reg = value; // Save local copy
    startTransmission();
    if (error = putByte(STAT_REG_W))
        return error;
    return putByte(value);
}

// Read status register
uint8_t readSR(uint8_t *result)
{
    uint8_t val;
    uint8_t error = 0;
#ifdef CRC_ENA
    _crc = bitrev(_stat_reg & SR_MASK); // Initialize CRC calculation
#endif
    startTransmission();
    if (error = putByte(STAT_REG_R)) {
        *result = 0xFF;
        return error;
    }
#ifdef CRC_ENA
    calcCRC(STAT_REG_R, &_crc); // Include command byte in CRC calculation
    *result = getByte(ACK);
    calcCRC(*result, &_crc);
    val = getByte(noACK);
    val = bitrev(val);
    if (val != _crc) {
        *result = 0xFF;
        error = S_Err_CRC;
    }
#else
    *result = getByte(noACK);
#endif
    return error;
}

// Public reset function
// Note: Soft reset returns sensor status register to default values
uint8_t reset(void)
{
    _stat_reg = 0x00;           // Sensor status register default state
    resetConnection();          // Reset communication link with sensor
    return putByte(SOFT_RESET); // Send soft reset command & return status
}


/******************************************************************************
 * Sensirion data communication
 ******************************************************************************/

// Write byte to sensor and check for acknowledge
uint8_t putByte(uint8_t value)
{
    uint8_t mask, i;
    uint8_t error = 0;
    gpioSetMode(DATA_PIN, PI_OUTPUT); // Set data line to output mode
    mask = 0x80;                      // Bit mask to transmit MSB first
    for (i = 8; i > 0; i--) {
        if (value & mask)
            gpioWrite(DATA_PIN, HIGH);
        else
            gpioWrite(DATA_PIN, LOW);

        usleep(PULSE_SHORT);
        gpioWrite(CLOCK_PIN, HIGH); // Generate clock pulse
        usleep(PULSE_LONG);
        gpioWrite(CLOCK_PIN, LOW);
        usleep(PULSE_SHORT);
        mask >>= 1; // Shift mask for next
    }
    gpioSetMode(DATA_PIN, PI_INPUT); // Return data line to input mode
#ifdef DATA_PU
    gpioWrite(DATA_PIN, DATA_PU); // Restore internal pullup state
#endif
    gpioWrite(CLOCK_PIN, HIGH); // Clock #9 for ACK
    usleep(PULSE_LONG);
    if (gpioRead(DATA_PIN)) // Verify ACK ('0') received from sensor
        error = S_Err_NoACK;
    usleep(usleep(PULSE_SHORT));
    gpioWrite(CLOCK_PIN, LOW); // Finish with clock in low state
    return error;
}

// Read byte from sensor and send acknowledge if "ack" is true
uint8_t getByte(bool ack)
{
    uint8_t i;
    uint8_t result = 0;
    for (i = 8; i > 0; i--) {
        result <<= 1;               // Shift received bits towards MSB
        gpioWrite(CLOCK_PIN, HIGH); // Generate clock pulse
        usleep(PULSE_SHORT);
        result |= gpioRead(DATA_PIN); // Merge next bit into LSB position
        gpioWrite(CLOCK_PIN, LOW);
        usleep(PULSE_SHORT);
    }
    gpioSetMode(DATA_PIN, PI_OUTPUT);
    gpioWrite(DATA_PIN, !ack); // Assert ACK ('0') if ack == 1
    usleep(PULSE_SHORT);
    gpioWrite(CLOCK_PIN, HIGH); // Clock #9 for ACK / noACK
    usleep(PULSE_LONG);
    gpioWrite(CLOCK_PIN, LOW); // Finish with clock in low state
    usleep(PULSE_SHORT);
    gpioSetMode(DATA_PIN, PI_INPUT); // Return data line to input mode
#ifdef DATA_PU
    gpioWrite(DATA_PIN, DATA_PU); // Restore internal pullup state
#endif
    return result;
}


/******************************************************************************
 * Sensirion signaling
 ******************************************************************************/

// Generate Sensirion-specific transmission start sequence
// This is where Sensirion does not conform to the I2C standard and is
// the main reason why the AVR TWI hardware support can not be used.
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
void startTransmission(void)
{
    gpioWrite(DATA_PIN, HIGH);        // Set data register high before turning on
    gpioSetMode(DATA_PIN, PI_OUTPUT); // output driver (avoid possible low pulse)
    usleep(PULSE_SHORT);
    gpioWrite(CLOCK_PIN, HIGH);
    usleep(PULSE_SHORT);
    gpioWrite(DATA_PIN, LOW);
    usleep(PULSE_SHORT);
    gpioWrite(CLOCK_PIN, LOW);
    usleep(PULSE_LONG);
    gpioWrite(CLOCK_PIN, HIGH);
    usleep(PULSE_SHORT);
    gpioWrite(DATA_PIN, HIGH);
    usleep(PULSE_SHORT);
    gpioWrite(CLOCK_PIN, LOW);
    usleep(PULSE_SHORT);
    // Unnecessary here since putByte always follows startTransmission
    //  gpioSetMode(DATA_PIN, PI_INPUT);
}

// Communication link reset
// At least 9 SCK cycles with DATA=1, followed by transmission start sequence
//      ______________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
void resetConnection(void)
{
    uint8_t i;
    gpioWrite(DATA_PIN, HIGH);        // Set data register high before turning on
    gpioSetMode(DATA_PIN, PI_OUTPUT); // output driver (avoid possible low pulse)
    usleep(PULSE_LONG);
    for (i = 0; i < 9; i++) {
        gpioWrite(CLOCK_PIN, HIGH);
        usleep(PULSE_LONG);
        gpioWrite(CLOCK_PIN, LOW);
        usleep(PULSE_LONG);
    }
    startTransmission();
}


/******************************************************************************
 * Helper Functions
 ******************************************************************************/

// Calculates temperature in degrees C from raw sensor data
float calcTemp(uint16_t rawData)
{
    if (_stat_reg & LOW_RES)
        return D1 + D2l * (float)rawData;
    else
        return D1 + D2h * (float)rawData;
}

// Calculates relative humidity from raw sensor data
//   (with temperature compensation)
float calcHumi(uint16_t rawData, float temp)
{
    float humi;
    if (_stat_reg & LOW_RES) {
        humi = C1 + C2l * rawData + C3l * rawData * rawData;
        humi = (temp - 25.0) * (T1 + T2l * rawData) + humi;
    } else {
        humi = C1 + C2h * rawData + C3h * rawData * rawData;
        humi = (temp - 25.0) * (T1 + T2h * rawData) + humi;
    }
    if (humi > 100.0)
        humi = 100.0;
    if (humi < 0.1)
        humi = 0.1;
    return humi;
}

// Calculates dew point in degrees C
float calcDewpoint(float humi, float temp)
{
    float k;
    // TODO  k = log(humi/100) + (17.62 * temp) / (243.12 + temp);
    return 243.12 * k / (17.62 - k);
}

#ifdef CRC_ENA
// Calculate CRC for a single byte
void calcCRC(uint8_t value, uint8_t *crc)
{
    const uint8_t POLY = 0x31; // Polynomial: x**8 + x**5 + x**4 + 1
    uint8_t i;
    *crc ^= value;
    for (i = 8; i > 0; i--) {
        if (*crc & 0x80)
            *crc = (*crc << 1) ^ POLY;
        else
            *crc = (*crc << 1);
    }
}

// Bit-reverse a byte (for CRC calculations)
uint8_t bitrev(uint8_t value)
{
    uint8_t i;
    uint8_t result = 0;
    for (i = 8; i > 0; i--) {
        result = (result << 1) | (value & 0x01);
        value >>= 1;
    }
    return result;
}
#endif
