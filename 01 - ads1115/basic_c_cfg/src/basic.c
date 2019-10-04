/* ads1115 - Basic configuration
 *
 * Copyright (C) 2018  Gabriele LUCULLI
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
 * @date 08 Jun 2018
 * @brief File containing a basci configuration for the DAC ads115 on the RPI 3B+
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main() {

  int ADS_address = 0x48;	// Address of our device on the I2C bus
  int I2CFile;

  uint8_t writeBuf[3];		// Buffer to store the 3 bytes that we write to the I2C device
  uint8_t readBuf[2];		// 2 byte buffer to store the data read from the I2C device

  int16_t val;				// Stores the 16 bit value of our ADC conversion

  // Open the I2C device
  if((I2CFile = open("/dev/i2c-1", O_RDWR)) < 0){
    perror("Failed to open the bus\n");
    return 1;
  }

  // Specify the address of the I2C Slave to communicate with
  if(ioctl(I2CFile, I2C_SLAVE, ADS_address) < 0){
    perror("Failed to connect to the sensor\n");
    return 1;
  }

  // These three bytes are written to the ADS1115 to set the config register and start a conversion
  writeBuf[0] = 0x01;		// This sets the pointer register so that the following two bytes write to the config register
  writeBuf[1] = 0xC3;   	// This sets the 8 MSBs of the config register (bits 15-8) to 11000011
  writeBuf[2] = 0xE3;  		// This sets the 8 LSBs of the config register (bits 7-0) to 11100011

  // Initialize the buffer used to read data from the ADS1115 to 0
  readBuf[0]= 0;
  readBuf[1]= 0;

  // Write writeBuf to the ADS1115, the 3 specifies the number of bytes we are writing,
  // this begins a single conversion
  write(I2CFile, writeBuf, 3);

  // Wait for the conversion to complete, this requires bit 15 to change from 0->1
  while ((readBuf[0] & 0x80) == 0)	// readBuf[0] contains 8 MSBs of config register, AND with 10000000 to select bit 15
  {
	  read(I2CFile, readBuf, 2);	// Read the config register into readBuf
  }

  writeBuf[0] = 0;			// Set pointer register to 0 to read from the conversion register
  write(I2CFile, writeBuf, 1);

  read(I2CFile, readBuf, 2);		// Read the contents of the conversion register into readBuf

  val = readBuf[0] << 8 | readBuf[1];	// Combine the two bytes of readBuf into a single 16 bit result 

  printf("Voltage Reading %f (V) \n", (float)val*4.096/32767.0);	// Print the result to terminal, first convert from binary value to mV

  close(I2CFile);
  return 0;

}


