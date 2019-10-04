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

#include <pigpio.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <stdbool.h>

extern void sht75_Init();
extern float sht75_readTemperature();
extern float sht75_readHumidity(const float _temp);

int main() {
	float temp=0, humidity=0;

	// init the sensor
	sht75_Init();

	while(1) {
		// read data
		temp = sht75_readTemperature();
		humidity = sht75_readHumidity(temp);

		printf("The <temperature, humidity> are %0.2f %0.2f\n", temp, humidity);
	}
}


