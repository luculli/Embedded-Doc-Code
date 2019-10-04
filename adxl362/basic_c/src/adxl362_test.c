/* adxl362 - Basic configuration
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
 * @brief File containing a basci configuration for the adxl362 on the RPI 3B+
 */

#include "adxl362.h"

#define MEASURES_NB 100

int main() {
    uint8_t mode = 0, bits = 8, res = 0;
    uint32_t speed = SPEED_8MHZ;
    int file, verbose = 0;

    float buffer[3] = {0.0, 0.0, 0.0};
    float offsets[AXIS_NB] = {0.0};

    verbose = 1;
    file = adxl362_init(mode, bits, speed, offsets, verbose);
    if (file < 0) {
        printf("ERROR during intialization\n");
        return -1;
    }

    // Test one shot accelerations
    for (int measure = 0; measure < MEASURES_NB; measure++) {
        adxl362_get_accelerations(file, offsets, buffer, verbose);
        printf("%0.2f %0.2f %0.2f\n", buffer[0], buffer[1], buffer[2]);
        usleep(50 * 1000);
    }

    // Test fifo reading
    // Read half fifo
    adxl362_get_data_fifo(file, offsets, verbose);

    // Read half fifo
    adxl362_get_data_fifo(file, offsets, verbose);

    adxl362_stop(file);

    return 0;
}
