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

#ifndef ADXL362_H_
#define ADXL362_H_

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define AXIS_NB 3
#define OFFSETS_MEASURES_NB 10
#define OFFSETS_THRESHOLD 3
#define MEAN_THRESHOLD 3

// Self test thresholds
#define SELF_TEST_X_MIN 0.2
#define SELF_TEST_X_MAX 2.8
#define SELF_TEST_Y_MIN -2.8
#define SELF_TEST_Y_MAX -0.2
#define SELF_TEST_Z_MIN 0.2
#define SELF_TEST_Z_MAX 2.8

// ADXL362 registers addresses
#define ADXL362_REG_ADDR_DEVID_AD 0x00
#define ADXL362_REG_ADDR_DEVID_MST 0x01
#define ADXL362_REG_ADDR_PARTID 0x02
#define ADXL362_REG_ADDR_REVID 0x03
#define ADXL362_REG_ADDR_STATUS 0x0B
#define ADXL362_REG_ADDR_XDATA_LSB 0x0E
#define ADXL362_REG_ADDR_XDATA_MSB 0x0F
#define ADXL362_REG_ADDR_YDATA_LSB 0x10
#define ADXL362_REG_ADDR_YDATA_MSB 0x11
#define ADXL362_REG_ADDR_ZDATA_LSB 0x12
#define ADXL362_REG_ADDR_ZDATA_MSB 0x13
#define ADXL362_REG_ADDR_RESET 0x1F
#define ADXL362_REG_ADDR_FIFO_CTL 0x28
#define ADXL362_REG_ADRR_FILTER_CTL 0x2C
#define ADXL362_REG_ADDR_POWER_CTL 0x2D
#define ADXL362_REG_ADDR_SELF_TEST 0x2E
#define ADXL362_REG_ADDR_FIFO_ENTRIES_L 0x0C
#define ADXL362_REG_ADDR_FIFO_ENTRIES_H 0x0D
#define ADXL362_REG_ADDR_FIFO_CTL 0x28
#define ADXL362_REG_ADDR_FIFO_SAMPLES 0x29

// ADXL362 register values
#define REG_VAL_DEVID_AD 0xAD
#define REG_VAL_DEVID_MST 0x1D
#define REG_VAL_PARTID 0xF2
#define REG_VAL_REVID 0x02

// Clock speeds
#define SPEED_8MHZ 8000000
#define SPEED_5MHZ 5000000
#define SPEED_1MHZ 1000000
#define SPEED_1KHZ 1000

// Commands
#define CMD_WRITE_REG 0x0A
#define CMD_READ_REG 0x0B
#define CMD_READ_FIFO 0x0D

// Sensitivity
#define ADXL362_SENSITIVITY_2G 0
#define ADXL362_SENSITIVITY_4G 1
#define ADXL362_SENSITIVITY_8G 2

#define ADXL362_SENSITIVITY ADXL362_SENSITIVITY_8G

#define ADXL362_SCALE_FACTOR_2G 0.001 // (1 mg/LSB for 2g)
#define ADXL362_SCALE_FACTOR_4G 0.002 // (2 mg/LSB for 4g)
#define ADXL362_SCALE_FACTOR_8G 0.004 // (4 mg/LSB for 8g)

#define ODR_12_5HZ 0
#define ODR_25HZ 1
#define ODR_50HZ 2
#define ODR_100HZ 3
#define ODR_200HZ 4
#define ODR_400HZ 5

#define HALF_BW 0
#define QUARTER_BW 1

int adxl362_init(uint8_t mode, uint8_t bits, uint32_t speed, float _offsets[3], uint8_t _verbose);
void adxl362_get_accelerations(int _file, float _offsets[3], float _buffer[3], uint8_t _verbose);
void adxl362_get_data_fifo(int file, float _offsets[3], uint8_t verbose);

int adxl362_spi_init(uint8_t mode, uint8_t bits, uint32_t speed, uint8_t _verbose);
float adxl362_get_scale_factor();
int16_t inv_2comp(const int16_t val);
void adxl362_stop(int file);
uint8_t adxl362_read_reg(int file, uint8_t reg_addr, uint8_t verbose);
void adxl362_set_sensitivity(int _file, int8_t _sensitivity, uint8_t _verbose);
void adxl362_set_odr(int _file, uint8_t _ODR, uint8_t _verbose);
void adxl362_set_bw(int _file, uint8_t _BW, uint8_t _verbose);
bool adxl362_check_reg_reading(int _file, uint8_t _verbose);
void adxl362_write_reg(int file, uint8_t reg_addr, uint8_t value, uint8_t verbose);
void adxl362_read_2reg(int file, uint8_t reg_addr, uint8_t verbose);
bool adxl362_selftest(int file, float _offset[3], uint8_t _verbose);
void adxl362_reset(int _file, uint8_t _verbose);
void adxl362_get_data(int file);
void adxl362_get_data_sample(int file, float *buffer);
void adxl362_read_nreg(int file, uint8_t reg_addr, uint8_t len, uint8_t *obuf, uint8_t verbose);
bool adxl362_compute_offsets(int _file, float _offsets[3], uint8_t _verbose);
// Fifo
void adxl362_read_nfifo(int file, uint16_t len, uint8_t *obuf, uint8_t verbose);
uint8_t adxl362_check_fifo_ovr(int file);
uint8_t adxl362_check_fifo_rdy(int file);
bool adxl362_compute_accel_mean(int _file, int _samplesNb, float _threshold, float _offsets[3],
                                float _buffer[3], uint8_t _verbose);

#endif
