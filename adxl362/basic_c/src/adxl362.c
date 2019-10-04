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

static const char *device = "/dev/spidev0.1";

// 2-complement inversion
int16_t inv_2comp(const int16_t val) {
    if ((val & 0x0800) == 0)
        return (val);
    else
        return (-(0x0FFF - (val - 1)));
}

int adxl362_spi_init(uint8_t mode, uint8_t bits, uint32_t speed, uint8_t _verbose) {
    int file;

    if ((file = open(device, O_RDWR)) < 0) {
        perror("SPI: Can't open device.");
        return -1;
    }

    if (ioctl(file, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("SPI: Can't set SPI mode.");
        return -1;
    }

    if (ioctl(file, SPI_IOC_RD_MODE, &mode) == -1) {
        perror("SPI: Can't get SPI mode.");
        return -1;
    }

    if (ioctl(file, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
        perror("SPI: Can't set bits per word.");
        return -1;
    }

    if (ioctl(file, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1) {
        perror("SPI: Can't get bits per word.");
        return -1;
    }

    if (ioctl(file, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("SPI: Can't set max speed HZ");
        return -1;
    }

    if (ioctl(file, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1) {
        perror("SPI: Can't get max speed HZ.");
        return -1;
    }

    printf("*** Init ADXL362 ***\n");
    printf("SPI mode: %d\n", mode);
    printf("Bits per word: %d\n", bits);
    printf("Speed: %d Hz\n\n", speed);

    return file;
}

// Init the device by setting its basic parameters
int adxl362_init(uint8_t mode, uint8_t bits, uint32_t speed, float _offsets[3], uint8_t _verbose) {
    int file = adxl362_spi_init(mode, bits, speed, _verbose);

    adxl362_reset(file, _verbose);

    // set measurement mode
    adxl362_write_reg(file, ADXL362_REG_ADDR_POWER_CTL, 2, _verbose);

    usleep(500 * 1000);

    // read some registers
    printf("*** Reading some control registers ***\n");

    if (adxl362_check_reg_reading(file, _verbose) == false) {
        printf("ERROR, failed to read registers\n");
        return -1;
    }

    if (adxl362_selftest(file, _offsets, _verbose)) {
        printf("Self test succeed\n");
    } else {
        printf("ERROR self test failed!!!\n");
        return -1;
    }

    // Set acquisition parameters
    adxl362_set_odr(file, ODR_400HZ, _verbose);
    adxl362_set_bw(file, HALF_BW, _verbose);
    adxl362_set_sensitivity(file, ADXL362_SENSITIVITY, _verbose);

    // Compute offsets
    if (adxl362_compute_offsets(file, _offsets, _verbose) == false) {
        printf("ERROR, failed to compute offsets\n");
        return -1;
    }

    usleep(500 * 1000); // sleep for 500ms

    return (file);
}

bool adxl362_compute_offsets(int _file, float _offsets[3], uint8_t _verbose) {
    float mean[AXIS_NB] = {0.0};
    float nullOffsets[AXIS_NB] = {0.0};

    for (int axis = 0; axis < AXIS_NB; axis++)
        _offsets[axis] = 0.0;

    if (adxl362_compute_accel_mean(_file, OFFSETS_MEASURES_NB, OFFSETS_THRESHOLD, nullOffsets,
                                   _offsets, _verbose)
        == false)
        return false;

    _offsets[AXIS_Z] -= 1;
    if (_verbose)
        printf("----------------------------> Offsets computed: %0.2f %0.2f %0.2f\n", _offsets[0],
               _offsets[1], _offsets[2]);

    return true;
}
// Stop the access to the deivce
void adxl362_stop(int file) {
    close(file);
}

bool adxl362_check_reg_reading(int _file, uint8_t _verbose) {
    if (adxl362_read_reg(_file, ADXL362_REG_ADDR_DEVID_AD, _verbose) != REG_VAL_DEVID_AD) {
        printf("ERROR, wrong value in DEVID_AD register\n");
        return false;
    }

    if (adxl362_read_reg(_file, ADXL362_REG_ADDR_DEVID_MST, _verbose) != REG_VAL_DEVID_MST) {
        printf("ERROR, wrong value in DEVID_MST register\n");
        return false;
    }

    if (adxl362_read_reg(_file, ADXL362_REG_ADDR_PARTID, _verbose) != REG_VAL_PARTID) {
        printf("ERROR, wrong value in ADDR_PARTID register\n");
        return false;
    }

    if (adxl362_read_reg(_file, ADXL362_REG_ADDR_REVID, _verbose) != REG_VAL_REVID) {
        printf("ERROR, wrong value in ADDR_REVID register\n");
        return false;
    }

    return true;
}

void adxl362_set_bw(int _file, uint8_t _BW, uint8_t _verbose) {
    uint8_t currVal = adxl362_read_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, _verbose);
    currVal = currVal & 0b11101111; // Set bit 4 to 0
    uint8_t newVal = currVal | ((_BW << 4) & 0b00010000); // Update bit 4

    adxl362_write_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, newVal, _verbose);
    newVal = adxl362_read_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, _verbose);

    if (_verbose)
        printf("Register control filter: was %02X now: %02X\n", currVal, newVal);
}

void adxl362_set_odr(int _file, uint8_t _ODR, uint8_t _verbose) {
    uint8_t currVal = adxl362_read_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, _verbose);
    currVal = currVal & 0b11111000;
    uint8_t newVal = currVal | (_ODR & 0b00000111);

    adxl362_write_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, newVal, _verbose);
    newVal = adxl362_read_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, _verbose);

    if (_verbose)
        printf("Register control filter: was %02X now: %02X\n", currVal, newVal);
}

void adxl362_set_sensitivity(int _file, int8_t _sensitivity, uint8_t _verbose) {
    uint8_t currVal = adxl362_read_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, _verbose);
    currVal = currVal & 0b00111111;
    uint8_t newVal = currVal | ((_sensitivity << 6) & 0b11000000);

    adxl362_write_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, newVal, _verbose);
    newVal = adxl362_read_reg(_file, ADXL362_REG_ADRR_FILTER_CTL, _verbose);

    if (_verbose)
        printf("set_sensitivity: Register control filter: was %02X now: %02X\n", currVal, newVal);
}

// Read a single register from the device
uint8_t adxl362_read_reg(int file, uint8_t reg_addr, uint8_t verbose) {
    unsigned char send[3] = {0}, receive[3] = {0};

    send[0] = CMD_READ_REG;
    send[1] = reg_addr;
    struct spi_ioc_transfer transfer = {
            .tx_buf = (unsigned long)send, .rx_buf = (unsigned long)receive, .len = 3,
    };

    if (ioctl(file, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        perror("Failed to send SPI message");
        return -1;
    }

    if (verbose)
        printf("Return value: reg[0x%x] = 0x%x\n", reg_addr, receive[2] & 0xff);

    return (receive[2]);
}

// Write a single register to the device
void adxl362_write_reg(int file, uint8_t reg_addr, uint8_t value, uint8_t verbose) {
    unsigned char send[3] = {0}, receive[3] = {0};

    send[0] = CMD_WRITE_REG;
    send[1] = reg_addr;
    send[2] = value & 0xff;

    struct spi_ioc_transfer transfer = {
            .tx_buf = (unsigned long)send, .rx_buf = (unsigned long)receive, .len = 3,
    };

    if (ioctl(file, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        perror("Failed to send SPI message");
    } else if (verbose)
        printf("Written value: 0x%x @ reg[0x%x]\n", value & 0xff, reg_addr);
}

float adxl362_get_scale_factor() {
    if (ADXL362_SENSITIVITY == ADXL362_SENSITIVITY_2G)
        return ADXL362_SCALE_FACTOR_2G;
    else if (ADXL362_SENSITIVITY == ADXL362_SENSITIVITY_4G)
        return ADXL362_SCALE_FACTOR_4G;
    else if (ADXL362_SENSITIVITY == ADXL362_SENSITIVITY_8G)
        return ADXL362_SCALE_FACTOR_8G;
    else {
        printf("ERROR, unknown acceleration sensitivity!!!\n");
        return -1;
    }
}

void adxl362_get_accelerations(int _file, float _offsets[3], float _buffer[3], uint8_t _verbose) {

    uint8_t rawBuffer[6] = {0};
    int16_t tempBuffer[AXIS_NB] = {0};
    float scaleFactor = adxl362_get_scale_factor();

    adxl362_read_nreg(_file, ADXL362_REG_ADDR_XDATA_LSB, 6, rawBuffer, 0);

    for (int axis = 0; axis < AXIS_NB; axis++) {
        tempBuffer[axis] =
                (int16_t)(((rawBuffer[axis * 2 + 1] & 0x0F) << 8) + (rawBuffer[axis * 2] & 0xFF));
        _buffer[axis] = (float)(inv_2comp(tempBuffer[axis]) * scaleFactor) - _offsets[axis];
    }

    if (_verbose)
        printf("%0.2f %0.2f %0.2f\n", _buffer[0], _buffer[1], _buffer[2]);
}

// Read len consecutive registers from the device
void adxl362_read_nreg(int file, uint8_t reg_addr, uint8_t len, uint8_t *obuf, uint8_t verbose) {
    unsigned char send[len + 2], receive[len + 2];

    if (len == 0) {
        printf("adxl362_read_nreg - ERROR, len must be > 0!!!\n");
        return;
    }

    memset(send, 0, sizeof(send));
    memset(receive, 0, sizeof(receive));
    send[0] = CMD_READ_REG;
    send[1] = reg_addr;

    struct spi_ioc_transfer transfer = {
            .tx_buf = (unsigned long)send, .rx_buf = (unsigned long)receive, .len = len + 2,
    };

    if (ioctl(file, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        perror("Failed to send SPI message\n");
    } else {
        if (verbose)
            for (int i = 0; i < len; i++) {
                printf("Return value: reg[0x%x] = 0x%x\n", reg_addr + i, receive[i + 2] & 0xff);
            }

        // copy data in obuf
        if (obuf != NULL)
            memcpy(obuf, receive + 2, len);
    }
}

// Read len consecutive registers from the device
void adxl362_read_nfifo(int file, uint16_t len, uint8_t *obuf, uint8_t verbose) {
    if (len == 0) {
        printf("adxl362_read_nfifo - ERROR, len must be > 0!!!\n");
        return;
    }

    if (adxl362_check_fifo_rdy(file) == 0) {
        printf("ERROR, FIFO not ready!!!\n");
        memset(obuf, 0, sizeof(len));
        return;
    }

    if (adxl362_check_fifo_ovr(file) != 0)
        printf("Warning, FIFO overrun detected!!\n");

    unsigned char send[len + 1], receive[len + 1];

    memset(send, 0, sizeof(send));
    memset(receive, 0, sizeof(receive));
    send[0] = CMD_READ_FIFO;

    struct spi_ioc_transfer transfer = {
            .tx_buf = (unsigned long)send, .rx_buf = (unsigned long)receive, .len = len + 1,
    };

    if (ioctl(file, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        perror("Failed to send SPI message\n");
        return;
    }

    if (verbose) {
        for (int i = 0; i < (len + 1); i++)
            printf("send[%d]: %04X receive[%d]: 0x%x\n", i, send[i], i, receive[i]);
    }

    // copy data in obuf
    if (obuf != NULL)
        memcpy(obuf, receive + 1, len);
}

void adxl362_reset(int _file, uint8_t _verbose) {
    adxl362_write_reg(_file, ADXL362_REG_ADDR_RESET, 0x52, _verbose);
    usleep(10 * 1000); // "A latency of approximately 0.5ms is required after soft reset"
}

bool adxl362_compute_accel_mean(int _file, int _samplesNb, float _threshold, float _offsets[3],
                                float _buffer[3], uint8_t _verbose) {
    float accel[AXIS_NB] = {0.0};
    float sum[AXIS_NB] = {0.0};
    float max[AXIS_NB] = {0.0};
    float min[AXIS_NB] = {100.0};

    if (_samplesNb <= 0) {
        printf("adxl362_compute_accel_mean - ERROR, samples number must be "
               "superior to 0\n");
        return false;
    }

    adxl362_get_accelerations(_file, _offsets, accel, _verbose);
    usleep(100 * 1000);
    for (int measure = 0; measure < _samplesNb; measure++) {
        adxl362_get_accelerations(_file, _offsets, accel, _verbose);
        for (int axis = 0; axis < AXIS_NB; axis++) {
            if (accel[axis] > max[axis])
                max[axis] = accel[axis];

            if (accel[axis] < min[axis])
                min[axis] = accel[axis];

            sum[axis] += accel[axis];
        }
        usleep(100 * 1000);
    }

    for (int axis = 0; axis < AXIS_NB; axis++) {
        if ((max[axis] - min[axis]) > _threshold) {
            printf("ERROR during offset computation of axis %d: min: %0.2lf max: "
                   "%0.2lf delta = "
                   "%0.2f\n",
                   axis, min[axis], max[axis], max[axis] - min[axis]);
            return false;
        }
    }

    for (int axis = 0; axis < AXIS_NB; axis++)
        _buffer[axis] = sum[axis] / _samplesNb;

    if (_verbose)
        printf("Mean acceleration: %0.2f %0.2f %0.2f\n", _buffer[0], _buffer[1], _buffer[2]);
    return true;
}

// Self-test of adxl362 to check its integrity
bool adxl362_selftest(int file, float _offsets[3], uint8_t _verbose) {
    bool res = true;

    // Self test requires 8g range
    int8_t savedRegFilterCtl = adxl362_read_reg(file, ADXL362_REG_ADRR_FILTER_CTL, _verbose);
    if (_verbose)
        printf("Filter control register before self test: %02X\n", savedRegFilterCtl);
    adxl362_set_sensitivity(file, ADXL362_SENSITIVITY_8G, _verbose);

    // Self test requires 100 Hz ODR
    adxl362_set_odr(file, ODR_100HZ, _verbose);

    // Clear half bw
    adxl362_set_bw(file, HALF_BW, _verbose);

    float accelBeforeTest[AXIS_NB] = {0};
    float accelAfterTest[AXIS_NB] = {0};

    int measuresNb = 16;

    if (adxl362_compute_accel_mean(file, measuresNb, MEAN_THRESHOLD, _offsets, accelBeforeTest,
                                   _verbose)
        == false)
        res = false;

    adxl362_write_reg(file, ADXL362_REG_ADDR_SELF_TEST, 1, _verbose);

    usleep(40 * 1000);

    if (adxl362_compute_accel_mean(file, measuresNb, MEAN_THRESHOLD, _offsets, accelAfterTest,
                                   _verbose)
        == false)
        res = false;

    adxl362_write_reg(file, ADXL362_REG_ADDR_SELF_TEST, 0, _verbose);

    float delta = accelAfterTest[AXIS_X] - accelBeforeTest[AXIS_X];
    if ((delta < SELF_TEST_X_MIN) || (delta > SELF_TEST_X_MAX)) {
        printf("ERROR self test failed on X: %0.2lf\n", delta);
        res = false;
    }

    delta = accelAfterTest[AXIS_Y] - accelBeforeTest[AXIS_Y];
    if ((delta < SELF_TEST_Y_MIN) || (delta > SELF_TEST_Y_MAX)) {
        printf("ERROR self test failed on Y: %0.2lf\n", delta);
        res = false;
    }

    delta = accelAfterTest[AXIS_Z] - accelBeforeTest[AXIS_Z];
    if ((delta < SELF_TEST_Z_MIN) || (delta > SELF_TEST_Z_MAX)) {
        printf("ERROR self test failed on Z: %0.2lf\n", delta);
        res = false;
    }

    // Restore filter control register
    adxl362_write_reg(file, ADXL362_REG_ADRR_FILTER_CTL, savedRegFilterCtl, _verbose);
    uint8_t restoredRegFilterCtl = adxl362_read_reg(file, ADXL362_REG_ADRR_FILTER_CTL, _verbose);
    if (savedRegFilterCtl != restoredRegFilterCtl) {
        printf("ERROR, failed to restore filter register!!!\n");
        res = false;
    }
    if (_verbose)
        printf("Filter control register after self test: %02X\n", savedRegFilterCtl);

    return res;
}
int16_t adxl362_get_fifo_size(int file) {
    uint8_t buf[2] = {
            0,
    };
    adxl362_read_nreg(file, ADXL362_REG_ADDR_FIFO_ENTRIES_L, 2, buf, 0);

    return (uint16_t)(((buf[1] & 0x03) << 8) | buf[0]);
}

// Check if the fifo is full
uint8_t adxl362_check_fifo_full(int file) {
    return (adxl362_read_reg(file, ADXL362_REG_ADDR_STATUS, 0) & 0x04);
}

// Check if the fifo is on overrun (i.e. too many data)
uint8_t adxl362_check_fifo_ovr(int file) {
    return (adxl362_read_reg(file, ADXL362_REG_ADDR_STATUS, 0) & 0x08);
}

// Check if there is 1 data sample at least in the fifo
uint8_t adxl362_check_fifo_rdy(int file) {
    return (adxl362_read_reg(file, ADXL362_REG_ADDR_STATUS, 0) & 0x02);
}

void adxl362_get_data_fifo(int file, float _offsets[3], uint8_t verbose) {
    uint8_t res = 0;
    uint8_t buf[512] = {
            0,
    };
    int16_t tempAccVal = 0;
    float xacc, yacc, zacc = -9999.0;
    int16_t size, sample = 0;
    int16_t tuplesNb = 0;
    int16_t halfSizeFIFO = 0;
    float scaleFactor = adxl362_get_scale_factor();

    printf("\n*** Reading Accelerations from FIFO ***\n");
    printf("Defined offsets: %0.2f %0.2f %0.2f\n", _offsets[0], _offsets[1], _offsets[2]);

    // set standby  mode
    adxl362_write_reg(file, ADXL362_REG_ADDR_POWER_CTL, 0x00, verbose);

    // set fifo size (9bits) 512 samples -> 170 tuples XYZ
    adxl362_write_reg(file, ADXL362_REG_ADDR_FIFO_SAMPLES, 0xFF, verbose);
    res = adxl362_read_reg(file, ADXL362_REG_ADDR_FIFO_SAMPLES, verbose);

    // set fifo in stream mode saving XYZ
    adxl362_write_reg(file, ADXL362_REG_ADDR_FIFO_CTL, 0b0001010,
                      verbose); // Above half, stream mode

    // set measurement mode + 16ms filter settling time
    res = adxl362_read_reg(file, ADXL362_REG_ADDR_POWER_CTL, verbose);
    adxl362_write_reg(file, ADXL362_REG_ADDR_POWER_CTL, 0x02, verbose); // Measurement mode

    usleep(1000);

    size = 0;
    if (verbose)
        printf("Waiting for fifo ready + data ...");

    while (size < 512) {
        if (adxl362_check_fifo_rdy(file)) {
            size = adxl362_get_fifo_size(file); // size is the total # of samples NOT tuples
                                                // printf("fifo size: %i\n", size);
        }
        usleep(1000);
    }
    if (verbose)
        printf("done\n");

    tuplesNb = size / 6 - 1;           // get an integer number of tuples <x,y,z> in 16-bits
    halfSizeFIFO = (tuplesNb / 2) * 6; // One tuple = 6 FIFO bytes

    // read FIFO Acc X,Y,Z
    printf("fifo size: %d tuples: %d (tuples/2)*6: %d\n", size, tuplesNb, (tuplesNb / 2) * 6);

    // Read half of the FIFO
    adxl362_read_nfifo(file, halfSizeFIFO, buf, 0);

    // convert and print data of fifo
    xacc = yacc = zacc = -9999;
    for (int byte = 0; byte < (halfSizeFIFO / 2); byte++) {
        uint16_t twoBytes = ((buf[byte * 2 + 1] << 8) & 0xFF00) + buf[byte * 2];
        tempAccVal = inv_2comp(twoBytes & 0xFFF);

        if (((twoBytes & 0xC000) >> 14) == 0) {
            xacc = ((float)tempAccVal) * scaleFactor - _offsets[AXIS_X];
        } else if (((twoBytes & 0xC000) >> 14) == 1) {
            yacc = ((float)tempAccVal) * scaleFactor - _offsets[AXIS_Y];
        } else if (((twoBytes & 0xC000) >> 14) == 2) {
            zacc = ((float)tempAccVal) * scaleFactor - _offsets[AXIS_Z];
            printf("Acceleration %d <x,y,z> (fifo mode): %0.1f %0.1f %0.1f\n", byte, xacc, yacc,
                   zacc);
            xacc = yacc = zacc = -9999;
        } else if (((twoBytes & 0xC000) >> 14) == 3) {
            printf("Was Temperature\n");
        } else
            printf("ERROR, unknown coord type in FIFO: %04X\n", twoBytes);
    }

    // put it in standby
    res = adxl362_read_reg(file, ADXL362_REG_ADDR_POWER_CTL, 0);
    adxl362_write_reg(file, ADXL362_REG_ADDR_POWER_CTL, res & 0x00, 0);
}
