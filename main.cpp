#include "mbed.h"
#include "math.h"

/**
 * All the following code is original, no libraries, other than what Mbed 
 * provides, were used.
 */

I2C i2c(p9, p10);

/*
Byte reference:

index: 7   6  5  4  | 3  2  1  0
value: 128 64 32 16 | 8  4  2  1
       8   4  2  1

I2C reference:

Addresses are shifted to the left once b/c the LSB in an I2C address
is used to indicate if the transaction is a read or write. This bit
will be set by the Mbed I2C API.
*/

/**
 * LiDAR constants
 */
const int LIDAR_ADDR = 0x62 << 1;

const char LIDAR_ACQ_CMD_REG = 0x00;
const char LIDAR_ACQ_CMD = 0x04;

const char LIDAR_STATUS_REG = 0x01;
const char LIDAR_STATUS_BUSY_MASK = 0x01;
const char LIDAR_STATUS_HEALTH_MASK = 0x20;

const char LIDAR_DELTA_VELOCITY_REG = 0x09;

// The high byte of the value is stored in 0x0f and the low byte in 0x10.
// The LiDAR sensor can automatically increment register read addresses if the
// MSB of an address to be set 1. 
// Setting the MSB of 0x0f to 1 = 0x8f, which triggers a read of first 0x0f then
// 0x10.
const char LIDAR_DIST_REG = 0x8f;

void die(char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    
    vprintf(fmt, args);
    printf("\r\n");
    
    va_end(args);
    
    exit(1);
}

/**
 * LiDAR status.
 */
typedef struct {
    /**
     * Indicates if the device is busy.
     * Boolean.
     */
    char busy;
    
    /**
     * Indicates if the device is healthy.
     * Boolean.
     */
    char healthy;
} lidar_status_t;

/**
 * Reads the LiDAR sensor's status.
 */
void lidar_read_status(lidar_status_t *lidar_status) {
    // Read register
    if (i2c.write(LIDAR_ADDR, &LIDAR_STATUS_REG, 1) != 0) {
        die("lidar: read_status: failed to select status register for read");
    }
    
    char buf;
    if (i2c.read(LIDAR_ADDR, &buf, 1) != 0) {
        die("lidar: read_status: failed to read status register");
    }

    // Unpack into lidar_status_t
    lidar_status->busy = buf & LIDAR_STATUS_BUSY_MASK;
    lidar_status->healthy = (buf * LIDAR_STATUS_HEALTH_MASK) >> 5;
}

/**
 * Exits program if the LiDAR sensor is in an error state.
 */
void lidar_die() {
    lidar_status_t lidar_status;
    lidar_read_status(&lidar_status);
    
    if (!lidar_status.healthy) {
        die("lidar: sensor is not healthy");
    }
}

/**
 * Sends an acquire data command to the LiDAR sensor.
 */
void lidar_write_acq_cmd() {
    char buf[2] = {
        LIDAR_ACQ_CMD_REG, 
        LIDAR_ACQ_CMD
    };
    if (i2c.write(LIDAR_ADDR, buf, 2) != 0) {
        die("lidar: write_acq_cmd: failed to write acquire command");
    }
}

/**
 * Reads the distance value from the LiDAR sensor.
 * Returns: Distance in centimeters.
 *
 */
uint16_t lidar_read_distance() {
    // Read
    if (i2c.write(LIDAR_ADDR, &LIDAR_DIST_REG, 1) != 0) {
        die("lidar: read_distance: failed to select distance registers for read");
    }
    
    char buf[2];
    if (i2c.read(LIDAR_ADDR, buf, 2) != 0) {
        die("lidar: read_distance: failed to read distance registers");
    }
    
    // Re-assemble
    return buf[1] | (buf[0] << 8);
}

/**
 * Reads the delta velocity value from the LiDAR sensor.
 * Returns: Difference in last velocity reading in cm.
 */
int8_t lidar_read_delta_velocity() {
    if (i2c.write(LIDAR_ADDR, &LIDAR_DELTA_VELOCITY_REG, 1) != 0) {
        die("lidar: read_delta_velocity: failed to select delta velocity register for read");
    }
    
    char buf;
    if (i2c.read(LIDAR_ADDR, &buf, 1) != 0) {
        die("lidar: read_delta_velocity: failed to read delta velocity register");
    }
    
    return (int8_t)buf;
}

int main() {
    int8_t lidar_velocity = 0;
    
    while (1) {
        // Send acquire command
        lidar_write_acq_cmd();
        
        // Wait until lidar isn't busy
        lidar_status_t lidar_status;
        
        do {
            lidar_read_status(&lidar_status);
            wait(0.5);
        } while (lidar_status.busy);
    
        // Read distance
        uint16_t lidar_distance = lidar_read_distance();
        
        if (lidar_distance >= ((uint16_t)9999)) {
            lidar_distance = 9999;
        }
        
        // Read velocity
        lidar_velocity += lidar_read_delta_velocity();
        
        printf("lidar: distance=%d cm, velocity=%d cm/s\r\n", lidar_distance,
            lidar_velocity);
    }
}
