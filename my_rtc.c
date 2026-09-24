/* my_rtc.c - Modified to use DS3231 RTC   JPB 9/22/2026 */

#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ff.h"

#define I2C_PORT i2c1
#define DS3231_ADDR 0x68

// Required by crash.c
time_t epochtime = 0;

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day_of_week;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} ds3231_time_t;

// Convert BCD to decimal
static uint8_t bcd_to_decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Read complete DS3231 time/date
static void read_ds3231_time(ds3231_time_t *time) {
    uint8_t buffer[7];
    uint8_t reg = 0x00;  // Start at Seconds register
    
    i2c_write_blocking(I2C_PORT, DS3231_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, DS3231_ADDR, buffer, 7, false);
    
    time->seconds = bcd_to_decimal(buffer[0]);
    time->minutes = bcd_to_decimal(buffer[1]);
    time->hours = bcd_to_decimal(buffer[2] & 0x3F);
    time->day_of_week = buffer[3];
    time->date = bcd_to_decimal(buffer[4]);
    time->month = bcd_to_decimal(buffer[5] & 0x1F);
    time->year = bcd_to_decimal(buffer[6]);
}

/**
 * @brief Get the current time in the FAT time format from DS3231
 */
DWORD get_fattime(void) {
    ds3231_time_t time;
    read_ds3231_time(&time);
    
    DWORD fattime = 0;
    fattime |= ((time.year + 20) & 0x7F) << 25;
    fattime |= (time.month & 0x0F) << 21;
    fattime |= (time.date & 0x1F) << 16;
    fattime |= (time.hours & 0x1F) << 11;
    fattime |= (time.minutes & 0x3F) << 5;
    fattime |= (time.seconds / 2) & 0x1F;
    
    return fattime;
}

void time_init() {
    // DS3231 doesn't need initialization - it has a battery backup
}
