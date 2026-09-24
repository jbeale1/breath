#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

#define VERSION "1.0.1"

// MPU-6050 Calibration Enable/Disable
#define ENABLE_CALIBRATION 1

// MPU-6050 Calibration Constants
// Measured on 2026-09-24 using cube face and edge method
// Apply when ENABLE_CALIBRATION = 1: x_cal = (x_raw - offset_x) * scale_x
#define CALIB_OFFSET_X 0.03092250f
#define CALIB_SCALE_X  1.00047773f
#define CALIB_OFFSET_Y -0.01023000f
#define CALIB_SCALE_Y  0.99378882f
#define CALIB_OFFSET_Z -0.07085750f
#define CALIB_SCALE_Z  0.97970780f

#define LED_PIN 25

// I2C for RTC (DS3231)
#define I2C_RTC i2c1
#define I2C_RTC_SDA 6
#define I2C_RTC_SCL 7
#define DS3231_ADDR 0x68

// I2C for MPU-6050
#define I2C_MPU i2c0
#define I2C_MPU_SDA 0
#define I2C_MPU_SCL 1
#define MPU6050_ADDR_LOW 0x68
#define MPU6050_ADDR_HIGH 0x69

#define BUFFER_SIZE 12

// MPU-6050 registers
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H 0x41

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day_of_week;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} ds3231_time_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float temperature;
    bool valid;
} mpu6050_reading_t;

typedef struct {
    uint32_t epoch;
    float accel_x;
    float accel_y;
    float accel_z;
    float temperature;
} data_point_t;

static ds3231_time_t current_time;
static data_point_t data_buffer[BUFFER_SIZE];
static int buffer_index = 0;
static uint8_t mpu6050_addr = 0;  // Will store detected address

// Apply calibration constants to acceleration values
void apply_calibration(float *x, float *y, float *z) {
#if ENABLE_CALIBRATION
    *x = (*x - CALIB_OFFSET_X) * CALIB_SCALE_X;
    *y = (*y - CALIB_OFFSET_Y) * CALIB_SCALE_Y;
    *z = (*z - CALIB_OFFSET_Z) * CALIB_SCALE_Z;
#endif
}

void blink_led(int times) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    for (int i = 0; i < times; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
    }
}

void blink_brief(int duration_ms) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
    sleep_ms(duration_ms);
    gpio_put(LED_PIN, 0);
}

void blink_double() {
    blink_brief(5);
    sleep_ms(100);
    blink_brief(5);
}

uint8_t bcd_to_decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

void read_ds3231_time(ds3231_time_t *time) {
    uint8_t buffer[7];
    uint8_t reg = 0x00;
    
    i2c_write_blocking(I2C_RTC, DS3231_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_RTC, DS3231_ADDR, buffer, 7, false);
    
    time->seconds = bcd_to_decimal(buffer[0]);
    time->minutes = bcd_to_decimal(buffer[1]);
    time->hours = bcd_to_decimal(buffer[2] & 0x3F);
    time->day_of_week = buffer[3];
    time->date = bcd_to_decimal(buffer[4]);
    time->month = bcd_to_decimal(buffer[5] & 0x1F);
    time->year = bcd_to_decimal(buffer[6]);
}

static uint32_t ds3231_to_epoch(ds3231_time_t *time) {
    uint32_t days = 0;
    
    for (int y = 0; y < time->year; y++) {
        days += (y % 4 == 0) ? 366 : 365;
    }
    
    int days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (time->year % 4 == 0) days_in_month[2] = 29;
    
    for (int m = 1; m < time->month; m++) {
        days += days_in_month[m];
    }
    
    days += time->date - 1;
    
    uint32_t epoch = (days + 10957) * 86400;
    epoch += time->hours * 3600;
    epoch += time->minutes * 60;
    epoch += time->seconds;
    
    return epoch;
}

// Detect MPU-6050 on I2C bus
void detect_mpu6050(void) {
    uint8_t reg = 0x75;  // WHO_AM_I register
    uint8_t data;
    
    printf("Detecting MPU-6050...\n");
    
    // Try 0x69 first
    i2c_write_blocking(I2C_MPU, MPU6050_ADDR_HIGH, &reg, 1, true);
    if (i2c_read_blocking(I2C_MPU, MPU6050_ADDR_HIGH, &data, 1, false) > 0 && data == 0x68) {
        mpu6050_addr = MPU6050_ADDR_HIGH;
        printf("MPU-6050 detected at address 0x69\n");
        return;
    }
    
    // Try 0x68
    i2c_write_blocking(I2C_MPU, MPU6050_ADDR_LOW, &reg, 1, true);
    if (i2c_read_blocking(I2C_MPU, MPU6050_ADDR_LOW, &data, 1, false) > 0 && data == 0x68) {
        mpu6050_addr = MPU6050_ADDR_LOW;
        printf("MPU-6050 detected at address 0x68\n");
        return;
    }
    
    printf("MPU-6050 not found on I2C bus\n");
}

// Initialize MPU-6050
void OLD_init_mpu6050(void) {
    if (!mpu6050_addr) return;
    
    // Wake up from sleep mode
    uint8_t cmd[2] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_MPU, mpu6050_addr, cmd, 2, false);
    sleep_ms(10);
    
    // Set accel config to ±2g range (0x00)
    cmd[0] = MPU6050_ACCEL_CONFIG;
    cmd[1] = 0x00;
    i2c_write_blocking(I2C_MPU, mpu6050_addr, cmd, 2, false);
}


void init_mpu6050(void) {
    if (!mpu6050_addr) return;
    
    // Wake up from sleep mode
    uint8_t cmd[2] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_MPU, mpu6050_addr, cmd, 2, false);
    sleep_ms(100);
    
    // Reset all registers
    cmd[0] = MPU6050_PWR_MGMT_1;
    cmd[1] = 0x80;  // Reset bit
    i2c_write_blocking(I2C_MPU, mpu6050_addr, cmd, 2, false);
    sleep_ms(100);
    
    // Wake up again
    cmd[0] = MPU6050_PWR_MGMT_1;
    cmd[1] = 0x00;
    i2c_write_blocking(I2C_MPU, mpu6050_addr, cmd, 2, false);
    sleep_ms(100);
    
    // Set accel config to ±2g range
    cmd[0] = MPU6050_ACCEL_CONFIG;
    cmd[1] = 0x00;
    i2c_write_blocking(I2C_MPU, mpu6050_addr, cmd, 2, false);
    sleep_ms(10);
    
    // Configure sample rate divider (lower value = faster sampling)
    cmd[0] = 0x19;  // SMPRT_DIV register
    cmd[1] = 0x00;  // Sample rate = 1000 / (1 + 0) = 1kHz
    i2c_write_blocking(I2C_MPU, mpu6050_addr, cmd, 2, false);
}

mpu6050_reading_t read_mpu6050(void) {
    mpu6050_reading_t result = {0, 0, 0, 0, false};
    
    if (!mpu6050_addr) return result;
    
    uint8_t buffer[8];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    
    i2c_write_blocking(I2C_MPU, mpu6050_addr, &reg, 1, true);
    i2c_read_blocking(I2C_MPU, mpu6050_addr, buffer, 8, false);
    
    // Convert accel data (16384 LSBs per g for ±2g range)
    int16_t accel_x_raw = ((int16_t)buffer[0] << 8) | buffer[1];
    int16_t accel_y_raw = ((int16_t)buffer[2] << 8) | buffer[3];
    int16_t accel_z_raw = ((int16_t)buffer[4] << 8) | buffer[5];
    
    result.accel_x = accel_x_raw / 16384.0f;
    result.accel_y = accel_y_raw / 16384.0f;
    result.accel_z = accel_z_raw / 16384.0f;
    
    // Apply calibration if enabled
    apply_calibration(&result.accel_x, &result.accel_y, &result.accel_z);
    
    // Convert temperature (raw / 340 + 36.53)
    int16_t temp_raw = ((int16_t)buffer[6] << 8) | buffer[7];
    result.temperature = (temp_raw / 340.0f) + 36.53f;
    
    result.valid = true;
    return result;
}

void list_directory(const char *path, int indent) {
    FRESULT fr;
    DIR dir;
    FILINFO fno;
    
    fr = f_opendir(&dir, path);
    if (fr != FR_OK) {
        printf("Failed to open directory: %s\n", path);
        return;
    }
    
    while (1) {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK || fno.fname[0] == 0) break;
        
        for (int i = 0; i < indent; i++) printf("  ");
        
        if (fno.fattrib & AM_DIR) {
            printf("[DIR]  %s\n", fno.fname);
            char subpath[256];
            snprintf(subpath, sizeof(subpath), "%s/%s", path, fno.fname);
            list_directory(subpath, indent + 1);
        } else {
            printf("[FILE] %s (%lu bytes)\n", fno.fname, fno.fsize);
        }
    }
    
    f_closedir(&dir);
}

void scan_i2c_bus(i2c_inst_t *i2c, const char *bus_name) {
    printf("\nScanning %s for I2C devices:\n", bus_name);
    printf("Found devices at addresses: ");
    
    bool found_any = false;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t rxdata;
        int result = i2c_read_blocking(i2c, addr, &rxdata, 1, false);
        if (result > 0) {
            printf("0x%02X ", addr);
            found_any = true;
        }
    }
    
    if (!found_any) {
        printf("(none)");
    }
    printf("\n\n");
}

void dump_registers(i2c_inst_t *i2c, uint8_t addr, const char *device_name) {
    printf("\nRegister dump for %s at 0x%02X:\n", device_name, addr);
    printf("Reg  | Hex Value\n");
    printf("-----+-----------\n");
    
    for (uint8_t reg = 0x00; reg < 0x80; reg++) {
        uint8_t data = 0;
        i2c_write_blocking(i2c, addr, &reg, 1, true);
        i2c_read_blocking(i2c, addr, &data, 1, false);
        
        if (reg % 16 == 0) printf("\n0x%02X | ", reg);
        printf("%02X ", data);
    }
    printf("\n\n");
}

typedef struct {
    float sum_x;
    float sum_y;
    float sum_z;
    float sum_temp;
    int count;
} mpu_accumulator_t;

mpu6050_reading_t average_mpu_readings(int num_readings) {
    mpu_accumulator_t acc = {0, 0, 0, 0, 0};
    
    for (int i = 0; i < num_readings; i++) {
        uint8_t buffer[8];
        uint8_t reg = MPU6050_ACCEL_XOUT_H;
        
        i2c_write_blocking(I2C_MPU, mpu6050_addr, &reg, 1, true);
        i2c_read_blocking(I2C_MPU, mpu6050_addr, buffer, 8, false);
        
        int16_t accel_x_raw = ((int16_t)buffer[0] << 8) | buffer[1];
        int16_t accel_y_raw = ((int16_t)buffer[2] << 8) | buffer[3];
        int16_t accel_z_raw = ((int16_t)buffer[4] << 8) | buffer[5];
        int16_t temp_raw = ((int16_t)buffer[6] << 8) | buffer[7];
        
        float x = accel_x_raw / 16384.0f;
        float y = accel_y_raw / 16384.0f;
        float z = accel_z_raw / 16384.0f;
        
        // Apply calibration if enabled
        apply_calibration(&x, &y, &z);
        
        acc.sum_x += x;
        acc.sum_y += y;
        acc.sum_z += z;
        acc.sum_temp += (temp_raw / 340.0f) + 36.53f;
        acc.count++;
    }
    
    mpu6050_reading_t result;
    result.accel_x = acc.sum_x / acc.count;
    result.accel_y = acc.sum_y / acc.count;
    result.accel_z = acc.sum_z / acc.count;
    result.temperature = acc.sum_temp / acc.count;
    result.valid = true;
    return result;
}

int main() {
    blink_led(3);  // initial poweron blinks happen first
    
    stdio_init_all();
    sleep_ms(4000);
    
    printf("Pico SD Card Data Logger Version %s\n", VERSION);

    // Initialize I2C for RTC (i2c1 on GP6/GP7)
    i2c_init(I2C_RTC, 400 * 1000);
    gpio_set_function(I2C_RTC_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_RTC_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_RTC_SDA);
    gpio_pull_up(I2C_RTC_SCL);
    
   
    // Initialize I2C for MPU-6050 (i2c0 on GP0/GP1)
    i2c_init(I2C_MPU, 400 * 1000);
    gpio_set_function(I2C_MPU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_MPU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_MPU_SDA);
    gpio_pull_up(I2C_MPU_SCL);
    
    // Scan both I2C buses  
    scan_i2c_bus(I2C_RTC, "I2C1 (RTC bus - GP6/GP7)");
    scan_i2c_bus(I2C_MPU, "I2C0 (MPU bus - GP0/GP1)");
    
    // Dump registers from device on MPU bus
    dump_registers(I2C_MPU, 0x68, "Device on I2C0");


    // Detect MPU-6050
    detect_mpu6050();
    if (mpu6050_addr) {
        init_mpu6050();
    }

    printf("Initializing SD card...\n");
    
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (FR_OK != fr) {
        panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    }
    
    blink_led(2);  // final blink pair to indicate SD card is ready
    
    printf("SD card mounted successfully\n");
    
    // Get free space
    DWORD free_clusters;
    FATFS *pfs;
    fr = f_getfree("", &free_clusters, &pfs);
    if (FR_OK == fr) {
        uint64_t free_bytes = (uint64_t)free_clusters * pfs->csize * 512;
        printf("Free space: %llu bytes (%.2f MB)\n", free_bytes, free_bytes / 1024.0 / 1024.0);
    }
    
    printf("Directory listing:\n");
    printf("==================\n");
    
    list_directory("", 0);
    
    printf("==================\n");
    
    // Read RTC and open/create CSV file for appending
    read_ds3231_time(&current_time);
    
    char filename[32];
    snprintf(filename, sizeof(filename), "%02d%02d%02d%02d.csv",
             current_time.month, current_time.date, current_time.hours, current_time.minutes);
    
    FIL fil;
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr) {
        printf("Error opening file: %s (%d)\n", filename, fr);
    } else {
        // Check if file is empty (new file)
        if (f_size(&fil) == 0) {
            f_printf(&fil, "epoch,accel_x,accel_y,accel_z,temp_C\n");
            f_printf(&fil, "# START: 20%02d-%02d-%02d %02d:%02d:%02d\n",
                     current_time.year, current_time.month, current_time.date,
                     current_time.hours, current_time.minutes, current_time.seconds);
        }
        printf("File: %s (appending)\n", filename);
    }
    
    printf("Reading data every 10 seconds, writing to SD every 2 minutes...\n");
    
    uint8_t last_seconds = 0xFF;
    uint32_t last_write_time = 0;
    uint32_t epoch = 0;
    
    while (1) {
        read_ds3231_time(&current_time);
        
        // Double blink and collect data at top of minute
        if ((current_time.seconds == 0) && (current_time.seconds != last_seconds)) {
            blink_double();
            // mpu6050_reading_t mpu = read_mpu6050();
            mpu6050_reading_t mpu = average_mpu_readings(50);

            
            epoch = ds3231_to_epoch(&current_time);
            if (mpu.valid) {
                printf("UTC: 20%02d-%02d-%02d %02d:%02d:%02d | Epoch: %lu | Ax=%.3f Ay=%.3f Az=%.3f T=%.2f°C\n",
                       current_time.year, current_time.month, current_time.date,
                       current_time.hours, current_time.minutes, current_time.seconds,
                       epoch, mpu.accel_x, mpu.accel_y, mpu.accel_z, mpu.temperature);
            } else {
                printf("UTC: 20%02d-%02d-%02d %02d:%02d:%02d | Epoch: %lu | MPU-6050 not available\n",
                       current_time.year, current_time.month, current_time.date,
                       current_time.hours, current_time.minutes, current_time.seconds, epoch);
            }
            
            // Add to buffer
            if (buffer_index < BUFFER_SIZE) {
                data_buffer[buffer_index].epoch = epoch;
                data_buffer[buffer_index].accel_x = mpu.accel_x;
                data_buffer[buffer_index].accel_y = mpu.accel_y;
                data_buffer[buffer_index].accel_z = mpu.accel_z;
                data_buffer[buffer_index].temperature = mpu.temperature;
                buffer_index++;
            }
        }
        // Single blink and collect data at even multiples of 10 seconds
        else if ((current_time.seconds % 10 == 0) && (current_time.seconds != last_seconds)) {
            blink_brief(5);
            // mpu6050_reading_t mpu = read_mpu6050();
            mpu6050_reading_t mpu = average_mpu_readings(50);
            
            epoch = ds3231_to_epoch(&current_time);
            if (mpu.valid) {
                printf("UTC: 20%02d-%02d-%02d %02d:%02d:%02d | Epoch: %lu | Ax=%.3f Ay=%.3f Az=%.3f T=%.2f°C\n",
                       current_time.year, current_time.month, current_time.date,
                       current_time.hours, current_time.minutes, current_time.seconds,
                       epoch, mpu.accel_x, mpu.accel_y, mpu.accel_z, mpu.temperature);
            } else {
                printf("UTC: 20%02d-%02d-%02d %02d:%02d:%02d | Epoch: %lu | MPU-6050 not available\n",
                       current_time.year, current_time.month, current_time.date,
                       current_time.hours, current_time.minutes, current_time.seconds, epoch);
            }
            
            // Add to buffer
            if (buffer_index < BUFFER_SIZE) {
                data_buffer[buffer_index].epoch = epoch;
                data_buffer[buffer_index].accel_x = mpu.accel_x;
                data_buffer[buffer_index].accel_y = mpu.accel_y;
                data_buffer[buffer_index].accel_z = mpu.accel_z;
                data_buffer[buffer_index].temperature = mpu.temperature;
                buffer_index++;
            }
        }
        
        // Write buffer to SD card every 120 seconds (2 minutes)
        if (buffer_index > 0 && (epoch - last_write_time) >= 120) {
            if (FR_OK == fr) {
                for (int i = 0; i < buffer_index; i++) {
                    f_printf(&fil, "%lu,%.3f,%.3f,%.3f,%.2f\n",
                             data_buffer[i].epoch,
                             data_buffer[i].accel_x,
                             data_buffer[i].accel_y,
                             data_buffer[i].accel_z,
                             data_buffer[i].temperature);
                }
                f_sync(&fil);
                printf("Wrote %d samples to SD card\n", buffer_index);
            }
            buffer_index = 0;
            last_write_time = epoch;
        }
        
        last_seconds = current_time.seconds;
        sleep_ms(250);
    }
    
    return 0;
}
