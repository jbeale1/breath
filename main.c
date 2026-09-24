#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "freq_counter.pio.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

// ============ PIN DEFINITIONS ============
#define LED_PIN 25
#define FREQ_PIN 3      // External ~1 MHz signal to measure

// I2C for RTC (DS3231)
#define I2C_RTC i2c1
#define I2C_RTC_SDA 6
#define I2C_RTC_SCL 7
#define DS3231_ADDR 0x68

// I2C for MPU-6050
#define I2C_MPU i2c0
#define I2C_MPU_SDA 0
#define I2C_MPU_SCL 1
#define MPU6050_ADDR 0x68

// Frequency counter PIO pins
#define GATE_PIN      2
#define PULSE_FIN_PIN 4

// ============ MPU-6050 CONFIGURATION ============
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H 0x41

// MPU-6050 Calibration Enable/Disable
#define ENABLE_CALIBRATION 1

// Calibration Constants (measured 2026-09-24)
#define CALIB_OFFSET_X 0.03092250f
#define CALIB_SCALE_X  1.00047773f
#define CALIB_OFFSET_Y -0.01023000f
#define CALIB_SCALE_Y  0.99378882f
#define CALIB_OFFSET_Z -0.07085750f
#define CALIB_SCALE_Z  0.97970780f

// ============ FREQUENCY COUNTER CONFIGURATION ============
#define SYS_CLOCK_HZ         125000000.0
#define GATE_NOMINAL_CYCLES  ((uint32_t)(SYS_CLOCK_HZ * 0.250))  // 250 ms

// ============ SD CARD CONFIGURATION ============
#define BUFFER_SIZE 40  // Write every 10 seconds (40 samples × 250 ms)

// ============ TYPE DEFINITIONS ============
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
    float sum_x;
    float sum_y;
    float sum_z;
    float sum_temp;
    int count;
} mpu_accumulator_t;

typedef struct {
    float elapsed_sec;
    float freq_hz;
    float accel_x;
    float accel_y;
    float accel_z;
    float temp_c;
} data_point_t;

// ============ GLOBAL STATE ============
static PIO pio = pio0;
static uint sm_gate, sm_clock, sm_pulse;

static volatile uint32_t g_clock_cycles;
static volatile uint32_t g_pulse_count;
static volatile bool g_result_ready;

static data_point_t data_buffer[BUFFER_SIZE];
static int buffer_index = 0;

static mpu_accumulator_t mpu_acc = {0, 0, 0, 0, 0};
static float elapsed_sec = 0.0f;  // Elapsed time in seconds (float)

static FIL fil;
static uint8_t mpu6050_addr = 0x68;

// ============ LED FUNCTIONS ============
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

// ============ RTC FUNCTIONS ============
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

// ============ MPU-6050 FUNCTIONS ============
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
    
    // Configure sample rate divider (1 kHz sampling)
    cmd[0] = 0x19;  // SMPRT_DIV register
    cmd[1] = 0x00;  // Sample rate = 1000 / (1 + 0) = 1kHz
    i2c_write_blocking(I2C_MPU, mpu6050_addr, cmd, 2, false);
}

void apply_calibration(float *x, float *y, float *z) {
#if ENABLE_CALIBRATION
    *x = (*x - CALIB_OFFSET_X) * CALIB_SCALE_X;
    *y = (*y - CALIB_OFFSET_Y) * CALIB_SCALE_Y;
    *z = (*z - CALIB_OFFSET_Z) * CALIB_SCALE_Z;
#endif
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
    
    // Apply calibration
    apply_calibration(&result.accel_x, &result.accel_y, &result.accel_z);
    
    // Convert temperature (raw / 340 + 36.53)
    int16_t temp_raw = ((int16_t)buffer[6] << 8) | buffer[7];
    result.temperature = (temp_raw / 340.0f) + 36.53f;
    
    result.valid = true;
    return result;
}

// ============ FREQUENCY COUNTER ISR ============
static void pio0_isr(void) {
    uint32_t raw_clock = pio_sm_get_blocking(pio, sm_clock);
    uint32_t raw_pulse = pio_sm_get_blocking(pio, sm_pulse);

    g_clock_cycles = 2u * (0xFFFFFFFFu - raw_clock);  // 2 cycles/iteration in clock_count
    g_pulse_count  = (0xFFFFFFFEu) - raw_pulse;        // pulse_count's X started at max-1
    g_result_ready = true;

    pio_interrupt_clear(pio, 0);  // lets gate SM proceed to the next window
}

// ============ SD CARD FUNCTIONS ============
void write_buffer_to_sd(void) {
    // Write buffered data to SD card
    for (int i = 0; i < buffer_index; i++) {
        f_printf(&fil, "%.2f,%.2f,%.4f,%.4f,%.4f,%.2f\n",
                 data_buffer[i].elapsed_sec,
                 data_buffer[i].freq_hz,
                 data_buffer[i].accel_x,
                 data_buffer[i].accel_y,
                 data_buffer[i].accel_z,
                 data_buffer[i].temp_c);
    }
    f_sync(&fil);
    printf("Wrote %d samples to SD card\n", buffer_index);
    buffer_index = 0;
    blink_brief(2);  // 2 ms LED blink after write
}

// ============ MAIN ============
int main() {
    stdio_init_all();
    
    // Initial boot pattern: 3 blinks
    blink_led(3);
    printf("Terminal ready\n");
    
    // Wait 4 seconds
    sleep_ms(4000);

    printf("Pico Frequency Counter + MPU-6050 Logger\n");

    // Initialize I2C for RTC
    i2c_init(I2C_RTC, 400 * 1000);
    gpio_set_function(I2C_RTC_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_RTC_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_RTC_SDA);
    gpio_pull_up(I2C_RTC_SCL);

    // Initialize I2C for MPU
    i2c_init(I2C_MPU, 400 * 1000);
    gpio_set_function(I2C_MPU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_MPU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_MPU_SDA);
    gpio_pull_up(I2C_MPU_SCL);

    // Read RTC for filename and timestamp
    ds3231_time_t start_time;
    read_ds3231_time(&start_time);
    printf("RTC time: 20%02d-%02d-%02d %02d:%02d:%02d UTC\n",
           start_time.year, start_time.month, start_time.date,
           start_time.hours, start_time.minutes, start_time.seconds);

    // Create filename from date/time
    char filename[32];
    snprintf(filename, sizeof(filename), "%02d%02d%02d%02d.csv",
             start_time.month, start_time.date, start_time.hours, start_time.minutes);

    // Initialize SD card
    printf("Initializing SD card...\n");
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (FR_OK != fr) {
        panic("f_mount error: %d\n", fr);
    }

    // Open/create CSV file
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr) {
        panic("Error opening file: %d\n", fr);
    }

    // Check if this is a new file (empty)
    bool is_new_file = (f_size(&fil) == 0);

    printf("File: %s (%s)\n", filename, is_new_file ? "new" : "appending");

    // Initialize MPU-6050
    init_mpu6050();
    printf("MPU-6050 initialized\n");
    
    // Boot complete pattern: 2 blinks
    blink_led(2);

    // Initialize frequency counter PIO
    uint offset_gate  = pio_add_program(pio, &gate_program);
    uint offset_clock = pio_add_program(pio, &clock_count_program);
    uint offset_pulse = pio_add_program(pio, &pulse_count_program);

    sm_gate  = pio_claim_unused_sm(pio, true);
    sm_clock = pio_claim_unused_sm(pio, true);
    sm_pulse = pio_claim_unused_sm(pio, true);

    gpio_init(FREQ_PIN);
    gpio_set_dir(FREQ_PIN, GPIO_IN);

    pio_gpio_init(pio, GATE_PIN);
    pio_gpio_init(pio, PULSE_FIN_PIN);

    // Configure gate state machine
    pio_sm_config c_gate = gate_program_get_default_config(offset_gate);
    sm_config_set_in_pins(&c_gate, FREQ_PIN);
    sm_config_set_sideset_pins(&c_gate, GATE_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm_gate, GATE_PIN, 1, true);
    pio_sm_init(pio, sm_gate, offset_gate, &c_gate);

    // Configure clock_count state machine
    pio_sm_config c_clock = clock_count_program_get_default_config(offset_clock);
    sm_config_set_in_pins(&c_clock, GATE_PIN);
    sm_config_set_jmp_pin(&c_clock, PULSE_FIN_PIN);
    pio_sm_init(pio, sm_clock, offset_clock, &c_clock);

    // Configure pulse_count state machine
    pio_sm_config c_pulse = pulse_count_program_get_default_config(offset_pulse);
    sm_config_set_in_pins(&c_pulse, GATE_PIN);  // offset 0 = GATE_PIN, offset 1 = FREQ_PIN
    sm_config_set_sideset_pins(&c_pulse, PULSE_FIN_PIN);
    sm_config_set_jmp_pin(&c_pulse, GATE_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm_pulse, PULSE_FIN_PIN, 1, true);
    pio_sm_init(pio, sm_pulse, offset_pulse, &c_pulse);

    // Preload each SM's one-time constant
    pio_sm_put_blocking(pio, sm_gate,  GATE_NOMINAL_CYCLES);
    pio_sm_put_blocking(pio, sm_clock, 0xFFFFFFFFu);
    pio_sm_put_blocking(pio, sm_pulse, 0xFFFFFFFEu);

    // Set up interrupt
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio0_isr);
    irq_set_enabled(PIO0_IRQ_0, true);

    uint32_t mask = (1u << sm_gate) | (1u << sm_clock) | (1u << sm_pulse);
    pio_enable_sm_mask_in_sync(pio, mask);

    printf("Ready to measure (250 ms gate windows)\n");

    // Read RTC again just before starting data collection for accurate start time
    ds3231_time_t recording_start_time;
    read_ds3231_time(&recording_start_time);
    
    // Write actual recording start time to SD card (only if new file)
    if (is_new_file) {
        f_printf(&fil, "# Start: 20%02d-%02d-%02d %02d:%02d:%02d UTC\n",
                 recording_start_time.year, recording_start_time.month, recording_start_time.date,
                 recording_start_time.hours, recording_start_time.minutes, recording_start_time.seconds);
        f_printf(&fil, "elapsed_sec,freq_hz,accel_x,accel_y,accel_z,temp_c\n");
        f_sync(&fil);
    }

    // ============ MAIN LOOP ============
    while (true) {
        // Continuously read MPU-6050 and accumulate samples
        mpu6050_reading_t mpu = read_mpu6050();
        if (mpu.valid) {
            mpu_acc.sum_x += mpu.accel_x;
            mpu_acc.sum_y += mpu.accel_y;
            mpu_acc.sum_z += mpu.accel_z;
            mpu_acc.sum_temp += mpu.temperature;
            mpu_acc.count++;
        }

        // When frequency measurement window completes
        if (g_result_ready) {
            g_result_ready = false;

            // Calculate frequency
            double freq_hz = (double)g_pulse_count * SYS_CLOCK_HZ / (double)g_clock_cycles;

            // Save sample count before resetting accumulator
            int samples_collected = mpu_acc.count;

            // Average MPU readings collected during this 250 ms window
            float avg_x = (mpu_acc.count > 0) ? mpu_acc.sum_x / mpu_acc.count : 0;
            float avg_y = (mpu_acc.count > 0) ? mpu_acc.sum_y / mpu_acc.count : 0;
            float avg_z = (mpu_acc.count > 0) ? mpu_acc.sum_z / mpu_acc.count : 0;
            float avg_temp = (mpu_acc.count > 0) ? mpu_acc.sum_temp / mpu_acc.count : 0;

            // Reset accumulator for next window
            mpu_acc.sum_x = mpu_acc.sum_y = mpu_acc.sum_z = mpu_acc.sum_temp = 0;
            mpu_acc.count = 0;

            // Store data point in buffer
            if (buffer_index < BUFFER_SIZE) {
                data_buffer[buffer_index].elapsed_sec = elapsed_sec;
                data_buffer[buffer_index].freq_hz = (float)freq_hz;
                data_buffer[buffer_index].accel_x = avg_x;
                data_buffer[buffer_index].accel_y = avg_y;
                data_buffer[buffer_index].accel_z = avg_z;
                data_buffer[buffer_index].temp_c = avg_temp;
                buffer_index++;

                printf("%.2f s: freq=%.2f Hz, ax=%.4f, ay=%.4f, az=%.4f, T=%.2f C (samples=%d)\n",
                       elapsed_sec, freq_hz, avg_x, avg_y, avg_z, avg_temp, samples_collected);
            }

            // Increment elapsed time (each window is 250 ms)
            elapsed_sec += 0.25f;

            // Write to SD card when buffer reaches 40 samples (10 seconds)
            if (buffer_index >= BUFFER_SIZE) {
                write_buffer_to_sd();
            }
        }

        tight_loop_contents();
    }

    return 0;
}
