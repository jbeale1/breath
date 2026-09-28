#ifndef WS2812_H_
#define WS2812_H_

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Put a pixel (GRB format) into the FIFO
// This version does NOT include the reset pulse - use ws2812_put_pixel_with_reset for automatic reset
static inline void ws2812_put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

// Put a pixel with automatic WS2812 reset pulse
// The WS2812 protocol requires a reset pulse (>50us LOW) after each color transmission
static inline void ws2812_put_pixel_with_reset(PIO pio, uint sm, uint8_t ws2812_pin, uint32_t pixel_grb) {
    ws2812_put_pixel(pio, sm, pixel_grb);

    // CRITICAL: Wait for the 24 bits to finish transmitting BEFORE doing reset pulse
    // At 2.4 MHz with ~10 cycles per bit: 24 bits * 10 cycles / 2.4 MHz = ~100 microseconds
    // Use 150us to ensure transmission is completely done before reset pulse
    sleep_us(150);

    // After transmission completes, the SM is idle and pin state is stable.
    // The delay above naturally provides the reset pulse timing while allowing
    // the data to transmit uninterrupted. Do NOT disable the SM during transmission!
}

// Convert RGB to GRB format
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

#ifdef __cplusplus
}
#endif

#endif // WS2812_H_
