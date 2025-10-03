

/*
 * SSD1306 SPI Driver for ESP32 (ESP-IDF)
 * --------------------------------------
 * Philip Glazzard style driver skeleton with comments.
 * 
 * Wiring (example):
 *   ESP32 MOSI -> SSD1306 SDA
 *   ESP32 SCLK -> SSD1306 SCL
 *   ESP32 DC   -> SSD1306 DC
 *   ESP32 RST  -> SSD1306 RES
 *   ESP32 CS   -> SSD1306 CS
 */

#include <stdio.h>
#include "driver/spi_master.h"// SPI master driver for ESP32
#include "driver/gpio.h"// GPIO control
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"// Logging macros
#include <string.h>// For memset, etc.

#define TAG "SSD1306"// Logging tag for this module


// ---- Display config ----
#define SSD1306_WIDTH   128      // Display width in pixels
#define SSD1306_HEIGHT   64      // Display height in pixels

// GPIO pin assignments 
#define PIN_NUM_MOSI    23       // SPI MOSI (data) white
#define PIN_NUM_CLK     18       // SPI clock blue
#define PIN_NUM_CS      5        // Chip select yelloe
#define PIN_NUM_DC      21       // Data/Command control green
#define PIN_NUM_RST     22       // Reset pin orange

// ---- SPI device handle ----
static spi_device_handle_t ssd1306_spi;// Handle for SPI device

// ---- Framebuffer ----
static uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];// Framebuffer: 1 bit per pixel, 8 pixels per byte


// ---- Low-level helpers ----

// Sends a single command byte to the display
static void ssd1306_send_command(uint8_t cmd) {
    gpio_set_level(PIN_NUM_DC, 0); // DC low = command mode
    spi_transaction_t t = {
        .length = 8,               // 8 bits
        .tx_buffer = &cmd,        // Pointer to command byte
    };
    spi_device_transmit(ssd1306_spi, &t); // Blocking transmit
}

// Sends a block of data bytes to the display
static void ssd1306_send_data(const uint8_t *data, size_t len) {
    gpio_set_level(PIN_NUM_DC, 1); // DC high = data mode
    spi_transaction_t t = {
        .length = len * 8,         // Total bits
        .tx_buffer = data,         // Pointer to data buffer
    };
    spi_device_transmit(ssd1306_spi, &t); // Blocking transmit
}

// ---- Initialization sequence ----
static void ssd1306_reset(void) {// Hardware reset using RST pin

    gpio_set_level(PIN_NUM_RST, 0);// Pull reset low
    vTaskDelay(pdMS_TO_TICKS(10));// Wait 10ms
    gpio_set_level(PIN_NUM_RST, 1);// Release reset
    vTaskDelay(pdMS_TO_TICKS(10));// Wait 10ms
}

static void ssd1306_init_cmds(void) {
    ssd1306_send_command(0xAE); // Display off
    ssd1306_send_command(0xD5); // Set display clock div
    ssd1306_send_command(0x80);
    ssd1306_send_command(0xA8); // Multiplex
    ssd1306_send_command(SSD1306_HEIGHT - 1);
    ssd1306_send_command(0xD3); // Display offset
    ssd1306_send_command(0x00);
    ssd1306_send_command(0x40 | 0x00); // Start line = 0
    ssd1306_send_command(0x8D); // Charge pump
    ssd1306_send_command(0x14);
    ssd1306_send_command(0x20); // Memory mode
    ssd1306_send_command(0x00); // Horizontal
    ssd1306_send_command(0xA1); // Segment remap
    ssd1306_send_command(0xC8); // COM scan dec
    ssd1306_send_command(0xDA); // COM pins
    ssd1306_send_command(0x12);
    ssd1306_send_command(0x81); // Contrast
    ssd1306_send_command(0x7F);
    ssd1306_send_command(0xD9); // Precharge
    ssd1306_send_command(0xF1);
    ssd1306_send_command(0xDB); // VCOM detect
    ssd1306_send_command(0x40);
    ssd1306_send_command(0xA4); // Resume RAM
    ssd1306_send_command(0xA6); // Normal display
    ssd1306_send_command(0xAF); // Display on
}

// ---- Public API ----

// Initialize SPI bus and SSD1306
void ssd1306_init(void) {
    esp_err_t ret;

    // Configure DC and RST GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<PIN_NUM_DC) | (1ULL<<PIN_NUM_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Configure SPI bus 
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,// Not used
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,// Not used
        .quadhd_io_num = -1,// Not used
        .max_transfer_sz = SSD1306_WIDTH * SSD1306_HEIGHT / 8,
    };

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Configure SPI device 
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000, // 8 MHz SPI clock
        .mode = 0,// SPI mode 0
        .spics_io_num = PIN_NUM_CS,// CS pin
        .queue_size = 1,// Only one transaction at a time
    };

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &ssd1306_spi);
    ESP_ERROR_CHECK(ret);

    // Reset and init display
    ssd1306_reset();
    ssd1306_init_cmds();

    // Clear buffer
    memset(buffer, 0, sizeof(buffer));
}

// Draw pixel into framebuffer
void ssd1306_draw_pixel(int x, int y, bool color) {
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;

    int byteIndex = x + (y / 8) * SSD1306_WIDTH;// Calculate byte index
    if (color)
    {
        buffer[byteIndex] |= (1 << (y & 7));// Set bit  
    }
    else
    {
        buffer[byteIndex] &= ~(1 << (y & 7));// Clear bit  
    }
}

// Update entire display from framebuffer
void ssd1306_update_display(void) {
    for (uint8_t page = 0; page < (SSD1306_HEIGHT / 8); page++) {
        ssd1306_send_command(0xB0 + page); // Set Page address
        ssd1306_send_command(0x00);        // Set lower column address
        ssd1306_send_command(0x10);        // Set higher column address
        ssd1306_send_data(&buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);// Send page data
    }
}

// Clear framebuffer (all pixels off)
void ssd1306_clear(void) {
    memset(buffer, 0, sizeof(buffer));
}



void app_main(void)
{
    ssd1306_init(); // Initialize display

    ssd1306_clear();// Clear framebuffer


    // Draw a diagonal line from top-left to bottom-right

    for (int i = 0; i < SSD1306_HEIGHT; i++) {
    
        ssd1306_draw_pixel(i, i, true);
         
    }
    ssd1306_update_display();// Push framebuffer to display



    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));// Idle loop

    }
}