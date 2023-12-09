/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "gpio_JRL/gpio.h"
#include "gpio_JRL/gpio.c"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/FreeRTOSConfig.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"


#include "esp_lcd_panel_vendor.h"


static const char *TAG = "example";

#define I2C_HOST  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           21
#define EXAMPLE_PIN_NUM_SCL           22
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical

#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8
//macros propias de JRL para aplicación
#define RED PIN14
#define GREEN PIN13
#define BLUE PIN12

extern void example_lvgl_demo_ui(lv_disp_t *disp, float);

/* The LVGL port component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called
lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done
callback from esp_lcd IO config structure. In IDF 5.1 and higher, it is solved inside LVGL port component. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

void app_main(void)
{ /*configurar i2c*/
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet

    };
    /*configure pantalla*/
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));


    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = 64 * 128,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t * disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);
/*configuracion de pines con driver de JRL*/
//  Definir el número de pin GPIO que deseas configurar
    uint8_t pinout = PIN4;  // Por ejemplo, configurando el pin GPIO5
    uint8_t pinON = PIN18;   //SW1
    uint8_t pinOFF = PIN19;  //SW2
    gpio_pinMode(pinout, OUTPUT);
    gpio_pinMode(RED, OUTPUT);
    gpio_pinMode(GREEN, OUTPUT);
    gpio_pinMode(BLUE, OUTPUT);
    gpio_pinMode(pinON, INPUT_PULLUP);
    gpio_pinMode(pinOFF, INPUT_PULLUP);
    gpio_write(RED, HIGH);
    gpio_write(GREEN, HIGH);
    gpio_write(BLUE, HIGH);


    /*configuración de adc y declaración de variables para la temperatura*/
    float tv, tr, y, temp;
    int adc_read = 0;
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc_read = adc1_get_raw(ADC1_CHANNEL_0);
    ESP_LOGI(TAG, "Display LVGL Scroll Text");

    //example_lvgl_demo_ui(disp, 0);

    while(1){

         if(gpio_read(pinON) == 0X00){
         gpio_write(pinout, HIGH);
         gpio_write(RED, LOW);
         vTaskDelay(1000/ (( TickType_t ) 1000 / 100));
         gpio_write(RED, HIGH);
         gpio_write(GREEN, LOW);
         vTaskDelay(1000/ (( TickType_t ) 1000 / 100));
         gpio_write(GREEN, HIGH);
         gpio_write(BLUE, LOW);
         vTaskDelay(1000/ (( TickType_t ) 1000 / 100));
         gpio_write(BLUE, HIGH);
         printf("HIGH\n");
        
        } 
        if(gpio_read(pinOFF) == 0X00){
          gpio_write(pinout, LOW);
          gpio_write(RED, HIGH);
          gpio_write(GREEN, HIGH);
          gpio_write(BLUE, HIGH);
        //vTaskDelay(1000/ (( TickType_t ) 1000 / 100));
        }
        tv = 3.3 * adc_read / 4095.0;
        tr = tv * 10000.0 / (3.3 - tv);
        y = log(tr/10000.0);
        y = (1.0/298.15) + (y *(1.0/4050.0));
        temp = 1.0/y;
        temp = temp -273.15;
        esp_lcd_panel_reset(panel_handle);
        example_lvgl_demo_ui(disp, temp);
        vTaskDelay(100/ (( TickType_t ) 1000 / 100));
    }

    
}
