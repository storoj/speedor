#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"

#include "lvgl.h"
#include "lv_demos.h"

#include "esp_lcd_nv3041a.h"
#include "esp_lcd_touch_gt911.h"

#include "ubxlib.h"

#include "sd_card.h"

void lv_app();
void init_gps(char *status_msg);
void init_gps_task(void*);

static lv_obj_t *speed_label;
static lv_obj_t *speed_meter;
static lv_meter_indicator_t *speed_needle;
static lv_obj_t *status_label;
static bool sd_card_ok = false;
static FILE *sd_card_file = NULL;

#define U_CFG_APP_GNSS_UART 1
#define U_CFG_APP_PIN_GNSS_TXD 17
#define U_CFG_APP_PIN_GNSS_RXD 18
// #define U_GNSS_UART_BAUD_RATE 115200

#include "u_cfg_app_platform_specific.h"

static const char *TAG = "example";
static SemaphoreHandle_t lvgl_mux = NULL;

#define LCD_HOST    SPI2_HOST
#define TOUCH_HOST  I2C_NUM_0

#define LCD_BIT_PER_PIXEL       (16)

#define EXAMPLE_LCD_H_RES              480
#define EXAMPLE_LCD_V_RES              272

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_LCD_CS            (GPIO_NUM_45)
#define EXAMPLE_PIN_NUM_LCD_PCLK          (GPIO_NUM_47)
#define EXAMPLE_PIN_NUM_LCD_DATA0         (GPIO_NUM_21)
#define EXAMPLE_PIN_NUM_LCD_DATA1         (GPIO_NUM_48)
#define EXAMPLE_PIN_NUM_LCD_DATA2         (GPIO_NUM_40)
#define EXAMPLE_PIN_NUM_LCD_DATA3         (GPIO_NUM_39)
#define EXAMPLE_PIN_NUM_LCD_RST           (-1)
#define EXAMPLE_PIN_NUM_BK_LIGHT          (GPIO_NUM_1)

#define EXAMPLE_PIN_NUM_TOUCH_SCL         (GPIO_NUM_4)
#define EXAMPLE_PIN_NUM_TOUCH_SDA         (GPIO_NUM_8)
#define EXAMPLE_PIN_NUM_TOUCH_RST         (GPIO_NUM_38)
#define EXAMPLE_PIN_NUM_TOUCH_INT         (GPIO_NUM_3)

esp_lcd_touch_handle_t tp = NULL;

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_lvgl_update_cb(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    }
}

static SemaphoreHandle_t touch_mux = NULL;

static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;
    assert(tp);

    uint16_t tp_x;
    uint16_t tp_y;
    uint8_t tp_cnt = 0;
    if (xSemaphoreTake(touch_mux, 0) == pdTRUE) {
        esp_lcd_touch_read_data(tp);
    }

    bool tp_pressed = esp_lcd_touch_get_coordinates(tp, &tp_x, &tp_y, NULL, &tp_cnt, 1);
    if (tp_pressed && tp_cnt > 0) {
        data->point.x = tp_x;
        data->point.y = tp_y;
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGD(TAG, "Touch position: %d,%d", tp_x, tp_y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void example_touch_isr_cb(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


static void example_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static char latLongToBits(int32_t thingX1e7,
                          int32_t *pWhole,
                          int32_t *pFraction)
{
    char prefix = '+';

    // Deal with the sign
    if (thingX1e7 < 0) {
        thingX1e7 = -thingX1e7;
        prefix = '-';
    }
    *pWhole = thingX1e7 / 10000000;
    *pFraction = thingX1e7 % 10000000;

    return prefix;
}

// Callback function to receive location.
static void callback(uDeviceHandle_t devHandle,
                     int32_t errorCode,
                     const uLocation_t *pLocation)
{
    char prefix[2] = {0};
    int32_t whole[2] = {0};
    int32_t fraction[2] = {0};

    // Not used
    (void) devHandle;

    if (errorCode == 0) {
        prefix[0] = latLongToBits(pLocation->longitudeX1e7, &(whole[0]), &(fraction[0]));
        prefix[1] = latLongToBits(pLocation->latitudeX1e7, &(whole[1]), &(fraction[1]));
        ESP_LOGI("Location", "I am here: https://maps.google.com/?q=%c%d.%07d,%c%d.%07d\n",
                 prefix[1], whole[1], fraction[1], prefix[0], whole[0], fraction[0]);
        ESP_LOGI("Location", "Speed: %.3f m/s", ((double)pLocation->speedMillimetresPerSecond) / 1000);
        ESP_LOGI("Location", "svs: %d", pLocation->svs);
        ESP_LOGI("Location", "time: %"PRId64"", pLocation->timeUtc);
    } else if (errorCode == U_ERROR_COMMON_TIMEOUT) {
        ESP_LOGI("Location", "* Timeout");
    } else {
        ESP_LOGI("Location", "Error %d", errorCode);
    }
}

static uint64_t counter = 0;
static void gps_ubx_callback(uDeviceHandle_t devHandle, const uGnssMessageId_t* pMessageId,
    int32_t errorCodeOrLength, void* pCallbackParam) {
    char* pBuffer = (char*)pCallbackParam;
    int32_t length;
    uGnssDec_t* pDec;
    uGnssDecUbxNavPvt_t* pUbxNavPvt;
    int64_t utcTimeNanoseconds;

    (void)pMessageId;

    if (errorCodeOrLength >= 0) {
        // Read the message into our buffer
        length = uGnssMsgReceiveCallbackRead(devHandle, pBuffer, errorCodeOrLength);
        if (length >= 0) {
            // Call the uGnssDec() API to decode the message
            pDec = pUGnssDecAlloc(pBuffer, length);
            if ((pDec != NULL) && (pDec->errorCode == 0)) {
                // No need to check pDec->id (or pMessageId) here since we have
                // only asked for UBX-NAV-PVT messages.
                pUbxNavPvt = &(pDec->pBody->ubxNavPvt);

                utcTimeNanoseconds = uGnssDecUbxNavPvtGetTimeUtc(pUbxNavPvt);

                int32_t lat = pUbxNavPvt->lat;
                int32_t lon = pUbxNavPvt->lon;
                int32_t alt = pUbxNavPvt->hMSL;
                int32_t hAcc = pUbxNavPvt->hAcc;

                uGnssDecUbxNavPvtFixType_t fix_status = pUbxNavPvt->fixType;

                uint8_t sats_used = pUbxNavPvt->numSV;
                int32_t speed_mm_s = pUbxNavPvt->gSpeed;
                int32_t speed_km_h = 9 * speed_mm_s / 2500;

                example_lvgl_lock(-1);
                lv_label_set_text_fmt(speed_label, "%d", (int)speed_km_h);
                lv_meter_set_indicator_value(speed_meter, speed_needle, speed_km_h);
                lv_label_set_text_fmt(status_label,
                        "lat: %"PRIi32 "\n"
                        "lon: %"PRIi32 "\n"
                        "alt: %"PRIi32 "\n"
                        "hacc: %"PRIi32"\n"
                        "fix: %d\n"
                        "sats: %d\n"
                        "spd_mm_s: %"PRIi32"\n"
                        "spd_km_h: %"PRIi32"\n",
                        lat, lon, alt, hAcc, fix_status, sats_used, speed_mm_s, speed_km_h);
                example_lvgl_unlock();


                if (sd_card_ok) {
                    if (!sd_card_file && utcTimeNanoseconds > 0) {
                        char filename[256];
                        sprintf(filename, "/sdcard/%"PRId64".dat", utcTimeNanoseconds);
                        ESP_LOGI("SD", "Filename: %s", filename);
                        sd_card_file = fopen(filename, "wb");
                        if (!sd_card_file) {
                            perror("fopen");
                        }
                        ESP_LOGI("SD", "File: %p", sd_card_file);
                    }

                    if (sd_card_file) {
                        fwrite(pUbxNavPvt, sizeof(uGnssDecUbxNavPvt_t), 1, sd_card_file);
                        if (counter % 100 == 0) {
                            ESP_LOGI("SD", "Flush");
                            fflush(sd_card_file);
                        }
                    }
                }

                counter++;

                ESP_LOGI("Location",
                        "t: %"PRId64" "
                        "lat: %"PRIi32 " "
                        "lon: %"PRIi32 " "
                        "alt: %"PRIi32 " "
                        "hacc: %"PRIi32" "
                        "fix: %d "
                        "spd_mm_s: %"PRIi32" "
                        "spd_km_h: %"PRIi32" ",
                        utcTimeNanoseconds, lat, lon, alt, hAcc, fix_status, speed_mm_s, speed_km_h);

                // xTaskNotify(telemetry_task, 0, eNoAction);
            }
            uGnssDecFree(pDec);
        }
    } else {
        ESP_LOGI("GPS", "Empty or bad message received.");
    }
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = NV3041A_PANEL_BUS_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA1,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA2,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA3,
                                                                 EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = NV3041A_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, example_notify_lvgl_flush_ready, &disp_drv);

    nv3041a_vendor_config_t vendor_config = {
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };

    ESP_LOGI(TAG, "Install NV3041A panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_nv3041a(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));

    ESP_LOGI(TAG, "Initialize I2C bus");
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = EXAMPLE_PIN_NUM_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400 * 1000,
    };
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_HOST, i2c_conf.mode, 0, 0, 0));

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle));

    touch_mux = xSemaphoreCreateBinary();
    assert(touch_mux);

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST,
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = example_touch_isr_cb,
    };

    ESP_LOGI(TAG, "Initialize touch controller GT911");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 30 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 30 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 30);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_update_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    
    lv_indev_drv_register(&indev_drv);

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    if (ESP_OK == sd_card_init("/sdcard")) {
        sd_card_ok = true;
    }

    ESP_LOGI(TAG, "Display LVGL demos");
    if (example_lvgl_lock(-1)) {
        // lv_demo_widgets();
        // lv_demo_music();
        // lv_demo_stress();
        // lv_demo_benchmark();
        lv_app();
        example_lvgl_unlock();
    }
    xTaskCreate(init_gps_task, "GPS_INIT", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, 3, NULL);
}

static void status_set_text_fmt(const char * fmt, ...) {
    example_lvgl_lock(-1);

    va_list args;
    va_start(args, fmt);
    lv_label_set_text_fmt(status_label, fmt, args);
    va_end(args);

    example_lvgl_unlock();
}

static char * status_append_fmt(char *status, char *tail, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    tail += vsprintf(tail, fmt, args);
    va_end(args);

    example_lvgl_lock(-1);
    lv_label_set_text(status_label, status);
    example_lvgl_unlock();

    return tail;
}

void init_gps_task(void *) {
    char buf[1024];
    init_gps(buf);
    vTaskDelete(NULL);
}

void init_gps(char *status_msg) {
    const uGnssModuleType_t module_type = U_GNSS_MODULE_TYPE_M10;

    *status_msg = 0;
    char *tail = status_msg;

    tail = status_append_fmt(status_msg, tail, "uPortInit...");
    int32_t ubx_err = uPortInit();
    if (ubx_err) {
        ESP_LOGE("GPS", "UBXLIB Port init error: %" PRIi32, ubx_err);
        tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
        return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    tail = status_append_fmt(status_msg, tail, "uGnssInit...");
    ubx_err = uGnssInit();
    if (ubx_err) {
        ESP_LOGE("GPS", "UBXLIB GNSS init error: %" PRIi32, ubx_err);
        tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
        return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    tail = status_append_fmt(status_msg, tail, "UART 9600...");
    uGnssTransportHandle_t transportHandle;
    transportHandle.uart = uPortUartOpen(
        U_CFG_APP_GNSS_UART, 9600, NULL, U_GNSS_UART_BUFFER_LENGTH_BYTES,
        U_CFG_APP_PIN_GNSS_TXD, U_CFG_APP_PIN_GNSS_RXD, U_CFG_APP_PIN_GNSS_CTS,
        U_CFG_APP_PIN_GNSS_RTS);

    uDeviceHandle_t devHandle = NULL;
    ubx_err = uGnssAdd(module_type, U_GNSS_TRANSPORT_UART, transportHandle, -1, true, &devHandle);
    if (ubx_err) {
      ESP_LOGE("GPS", "UBXLIB 9600 GNSS add error: %" PRIi32, ubx_err);
      tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
      return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    U_GNSS_CFG_SET_VAL_RAM(devHandle, UART1_BAUDRATE_U4, 921600);
    vTaskDelay(pdMS_TO_TICKS(500));

    uGnssRemove(devHandle);
    uPortUartClose(transportHandle.uart);

    tail = status_append_fmt(status_msg, tail, "UART 921600...");
    transportHandle.uart = uPortUartOpen(
        U_CFG_APP_GNSS_UART, 921600, NULL, U_GNSS_UART_BUFFER_LENGTH_BYTES,
        U_CFG_APP_PIN_GNSS_TXD, U_CFG_APP_PIN_GNSS_RXD, U_CFG_APP_PIN_GNSS_CTS,
        U_CFG_APP_PIN_GNSS_RTS);

    ubx_err = uGnssAdd(module_type, U_GNSS_TRANSPORT_UART, transportHandle, -1, true, &devHandle);
    if (ubx_err) {
      ESP_LOGE("GPS", "UBXLIB 921600 GNSS add error: %" PRIi32, ubx_err);
      tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
      return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    uGnssSetUbxMessagePrint(devHandle, true);
    tail = status_append_fmt(status_msg, tail, "Disable NMEA...");
    ubx_err = uGnssCfgSetProtocolOut(devHandle, U_GNSS_PROTOCOL_NMEA, false);
    if (ubx_err) {
      ESP_LOGE("GPS", "NMEA disable error %" PRIi32, ubx_err);
      tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
      return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    tail = status_append_fmt(status_msg, tail, "Enable UBX...");
    ubx_err = uGnssCfgSetProtocolOut(devHandle, U_GNSS_PROTOCOL_UBX, true);
    if (ubx_err) {
      ESP_LOGE("GPS", "UBX enable error %" PRIi32, ubx_err);
      tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
      return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    // Power up the GNSS module
    uGnssPwrOn(devHandle);

    tail = status_append_fmt(status_msg, tail, "Set UBX Rate...");
    ubx_err = uGnssCfgValSet(devHandle, U_GNSS_CFG_VAL_KEY_ID_MSGOUT_UBX_NAV_PVT_UART1_U1,
            1, U_GNSS_CFG_VAL_TRANSACTION_NONE, U_GNSS_CFG_VAL_LAYER_RAM);
    if (ubx_err) {
      ESP_LOGE("GPS", "UBX Set UBX rate error: %" PRIi32, ubx_err);
      tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
      return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    // ubx_err = uGnssCfgValSet(devHandle, U_GNSS_CFG_VAL_KEY_ID_NAVSPG_USE_PPP_L,
    //         1, U_GNSS_CFG_VAL_TRANSACTION_NONE, U_GNSS_CFG_VAL_LAYER_RAM);
    // if (ubx_err != 0) {
    //   ESP_LOGE("GPS", "UBX Set PPP error: %" PRIi32, ubx_err);
    // }

    int32_t dynamic_model = uGnssCfgGetDynamic(devHandle);
    ESP_LOGI("CFG", "Dynamic model before: %"PRIi32, dynamic_model);
    tail = status_append_fmt(status_msg, tail, "Dynamic model before: %"PRIi32"\n", dynamic_model);

    tail = status_append_fmt(status_msg, tail, "Set dynamic model...");
    ubx_err = uGnssCfgSetDynamic(devHandle, U_GNSS_DYNAMIC_AUTOMOTIVE);
    if (ubx_err) {
      ESP_LOGE("GPS", "UBX Set dynmodel error: %" PRIi32, ubx_err);
      tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
      return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    dynamic_model = uGnssCfgGetDynamic(devHandle);
    ESP_LOGI("CFG", "Dynamic model after: %"PRIi32, dynamic_model);
    tail = status_append_fmt(status_msg, tail, "Dynamic model after: %"PRIi32"\n", dynamic_model);

    int32_t measprdms = 100;
    int32_t measpernav = 1;

    tail = status_append_fmt(status_msg, tail, "Set Rate 10hz...");
    ubx_err = uGnssCfgSetRate(devHandle, measprdms, measpernav, -1);
    if (ubx_err) {
        ESP_LOGE("GPS", "UBXLIB GNSS rate error: %" PRIi32, ubx_err);
        tail = status_append_fmt(status_msg, tail, " Fail: %"PRIi32, ubx_err);
        return;
    }
    tail = status_append_fmt(status_msg, tail, " OK\n");

    char *gps_ubx_buf = (char*)pUPortMalloc(92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES);

    uGnssMessageId_t messageId;
    messageId.type = U_GNSS_PROTOCOL_UBX;
    messageId.id.ubx = U_GNSS_UBX_MESSAGE(
            U_GNSS_DEC_UBX_NAV_PVT_MESSAGE_CLASS,
            U_GNSS_DEC_UBX_NAV_PVT_MESSAGE_ID
            );

    uGnssMsgReceiveStart(devHandle, &messageId, gps_ubx_callback, gps_ubx_buf);

    ESP_LOGI("GPS", "Successfully init GPS!");
}

void lv_app() {
    lv_obj_t *screen = lv_scr_act();

    lv_style_t style_title;
    lv_style_init(&style_title);
    const lv_font_t * font_large = LV_FONT_DEFAULT;
#if LV_FONT_MONTSERRAT_48
    font_large     = &lv_font_montserrat_48;
#endif
    lv_style_set_text_font(&style_title, &lv_font_montserrat_24);

    lv_obj_t *cont = lv_obj_create(screen);

    lv_obj_set_height(cont, lv_pct(100));
    lv_obj_set_width(cont, lv_pct(100));
    lv_obj_set_style_bg_color(cont, lv_color_black(), LV_STATE_DEFAULT);

    lv_obj_t * meter = lv_meter_create(screen);
    // lv_obj_remove_style(meter, NULL, LV_PART_MAIN);
    // lv_obj_remove_style(meter, NULL, LV_PART_INDICATOR);
    lv_obj_set_size(meter, 260, 260);
    lv_obj_align(meter, LV_ALIGN_CENTER, 0, 0);

    // lv_obj_set_style_pad_hor(meter, 10, 0);
    lv_obj_set_style_size(meter, 10, LV_PART_INDICATOR);
    lv_obj_set_style_radius(meter, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(meter, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(meter, lv_palette_darken(LV_PALETTE_GREY, 4), LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(meter, lv_color_white(), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(meter, 3, LV_PART_INDICATOR);
    lv_obj_set_style_text_color(meter, lv_color_white(), LV_PART_TICKS);

    lv_meter_scale_t * scale;
    lv_meter_indicator_t * indic;
    scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale, 10, 90, 220, 360 - 220);
    lv_meter_set_scale_ticks(meter, scale, 21, 3, 17, lv_color_white());
    lv_meter_set_scale_major_ticks(meter, scale, 4, 4, 22, lv_color_white(), 15);

    indic = lv_meter_add_arc(meter, scale, 10, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 60);

    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_darken(LV_PALETTE_GREEN, 3),
                                     lv_palette_darken(LV_PALETTE_GREEN, 3), true, 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 60);

    indic = lv_meter_add_arc(meter, scale, 10, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 60);
    lv_meter_set_indicator_end_value(meter, indic, 90);

    indic = lv_meter_add_scale_lines(
        meter, scale, lv_palette_darken(LV_PALETTE_RED, 3),
        lv_palette_darken(LV_PALETTE_RED, 3), true, 0);
    lv_meter_set_indicator_start_value(meter, indic, 60);
    lv_meter_set_indicator_end_value(meter, indic, 90);

    lv_meter_indicator_t *needle = lv_meter_add_needle_line(
        meter, scale, 4, lv_palette_darken(LV_PALETTE_GREY, 4), -25);

    lv_obj_t * spd_label = lv_label_create(screen);
    lv_obj_set_style_text_color(spd_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(spd_label, &lv_font_montserrat_12, 0);

    lv_obj_t * spd_unit_label = lv_label_create(meter);
    lv_label_set_text(spd_unit_label, "-");
    lv_obj_set_style_text_font(spd_unit_label, &lv_font_montserrat_48, 0);
    lv_obj_align(spd_unit_label, LV_ALIGN_CENTER, 60, 60);

    lv_obj_align(spd_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    speed_label = spd_unit_label;
    speed_needle = needle;
    speed_meter = meter;
    status_label = spd_label;
}
