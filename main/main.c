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

                ESP_LOGI("Location",
                        "t: %"PRId64", lat: %"PRIi32 ", lon: %"PRIi32 ", alt: %"PRIi32 ", hacc: %"PRIi32", fix: %d",
                        utcTimeNanoseconds, lat, lon, alt, hAcc, fix_status);

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

    const uGnssModuleType_t module_type = U_GNSS_MODULE_TYPE_M10;

    int32_t ubx_err = uPortInit();
    if (ubx_err) {
        ESP_LOGE("GPS", "UBXLIB Port init error: %" PRIi32, ubx_err);
        return;
    }
    ubx_err = uGnssInit();
    if (ubx_err) {
        ESP_LOGE("GPS", "UBXLIB GNSS init error: %" PRIi32, ubx_err);
        return;
    }


    uGnssTransportHandle_t transportHandle;
    transportHandle.uart = uPortUartOpen(
        U_CFG_APP_GNSS_UART, 9600, NULL, U_GNSS_UART_BUFFER_LENGTH_BYTES,
        U_CFG_APP_PIN_GNSS_TXD, U_CFG_APP_PIN_GNSS_RXD, U_CFG_APP_PIN_GNSS_CTS,
        U_CFG_APP_PIN_GNSS_RTS);

    uDeviceHandle_t devHandle = NULL;
    ubx_err = uGnssAdd(module_type, U_GNSS_TRANSPORT_UART,
                       transportHandle, -1, true, &devHandle);
    if (ubx_err) {
      ESP_LOGE("GPS", "UBXLIB GNSS add error: %" PRIi32, ubx_err);
      return;
    }

    uGnssSetUbxMessagePrint(devHandle, true);
    ubx_err = uGnssCfgSetProtocolOut(devHandle, U_GNSS_PROTOCOL_NMEA, false);
    if (ubx_err) {
      ESP_LOGE("GPS", "NMEA disable error %" PRIi32, ubx_err);
    }
    ubx_err = uGnssCfgSetProtocolOut(devHandle, U_GNSS_PROTOCOL_UBX, true);
    if (ubx_err) {
      ESP_LOGE("GPS", "UBX enable error %" PRIi32, ubx_err);
    }

    // Power up the GNSS module
    uGnssPwrOn(devHandle);

    ubx_err = uGnssCfgValSet(devHandle, U_GNSS_CFG_VAL_KEY_ID_MSGOUT_UBX_NAV_PVT_UART1_U1,
            1, U_GNSS_CFG_VAL_TRANSACTION_NONE, U_GNSS_CFG_VAL_LAYER_RAM);
    if (ubx_err != 0) {
      ESP_LOGE("GPS", "UBX Set UBX rate error: %" PRIi32, ubx_err);
    }

    // ubx_err = uGnssCfgValSet(devHandle, U_GNSS_CFG_VAL_KEY_ID_NAVSPG_USE_PPP_L,
    //         1, U_GNSS_CFG_VAL_TRANSACTION_NONE, U_GNSS_CFG_VAL_LAYER_RAM);
    // if (ubx_err != 0) {
    //   ESP_LOGE("GPS", "UBX Set PPP error: %" PRIi32, ubx_err);
    // }

    ESP_LOGI("CFG", "Dynamic model before: %"PRIi32, uGnssCfgGetDynamic(devHandle));

    ubx_err = uGnssCfgSetDynamic(devHandle, U_GNSS_DYNAMIC_AUTOMOTIVE);
    if (ubx_err != 0) {
      ESP_LOGE("GPS", "UBX Set dynmodel error: %" PRIi32, ubx_err);
    }
    ESP_LOGI("CFG", "Dynamic model after: %"PRIi32, uGnssCfgGetDynamic(devHandle));

    int32_t measprdms = 100;
    int32_t measpernav = 1;

    ubx_err = uGnssCfgSetRate(devHandle, measprdms, measpernav, -1);
    if (ubx_err) {
        ESP_LOGE("GPS", "UBXLIB GNSS rate error: %" PRIi32, ubx_err);
        return;
    }

    char *gps_ubx_buf = (char*)pUPortMalloc(92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES);

    uGnssMessageId_t messageId;
    messageId.type = U_GNSS_PROTOCOL_UBX;
    messageId.id.ubx = U_GNSS_UBX_MESSAGE(
            U_GNSS_DEC_UBX_NAV_PVT_MESSAGE_CLASS,
            U_GNSS_DEC_UBX_NAV_PVT_MESSAGE_ID
            );

    uGnssMsgReceiveStart(devHandle, &messageId, gps_ubx_callback, gps_ubx_buf);

    ESP_LOGI("GPS", "Successfully init GPS!");

/*
    uint32_t ret = 0;
    const uDeviceCfg_t gDeviceCfg = {
        .deviceType = U_DEVICE_TYPE_GNSS,
        .deviceCfg = {
            .cfgGnss = {
                .moduleType = module_type,
                .pinEnablePower = U_CFG_APP_PIN_GNSS_ENABLE_POWER,
                .pinDataReady = -1
            },
        },
        .transportType = U_DEVICE_TRANSPORT_TYPE_UART,
        .transportCfg = {
            .cfgUart = {
                .uart = U_CFG_APP_GNSS_UART,
                .baudRate = U_GNSS_UART_BAUD_RATE, 
                .pinTxd = U_CFG_APP_PIN_GNSS_TXD,  // Use -1 if on Zephyr or Linux or Windows
                .pinRxd = U_CFG_APP_PIN_GNSS_RXD,  // Use -1 if on Zephyr or Linux or Windows
                .pinCts = U_CFG_APP_PIN_GNSS_CTS,  // Use -1 if on Zephyr
                .pinRts = U_CFG_APP_PIN_GNSS_RTS,  // Use -1 if on Zephyr
                .pPrefix = NULL,
            },
        },
    };

    // NETWORK configuration for GNSS
    const uNetworkCfgGnss_t gNetworkCfg = {
        .type = U_NETWORK_TYPE_GNSS,
        .moduleType = module_type,
        .devicePinPwr = -1,
        .devicePinDataReady = -1
    };
    ret = uDeviceOpen(&gDeviceCfg, &devHandle);
    uPortLog("Opened device with return code %d.\n", ret);
    if (ret == 0) {
        // Bring up the GNSS network interface
        uPortLog("Bringing up the network...\n");
        if (uNetworkInterfaceUp(devHandle, U_NETWORK_TYPE_GNSS, &gNetworkCfg) == 0) {
            if (0 != uGnssCfgValSet(devHandle, U_GNSS_CFG_VAL_KEY_ITEM_RATE_MEAS_U2, 100, U_GNSS_CFG_VAL_TRANSACTION_NONE, U_GNSS_CFG_VAL_LAYER_RAM))
            {
                ESP_LOGI("Config", "Failed to set rate");
            }
            

            // Start to get location
            uPortLog("Starting continuous location.\n");
            uLocationGetContinuousStart(devHandle,
                    100,
                    U_LOCATION_TYPE_GNSS,
                    NULL, NULL, callback);

            // // Stop getting location
            // uLocationGetStop(devHandle);
            //
            // // When finished with the GNSS network layer
            // uPortLog("Taking down GNSS...\n");
            // uNetworkInterfaceDown(devHandle, U_NETWORK_TYPE_GNSS);
        } else {
            uPortLog("Unable to bring up GNSS!\n");
        }
    }
    */

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL demos");
    if (example_lvgl_lock(-1)) {
        lv_demo_widgets();
        // lv_demo_music();
        // lv_demo_stress();
        // lv_demo_benchmark();

        example_lvgl_unlock();
    }
}
