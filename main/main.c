#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"


#include "sdkconfig.h"

#define GATTS_TAG "GATTS_SMART_BIN_SERVER"

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
// static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

#define TEST_DEVICE_NAME            "SMART_BIN_V0"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024


/*
- FLAG, VARIABLE and STRUCT/////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
static const char *TAG = "SERVO";
#define SERVO_CHECK(a, str, ret_val) \
    if (!(a)) { \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val); \
    }

#define DEBUG_ENABLED              1

#define SENSOR_OP_BIN1_PIN         27
#define SENSOR_OP_BIN2_PIN         21
#define SENSOR_OP_BOX_PIN          13
#define SENSOR_CK_BIN1_PIN         23
#define SENSOR_CK_BIN2_PIN         22

#define  SERVO_CH0_PIN             4
#define  SERVO_CH1_PIN             16
#define  SERVO_CH2_PIN             17

#define SER0_O                     120
#define SER0_C                     0
#define SER1_O                     111
#define SER1_C                     0
#define SER2_O                     90
#define SER2_C                     0

#define CK_DELAY                   500
#define OP_DELAY                   20

#define SERVO_LEDC_INIT_BITS       LEDC_TIMER_10_BIT
#define FULL_DUTY                  ((1 << SERVO_LEDC_INIT_BITS) - 1)
#define SERVO_CHANNEL_MAX          5
#define MAX_SERVO_ANGLE            180
#define MIN_SERVO_ANGLE            0
#define MIN_WIDTH_US               500
#define MAX_WIDTH_US               2500
#define SERVO_FREQ                 50

#define DRIVER_IN1                 32
#define DRIVER_IN2                 33
#define DRIVER_IN3                 25
#define DRIVER_IN4                 26

#define SPEED                      (FULL_DUTY*2/3)

static bool check_en_connect = false;
static bool check_motor_control = false;
static uint32_t motor_status[4] = {0,0,0,0};
static uint8_t servo_status[3] = {SER0_C,SER1_C,SER2_C};
static uint8_t cur_servo_status[3] = {SER0_C,SER1_C,SER2_C};
static bool bin_1_open_by_phone = false;
static bool bin_2_open_by_phone = false;
static uint8_t servo_st_cnt_on[3] = {0,0,0};
static uint8_t servo_st_cnt_off[3] = {0,0,0};
static uint16_t check_bin_cnt_full[2] = {0,0};
static uint16_t check_bin_cnt_free[2] = {0,0};
static uint8_t check_bin[2] = {0,0};
static uint8_t old_check_bin[2] = {0,0};
static uint8_t channel_servo[3] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};

typedef struct {
    gpio_num_t servo_pin[SERVO_CHANNEL_MAX];     
    ledc_channel_t ch[SERVO_CHANNEL_MAX];    
} servo_channel_t;

typedef struct {
    ledc_timer_t timer_number; 
    servo_channel_t channels;    
} servo_config_t;

static uint32_t calculate_duty(float angle)
{
    float angle_us = angle / MAX_SERVO_ANGLE * (MAX_WIDTH_US - MIN_WIDTH_US) + MIN_WIDTH_US;
    //ESP_LOGE(GATTS_TAG, "angle us: %f", angle_us);
    uint32_t duty = (uint32_t)(FULL_DUTY * (angle_us) * SERVO_FREQ / (1000000.0f));
    return duty;
}

esp_err_t servo_init(ledc_mode_t speed_mode, const servo_config_t *config)
{
    esp_err_t ret;
    SERVO_CHECK(NULL != config, "Pointer of config is invalid", ESP_ERR_INVALID_ARG);
    uint64_t pin_mask = 0;
    uint32_t ch_mask = 0;
    for (size_t i = 0; i < SERVO_CHANNEL_MAX; i++) {
        uint64_t _pin_mask = 1ULL << config->channels.servo_pin[i];
        uint32_t _ch_mask = 1UL << config->channels.ch[i];
        SERVO_CHECK(!(pin_mask & _pin_mask), "servo gpio has a duplicate", ESP_ERR_INVALID_ARG);
        SERVO_CHECK(!(ch_mask & _ch_mask), "servo channel has a duplicate", ESP_ERR_INVALID_ARG);
        SERVO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(config->channels.servo_pin[i]), "servo gpio invalid", ESP_ERR_INVALID_ARG);
        pin_mask |= _pin_mask;
        ch_mask |= _ch_mask;
    }

    ledc_timer_config_t ledc_timer = {
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = SERVO_LEDC_INIT_BITS,     // resolution of PWM duty
        .freq_hz = SERVO_FREQ,                     // frequency of PWM signal
        .speed_mode = speed_mode,            // timer mode
        .timer_num = config->timer_number            // timer index
    };
    ret = ledc_timer_config(&ledc_timer);
    SERVO_CHECK(ESP_OK == ret, "ledc timer configuration failed", ESP_FAIL);
    for (size_t i = 0; i < SERVO_CHANNEL_MAX; i++) {
        ledc_channel_config_t ledc_ch = {
            .intr_type  = LEDC_INTR_DISABLE,
            .channel    = config->channels.ch[i],
            .duty       = calculate_duty(0),
            .gpio_num   = config->channels.servo_pin[i],
            .speed_mode = speed_mode,
            .timer_sel  = config->timer_number,
            .hpoint     = 0
        };
        ret = ledc_channel_config(&ledc_ch);
        SERVO_CHECK(ESP_OK == ret, "ledc channel configuration failed", ESP_FAIL);
    }

    return ESP_OK;
}

esp_err_t servo_write(ledc_mode_t speed_mode, uint8_t channel, float angle)
{
    SERVO_CHECK(speed_mode < LEDC_SPEED_MODE_MAX, "LEDC speed mode invalid", ESP_ERR_INVALID_ARG);
    SERVO_CHECK(channel < LEDC_CHANNEL_MAX, "LEDC channel number too large", ESP_ERR_INVALID_ARG);
    SERVO_CHECK(angle >= 0.0f, "Angle can't to be negative", ESP_ERR_INVALID_ARG);
    esp_err_t ret;
    uint32_t duty = calculate_duty(angle);
    ret = ledc_set_duty(speed_mode, (ledc_channel_t)channel, duty);
    ret |= ledc_update_duty(speed_mode, (ledc_channel_t)channel);
    SERVO_CHECK(ESP_OK == ret, "write servo angle failed", ESP_FAIL);
    return ESP_OK;
}

esp_err_t motor_write(ledc_mode_t speed_mode, uint8_t channel, uint32_t speed)
{
    esp_err_t ret;
    ret = ledc_set_duty(speed_mode, (ledc_channel_t)channel, speed);
    ret |= ledc_update_duty(speed_mode, (ledc_channel_t)channel);
    SERVO_CHECK(ESP_OK == ret, "write servo angle failed", ESP_FAIL);
    return ESP_OK;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}


static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
        }
        uint8_t *input_data_p = param->write.value;
        uint8_t input_data_ck = input_data_p[0];
        uint8_t input_data_ctrl = input_data_p[1];
        switch (input_data_ck)
        {
        case 0x01:
            if(!check_en_connect) {check_en_connect = true;}
            if(check_en_connect)
            {  
                esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_A_APP_ID].gatts_if , gl_profile_tab[PROFILE_A_APP_ID].conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof( old_check_bin),  old_check_bin, true);
            }  
            break;
        
        case 0x10:
            check_motor_control = true;
            if(input_data_ctrl == 1){
                motor_status[0] = SPEED; motor_status[1] = 0; motor_status[2] = SPEED; motor_status[3] = 0;
            }else if(input_data_ctrl == 2){
                motor_status[0] = SPEED; motor_status[1] = 0; motor_status[2] = FULL_DUTY - SPEED; motor_status[3] = 1;
            }else if(input_data_ctrl == 3){
                motor_status[0] = FULL_DUTY - SPEED; motor_status[1] = 1; motor_status[2] = FULL_DUTY - SPEED; motor_status[3] = 1;
            }else if(input_data_ctrl == 4){
                motor_status[0] = FULL_DUTY - SPEED; motor_status[1] = 1; motor_status[2] = SPEED; motor_status[3] = 0;
            }else if(input_data_ctrl == 5){
                motor_status[0] = 0; motor_status[1] = 0; motor_status[2] = 0; motor_status[3] = 0;
            }
            break;

        case 0x11:
            if(input_data_ctrl == 1){
                servo_status[0] = SER0_O;
                bin_1_open_by_phone = true;
            }else if(input_data_ctrl == 2){
                servo_status[0] = SER0_C;
                bin_1_open_by_phone = false;
            }
            break;
        
        case 0x12:
            if(input_data_ctrl == 1){
                servo_status[1] = SER1_O;
                bin_2_open_by_phone = true;
            }else if(input_data_ctrl == 2){
                servo_status[1] = SER1_C;
                bin_2_open_by_phone = false;
            }
            break;

        default:
            break;
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo length = %x\n", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT ,conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        gl_profile_tab[PROFILE_A_APP_ID].gatts_if = gatts_if;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        //check_en_connect = true;
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        check_en_connect = false;
        bin_1_open_by_phone = false;
        bin_2_open_by_phone = false;
        // Stop motor when Disconnect BLE
        check_motor_control = true;
        motor_status[0] = 0; motor_status[1] = 0; motor_status[2] = 0; motor_status[3] = 0;
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


void initialize_NVS(){
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
}

void BLE_config(){
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}

void Config_GPIO(){
    // Config input pin ////////////////////////////////////////////////
    gpio_config_t GPIO_config_IN = {};
    GPIO_config_IN.pin_bit_mask = ((uint64_t)1 << SENSOR_OP_BIN1_PIN) | ((uint64_t)1 << SENSOR_OP_BIN2_PIN) | ((uint64_t)1 << SENSOR_CK_BIN1_PIN) | ((uint64_t)1 << SENSOR_CK_BIN2_PIN) | ((uint64_t)1 << SENSOR_OP_BOX_PIN);
    GPIO_config_IN.mode = GPIO_MODE_INPUT;
    GPIO_config_IN.pull_up_en = 0;
    GPIO_config_IN.pull_down_en = 0;
    GPIO_config_IN.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&GPIO_config_IN);

    // Config output pin ///////////////////////////////////////////////
    gpio_config_t GPIO_config_OUT = {};
    GPIO_config_OUT.pin_bit_mask = ((uint64_t)1 << DRIVER_IN1) | ((uint64_t)1 << DRIVER_IN2) | ((uint64_t)1 << DRIVER_IN3) | ((uint64_t)1 << DRIVER_IN4);
#ifdef DEBUG_ENABLED
    ESP_LOGE(GATTS_TAG, "pin_bit_mask config: %" PRIu64 " ", GPIO_config_OUT.pin_bit_mask);
#endif    
    GPIO_config_OUT.mode = GPIO_MODE_OUTPUT;
    GPIO_config_OUT.pull_up_en = 0;
    GPIO_config_OUT.pull_down_en = 0;
    GPIO_config_OUT.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&GPIO_config_OUT);
}

void ControlTask(void *pvParameters) {
    while(1){
        if(old_check_bin[0] != check_bin[0] || old_check_bin[1] != check_bin[1]){
            old_check_bin[0] = check_bin[0];
            old_check_bin[1] = check_bin[1];
#ifdef DEBUG_ENABLED
            ESP_LOGE(GATTS_TAG, "Switch State: %d - %d", old_check_bin[1],  old_check_bin[1]);
#endif
            if(check_en_connect)
            {  
                esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_A_APP_ID].gatts_if , gl_profile_tab[PROFILE_A_APP_ID].conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof( old_check_bin),  old_check_bin, true);
            }  
        }

        if(check_motor_control){
            check_motor_control = false;
#ifdef DEBUG_ENABLED
            ESP_LOGE(GATTS_TAG, "get motor change");
#endif           
            motor_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, motor_status[0]);
            gpio_set_level(DRIVER_IN2, motor_status[1]);
            motor_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, motor_status[2]);
            gpio_set_level(DRIVER_IN4, motor_status[3]);

        }
    } 
}

void ReadSensorTask(void *pvParameters) {
    while (1) {
        // Read check full Bin 1
        if(gpio_get_level(SENSOR_CK_BIN1_PIN) == 1){
#ifdef DEBUG_ENABLED
            //ESP_LOGE(GATTS_TAG, "full check 1: %d", check_bin_cnt_full[0]);
#endif
            check_bin_cnt_full[0]++;
            check_bin_cnt_free[0] = 0;
            if(check_bin_cnt_full[0] >= CK_DELAY){
                check_bin[0] = 1;
                check_bin_cnt_full[0] = 0;
            }
        }else{
            check_bin_cnt_free[0]++;
            check_bin_cnt_full[0] = 0;
            if(check_bin_cnt_free[0] >= CK_DELAY){
                check_bin[0] = 0;
                check_bin_cnt_free[0] = 0;
            }
        }
        // Read check full Bin 2
        if(gpio_get_level(SENSOR_CK_BIN2_PIN) == 1){
#ifdef DEBUG_ENABLED
            ESP_LOGE(GATTS_TAG, "full check 2: %d", check_bin_cnt_full[1]);
#endif
            check_bin_cnt_full[1]++;
            check_bin_cnt_free[1] = 0;
            if(check_bin_cnt_full[1] >= CK_DELAY){
                check_bin[1] = 1;
                check_bin_cnt_full[1] = 0;
            }
        }else{
            check_bin_cnt_free[1]++;
            check_bin_cnt_full[1] = 0;
            if(check_bin_cnt_free[1] >= CK_DELAY){
                check_bin[1] = 0;
                check_bin_cnt_free[1] = 0;
            }
        }
        // Read open or close
        if(!bin_1_open_by_phone){
            if(gpio_get_level(SENSOR_OP_BIN1_PIN) == 1){
                servo_st_cnt_on[0]++;
                servo_st_cnt_off[0] = 0;
                if(servo_st_cnt_on[0] >= OP_DELAY){servo_st_cnt_on[0] = 0; servo_status[0] = SER0_O;}
            }else{
                servo_st_cnt_off[0]++;
                servo_st_cnt_on[0] = 0;
                if(servo_st_cnt_off[0] >= OP_DELAY){servo_st_cnt_off[0] = 0; servo_status[0] = SER0_C;}
            }
        }
        if(!bin_2_open_by_phone){
            if(gpio_get_level(SENSOR_OP_BIN2_PIN) == 1){
                servo_st_cnt_on[1]++;
                servo_st_cnt_off[1] = 0;
                if(servo_st_cnt_on[1] >= OP_DELAY){servo_st_cnt_on[1] = 0; servo_status[1] = SER1_O;}
            }else{
                servo_st_cnt_off[1]++;
                servo_st_cnt_on[1] = 0;
                if(servo_st_cnt_off[1] >= OP_DELAY){servo_st_cnt_off[1] = 0; servo_status[1] = SER1_C;}
            }
        }
        if(gpio_get_level(SENSOR_OP_BOX_PIN) == 1){
#ifdef DEBUG_ENABLED
            ESP_LOGE(GATTS_TAG, "polling box open: %d", servo_st_cnt_on[2]);
#endif
            servo_st_cnt_on[2]++;
            servo_st_cnt_off[2] = 0;
            if(servo_st_cnt_on[2] >= OP_DELAY){servo_st_cnt_on[2] = 0; servo_status[2] = SER2_O;}
        }else{
#ifdef DEBUG_ENABLED
            //ESP_LOGE(GATTS_TAG, "polling box close: %d", servo_st_cnt_off[2]);
#endif
            servo_st_cnt_off[2]++;
            servo_st_cnt_on[2] = 0;
            if(servo_st_cnt_off[2] >= OP_DELAY){servo_st_cnt_off[2] = 0; servo_status[2] = SER2_C;}
        }
      
        vTaskDelay(pdMS_TO_TICKS(10)); // Đợi 10ms
    }
}

void ServoBreathing(void *pvParameters) {
    while (1) {
        for(int i = 0; i < 3; i++){
            if(cur_servo_status[i] < servo_status[i]){
                cur_servo_status[i] += 3;
                servo_write(LEDC_LOW_SPEED_MODE, channel_servo[i], cur_servo_status[i]);
#ifdef DEBUG_ENABLED
                ESP_LOGE(GATTS_TAG, "servo %d State %d", i, cur_servo_status[i]);
#endif
            }else if(cur_servo_status[i] > servo_status[i]){
                cur_servo_status[i] -= 3;
                servo_write(LEDC_LOW_SPEED_MODE, channel_servo[i], cur_servo_status[i]);
#ifdef DEBUG_ENABLED
                ESP_LOGE(GATTS_TAG, "servo %d State %d", i, cur_servo_status[i]);
#endif

            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Đợi 5ms
    }
}

void app_main(void)
{
    initialize_NVS();
    BLE_config();
    Config_GPIO();
    servo_config_t servo_cfg = {
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_CH0_PIN,
                SERVO_CH1_PIN,
                SERVO_CH2_PIN,
                DRIVER_IN1,
                DRIVER_IN3
                },
            .ch = {
                LEDC_CHANNEL_0,
                LEDC_CHANNEL_1,
                LEDC_CHANNEL_2,
                LEDC_CHANNEL_3,
                LEDC_CHANNEL_4
            },
        },
    };
    servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    servo_write(LEDC_LOW_SPEED_MODE, channel_servo[0], SER0_C);
    servo_write(LEDC_LOW_SPEED_MODE, channel_servo[1], SER1_C);
    servo_write(LEDC_LOW_SPEED_MODE, channel_servo[2], SER2_C);
    
    motor_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, motor_status[0]);
    gpio_set_level(DRIVER_IN2, motor_status[1]);
    motor_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, motor_status[2]);
    gpio_set_level(DRIVER_IN4, motor_status[3]);
    
    //while(1);
    xTaskCreate(ReadSensorTask, "ReadSensorTask", 4096, NULL, 0, NULL);
    xTaskCreate(ControlTask, "ControlTask", 4096, NULL, 0, NULL);
    xTaskCreate(ServoBreathing, "ServoBreathing", 4096, NULL, 0, NULL);
    return;
}