| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- |

# Giải thích và hướng dẫn config code (Cơ bản)
- *Trong bài project này tôi chỉ tạo 1 service cho Server (GATTS_SERVICE_UUID_TEST_A) dùng để cho các client đọc, ghi và gửi dữ liệu cho Server, các dữ liệu liên quan đến UUID,Advertisement sẽ được đặt mặc định*

## Chỉnh sửa các thông số 

```code.c

#define TEST_DEVICE_NAME            "SMART_BIN_V0"   // Chỉnh sửa tên thiết bị

#define DEBUG_ENABLED              1  // Đăt 1 muốn nếu muốn Debug lỗi trong qua trình điều khiển

#define SENSOR_OP_BIN1_PIN         27  // Chân tín hiệu cảm biến hồng ngoại của Thùng rác 1
#define SENSOR_OP_BIN2_PIN         21  // Chân tín hiệu cảm biến hồng ngoại của Thùng rác 2
#define SENSOR_OP_BOX_PIN          13  // Chân tín hiệu cảm biến hồng ngoại của Hộp giấy
#define SENSOR_CK_BIN1_PIN         23  // Chân tín hiệu cảm biến hồng ngoại của Thùng rác 1 để kiểm tra thùng rác đã đầy hay chưa
#define SENSOR_CK_BIN2_PIN         22  // Chân tín hiệu cảm biến hồng ngoại của Thùng rác 1 để kiểm tra thùng rác đã đầy hay chưa

#define  SERVO_CH0_PIN             4   // Chân điều khiển nắp của thùng rác 1
#define  SERVO_CH1_PIN             16  // Chân điều khiển nắp của thùng rác 2
#define  SERVO_CH2_PIN             17  // Chân điều khiển nắp hộp giấy

// Lưu ý quan trọng: chọn góc là các số chia hết cho 3
#define SER0_O                     120 // góc mở nắp của thùng rác 1
#define SER0_C                     0   // góc mặc định khi đóng của thùng rác 1
#define SER1_O                     111 // góc mở nắp của thùng rác 2
#define SER1_C                     0   // góc mặc định khi đóng của thùng rác 2
#define SER2_O                     90  // góc mở nắp của hộp giấy
#define SER2_C                     0   // góc mặc định khi đóng của hộp giấy

#define DRIVER_IN1                 32  // Chân điều khiển động cơ A
#define DRIVER_IN2                 33  // Chân điều khiển động cơ A
#define DRIVER_IN3                 25  // Chân điều khiển động cơ B
#define DRIVER_IN4                 26  // Chân điều khiển động cơ B

#define SPEED                      (FULL_DUTY*2/3)    // Điều chỉnh tốc độ của xe
```

## Nguyên lý điều khiển

### Điều khiển Servo và đông cơ
- Trong ESP32 có 2 cách để băm xung (Xem trong Datasheet)
  + Băm xung bằng LED Control (LEDC)
  + Băm xung bằng Motor Control Pulse Width Modulator (MCPWM)

- Trong Project này tôi sẽ chọn điều khiển bằng LEDC vì LEDC có 2 bộ timer, mỗi bộ có 8 chân channel (nhiều hơn MCPWM).
- Đầu tiên, project sẽ khai báo và config các chân cần băm xung
```code.c

// Chi tiết hàm xem trong code
/*
* Nguyên lý điều khiển servo bằng cách băm xung xem ở link sau:
* https://istem.com.vn/blog/dong-co-servo-hoat-dong-dieu-khien-servo-bang-arduino
*/
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
```
- Trước đó tôi chỉ khai báo và config các chân băm xung cho servo, sau này cần điều khiển cả tốc độ động cơ chạy của thùng rác nên dùng chung timer với servo và khai báo thêm 2 channel là **LEDC_CHANNEL_3**, **LEDC_CHANNEL_4**

- Servo sẽ được điều khiển riêng trong qua trình hoặt động thông qua task **void ServoBreathing(void *pvParameters)**. Hàm xe được RTOS phân công thực hiện trong quá trình chạy
- Trong mỗi vòng loop, hàm sẽ dựa vào góc lệnh (**servo_status**) và góc hiện tại (**cur_servo_status**) để dii chuyển lên hoặc xuống sao cho **servo_status** = **cur_servo_status**
```code.c

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
```
- Động cơ (motor) cũng sẽ dựa vào **motor_status** để điều chỉnh 4 chân output, gửi tín hiệu cho L298 để điều khiển động cơ
```code.c

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
```

- Khi bắt đầu chạy, các góc servo và tốc độ motor sẽ được đưa về mặc định 
```code.c

    servo_write(LEDC_LOW_SPEED_MODE, channel_servo[0], SER0_C);
    servo_write(LEDC_LOW_SPEED_MODE, channel_servo[1], SER1_C);
    servo_write(LEDC_LOW_SPEED_MODE, channel_servo[2], SER2_C);
    
    motor_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, motor_status[0]);
    gpio_set_level(DRIVER_IN2, motor_status[1]);
    motor_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, motor_status[2]);
    gpio_set_level(DRIVER_IN4, motor_status[3]);
```

### Kiểm tra tín hiệu cảm biến 
- Để tránh nhiễu, project đã dùng các biến như **check_bin_cnt_full**, **check_bin_cnt_free**,... để xử lý nhiễu, cũng như tạo delay (Không dùng delay thường vì lúc đó ESP32 bị treo, sẽ ảnh hưởng nhiều nếu chạy lâu dài hoẵ gây lãng phí). 
- Từ tín hiệu của cảm biến, ta sẽ viết các góc để servo quay vào góc lệnh (**servo_status**) hay viết các trạng thái đầy hoặc không đầy qua biến **check_bin** để gửi về điện thoại.
```code.c

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
```

## Giao tiếp với client (Điện thoại)

- Khi tín hiệu kiểm tra rác trong các thùng đã đầy qua biến **check_bin**, ESP32 sẽ gửi tín hiệu lên điện thoại
```code.c

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
```

- Khi điện thoại gửi lệnh đến Server, khi đó hàm callback **gatts_profile_a_event_handler** sẽ được kích hoặt với cờ **ESP_GATTS_WRITE_EVT**. Khi đó, ta sẽ viết các tín hiệu hay các góc cần điều khiển vào các biến **servo_status** hay **motor_status** để **ControlTask** và **ServoBreathing** điều khiển
```code.c

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
```


## *Còn nhiều phần code để xử lý bug để quá trình chạy ổn định hơn sẽ cần phải hiểu sâu hơn và giải thích chi tiết hơn*