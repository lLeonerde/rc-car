/*
 * ESP32 BLE Drift Car Controller
 * Based on ESP-IDF BLE throughput example
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_system.h"
 #include "esp_log.h"
 #include "nvs_flash.h"
 #include "esp_bt.h"
 #include "esp_gap_ble_api.h"
 #include "esp_gatts_api.h"
 #include "esp_bt_defs.h"
 #include "esp_bt_main.h"
 #include "esp_bt_device.h"
 #include "esp_gatt_common_api.h"
 #include "driver/ledc.h"  // For PWM control
 #include "esp_timer.h" // For servo auto-detach timer
 #include "esp_timer.h" // For servo auto-detach timer
 
 #define GATTS_TAG "DRIFT_CAR_B+LE"
 
 // Device name
 static char device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "DRIFT_CAR";
 
 // PWM configuration
 #define PWM_MOTOR_CHANNEL        LEDC_CHANNEL_0
 #define PWM_STEERING_CHANNEL     LEDC_CHANNEL_1
 #define PWM_STEERING_TIMER       LEDC_TIMER_0 // Renamed from PWM_TIMER
 #define PWM_MOTOR_TIMER          LEDC_TIMER_1 // New timer for motor
 #define PWM_RESOLUTION           LEDC_TIMER_8_BIT // For motor
 #define PWM_STEERING_RESOLUTION  LEDC_TIMER_12_BIT // For steering
 #define PWM_STEERING_FREQ_HZ     50 // Renamed from PWM_FREQ
 #define PWM_MOTOR_FREQ_HZ        30000 // New frequency for motor
 #define MOTOR_GPIO               10  // Change to your motor control GPIO
 #define STEERING_GPIO            3  // Change to your steering control GPIO
 #define STEERING_DEADBAND_THRESHOLD 3 // Degrees
 #define PWM_STEERING_MIN_DUTY 225
 #define PWM_STEERING_MAX_DUTY 390
 #define SERVO_AUTO_DETACH_MS 50
 
 // BLE Service and Characteristic UUIDs
 #define DRIFT_CAR_SERVICE_UUID           0xABCD
 #define CONTROL_CHARACTERISTIC_UUID      0x1234
 #define STATUS_CHARACTERISTIC_UUID       0x5678
 #define ESP_GATT_UUID_CHAR_CLIENT_CONFIG    0x2902 // Standard CCCD UUID

 // Define the structure for a profile
 #define PROFILE_NUM 1
 #define PROFILE_APP_ID 0
 
 struct gatts_profile_inst {
     esp_gatts_cb_t gatts_cb;
     uint16_t gatts_if;
     uint16_t app_id;
     uint16_t conn_id;
     uint16_t service_handle;
     esp_gatt_srvc_id_t service_id;
     uint16_t control_char_handle;
     uint16_t status_char_handle;
     esp_bt_uuid_t control_char_uuid;
     esp_bt_uuid_t status_char_uuid;
 };
 
 // Control command structure
 typedef struct {
     uint8_t throttle;  // 0-100 for motor speed
     uint8_t steering;  // 0-180 for steering angle (90 is center)
 } car_control_t;
 
 static car_control_t car_state = {
     .throttle = 0,
     .steering = 90
 };
 static esp_timer_handle_t servo_detach_timer_handle;
 static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

 // Profile instance
 static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
     [PROFILE_APP_ID] = {
         .gatts_cb = gatts_profile_event_handler,
         .gatts_if = ESP_GATT_IF_NONE,
     },
 };
 
  // Prepare write buffer
  #define PREPARE_BUF_MAX_SIZE 1024
  typedef struct {
      uint8_t *prepare_buf;
      int prepare_len;
  } prepare_type_env_t;
  

 // Prepare write environment variable
 static prepare_type_env_t prepare_write_env;
 
 // Declare the static function
 
 // Advertising data
 static uint8_t adv_config_done = 0;
 #define adv_config_flag      (1 << 0)
 #define scan_rsp_config_flag (1 << 1)
 
 static esp_ble_adv_data_t adv_data = {
     .set_scan_rsp = false,
     .include_name = true,
     .include_txpower = true,
     .min_interval = 0x0006, // Slave connection min interval, Time = min_interval * 1.25 msec
     .max_interval = 0x000C, // Slave connection max interval, Time = max_interval * 1.25 msec
     .appearance = 0x00,
     .manufacturer_len = 0,
     .p_manufacturer_data = NULL,
     .service_data_len = 0,
     .p_service_data = NULL,
     .service_uuid_len = 0,
     .p_service_uuid = NULL,
     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
 };
 
 static esp_ble_adv_data_t scan_rsp_data = {
     .set_scan_rsp = true,
     .include_name = true,
     .include_txpower = true,
     .min_interval = 0x0006,
     .max_interval = 0x000C,
     .appearance = 0x00,
     .manufacturer_len = 0,
     .p_manufacturer_data = NULL,
     .service_data_len = 0,
     .p_service_data = NULL,
     .service_uuid_len = 0,
     .p_service_uuid = NULL,
     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
 };
 
 static esp_ble_adv_params_t adv_params = {
     .adv_int_min = 0x20,
     .adv_int_max = 0x40,
     .adv_type = ADV_TYPE_IND,
     .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
     .channel_map = ADV_CHNL_ALL,
     .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
 };
 

 // Forward declarations for required handling functions
 void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
 void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

 // Function to disengage the steering servo
static void detach_steering_servo(void) {
    esp_err_t err_stop = ledc_stop(LEDC_LOW_SPEED_MODE, PWM_STEERING_CHANNEL, 1); // Set output low when stopped
    if (err_stop != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to stop steering servo PWM: %s", esp_err_to_name(err_stop));
    }
    // ESP_LOGI(GATTS_TAG, "Steering servo detached"); // Optional: for debugging
}
static void servo_detach_timer_callback(void* arg) {
    ESP_LOGI(GATTS_TAG, "Servo auto-detach timer expired");
    detach_steering_servo();
}
 
 // Setup PWM channels for motor and steering control
 static void init_pwm(void) {
     // Configure steering timer
     ledc_timer_config_t steering_ledc_timer = {
        .duty_resolution = PWM_STEERING_RESOLUTION, // Use new 10-bit resolution
         .freq_hz = PWM_STEERING_FREQ_HZ, // Use renamed steering freq
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .timer_num = PWM_STEERING_TIMER, // Use renamed steering timer
         .clk_cfg = LEDC_AUTO_CLK,
     };
     ledc_timer_config(&steering_ledc_timer);
 
     // Configure motor timer
     ledc_timer_config_t motor_ledc_timer = {
         .duty_resolution = PWM_RESOLUTION, 
         .freq_hz = PWM_MOTOR_FREQ_HZ,      // Use new motor freq
         .speed_mode = LEDC_LOW_SPEED_MODE, 
         .timer_num = PWM_MOTOR_TIMER,      // Use new motor timer
         .clk_cfg = LEDC_AUTO_CLK,
     };
     ledc_timer_config(&motor_ledc_timer);
 
     // Configure motor channel
     ledc_channel_config_t motor_channel = {
         .channel    = PWM_MOTOR_CHANNEL,
         .duty       = 0,
         .gpio_num   = MOTOR_GPIO,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint     = 0,
         .timer_sel  = PWM_MOTOR_TIMER, // Assign motor channel to motor timer
     };
     ledc_channel_config(&motor_channel);
 
     // Configure steering channel
     ledc_channel_config_t steering_channel = {
         .channel    = PWM_STEERING_CHANNEL,
         .duty       = (PWM_STEERING_MIN_DUTY + PWM_STEERING_MAX_DUTY) / 2, // Auto-calculated center
         .gpio_num   = STEERING_GPIO,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint     = 0,
         .timer_sel  = PWM_STEERING_TIMER, // Assign steering channel to steering timer
     };
     ledc_channel_config(&steering_channel);
}

// Function to engage/update the steering servo
static void attach_steering_servo(uint32_t duty_cycle) {
    esp_err_t err_set_duty = ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_STEERING_CHANNEL, duty_cycle);
    if (err_set_duty != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to set steering duty: %s", esp_err_to_name(err_set_duty));
    }
    esp_err_t err_update_duty = ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_STEERING_CHANNEL);
    if (err_update_duty != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to update steering duty: %s", esp_err_to_name(err_update_duty));
    }
    // ESP_LOGI(GATTS_TAG, "Steering servo attached/updated, duty: %u", (unsigned int)duty_cycle); // Optional: for debugging
}


 
 // Update PWM values based on control commands
 static void update_car_control(void) {
     static uint8_t last_commanded_steering_angle = 90; // Initialized to center
 
     // Convert throttle (0-100) to PWM duty cycle (0-255)
     uint8_t motor_duty = (car_state.throttle * 255) / 100;
     
     // Set motor duty cycle first (unrelated to steering deadband)
     ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_MOTOR_CHANNEL, motor_duty);
     ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_MOTOR_CHANNEL);

     // Steering deadband logic
     if (abs(car_state.steering - last_commanded_steering_angle) > STEERING_DEADBAND_THRESHOLD) {
         uint32_t steering_duty = PWM_STEERING_MIN_DUTY + (car_state.steering * (PWM_STEERING_MAX_DUTY - PWM_STEERING_MIN_DUTY)) / 180;
         
         // Stop any existing timer before starting a new sequence
         esp_err_t err_timer_stop = esp_timer_stop(servo_detach_timer_handle);
         if (err_timer_stop == ESP_OK) {
             // ESP_LOGI(GATTS_TAG, "Previously running servo_detach_timer stopped."); // Optional debug log
         } else if (err_timer_stop == ESP_ERR_INVALID_STATE) {
             // This means the timer was not running, which is fine.
             // ESP_LOGI(GATTS_TAG, "servo_detach_timer was not running, no need to stop."); // Optional debug log
         } else {
             ESP_LOGE(GATTS_TAG, "Failed to stop servo_detach_timer: %s", esp_err_to_name(err_timer_stop));
         }

         attach_steering_servo(steering_duty); // This function now handles set_duty and update_duty
         
         // Start/Restart the one-shot timer to detach the servo later
         esp_err_t err_timer_start = esp_timer_start_once(servo_detach_timer_handle, SERVO_AUTO_DETACH_MS * 1000); // Convert ms to microseconds
         if (err_timer_start != ESP_OK) {
             ESP_LOGE(GATTS_TAG, "Failed to start servo detach timer: %s", esp_err_to_name(err_timer_start));
         }
         
         last_commanded_steering_angle = car_state.steering;

         ESP_LOGI(GATTS_TAG, "Steering updated to %d (Duty: %u). Detach timer started for %d ms.", 
                  car_state.steering, (unsigned int)steering_duty, SERVO_AUTO_DETACH_MS);
     } else {
         // Steering change is within deadband. Servo state is not actively changed here.
         // If timer was running, it will continue. If servo was detached, it stays detached.
         ESP_LOGI(GATTS_TAG, "Steering change (to %d) within deadband from %d. Servo state unchanged by this command.", 
                  car_state.steering, last_commanded_steering_angle);
     }
 }
 
 // Parse control commands from BLE
 static void parse_control_command(uint8_t *data, uint16_t length) {
     // Simple text command parsing
     // Format: "T:50,S:90" for throttle 50%, steering 90 degrees
     char cmd[50] = {0};
     memcpy(cmd, data, length < 49 ? length : 49);
     cmd[length] = '\0';
     
     // Log received command
     ESP_LOGI(GATTS_TAG, "Received command: %s", cmd);
     
     // Parse throttle
     char *throttle_str = strstr(cmd, "T:");
     if (throttle_str) {
         int throttle = atoi(throttle_str + 2);
         // Constrain to valid range
         car_state.throttle = (throttle < 0) ? 0 : (throttle > 100) ? 100 : throttle;
     }
     
     // Parse steering
     char *steering_str = strstr(cmd, "S:");
     if (steering_str) {
         int steering = atoi(steering_str + 2);
         // Constrain to valid range
         car_state.steering = (steering < 0) ? 0 : (steering > 180) ? 180 : steering;
     }
     
     // Update PWM controls
     update_car_control();
 }
 
 // GAP event handler
 static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
     switch (event) {
         case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
             adv_config_done &= (~adv_config_flag);
             if (adv_config_done == 0) {
                 esp_ble_gap_start_advertising(&adv_params);
             }
             break;
         case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
             adv_config_done &= (~scan_rsp_config_flag);
             if (adv_config_done == 0) {
                 esp_ble_gap_start_advertising(&adv_params);
             }
             break;
         case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
             // Advertising start complete event to indicate advertising start successfully or failed
             if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                 ESP_LOGE(GATTS_TAG, "Advertising start failed");
             } else {
                 ESP_LOGI(GATTS_TAG, "Advertising started successfully");
             }
             break;
         case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
             if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                 ESP_LOGE(GATTS_TAG, "Advertising stop failed");
             } else {
                 ESP_LOGI(GATTS_TAG, "Advertising stopped successfully");
             }
             break;
         default:
             break;
     }
 }
 
 // GATTS profile event handler
 static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "GATT service registered, status %d, app_id %d", param->reg.status, param->reg.app_id);
            if (param->reg.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
                return;
            }
            gl_profile_tab[PROFILE_APP_ID].gatts_if = gatts_if; // Store gatts_if like in example

            // Set device name
            esp_ble_gap_set_device_name(device_name);

            // Config adv data
            esp_ble_gap_config_adv_data(&adv_data);
            adv_config_done |= adv_config_flag;

            // Config scan response data
            esp_ble_gap_config_adv_data(&scan_rsp_data);
            adv_config_done |= scan_rsp_config_flag;

            // Create service
            gl_profile_tab[PROFILE_APP_ID].service_id.is_primary = true;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = DRIFT_CAR_SERVICE_UUID;

            // Correct handle count was already identified (7)
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_APP_ID].service_id, 7);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "Service created, status %d, service_handle %d", param->create.status, param->create.service_handle);
             if (param->create.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "Create service failed, status %d", param->create.status);
                return;
            }
            gl_profile_tab[PROFILE_APP_ID].service_handle = param->create.service_handle;

            // Start service
            esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_ID].service_handle);

            // --- Add FIRST characteristic (Control Characteristic) ---
            gl_profile_tab[PROFILE_APP_ID].control_char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_ID].control_char_uuid.uuid.uuid16 = CONTROL_CHARACTERISTIC_UUID;

            // Note: Using NULL for initial value and auto handle response
            esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_ID].service_handle,
                                   &gl_profile_tab[PROFILE_APP_ID].control_char_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // Permissions for the value itself
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY, // Properties declared
                                   NULL, // No initial static value (use NULL or esp_attr_value_t struct)
                                   NULL); // Use default response handler (NULL) or provide custom one
            break;

        case ESP_GATTS_ADD_CHAR_EVT: { // Use braces for scope
            esp_bt_uuid_t cccd_uuid;
            cccd_uuid.len = ESP_UUID_LEN_16;
            cccd_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG; // Standard CCCD UUID

            ESP_LOGI(GATTS_TAG, "Characteristic added, status %d, attr_handle %d, service_handle %d, char_uuid 0x%04x",
                     param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle, param->add_char.char_uuid.uuid.uuid16);

            if (param->add_char.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "Add char failed, error code: 0x%x", param->add_char.status);
                break; // Exit if char add failed
            }

            // --- Check WHICH characteristic was just added ---
            if (param->add_char.char_uuid.uuid.uuid16 == CONTROL_CHARACTERISTIC_UUID) {
                gl_profile_tab[PROFILE_APP_ID].control_char_handle = param->add_char.attr_handle;
                ESP_LOGI(GATTS_TAG, "Control characteristic added, handle: %d", param->add_char.attr_handle);

                // --- Add CCCD for the Control Characteristic ---
                esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
                    gl_profile_tab[PROFILE_APP_ID].service_handle,
                    &cccd_uuid, // Use the CCCD UUID
                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // CCCD needs read/write permission from client
                    NULL, // No initial static value for CCCD needed here
                    NULL); // Use default response handler
                if (add_descr_ret) {
                    ESP_LOGE(GATTS_TAG, "Add CCCD for control char failed, error code =%x", add_descr_ret);
                } else {
                     ESP_LOGI(GATTS_TAG, "Initiated adding CCCD for Control Characteristic");
                }
                // ---- DO NOT add the status characteristic here ----

            } else if (param->add_char.char_uuid.uuid.uuid16 == STATUS_CHARACTERISTIC_UUID) {
                gl_profile_tab[PROFILE_APP_ID].status_char_handle = param->add_char.attr_handle;
                ESP_LOGI(GATTS_TAG, "Status characteristic added, handle: %d", param->add_char.attr_handle);

                // --- Add CCCD for the Status Characteristic ---
                 esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
                    gl_profile_tab[PROFILE_APP_ID].service_handle,
                    &cccd_uuid, // Use the CCCD UUID
                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                    NULL,
                    NULL);
                if (add_descr_ret) {
                    ESP_LOGE(GATTS_TAG, "Add CCCD for status char failed, error code =%x", add_descr_ret);
                } else {
                     ESP_LOGI(GATTS_TAG, "Initiated adding CCCD for Status Characteristic");
                }
            }
            break; // Break for ESP_GATTS_ADD_CHAR_EVT
        } // End scope for ESP_GATTS_ADD_CHAR_EVT

        case ESP_GATTS_ADD_CHAR_DESCR_EVT: // *** Handle Descriptor Add Confirmation ***
             ESP_LOGI(GATTS_TAG, "Descriptor added, status %d, attr_handle %d, service_handle %d, descr_uuid: 0x%04x",
                             param->add_char_descr.status,
                             param->add_char_descr.attr_handle,
                             param->add_char_descr.service_handle,
                             param->add_char_descr.descr_uuid.uuid.uuid16);

            if (param->add_char_descr.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "Add descriptor failed, status %d", param->add_char_descr.status);
                break; // Exit if desc add failed
            }

            if (gl_profile_tab[PROFILE_APP_ID].status_char_handle == 0) { // Check if status char handle is uninitialized (or use a state flag)
               ESP_LOGI(GATTS_TAG, "Control CCCD added. Now adding Status Characteristic.");

                // --- Add SECOND characteristic (Status Characteristic) ---
                gl_profile_tab[PROFILE_APP_ID].status_char_uuid.len = ESP_UUID_LEN_16;
                gl_profile_tab[PROFILE_APP_ID].status_char_uuid.uuid.uuid16 = STATUS_CHARACTERISTIC_UUID;

                esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_ID].service_handle,
                                      &gl_profile_tab[PROFILE_APP_ID].status_char_uuid,
                                      ESP_GATT_PERM_READ, // Only needs read permission for the value itself
                                      ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, // Properties declared
                                      NULL,
                                      NULL);
            } else {
                ESP_LOGI(GATTS_TAG, "Status CCCD added. GATT profile setup complete.");
                // Both characteristics and their CCCDs are now added.
            }
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TAG, "Read event, conn_id %d, trans_id %ld, handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
            // --- Handle reads for BOTH characteristics AND potentially CCCDs ---
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;

            if (param->read.handle == gl_profile_tab[PROFILE_APP_ID].control_char_handle) {
                // Return current control values (as string)
                char status_str[50];
                int len = snprintf(status_str, sizeof(status_str), "T:%d,S:%d", car_state.throttle, car_state.steering);
                rsp.attr_value.len = len;
                memcpy(rsp.attr_value.value, status_str, len);
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            }
            else if (param->read.handle == gl_profile_tab[PROFILE_APP_ID].status_char_handle) {
                // Prepare and send status data (e.g., battery voltage, sensor readings - currently none implemented)
                // For now, just send a static message or the control state again
                char status_str[50];
                int len = snprintf(status_str, sizeof(status_str), "Status: OK T:%d,S:%d", car_state.throttle, car_state.steering); // Example status
                rsp.attr_value.len = len;
                memcpy(rsp.attr_value.value, status_str, len);
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            }
            // Add checks here if you need to respond to reads of the CCCD handles explicitly
            // else {
            //    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_READ_NOT_PERMIT, NULL);
            // }
            break;

        case ESP_GATTS_WRITE_EVT:
           ESP_LOGI(GATTS_TAG, "Write event, conn_id %d, trans_id %ld, handle %d, len %d, is_prep %d",
                    param->write.conn_id, param->write.trans_id, param->write.handle, param->write.len, param->write.is_prep);

           // --- Handle writes for Control Characteristic AND CCCDs ---
           // Check if it's a prepare write request
            if (param->write.is_prep) {
                example_write_event_env(gatts_if, &prepare_write_env, param);
            }
            // It's a non-prepared write (normal write or CCCD write)
            else {
                // Check if writing to the Control Characteristic Value Handle
                if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].control_char_handle) {
                    ESP_LOGI(GATTS_TAG, "Write to Control Characteristic Value");
                    ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
                    parse_control_command(param->write.value, param->write.len);

                    // Send response if requested
                    if (param->write.need_rsp) {
                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    }
                }
                // Check if writing to a CCCD handle (Need to know CCCD handles!)
                // NOTE: We didn't store the CCCD handles explicitly. This requires more robust state.
                // For now, we assume any other write might be to a CCCD.
                // A common pattern is writing 0x0001 (Notify) or 0x0002 (Indicate) to enable.
                // Let's just log it for now.
                else {
                   ESP_LOGI(GATTS_TAG, "Write to other handle (possibly CCCD?)");
                   ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
                   // Example CCCD handling (needs actual CCCD handle check):
                   // uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                   // if (descr_value == 0x0001){ ESP_LOGI(GATTS_TAG, "Notification/Indication ENABLED"); }
                   // else if (descr_value == 0x0000){ ESP_LOGI(GATTS_TAG, "Notification/Indication DISABLED"); }

                   // Send response if requested for CCCD write
                   if (param->write.need_rsp) {
                       esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                   }
                }
            }
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TAG, "Execute write event");
            esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id, param->exec_write.trans_id, ESP_GATT_OK, NULL); // Always send response for exec write
            example_exec_write_event_env(&prepare_write_env, param);
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TAG, "MTU event, MTU: %d", param->mtu.mtu);
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Connection established, conn_id: %d, remote "ESP_BD_ADDR_STR"",
                     param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
            gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;
            // Reset car controls to safe values when connected
            car_state.throttle = 0;
            car_state.steering = 90;
            update_car_control();
            // Optionally: update connection parameters like in gatts_demo example
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Disconnected, conn_id %d, reason 0x%x", param->disconnect.conn_id, param->disconnect.reason);
            gl_profile_tab[PROFILE_APP_ID].conn_id = 0; // Reset conn_id
            // Reset car controls to safe values when disconnected
            car_state.throttle = 0;
            car_state.steering = 90;
            update_car_control();
            // Start advertising again
            esp_ble_gap_start_advertising(&adv_params);
            break;

       // Add other cases from examples if needed (CONF, START, STOP etc.)
        case ESP_GATTS_START_EVT:
             ESP_LOGI(GATTS_TAG, "Service started, status %d, service_handle %d", param->start.status, param->start.service_handle);
             break;
        case ESP_GATTS_STOP_EVT:
             ESP_LOGI(GATTS_TAG, "Service stopped, status %d, service_handle %d", param->stop.status, param->stop.service_handle);
             break;
        case ESP_GATTS_DELETE_EVT:
             ESP_LOGI(GATTS_TAG, "Service deleted, status %d, service_handle %d", param->del.status, param->del.service_handle);
             break;
        case ESP_GATTS_CONF_EVT: // Confirmation for indications
             ESP_LOGI(GATTS_TAG, "Confirmation received, status %d, handle %d", param->conf.status, param->conf.handle);
             break;
        // Other events...
        default:
            ESP_LOGD(GATTS_TAG, "Unhandled GATT Event: %d", event); // Use Debug level for unhandled
            break;
    }
}
 
 // GATTS event handler
 static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
     // If event is register event, store the gatts_if for each profile
     if (event == ESP_GATTS_REG_EVT) {
         if (param->reg.status == ESP_GATT_OK) {
             gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
         } else {
             ESP_LOGI(GATTS_TAG, "Reg app failed, app_id: %04x, status: %d", param->reg.app_id, param->reg.status);
             return;
         }
     }
 
     // Call profile event handler
     for (int idx = 0; idx < PROFILE_NUM; idx++) {
         if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
             if (gl_profile_tab[idx].gatts_cb) {
                 gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
             }
         }
     }
 }
 
 // Prepare write event handling (for large data writes)
 void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
     esp_gatt_status_t status = ESP_GATT_OK;
     
     if (param->write.need_rsp) {
         if (param->write.is_prep) {
             if (prepare_write_env->prepare_buf == NULL) {
                 prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                 prepare_write_env->prepare_len = 0;
                 if (prepare_write_env->prepare_buf == NULL) {
                     ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
                     status = ESP_GATT_NO_RESOURCES;
                 }
             }
             
             esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
             if (gatt_rsp) {
                 gatt_rsp->attr_value.len = param->write.len;
                 gatt_rsp->attr_value.handle = param->write.handle;
                 gatt_rsp->attr_value.offset = param->write.offset;
                 gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                 memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                 esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                 free(gatt_rsp);
             } else {
                 status = ESP_GATT_NO_RESOURCES;
             }
             
             if (status != ESP_GATT_OK) {
                 return;
             }
             
             memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value, param->write.len);
             prepare_write_env->prepare_len += param->write.len;
         } else {
             esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
         }
     }
 }
 
 // Execute write event handling (for large data writes)
 void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
     if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf) {
         // Execute the command
         parse_control_command(prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
     }
     
     // Free the buffer
     if (prepare_write_env->prepare_buf) {
         free(prepare_write_env->prepare_buf);
         prepare_write_env->prepare_buf = NULL;
     }
     prepare_write_env->prepare_len = 0;
 }
 
 void app_main(void) {
     esp_err_t ret;
 
     // Initialize NVS
     ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
 
     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
 
     // Initialize BT controller
     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
     ret = esp_bt_controller_init(&bt_cfg);
     if (ret) {
         ESP_LOGE(GATTS_TAG, "%s initialize controller failed", __func__);
         return;
     }
 
     // Enable BT controller
     ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
     if (ret) {
         ESP_LOGE(GATTS_TAG, "%s enable controller failed", __func__);
         return;
     }
 
     // Initialize Bluedroid
     ret = esp_bluedroid_init();
     if (ret) {
         ESP_LOGE(GATTS_TAG, "%s init bluetooth failed", __func__);
         return;
     }
     ret = esp_bluedroid_enable();
     if (ret) {
         ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed", __func__);
         return;
     }
 
     // Register callbacks
     ret = esp_ble_gatts_register_callback(gatts_event_handler);
     if (ret) {
         ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
         return;
     }
     ret = esp_ble_gap_register_callback(gap_event_handler);
     if (ret) {
         ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
         return;
     }
     ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
     if (ret) {
         ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
         return;
     }
 
     // Set local MTU
     esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(517);
     if (local_mtu_ret) {
         ESP_LOGE(GATTS_TAG, "set local MTU failed, error code = %x", local_mtu_ret);
     }
 
     // Initialize PWM
     init_pwm();
     
     esp_timer_create_args_t servo_detach_timer_args = {
        .callback = &servo_detach_timer_callback,
        .name = "servo_detach_timer"
        // .arg = NULL, // Optional, can be omitted if NULL
        // .dispatch_method = ESP_TIMER_TASK, // Default
        // .skip_unhandled_events = false, // Default
    };

    esp_err_t err_timer_create = esp_timer_create(&servo_detach_timer_args, &servo_detach_timer_handle);
    if (err_timer_create != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to create servo detach timer: %s", esp_err_to_name(err_timer_create));
        // Handle error, perhaps by not continuing if timer is critical
    } else {
        ESP_LOGI(GATTS_TAG, "Servo detach timer created");
    }

     // Set initial car state (stopped, centered)
     car_state.throttle = 0;
     car_state.steering = 90;
     update_car_control();
     
     ESP_LOGI(GATTS_TAG, "BLE Drift Car Controller initialized");
 }