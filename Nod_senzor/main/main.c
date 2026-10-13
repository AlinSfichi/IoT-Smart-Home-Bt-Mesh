/* 
 * Proiect Licenta: Sistem IoT Smart Home bazat pe Bluetooth Mesh
 * Autor: Sfichi Alin-Ionuț
 * Nod: Senzor
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"

#include "ble_mesh_example_init.h"
#include "board.h"
#include "dht.h" // Driver-ul tau

#define TAG "SENZOR_SERVER"
#define CID_ESP     0x02E5
#define PIN_DHT22   3

#define RESET_BUTTON_GPIO  4

/* Standard Bluetooth Mesh Property IDs */
#define SENSOR_PROPERTY_ID_TEMP     0x0056  /* Ambient Temperature */
#define SENSOR_PROPERTY_ID_HUMID    0x002A  /* Ambient Humidity */

/* Variabile globale pentru stocarea valorilor citite (2 octeti pt precizie) */
static int16_t current_temp_mesh = 0;   
static uint16_t current_humid_mesh = 0; 

/* Definire buffere pentru datele senzorului (2 octeti) */
NET_BUF_SIMPLE_DEFINE_STATIC(sensor_data_temp, 2);
NET_BUF_SIMPLE_DEFINE_STATIC(sensor_data_humid, 2);

static uint8_t dev_uuid[16] = { 0xdd, 0xdd, 0x02, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0f };

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
    .default_ttl = 7,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_sensor_state_t sensor_states[2] = {
    [0] = {
        .sensor_property_id = SENSOR_PROPERTY_ID_TEMP,
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B,
            .length = 1, // 1 inseamna 2 octeti conform spec (N-1)
            .raw_value = &sensor_data_temp,
        },
    },
    [1] = {
        .sensor_property_id = SENSOR_PROPERTY_ID_HUMID,
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B,
            .length = 1, 
            .raw_value = &sensor_data_humid,
        },
    },
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 20, ROLE_NODE);
static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = { .uuid = dev_uuid };

void reset_button_task(void *arg) {
    gpio_config_t cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .pull_up_en = 1
    };
    gpio_config(&cfg);

    while (1) {
        if (gpio_get_level(RESET_BUTTON_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(5000)); // 5 secunde
            if (gpio_get_level(RESET_BUTTON_GPIO) == 0) {
                ESP_LOGW(TAG, "Factory reset!");
                esp_ble_mesh_node_local_reset();
                vTaskDelay(pdMS_TO_TICKS(500)); // lasa timp sa se scrie NVS
                esp_restart();  // <-- restart la placuta ca sa reinceapa procesu de provisioning
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- TASK CITIRE DHT22 ---
void DHT_mesh_task(void *pvParameter) {
    uint16_t temperature, humidity;
    DHTinit(PIN_DHT22);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while(1) {
        int r = DHTget(&temperature, &humidity);
        if(r >= 0) {
            // Conversie pentru Mesh Precizie Inalta (scara 1/100)
            // DHT returneaza 258 pentru 25.8, Mesh vrea 2580 (25.80)
            current_temp_mesh = (int16_t)(temperature * 10);
            current_humid_mesh = (uint16_t)(humidity * 10);

            net_buf_simple_reset(&sensor_data_temp);
            net_buf_simple_add_le16(&sensor_data_temp, current_temp_mesh);
            
            net_buf_simple_reset(&sensor_data_humid);
            net_buf_simple_add_le16(&sensor_data_humid, current_humid_mesh);

            // FIX: Afișare corectă cu zecimale pe monitorul serial local
            ESP_LOGI(TAG, "Senzor Update: T=%d.%dC, H=%d.%d%%", 
                     temperature/10, temperature%10, 
                     humidity/10, humidity%10);
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

// --- CALLBACKS MESH ---
static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param) {
    if (event == ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT) {
        ESP_LOGI(TAG, "Senzor Provizionat! Adresa: 0x%04x", param->node_prov_complete.addr);
    }
}

static void example_ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event, esp_ble_mesh_sensor_server_cb_param_t *param) {
    if (event == ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT) {
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_SENSOR_GET) {
            ESP_LOGI(TAG, "Gateway a cerut datele senzorului...");
            
            uint8_t status_msg[12];
            uint16_t len = 0;

            // Temperatura (Format B, 2 octeti date: (1 << 1) | 1)
            status_msg[len++] = (1 << 1) | 1; 
            status_msg[len++] = (uint8_t)(SENSOR_PROPERTY_ID_TEMP & 0xFF);
            status_msg[len++] = (uint8_t)(SENSOR_PROPERTY_ID_TEMP >> 8);
            status_msg[len++] = (uint8_t)(current_temp_mesh & 0xFF);
            status_msg[len++] = (uint8_t)(current_temp_mesh >> 8);

            // Umiditatea (Format B, 2 octeti date)
            status_msg[len++] = (1 << 1) | 1; 
            status_msg[len++] = (uint8_t)(SENSOR_PROPERTY_ID_HUMID & 0xFF);
            status_msg[len++] = (uint8_t)(SENSOR_PROPERTY_ID_HUMID >> 8);
            status_msg[len++] = (uint8_t)(current_humid_mesh & 0xFF);
            status_msg[len++] = (uint8_t)(current_humid_mesh >> 8);

            esp_ble_mesh_server_model_send_msg(param->model, &param->ctx, 
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, 
                                             len, status_msg);
        }
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param) { }

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    bluetooth_init();
    
    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_sensor_server_callback(example_ble_mesh_sensor_server_cb);

    esp_ble_mesh_init(&provision, &composition);
    esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);

    xTaskCreate(DHT_mesh_task, "DHT_mesh_task", 4096, NULL, 5, NULL);
    xTaskCreate(reset_button_task, "rst_btn", 2048, NULL, 5, NULL);
}