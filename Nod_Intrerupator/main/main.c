/* 
 * Proiect Licenta: Sistem IoT Smart Home bazat pe Bluetooth Mesh
 * Autor: Sfichi Alin-Ionuț
 * Nod: Intrerupator
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"

#define TAG "SWITCH_NODE"
#define CID_ESP 0x02E5
#define BUTTON_IO GPIO_NUM_21
#define RESET_BUTTON_GPIO  4  

static uint8_t dev_uuid[16] = { 0xdd, 0xdd, 0x03 }; 

static struct example_info_store {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  onoff; // Ultima stare cunoscuta a becului
    uint8_t  tid;
} __attribute__((packed)) store = {
    .net_idx = ESP_BLE_MESH_KEY_UNUSED,
    .app_idx = ESP_BLE_MESH_KEY_UNUSED,
    .onoff = 0,
    .tid = 0x0,
};

static nvs_handle_t NVS_HANDLE;
static const char * NVS_KEY = "onoff_client";
static esp_ble_mesh_client_t onoff_client;

// Flag pentru a sti ca asteptam un raspuns STATUS pentru a executa Toggle
static bool is_toggle_pending = false;

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
    .default_ttl = 7,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 2 + 1, ROLE_NODE);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_pub, &onoff_client), // Index 1
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

// --- FUNCTIE TRIMITERE SET (EXECUTIE) ---

void send_mesh_onoff_set_action(uint8_t next_state) {
    esp_ble_mesh_generic_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {0};

    // Folosim aceeasi structura de model si context ca in cererea GET
    common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
    common.model = onoff_client.model; 
    common.ctx.net_idx = 0; // Ca in Gateway
    common.ctx.app_idx = 0; // Ca in Gateway
    common.ctx.addr = onoff_cli_pub.publish_addr; 
    common.ctx.send_ttl = 3;


    set.onoff_set.op_en = false;
    set.onoff_set.onoff = next_state;
    set.onoff_set.tid = store.tid++;

    if (esp_ble_mesh_generic_client_set_state(&common, &set) == ESP_OK) {
        store.onoff = next_state;
        ble_mesh_nvs_store(NVS_HANDLE, NVS_KEY, &store, sizeof(store));
        board_led_operation(LED_G, store.onoff);
        ESP_LOGI(TAG, "Toggle executat! Noua stare trimisa: %d", next_state);
    }
}

// --- CALLBACK CLIENT (RECEPTIE STATUS) ---

static void example_ble_mesh_generic_client_cb(
    esp_ble_mesh_generic_client_cb_event_t event,
    esp_ble_mesh_generic_client_cb_param_t *param) {
    
    if (param->error_code != 0) {
        //ESP_LOGE(TAG, "Eroare: 0x%02x", param->error_code);
        ESP_LOGE(TAG, "Eroare callback: error_code=0x%02x, event=%d, opcode=0x%04" PRIx32,
        param->error_code,
        event,
        param->params ? param->params->opcode : 0);
        is_toggle_pending = false;
        return;
    }

    switch (event) {
        case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
            // Raspuns la GET — statusul e valid direct
            if (is_toggle_pending) {
                is_toggle_pending = false;
                uint8_t cur = param->status_cb.onoff_status.present_onoff;
                ESP_LOGI(TAG, "Status primit: %d, execut toggle", cur);
                send_mesh_onoff_set_action(cur == 0 ? 1 : 0);
            }
            break;

        case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
            // Publicare nesolicitate de la bec
            store.onoff = param->status_cb.onoff_status.present_onoff;
            board_led_operation(LED_G, store.onoff);
            break;

        case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
            ESP_LOGE(TAG, "TIMEOUT: opcode=0x%04" PRIx32 ", addr=0x%04x",
                param->params->opcode,
                param->params->ctx.addr);
            is_toggle_pending = false;
            break;

        default:
            break;
    }
}

// --- LOGICA TRIGGER (TIMING SI PARAMETRI COPIATI DIN GATEWAY) ---

void example_ble_mesh_send_gen_onoff_set(void) {
    
    
    
    esp_ble_mesh_generic_client_get_state_t get = {0};
    
    // Verificam daca modelul este initializat (pentru a evita Guru Meditation anterior)
    if (onoff_client.model == NULL) {
        ESP_LOGE(TAG, "Eroare: Modelul Client nu este initializat de stiva Mesh!");
        return;
    }

    if (onoff_cli_pub.publish_addr == 0x0000) {
        ESP_LOGW(TAG, "Switch neconfigurat. Mapati becul in Dashboard!");
        return;
    }

    ESP_LOGI(TAG, "Interogare stare bec 0x%04x (Logica similara Gateway)...", onoff_cli_pub.publish_addr);

    // Parametri adaptati dupa request_mesh_onoff_status din Gateway
    esp_ble_mesh_client_common_param_t common = {
        .opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET, 
        .model = onoff_client.model, 
        .ctx.net_idx = 0, 
        .ctx.app_idx = 0, 
        .ctx.addr = onoff_cli_pub.publish_addr, 
        .ctx.send_ttl = 3
    };

     ESP_LOGW(TAG, "GET params: model=%p, net_idx=%d, app_idx=%d, addr=0x%04x, pub_addr=0x%04x",
        onoff_client.model,
        common.ctx.net_idx,
        common.ctx.app_idx, 
        common.ctx.addr,
        onoff_cli_pub.publish_addr);

    is_toggle_pending = true; 
    if (esp_ble_mesh_generic_client_get_state(&common, &get) != ESP_OK) {
        is_toggle_pending = false;
        ESP_LOGE(TAG, "Eroare la apelul esp_ble_mesh_generic_client_get_state");
    }
}

// --- TASK BUTON GPIO 9 ---

void button_tap_task(void *arg) {
    gpio_config_t io_conf = { .intr_type = GPIO_INTR_DISABLE, .mode = GPIO_MODE_INPUT, .pin_bit_mask = (1ULL << BUTTON_IO), .pull_up_en = 1 };
    gpio_config(&io_conf);
    uint8_t last_st = 1;
    while (1) {
        uint8_t curr_st = gpio_get_level(BUTTON_IO);
        if (last_st == 1 && curr_st == 0) {
            ESP_LOGI(TAG, "Buton apasat (GPIO 9)");
            example_ble_mesh_send_gen_onoff_set(); 
            vTaskDelay(pdMS_TO_TICKS(500)); 
        }
        last_st = curr_st;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// --- CALLBACKS PROV / CONFIG ---

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param) {
    if (event == ESP_BLE_MESH_PROV_REGISTER_COMP_EVT) {
        ble_mesh_nvs_restore(NVS_HANDLE, NVS_KEY, &store, sizeof(store), NULL);
    }
    else if (event == ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT) {
        store.net_idx = param->node_prov_complete.net_idx;
        board_led_operation(LED_G, LED_OFF);
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param) {
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND) {
            if (param->value.state_change.mod_app_bind.model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI) {
                store.app_idx = param->value.state_change.mod_app_bind.app_idx;
                ble_mesh_nvs_store(NVS_HANDLE, NVS_KEY, &store, sizeof(store));
                ESP_LOGI(TAG, "AppKey Bound. AppIdx: %d", store.app_idx);
            }
        }
        else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET) {
            ESP_LOGI(TAG, "Adresa publicare actualizata: 0x%04x", onoff_cli_pub.publish_addr);
        }
    }
}

void app_main(void) {
    board_init();
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) { ESP_ERROR_CHECK(nvs_flash_erase()); err = nvs_flash_init(); }
    ESP_ERROR_CHECK(err);

    bluetooth_init();
    ble_mesh_nvs_open(&NVS_HANDLE);
    
    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_generic_client_callback(example_ble_mesh_generic_client_cb); 
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);

    // Initializarea Mesh populeaza pointerul onoff_client.model
    err = esp_ble_mesh_init(&provision, &composition);
    ESP_ERROR_CHECK(err);

    esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    
    xTaskCreate(button_tap_task, "btn_task", 3072, NULL, 5, NULL);
    xTaskCreate(reset_button_task, "rst_btn", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Nod Intrerupator cu Sincronizare Activat.");
}