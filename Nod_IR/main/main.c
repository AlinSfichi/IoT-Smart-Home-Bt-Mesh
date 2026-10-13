/* 
 * Proiect Licenta: Sistem IoT Smart Home bazat pe Bluetooth Mesh
 * Autor: Sfichi Alin-Ionuț
 * Nod: IR
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"

#include "ble_mesh_example_init.h"
#include "board.h"

#define TAG "IR_NODE"
#define GATEWAY_ADDR  0x0001  // adresa fixă a gateway-ului

#define RESET_BUTTON_GPIO  4

// --- CONFIG HARDWARE IR ---
#define RMT_TX_GPIO_NUM     5
#define RMT_RX_GPIO_NUM     3
#define BUTTON_LEARN_GPIO   9
#define BUTTON_SEND_GPIO    10
#define RMT_BUFFER_SIZE     64
#define IR_CARRIER_FREQ_HZ  38000
#define IR_RESOLUTION_HZ    1000000

// --- CONFIG SLOTURI NVS ---
#define MAX_SLOTS           5
#define NVS_NAMESPACE       "ir_store"
#define NVS_KEY_LAST_SLOT   "last_slot"

// --- CONFIG MESH VENDOR ---
#define CID_ESP                          0x02E5
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER 0x0001
#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT 0x0000

#define ESP_BLE_MESH_VND_MODEL_OP_SEND   ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

// CODURI STATUS PENTRU FEEDBACK
#define STATUS_LEARN_STARTED   0x01
#define STATUS_LEARN_SUCCESS   0x02
#define STATUS_LEARN_TIMEOUT   0x03
#define STATUS_SEND_SUCCESS    0x04
#define STATUS_SEND_ERR_EMPTY  0x05

static rmt_channel_handle_t tx_channel = NULL;
static rmt_channel_handle_t rx_channel = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;
static TaskHandle_t ir_rx_task_handle = NULL;

static rmt_symbol_word_t learned_signal_buf[RMT_BUFFER_SIZE];
static size_t learned_signal_len = 0;
static uint8_t active_slot = 0;       
static uint8_t last_reproduced_slot = 0; 
static QueueHandle_t gpio_evt_queue = NULL;

// Variabile globale pentru a raspunde asincron in Mesh
static esp_ble_mesh_msg_ctx_t last_mesh_ctx;
static esp_ble_mesh_model_t *vnd_model_ptr = NULL;

static uint8_t dev_uuid[16] = { 0xdd, 0xdd }; 

static rmt_receive_config_t receive_config = {
    .signal_range_min_ns = 1000,
    .signal_range_max_ns = 12000000,
};

static volatile bool ir_rx_triggered_from_mesh = false;

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

// --- LOGICA FEEDBACK MESH ---

void send_mesh_status(uint8_t status, uint8_t slot, bool is_async) {
    if (!vnd_model_ptr) return;

    uint8_t msg[2] = { status, slot };
    esp_err_t err;

    if (!is_async) {
        // Răspuns IMEDIAT în callback — stiva știe contextul, trimite automat la sursă
        err = esp_ble_mesh_server_model_send_msg(
            vnd_model_ptr,
            &last_mesh_ctx,                      // contextul original, neatins
            ESP_BLE_MESH_VND_MODEL_OP_STATUS,
            sizeof(msg), msg
        );
    } else {
        // Mesaj ASINCRON — tranzacția originală a expirat, construim context nou
        esp_ble_mesh_msg_ctx_t async_ctx = {
            .net_idx  = last_mesh_ctx.net_idx,
            .app_idx  = last_mesh_ctx.app_idx,
            .addr     = GATEWAY_ADDR,            // destinație explicită
            .send_ttl = 3,
        };
        err = esp_ble_mesh_server_model_send_msg(
            vnd_model_ptr,
            &async_ctx,
            ESP_BLE_MESH_VND_MODEL_OP_STATUS,
            sizeof(msg), msg
        );
    }

    ESP_LOGI(TAG, "send_mesh_status: status=0x%02x slot=%d async=%d err=0x%x",
             status, slot, is_async, err);
}

// --- LOGICA PERSISTENTA SLOTURI ---

void save_slot_to_nvs(uint8_t slot) {
    if (slot >= MAX_SLOTS) return;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;

    char key_data[16], key_len[16];
    snprintf(key_data, sizeof(key_data), "sig_d_%u", slot);
    snprintf(key_len, sizeof(key_len), "sig_l_%u", slot);

    nvs_set_u32(h, key_len, (uint32_t)learned_signal_len);
    if (learned_signal_len > 0) {
        nvs_set_blob(h, key_data, learned_signal_buf, learned_signal_len * sizeof(rmt_symbol_word_t));
    }
    
    nvs_set_u8(h, NVS_KEY_LAST_SLOT, slot);
    last_reproduced_slot = slot;

    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "✓ Slot %u salvat in NVS", slot);
}

bool load_slot_from_nvs(uint8_t slot) {
    if (slot >= MAX_SLOTS) return false;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    char key_data[16], key_len[16];
    snprintf(key_data, sizeof(key_data), "sig_d_%u", slot);
    snprintf(key_len, sizeof(key_len), "sig_l_%u", slot);

    uint32_t temp_len = 0;
    if (nvs_get_u32(h, key_len, &temp_len) == ESP_OK && temp_len > 0) {
        size_t size = temp_len * sizeof(rmt_symbol_word_t);
        if (nvs_get_blob(h, key_data, learned_signal_buf, &size) == ESP_OK) {
            learned_signal_len = temp_len;
            nvs_close(h);
            return true;
        }
    }
    nvs_close(h);
    return false;
}

void load_system_config() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, NVS_KEY_LAST_SLOT, &last_reproduced_slot);
        nvs_close(h);
        ESP_LOGI(TAG, "Sistem: Ultimul slot utilizat a fost %u", last_reproduced_slot);
    }
}

// --- LOGICA IR ---

static bool IRAM_ATTR on_receive_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx) {
    size_t copy_len = edata->num_symbols;
    if (copy_len > RMT_BUFFER_SIZE) copy_len = RMT_BUFFER_SIZE;
    memcpy(learned_signal_buf, edata->received_symbols, copy_len * sizeof(rmt_symbol_word_t));
    learned_signal_len = copy_len;
    
    // Notificam Task-ul ca am primit date
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(ir_rx_task_handle, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void transmit_ir_from_slot(uint8_t slot, bool from_mesh) {
    if (load_slot_from_nvs(slot)) {
        rmt_transmit_config_t tx_config = { .loop_count = 0, .flags.eot_level = 0 };
        size_t data_size = learned_signal_len * sizeof(rmt_symbol_word_t);
        if (rmt_transmit(tx_channel, copy_encoder, learned_signal_buf, data_size, &tx_config) == ESP_OK) {
            rmt_tx_wait_all_done(tx_channel, 1000);
            ESP_LOGI(TAG, "✓ Slot %u trimis!", slot);
            
            nvs_handle_t h;
            if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_u8(h, NVS_KEY_LAST_SLOT, slot);
                nvs_commit(h);
                nvs_close(h);
            }
            last_reproduced_slot = slot;
            if (from_mesh) send_mesh_status(STATUS_SEND_SUCCESS, slot, true);
        }
    } else {
        ESP_LOGW(TAG, "Slot %u este gol!", slot);
        if (from_mesh) send_mesh_status(STATUS_SEND_ERR_EMPTY, slot, true);
    }
}

// --- LOGICA MESH ---

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
    .default_ttl = 7,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 0),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER, vnd_op, NULL, NULL),
};

static esp_ble_mesh_model_t root_models[] = { ESP_BLE_MESH_MODEL_CFG_SRV(&config_server) };
static esp_ble_mesh_elem_t elements[] = { ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models) };
static esp_ble_mesh_comp_t composition = { .cid = CID_ESP, .element_count = 1, .elements = elements };
static esp_ble_mesh_prov_t provision = { .uuid = dev_uuid };

static void vendor_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param) {
    if (event == ESP_BLE_MESH_MODEL_OPERATION_EVT && param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
        uint8_t action = param->model_operation.msg[0]; 
        uint8_t slot   = param->model_operation.msg[1]; 
        
        if (slot >= MAX_SLOTS) return;
        
        // Salvam contextul pentru feedback-ul asincron
        vnd_model_ptr = param->model_operation.model;
        memcpy(&last_mesh_ctx, param->model_operation.ctx, sizeof(esp_ble_mesh_msg_ctx_t));

        if (action == 1) {
            active_slot = slot;
            ir_rx_triggered_from_mesh = true;
            send_mesh_status(STATUS_LEARN_STARTED, slot, false); // ACK Imediat (is_async=false)
            xTaskNotifyGive(ir_rx_task_handle);
        } else if (action == 2) {
            transmit_ir_from_slot(slot, true);
        }
    }
}

void ir_rx_task(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // așteaptă trigger
        
        learned_signal_len = 0;
        
        // Pornește recepția RMT
        rmt_receive(rx_channel, learned_signal_buf,
                    sizeof(learned_signal_buf), &receive_config);
        
        // Acum on_receive_done va face vTaskNotifyGiveFromISR
        // Așteptăm 5 secunde
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) > 0) {
            ESP_LOGI(TAG, "Slot %u invatat!", active_slot);
            save_slot_to_nvs(active_slot);
            
            // Adaugă delay mic să lase stiva să respire
            vTaskDelay(pdMS_TO_TICKS(100));
            send_mesh_status(STATUS_LEARN_SUCCESS, active_slot, true);
        } else {
            rmt_disable(rx_channel);   // oprește RMT ca să nu rămână activ
            rmt_enable(rx_channel);    // resetează pentru utilizare viitoare
            
            ESP_LOGE(TAG, "Timeout învatare slot %u", active_slot);
            vTaskDelay(pdMS_TO_TICKS(100));
            send_mesh_status(STATUS_LEARN_TIMEOUT, active_slot, true);
        }
        
        ir_rx_triggered_from_mesh = false;
    }
}

void button_task(void *arg) {
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(io_num) == 0) {
                if (io_num == BUTTON_LEARN_GPIO) {
                    active_slot = last_reproduced_slot; 
                    xTaskNotifyGive(ir_rx_task_handle);
                } else if (io_num == BUTTON_SEND_GPIO) {
                    transmit_ir_from_slot(last_reproduced_slot, false);
                }
            }
        }
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    load_system_config(); 

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_config_t btn_cfg = {
        .intr_type = GPIO_INTR_NEGEDGE, .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_LEARN_GPIO) | (1ULL << BUTTON_SEND_GPIO), .pull_up_en = 1
    };
    gpio_config(&btn_cfg);
    gpio_isr_handler_add(BUTTON_LEARN_GPIO, gpio_isr_handler, (void*)BUTTON_LEARN_GPIO);
    gpio_isr_handler_add(BUTTON_SEND_GPIO, gpio_isr_handler, (void*)BUTTON_SEND_GPIO);

    // TX init
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT, .gpio_num = RMT_TX_GPIO_NUM,
        .mem_block_symbols = 64, .resolution_hz = IR_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_channel));
    rmt_apply_carrier(tx_channel, &(rmt_carrier_config_t){ .frequency_hz = IR_CARRIER_FREQ_HZ, .duty_cycle = 0.33 });
    rmt_enable(tx_channel);
    rmt_new_copy_encoder(&(rmt_copy_encoder_config_t){}, &copy_encoder);

    // RX init
    rmt_rx_channel_config_t rx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT, .gpio_num = RMT_RX_GPIO_NUM,
        .mem_block_symbols = RMT_BUFFER_SIZE, .resolution_hz = IR_RESOLUTION_HZ, .flags.invert_in = true,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_channel));
    rmt_rx_register_event_callbacks(rx_channel, &(rmt_rx_event_callbacks_t){ .on_recv_done = on_receive_done }, NULL);
    rmt_enable(rx_channel);

    xTaskCreate(ir_rx_task, "ir_rx", 4096, NULL, 10, &ir_rx_task_handle);
    xTaskCreate(button_task, "btn_task", 2048, NULL, 10, NULL);
    xTaskCreate(reset_button_task, "rst_btn", 2048, NULL, 5, NULL);

    bluetooth_init();
    esp_ble_mesh_register_prov_callback(NULL);
    esp_ble_mesh_register_custom_model_callback(vendor_model_cb);
    esp_ble_mesh_init(&provision, &composition);
    esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);

    ESP_LOGI(TAG, "Nod IR Ready!");
}