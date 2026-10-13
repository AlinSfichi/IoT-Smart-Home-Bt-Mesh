/* 
 * Proiect Licenta: Sistem IoT Smart Home bazat pe Bluetooth Mesh
 * Autor: Sfichi Alin-Ionuț
 * Nod: Control Iluminat
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_health_model_api.h"


#include "board.h"
#include "ble_mesh_example_init.h"

#define TAG "BEC_SERVER"

#define CID_ESP 0x02E5

// --- Definitii PWM ---
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (2) // GPIO 2 (Pinul pentru MOSFET-ul benzii LED)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Rezolutie 13 biti (0 - 8191)
#define LEDC_FREQUENCY          (5000)      // Frecventa 5 kHz
#define LEDC_MAX_DUTY           ((1 << LEDC_DUTY_RES) - 1) // Valoarea maxima (8191)

#define GATEWAY_UNICAST_ADDR 0x0001 // Adresa standard a provisioner-ului (Gateway)

#define RESET_BUTTON_GPIO  4

static uint8_t dev_uuid[16] = {
    0xdd, 0xdd, 0x01, 0x02,
    0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x09, 0x0a,
    0x0b, 0x0c, 0x0d, 0x0e
};


// Variabila pentru a salva starea PWM curenta
static uint32_t current_pwm_duty = 0;
// Level-ul salvat este pe 16 biti (0 la 32767), necesar pentru a reveni la luminozitatea anterioara dupa un OFF
static int16_t last_known_level = 32767; 

// Level Mesh [-32768, 32767]
const int32_t MAX_MESH_LEVEL = 32767;
const int32_t MIN_MESH_LEVEL = -32768;



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
// *******************************************************
// 1. Functii de control PWM (LEDC)
// *******************************************************

void led_pwm_init() {
    // 1. Configurare Timer LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. Configurare Canal LEDC
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .gpio_num       = LEDC_OUTPUT_IO,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI("PWM", "LEDC PWM initialized on GPIO %d (Max Duty: %u)", LEDC_OUTPUT_IO, LEDC_MAX_DUTY);
}

void set_pwm_duty(uint32_t duty) {
    if (duty > LEDC_MAX_DUTY) {
        duty = LEDC_MAX_DUTY;
    }
    current_pwm_duty = duty;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, current_pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    ESP_LOGI("PWM", "Duty Cycle set to %u", current_pwm_duty);
}

// Functie de conversie Level Mesh (16bit) la Duty PWM (13bit)
uint32_t level_to_pwm_duty(int16_t level) {
    

    if (level > MAX_MESH_LEVEL) level = MAX_MESH_LEVEL;
    if (level < MIN_MESH_LEVEL) level = MIN_MESH_LEVEL;

    // 1. Scalăm întregul interval Level la [0, 65535] (pentru mapare ușoară)
    // Offset: Adăugăm 32768
    uint32_t scaled_level = (uint32_t)level + 32768;
    
    // 2. Mapăm Level-ul scalat [0, 65535] la Duty PWM [0, 8191]
    // P = (L_scaled / 65535) * 8191
    uint32_t duty = (uint32_t)(((double)scaled_level / (MAX_MESH_LEVEL - MIN_MESH_LEVEL + 1)) * LEDC_MAX_DUTY);
    
    // Asigurăm că duty este cel puțin 1 dacă level > MIN_MESH_LEVEL
    if (duty == 0 && level > MIN_MESH_LEVEL) return 1;

    return duty;
}

// *******************************************************
// 2. Structuri Mesh (Configurare si Model)
// *******************************************************

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_ENABLED, //RELAY=ENABLED
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(health_pub, 1, ROLE_NODE);

static uint8_t test_ids[] = { 0x01 };

static esp_ble_mesh_health_srv_t health_server = {
    .health_test = {
        .id_count = 1,
        .test_ids = test_ids,
        .company_id = CID_ESP,   // SIG company ID
    },
};


// Model 1: Generic On/Off Server
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // MODIFICAT
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // MODIFICAT
    },
    .state.onoff = 0x00, // Initial OFF
};

// Model 2: Generic Level Server
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub_0, 2 + 5, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // MODIFICAT
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // MODIFICAT
    },
    .state.level = MIN_MESH_LEVEL,
    .state.target_level = MIN_MESH_LEVEL,
};


// Pastram doar un singur Element (Element 0)
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_HEALTH_SRV(&health_server,&health_pub),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub_0, &level_server_0), 
    // AICI VOM ADAUGA SI GENERIC LEVEL ULTERIOR
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

/* Disable OOB security for SILabs Android app */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_size = 4,
    .input_actions = ESP_BLE_MESH_PUSH,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};

// *******************************************************
// 3. Functii de Callback Mesh
// *******************************************************

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
    // board_led_operation(LED_G, LED_OFF); // Dezactivam LED-ul de stare dupa provizionare
}

// Declararea funcției Level înainte de a fi folosită
static void example_handle_gen_level_msg(esp_ble_mesh_model_t *model,
                                         esp_ble_mesh_msg_ctx_t *ctx,
                                         esp_ble_mesh_server_recv_gen_level_set_t *set);


// Rescriem functia care gestioneaza mesajul ON/OFF pentru a folosi PWM
static void example_handle_gen_onoff_msg(esp_ble_mesh_model_t *model,
                                        esp_ble_mesh_msg_ctx_t *ctx,
                                        esp_ble_mesh_server_recv_gen_onoff_set_t *set)
{
    esp_ble_mesh_gen_onoff_srv_t *srv = (esp_ble_mesh_gen_onoff_srv_t *)model->user_data;

    // Obține Level Server-ul (se află la indexul 2)
    esp_ble_mesh_model_t *level_model = &root_models[3];
    esp_ble_mesh_gen_level_srv_t *level_srv = (esp_ble_mesh_gen_level_srv_t *)level_model->user_data;

    switch (ctx->recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        // Raspuns cu starea curenta On/Off
        esp_ble_mesh_server_model_send_msg(model, ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        break;

    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        // 1. Actiune fizica (PWM)
        int16_t target_level = 0;

        if (set->onoff == 0x01) { // ON
            if (last_known_level == MIN_MESH_LEVEL) last_known_level = 32767; // Pornim la maxim daca aveam luminozitate 0%
            target_level = last_known_level;
            set_pwm_duty(level_to_pwm_duty(target_level)); 
            srv->state.onoff = 0x01;
            ESP_LOGI(TAG, "Lumini ON la Level %d", target_level);
        } else { // OFF
            set_pwm_duty(0);
            srv->state.onoff = 0x00;
            target_level = MIN_MESH_LEVEL;
            ESP_LOGI(TAG, "Lumini OFF");
        }

        // 2. Sincronizarea Level Server-ului (necesară pentru ca slider-ul să știe starea)
        level_srv->state.level = target_level;
        level_srv->state.target_level = target_level;

        // 3. Raspuns/Publicare (STATUS)
        if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            // Trimite STATUS doar daca este un mesaj ACK (cu confirmare)
            esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        }
        // Publica starea curenta (utile pentru sincronizare)
        esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            sizeof(srv->state.onoff), &srv->state.onoff, ROLE_NODE);

    //     //if (ctx->addr != GATEWAY_UNICAST_ADDR) 
    //     {
    //     esp_ble_mesh_msg_ctx_t gateway_ctx = {
    //     .net_idx  = ctx->net_idx,
    //     .app_idx  = ctx->app_idx,
    //     .addr = GATEWAY_UNICAST_ADDR,
    //     .send_ttl = 3,
    // };

    // // Trimitem statusul direct către Gateway (Unicast)
    // esp_ble_mesh_server_model_send_msg(model, &gateway_ctx,
    //     ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
    
    // ESP_LOGI(TAG, "Status raportat la Gateway (0x%04x): %s", 
    //          GATEWAY_UNICAST_ADDR, srv->state.onoff ? "ON" : "OFF");
    //     }

         break;
    default:
        break;
    }
}

// Funcție de tratare a mesajelor Level
static void example_handle_gen_level_msg(esp_ble_mesh_model_t *model,
                                        esp_ble_mesh_msg_ctx_t *ctx,
                                        esp_ble_mesh_server_recv_gen_level_set_t *set)
{
    esp_ble_mesh_gen_level_srv_t *srv = (esp_ble_mesh_gen_level_srv_t *)model->user_data;
    
    // Obține OnOff Server-ul (se află la indexul 1)
    esp_ble_mesh_model_t *onoff_model = &root_models[2];
    esp_ble_mesh_gen_onoff_srv_t *onoff_srv = (esp_ble_mesh_gen_onoff_srv_t *)onoff_model->user_data;
    
    int16_t target_level = set ? set->level : srv->state.level; // CORECTAT
    uint32_t duty = level_to_pwm_duty(target_level);
    
    // 1. Acțiune fizică (PWM)
    if (set != NULL) { // Doar pentru SET
        set_pwm_duty(duty);

        // 2. Actualizează starea internă
        srv->state.level = target_level; // CORECTAT
        srv->state.target_level = target_level;
        
        uint32_t duty = level_to_pwm_duty(target_level);
        // Sincronizează Level-ul cu On/Off Server
        if (duty > 0) {
            onoff_srv->state.onoff = 0x01; // Setează ON
            last_known_level = target_level; // Salvează Level-ul
        } else {
            onoff_srv->state.onoff = 0x00; // Setează OFF
        }

        // Publică starea On/Off sincronizată
        esp_ble_mesh_model_publish(onoff_model,
                                   ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
                                   sizeof(onoff_srv->state.onoff),
                                   &onoff_srv->state.onoff,
                                   ROLE_NODE);
    }

    // 3. Răspuns/Publicare (STATUS)
    if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET || ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET) {
        // Trimite STATUS dacă este un mesaj ACK sau GET
        esp_ble_mesh_server_model_send_msg(model, ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(srv->state.level), (uint8_t *)&srv->state.level); // CORECTAT
    }
    // Publică starea curentă
    if (set != NULL) {
        esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS,
            sizeof(srv->state.level), (uint8_t *)&srv->state.level, ROLE_NODE); // CORECTAT
    }
    
    ESP_LOGI(TAG, "Level Set/Get: %d (Duty: %u)", srv->state.level, duty); // CORECTAT
}


static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    esp_ble_mesh_gen_onoff_srv_t *srv;
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    // Variabilă temporară necesară pentru STATE_CHANGE_EVT
    esp_ble_mesh_server_recv_gen_onoff_set_t tmp_onoff_set;
    esp_ble_mesh_server_recv_gen_level_set_t tmp_level_set;

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        // MODIFICAT: Logica de executie hardware este mutata exclusiv in RECV_SET_MSG_EVT pentru a evita procesarea dubla

        // Verificăm dacă schimbarea de stare vine de la modelul OnOff
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET || 
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            
            // Verificăm dacă sursa (cel care a dat comanda) NU este Gateway-ul
            if (param->ctx.addr != GATEWAY_UNICAST_ADDR) {
                
                esp_ble_mesh_gen_onoff_srv_t *srv = (esp_ble_mesh_gen_onoff_srv_t *)param->model->user_data;
                
                esp_ble_mesh_msg_ctx_t gateway_ctx = {
                    .net_idx  = param->ctx.net_idx,
                    .app_idx  = param->ctx.app_idx,
                    .addr     = GATEWAY_UNICAST_ADDR,
                    .send_ttl = 3,
                };

                // Trimiterea acum va avea succes deoarece pachetul inițial a fost deja procesat
                esp_err_t err = esp_ble_mesh_server_model_send_msg(param->model, &gateway_ctx,
                                    ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, 
                                    sizeof(srv->state.onoff), &srv->state.onoff);
                
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Status sincronizat cu Gateway (de la src 0x%04x)", param->ctx.addr);
                } else {
                    ESP_LOGE(TAG, "Eroare trimitere status Gateway: 0x%02x", err);
                }
            }
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            srv = (esp_ble_mesh_gen_onoff_srv_t *)param->model->user_data;
            ESP_LOGI(TAG, "onoff 0x%02x", srv->state.onoff);
            example_handle_gen_onoff_msg(param->model, &param->ctx, NULL);
        }
        else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET) {
                example_handle_gen_level_msg(param->model, &param->ctx, NULL);
        }
        break;

    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x, tid 0x%02x", param->value.set.onoff.onoff, param->value.set.onoff.tid);
            example_handle_gen_onoff_msg(param->model, &param->ctx, &param->value.set.onoff);
        }
        else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
                   param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK) {
                ESP_LOGI(TAG, "level %d, tid 0x%02x", param->value.set.level.level, param->value.set.level.tid);
                example_handle_gen_level_msg(param->model, &param->ctx, &param->value.set.level);
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}


// *******************************************************
// 4. Initializare (app_main)
// *******************************************************

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);

    // Initializare Mesh
    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }


    bool provisioned = esp_ble_mesh_node_is_provisioned();
if (provisioned) {
    ESP_LOGI(TAG, "Node is already provisioned");
} else {
    ESP_LOGI(TAG, "Node is unprovisioned, enabling provisioning");
    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }
}
    

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    //board_led_operation(LED_G, LED_ON);

    return err;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    board_init();

    led_pwm_init(); 

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    // 5. Setare UUID constant (important pentru ca nodul să fie recunoscut după reboot)
    provision.uuid = dev_uuid;

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
    xTaskCreate(reset_button_task, "rst_btn", 2048, NULL, 5, NULL);

    
}