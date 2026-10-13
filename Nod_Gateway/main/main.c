/* 
 * Proiect Licenta: Sistem IoT Smart Home
 * Autor: Sfichi Alin-Ionuț
 * Nod: Gateway
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdarg.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "cJSON.h"

#include "esp_timer.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"

#include "ble_mesh_example_init.h"

#define TAG "GATEWAY_MASTER"

// --- CONFIGURARE WI-FI SI MQTT ---
#define WIFI_SSID      "test" // SSID retea
#define WIFI_PASS      "Test1234" // parola WiFi
// #define MQTT_URL       "mqtt://192.168.1.137" // adresa broker MQTT
// #define MQTT_URL       "mqtt://10.208.38.87" // adresa broker MQTT
#define MQTT_URL       "mqtt://alin.local" // adresa broker MQTT
#define TOPIC_OUT      "mesh/out" // topic iesire
#define TOPIC_IN       "mesh/in" // topic intrare

#define RESET_BUTTON_GPIO  9 // pin buton reset

// --- STARE SISTEM ---
static bool prov_mode_auto = false; // provisioning automat
static esp_mqtt_client_handle_t mqtt_client = NULL; // client MQTT
static bool mqtt_connected = false; // status conexiune

// --- CONFIG MESH VENDOR ---
#define CID_ESP                          0x02E5
#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT 0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER 0x0001 
#define ESP_BLE_MESH_VND_MODEL_OP_SEND   ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

#define PROV_OWN_ADDR       0x0001 // adresa gateway
#define MSG_SEND_TTL        3 // TTL mesh
#define APP_KEY_OCTET       0x12 // cheie aplicatie

#define SENSOR_PROPERTY_ID_TEMP     0x0056
#define SENSOR_PROPERTY_ID_HUMID    0x002A

#define DB_SIZE 10 // max noduri stocate
void reset_node(uint16_t addr);

// Structura nod Mesh
typedef struct {
    uint16_t addr; // adresa nod
    bool has_onoff; // suporta ON/OFF server
    bool has_onoff_client; // suporta ON/OFF client
    bool has_level; // suporta PWM
    bool has_sensor; // nod senzor
    bool has_ir; // nod IR
    bool discovered; // detectat
    bool in_config; // in configurare
    uint8_t config_step; // pas configurare
    int64_t config_start_time; // timp start configurare
} node_cache_t;

static node_cache_t node_db[DB_SIZE]; // baza de date noduri
static uint8_t dev_uuid[16];

typedef struct {
    uint8_t uuid[16];
    esp_ble_mesh_addr_type_t addr_type;
    uint8_t addr[6];
    esp_ble_mesh_prov_bearer_t bearer;
    int64_t last_report_time;
    bool occupied;
} scanned_device_t;

static scanned_device_t scan_results[DB_SIZE];

static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  app_key[16];
} prov_key;

static esp_ble_mesh_client_t config_client;
static esp_ble_mesh_client_t onoff_client;
static esp_ble_mesh_client_t level_client;
static esp_ble_mesh_client_t sensor_client;

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
    .default_ttl = 7,
};

static esp_ble_mesh_model_op_t vendor_client_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(NULL, &onoff_client),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_CLI(NULL, &level_client),
    ESP_BLE_MESH_MODEL_SENSOR_CLI(NULL, &sensor_client),
};

static const esp_ble_mesh_client_op_pair_t vendor_op_pair[] = {
    { ESP_BLE_MESH_VND_MODEL_OP_SEND, ESP_BLE_MESH_VND_MODEL_OP_STATUS },
};

static esp_ble_mesh_client_t vendor_client_data = {
    .op_pair_size = ARRAY_SIZE(vendor_op_pair),
    .op_pair = vendor_op_pair,
};

static esp_ble_mesh_model_t vendor_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_CLIENT,
                               vendor_client_op, NULL, &vendor_client_data),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vendor_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid = dev_uuid,
    .prov_unicast_addr = PROV_OWN_ADDR,
    .prov_start_address = 0x0005,
};

// =============================================================================
// FUNCTII HELPER JSON
// =============================================================================

// Trimite date (key-value)
static void publish_json(cJSON *root) {
    if (!root) return;

    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!str) return;

    printf("[MQTT OUT] %s\n", str);

    if (mqtt_connected && mqtt_client) {
        esp_mqtt_client_publish(mqtt_client, TOPIC_OUT, str, strlen(str), 1, 0);
    }

    free(str);
}

// Helper: addr uint16 -> string "0x0005"
static void addr_to_str(uint16_t addr, char *buf, size_t len) {
    snprintf(buf, len, "0x%04x", addr);
}

// --- Emitere mesaje JSON standardizate ---

static void json_send_data(uint16_t addr, const char *key, const char *val) {
    char addr_str[8];
    addr_to_str(addr, addr_str, sizeof(addr_str));
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "data");
    cJSON_AddStringToObject(root, "addr", addr_str);
    cJSON_AddStringToObject(root, "key",  key);
    cJSON_AddStringToObject(root, "val",  val);
    publish_json(root);
}

static void json_send_data_f(uint16_t addr, const char *key, float val) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", val);
    json_send_data(addr, key, buf);
}

static void json_send_data_i(uint16_t addr, const char *key, int val) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", val);
    json_send_data(addr, key, buf);
}

// Trimite eveniment
static void json_send_event(const char *name, uint16_t addr) {
    char addr_str[8];
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "event");
    cJSON_AddStringToObject(root, "name", name);
    if (addr != 0) {
        addr_to_str(addr, addr_str, sizeof(addr_str));
        cJSON_AddStringToObject(root, "addr", addr_str);
    }
    publish_json(root);
}

static void json_send_event_noaddr(const char *name) {
    json_send_event(name, 0);
}

static void json_send_ack(const char *cmd, cJSON *extra_fields) {
    // extra_fields: obiect cJSON cu campuri suplimentare, poate fi NULL
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "ack");
    cJSON_AddStringToObject(root, "cmd",  cmd);
    if (extra_fields) {
        // Mutam campurile din extra_fields in root
        cJSON *child = extra_fields->child;
        while (child) {
            cJSON *next = child->next;
            cJSON_DetachItemViaPointer(extra_fields, child);
            cJSON_AddItemToObject(root, child->string, child);
            child = next;
        }
        cJSON_Delete(extra_fields);
    }
    publish_json(root);
}

// Trimite eroare
static void json_send_error(const char *msg, const char *addr_str) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "error");
    cJSON_AddStringToObject(root, "msg",  msg);
    if (addr_str) cJSON_AddStringToObject(root, "addr", addr_str);
    publish_json(root);
}

static void json_send_db_node(node_cache_t *node) {
    char addr_str[8];
    addr_to_str(node->addr, addr_str, sizeof(addr_str));
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type",            "db_node");
    cJSON_AddStringToObject(root, "addr",            addr_str);
    cJSON_AddBoolToObject  (root, "has_onoff",       node->has_onoff);
    cJSON_AddBoolToObject  (root, "has_onoff_client",node->has_onoff_client);
    cJSON_AddBoolToObject  (root, "has_level",       node->has_level);
    cJSON_AddBoolToObject  (root, "has_sensor",      node->has_sensor);
    cJSON_AddBoolToObject  (root, "has_ir",          node->has_ir);
    publish_json(root);
}

static void json_send_unprov(const char *uuid_str, int8_t rssi) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "unprov");
    cJSON_AddStringToObject(root, "uuid", uuid_str);
    cJSON_AddNumberToObject(root, "rssi", rssi);
    publish_json(root);
}

// =============================================================================
// FUNCTII HELPER GENERALE
// =============================================================================

void hex_to_bytes(const char *src, uint8_t *dst, int len) {
    for (int i = 0; i < len; i++) sscanf(src + i * 2, "%02hhx", &dst[i]);
}

node_cache_t* get_node(uint16_t addr) {
    for (int i = 0; i < DB_SIZE; i++) {
        if (node_db[i].addr == addr) return &node_db[i];
    }
    for (int i = 0; i < DB_SIZE; i++) {
        if (node_db[i].addr == 0) { node_db[i].addr = addr; return &node_db[i]; }
    }
    return NULL;
}

node_cache_t* find_node_only(uint16_t addr) {
    for (int i = 0; i < DB_SIZE; i++) {
        if (node_db[i].addr == addr) return &node_db[i];
    }
    return NULL;
}

void save_node_db_to_nvs(void) {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_blob(handle, "mesh_node_db", node_db, sizeof(node_db));
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Node DB salvat in NVS");
    }
}

void load_node_db_from_nvs(void) {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK) {
        size_t size = sizeof(node_db);
        if (nvs_get_blob(handle, "mesh_node_db", node_db, &size) == ESP_OK) {
            for (int i = 0; i < DB_SIZE; i++) node_db[i].in_config = false;
            ESP_LOGI(TAG, "Node DB incarcat din NVS");
        }
        nvs_close(handle);
    }
}

void delete_node_db(void) {
    memset(node_db, 0, sizeof(node_db));
    save_node_db_to_nvs();
    json_send_event_noaddr("DB_CLEARED");
    ESP_LOGW(TAG, "Toata baza de date a fost stearsa");
}

void delete_node_by_addr(uint16_t addr) {
    char addr_str[8];
    addr_to_str(addr, addr_str, sizeof(addr_str));
    for (int i = 0; i < DB_SIZE; i++) {
        if (node_db[i].addr == addr) {
            memset(&node_db[i], 0, sizeof(node_cache_t));
            save_node_db_to_nvs();
            json_send_event("NODE_DELETED", addr);
            ESP_LOGW(TAG, "Nod 0x%04x sters din DB", addr);
            return;
        }
    }
    json_send_error("NODE_NOT_FOUND", addr_str);
}

void list_node_db(void) {
    json_send_event_noaddr("DB_LIST_START");
    for (int i = 0; i < DB_SIZE; i++) {
        if (node_db[i].addr != 0 && node_db[i].discovered) {
            json_send_db_node(&node_db[i]);
        }
    }
    json_send_event_noaddr("DB_LIST_END");
}

void set_switch_publication(uint16_t switch_addr, uint16_t target_addr) {
    esp_ble_mesh_cfg_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {
        .model = config_client.model, .ctx.net_idx = 0, .ctx.addr = switch_addr,
        .ctx.send_ttl = MSG_SEND_TTL, .opcode = ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET
    };
    set.model_pub_set.element_addr    = switch_addr;
    set.model_pub_set.publish_addr    = target_addr;
    set.model_pub_set.publish_app_idx = 0;
    set.model_pub_set.publish_ttl     = MSG_SEND_TTL;
    set.model_pub_set.model_id        = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI;
    set.model_pub_set.company_id      = ESP_BLE_MESH_CID_NVAL;
    esp_ble_mesh_config_client_set_state(&common, &set);

    char from_str[8], to_str[8];
    addr_to_str(switch_addr, from_str, sizeof(from_str));
    addr_to_str(target_addr,  to_str,  sizeof(to_str));
    cJSON *extra = cJSON_CreateObject();
    cJSON_AddStringToObject(extra, "from", from_str);
    cJSON_AddStringToObject(extra, "to",   to_str);
    json_send_ack("PUB_SET", extra);
}

void send_ir_command(uint16_t target_addr, uint8_t action, uint8_t slot) {
    char addr_str[8];
    addr_to_str(target_addr, addr_str, sizeof(addr_str));
    node_cache_t *node = find_node_only(target_addr);
    if (!node)          { json_send_error("NODE_NOT_PROVISIONED", addr_str); return; }
    if (!node->has_ir)  { json_send_error("NODE_NO_IR_MODEL",     addr_str); return; }

    uint8_t msg[2] = { action, slot };
    esp_ble_mesh_msg_ctx_t ctx = { .net_idx = 0, .app_idx = 0, .addr = target_addr, .send_ttl = MSG_SEND_TTL };
    esp_ble_mesh_client_model_send_msg(
        &vendor_models[0], &ctx,
        ESP_BLE_MESH_VND_MODEL_OP_SEND,
        sizeof(msg), msg, 0, false, ROLE_PROVISIONER
    );

    ESP_LOGI(TAG, "Trimit IR catre 0x%04x, Action=%d, Slot=%d", target_addr, action, slot);
    cJSON *extra = cJSON_CreateObject();
    cJSON_AddStringToObject(extra, "addr",   addr_str);
    cJSON_AddNumberToObject(extra, "action", action);
    cJSON_AddNumberToObject(extra, "slot",   slot);
    json_send_ack("IR_CMD_SENT", extra);
}

void request_mesh_onoff_status(uint16_t target_addr) {
    node_cache_t *node = find_node_only(target_addr);
    if (!node || !node->has_onoff) return;
    esp_ble_mesh_generic_client_get_state_t get = {0};
    esp_ble_mesh_client_common_param_t common = {
        .opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET, .model = onoff_client.model,
        .ctx.net_idx = 0, .ctx.app_idx = 0, .ctx.addr = target_addr, .ctx.send_ttl = MSG_SEND_TTL
    };
    esp_ble_mesh_generic_client_get_state(&common, &get);
}

void request_mesh_level_status(uint16_t target_addr) {
    node_cache_t *node = find_node_only(target_addr);
    if (!node || !node->has_level) return;
    esp_ble_mesh_generic_client_get_state_t get = {0};
    esp_ble_mesh_client_common_param_t common = {
        .opcode = ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET, .model = level_client.model,
        .ctx.net_idx = 0, .ctx.app_idx = 0, .ctx.addr = target_addr, .ctx.send_ttl = MSG_SEND_TTL
    };
    esp_ble_mesh_generic_client_get_state(&common, &get);
}

void request_mesh_sensor_data(uint16_t target_addr) {
    node_cache_t *node = find_node_only(target_addr);
    if (!node || !node->has_sensor) return;
    esp_ble_mesh_sensor_client_get_state_t get = { .sensor_get.op_en = false };
    esp_ble_mesh_client_common_param_t common = {
        .opcode = ESP_BLE_MESH_MODEL_OP_SENSOR_GET, .model = sensor_client.model,
        .ctx.net_idx = 0, .ctx.app_idx = 0, .ctx.addr = target_addr, .ctx.send_ttl = MSG_SEND_TTL,
    };
    esp_ble_mesh_sensor_client_get_state(&common, &get);
}

void send_mesh_onoff(uint16_t target_addr, uint8_t state) {
    node_cache_t *node = find_node_only(target_addr);
    if (!node || !node->has_onoff) return;
    esp_ble_mesh_generic_client_set_state_t set = { .onoff_set.onoff = state, .onoff_set.tid = 0 };
    esp_ble_mesh_client_common_param_t common = {
        .opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, .model = onoff_client.model,
        .ctx.net_idx = 0, .ctx.app_idx = 0, .ctx.addr = target_addr, .ctx.send_ttl = MSG_SEND_TTL
    };
    esp_ble_mesh_generic_client_set_state(&common, &set);
}

void send_mesh_level(uint16_t target_addr, int16_t level) {
    node_cache_t *node = find_node_only(target_addr);
    if (!node || !node->has_level) return;
    esp_ble_mesh_generic_client_set_state_t set = { .level_set.level = level, .level_set.tid = 0 };
    esp_ble_mesh_client_common_param_t common = {
        .opcode = ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK, .model = level_client.model,
        .ctx.net_idx = 0, .ctx.app_idx = 0, .ctx.addr = target_addr, .ctx.send_ttl = MSG_SEND_TTL
    };
    esp_ble_mesh_generic_client_set_state(&common, &set);
}

// =============================================================================
// CASCADA DE CONFIGURARE
// =============================================================================

void trigger_next_config_step(node_cache_t *node) {
    if (!node || !node->in_config) return;
    esp_ble_mesh_client_common_param_t common = {
        .model = config_client.model, .ctx.net_idx = 0,
        .ctx.addr = node->addr, .ctx.send_ttl = MSG_SEND_TTL
    };
    esp_ble_mesh_cfg_client_set_state_t bind = {0};
    bind.model_app_bind.element_addr   = node->addr;
    bind.model_app_bind.model_app_idx  = 0;

    if (node->config_step == 0) {
        common.opcode = ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET;
        esp_ble_mesh_cfg_client_get_state_t get = { .comp_data_get.page = 0 };
        esp_ble_mesh_config_client_get_state(&common, &get);
    }
    else if (node->config_step == 1) {
        if (node->has_onoff) {
            common.opcode = ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND;
            bind.model_app_bind.model_id   = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
            bind.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
            esp_ble_mesh_config_client_set_state(&common, &bind);
        } else { node->config_step = 2; trigger_next_config_step(node); }
    }
    else if (node->config_step == 2) {
        if (node->has_level) {
            common.opcode = ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND;
            bind.model_app_bind.model_id   = ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_SRV;
            bind.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
            esp_ble_mesh_config_client_set_state(&common, &bind);
        } else { node->config_step = 3; trigger_next_config_step(node); }
    }
    else if (node->config_step == 3) {
        if (node->has_sensor) {
            common.opcode = ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND;
            bind.model_app_bind.model_id   = ESP_BLE_MESH_MODEL_ID_SENSOR_SRV;
            bind.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
            esp_ble_mesh_config_client_set_state(&common, &bind);
        } else { node->config_step = 4; trigger_next_config_step(node); }
    }
    else if (node->config_step == 4) {
        if (node->has_ir) {
            common.opcode = ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND;
            bind.model_app_bind.model_id   = ESP_BLE_MESH_VND_MODEL_ID_SERVER;
            bind.model_app_bind.company_id = CID_ESP;
            esp_ble_mesh_config_client_set_state(&common, &bind);
        } else { node->config_step = 5; trigger_next_config_step(node); }
    }
    else if (node->config_step == 5) {
        if (node->has_onoff_client) {
            common.opcode = ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND;
            bind.model_app_bind.model_id   = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI;
            bind.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
            esp_ble_mesh_config_client_set_state(&common, &bind);
        } else { node->config_step = 6; trigger_next_config_step(node); }
    }
    else if (node->config_step == 6) {
        if (node->has_onoff) {
            common.opcode = ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET;
            bind.model_pub_set.element_addr      = node->addr;
            bind.model_pub_set.model_id          = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
            bind.model_pub_set.company_id        = ESP_BLE_MESH_CID_NVAL;
            bind.model_pub_set.publish_addr      = PROV_OWN_ADDR;
            bind.model_pub_set.publish_app_idx   = 0;
            bind.model_pub_set.publish_ttl       = MSG_SEND_TTL;
            bind.model_pub_set.publish_period    = 0;
            bind.model_pub_set.publish_retransmit= ESP_BLE_MESH_TRANSMIT(2, 20);
            esp_ble_mesh_config_client_set_state(&common, &bind);
        } else { node->config_step = 7; trigger_next_config_step(node); }
    }
    else {
        json_send_event("CONFIG_DONE", node->addr);
        node->in_config = false;
        save_node_db_to_nvs();
    }
}

static esp_timer_handle_t config_step_timer = NULL;
static node_cache_t *pending_config_node    = NULL;

static void config_step_timer_cb(void *arg) {
    trigger_next_config_step(pending_config_node);
}

static void schedule_config_step(node_cache_t *node, uint32_t delay_ms) {
    pending_config_node = node;
    if (config_step_timer == NULL) {
        esp_timer_create_args_t args = { .callback = config_step_timer_cb, .name = "cfg_step_timer" };
        esp_timer_create(&args, &config_step_timer);
    } else {
        esp_timer_stop(config_step_timer);
    }
    esp_timer_start_once(config_step_timer, delay_ms * 1000ULL);
}

// =============================================================================
// CALLBACKS MESH
// =============================================================================

static void example_ble_mesh_config_client_cb(
    esp_ble_mesh_cfg_client_cb_event_t event,
    esp_ble_mesh_cfg_client_cb_param_t *param)
{
    if (!param || !param->params) return;
    uint16_t addr = param->params->ctx.addr;
    node_cache_t *node = get_node(addr);
    if (!node) return;

    if (param->error_code != 0) {
        ESP_LOGE(TAG, "Config Error %d la 0x%04x", param->error_code, addr);
        node->in_config = false;
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET) {
            uint8_t  *data = param->status_cb.comp_data_status.composition_data->data;
            uint16_t  len  = param->status_cb.comp_data_status.composition_data->len;
            node->has_onoff = node->has_onoff_client = node->has_level =
                              node->has_sensor = node->has_ir = false;
            int offset = 10;
            while (offset < len) {
                offset += 2;
                uint8_t num_s = data[offset++];
                uint8_t num_v = data[offset++];
                for (int i = 0; i < num_s; i++) {
                    uint16_t mid = data[offset] | (data[offset+1] << 8);
                    if (mid == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV) node->has_onoff        = true;
                    if (mid == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI) node->has_onoff_client = true;
                    if (mid == ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_SRV) node->has_level        = true;
                    if (mid == ESP_BLE_MESH_MODEL_ID_SENSOR_SRV)    node->has_sensor       = true;
                    offset += 2;
                }
                for (int i = 0; i < num_v; i++) {
                    uint16_t cid = data[offset]   | (data[offset+1] << 8);
                    uint16_t mid = data[offset+2] | (data[offset+3] << 8);
                    if (cid == CID_ESP && mid == ESP_BLE_MESH_VND_MODEL_ID_SERVER) node->has_ir = true;
                    offset += 4;
                }
            }
            node->discovered = true;

            // Log local
            ESP_LOGI(TAG, "Discovery 0x%04x: OnOff=%d Switch=%d Level=%d Sensor=%d IR=%d",
                     addr, node->has_onoff, node->has_onoff_client,
                     node->has_level, node->has_sensor, node->has_ir);

            // Trimitem JSON de info discovery
            {
                char addr_str[8];
                addr_to_str(addr, addr_str, sizeof(addr_str));
                cJSON *root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "type", "discovery");
                cJSON_AddStringToObject(root, "addr", addr_str);
                cJSON_AddBoolToObject(root, "has_onoff",        node->has_onoff);
                cJSON_AddBoolToObject(root, "has_onoff_client", node->has_onoff_client);
                cJSON_AddBoolToObject(root, "has_level",        node->has_level);
                cJSON_AddBoolToObject(root, "has_sensor",       node->has_sensor);
                cJSON_AddBoolToObject(root, "has_ir",           node->has_ir);
                publish_json(root);
            }

            // Adaugam AppKey
            esp_ble_mesh_client_common_param_t common = {
                .model = config_client.model, .ctx.net_idx = 0,
                .ctx.addr = addr, .ctx.send_ttl = MSG_SEND_TTL,
                .opcode = ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD
            };
            esp_ble_mesh_cfg_client_set_state_t set_state = {
                .app_key_add.net_idx = 0, .app_key_add.app_idx = 0
            };
            memcpy(set_state.app_key_add.app_key, prov_key.app_key, 16);
            esp_ble_mesh_config_client_set_state(&common, &set_state);
        }
        break;

    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD) {
            node->config_step = 1;
            schedule_config_step(node, 200);
        } else if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND ||
                   param->params->opcode == ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET) {
            node->config_step++;
            schedule_config_step(node, 200);
        }
        break;

    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        json_send_event("NODE_CONFIG_TIMEOUT", addr);
        reset_node(addr);
        break;

    default:
        break;
    }
}

static void example_ble_mesh_sensor_client_cb(
    esp_ble_mesh_sensor_client_cb_event_t event,
    esp_ble_mesh_sensor_client_cb_param_t *param)
{
    if (param->error_code != 0 || !param->status_cb.sensor_status.marshalled_sensor_data) return;
    uint8_t  *data = param->status_cb.sensor_status.marshalled_sensor_data->data;
    uint16_t  len  = param->status_cb.sensor_status.marshalled_sensor_data->len;
    uint16_t  addr = param->params->ctx.addr;
    int offset = 0;
    while (offset < len) {
        uint8_t  header  = data[offset++];
        uint16_t prop_id = 0;
        uint8_t  val_len = 0;
        if ((header & BIT(0)) == 0) {
            val_len  = ((header >> 1) & 0x0F) + 1;
            prop_id  = (header >> 5);
        } else {
            val_len  = ((header >> 1) & 0x7F) + 1;
            prop_id  = data[offset] | (data[offset+1] << 8);
            offset  += 2;
        }
        if (prop_id == SENSOR_PROPERTY_ID_TEMP) {
            int16_t raw = data[offset] | (data[offset+1] << 8);
            json_send_data_f(addr, "TEMP", (float)raw / 100.0f);
        } else if (prop_id == SENSOR_PROPERTY_ID_HUMID) {
            uint16_t raw = data[offset] | (data[offset+1] << 8);
            json_send_data_f(addr, "HUMID", (float)raw / 100.0f);
        }
        offset += val_len;
    }
}

static void example_ble_mesh_custom_model_cb(
    esp_ble_mesh_model_cb_event_t event,
    esp_ble_mesh_model_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_MODEL_OPERATION_EVT &&
        event != ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT) return;

    uint32_t opcode; uint8_t *msg; uint16_t len; uint16_t addr;
    if (event == ESP_BLE_MESH_MODEL_OPERATION_EVT) {
        opcode = param->model_operation.opcode;
        msg    = param->model_operation.msg;
        len    = param->model_operation.length;
        addr   = param->model_operation.ctx->addr;
    } else {
        opcode = param->client_recv_publish_msg.opcode;
        msg    = param->client_recv_publish_msg.msg;
        len    = param->client_recv_publish_msg.length;
        addr   = param->client_recv_publish_msg.ctx->addr;
    }

    if (opcode != ESP_BLE_MESH_VND_MODEL_OP_STATUS || len < 2) return;

    uint8_t status = msg[0];
    uint8_t slot   = msg[1];

    const char *status_str = NULL;
    switch (status) {
        case 0x01: status_str = "LEARN_STARTED";      break;
        case 0x02: status_str = "LEARN_SUCCESS";       break;
        case 0x03: status_str = "LEARN_ERROR_TIMEOUT"; break;
        case 0x04: status_str = "SEND_SUCCESS";        break;
        case 0x05: status_str = "SEND_ERROR_EMPTY";    break;
        default:   status_str = "UNKNOWN";             break;
    }

    char addr_str[8];
    addr_to_str(addr, addr_str, sizeof(addr_str));
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type",   "data");
    cJSON_AddStringToObject(root, "addr",   addr_str);
    cJSON_AddStringToObject(root, "key",    "IR_STATUS");
    cJSON_AddStringToObject(root, "val",    status_str);
    cJSON_AddNumberToObject(root, "slot",   slot);
    publish_json(root);
}

// --- CALLBACK PROVIZIONARE ---
static void example_ble_mesh_provisioning_cb(
    esp_ble_mesh_prov_cb_event_t event,
    esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT: {
        uint8_t *u   = param->provisioner_recv_unprov_adv_pkt.dev_uuid;
        int64_t  now = esp_timer_get_time();
        int existing_idx = -1;
        int empty_idx    = -1;

        for (int i = 0; i < DB_SIZE; i++) {
            if (scan_results[i].occupied && memcmp(scan_results[i].uuid, u, 16) == 0) {
                existing_idx = i; break;
            }
            if (empty_idx == -1 && !scan_results[i].occupied) empty_idx = i;
        }

        int target_slot = (existing_idx != -1) ? existing_idx : empty_idx;
        if (target_slot != -1) {
            bool is_truly_new = (existing_idx == -1);
            memcpy(scan_results[target_slot].uuid, u, 16);
            scan_results[target_slot].addr_type = param->provisioner_recv_unprov_adv_pkt.addr_type;
            memcpy(scan_results[target_slot].addr, param->provisioner_recv_unprov_adv_pkt.addr, 6);
            scan_results[target_slot].bearer   = param->provisioner_recv_unprov_adv_pkt.bearer;
            scan_results[target_slot].occupied = true;

            if (is_truly_new || (now - scan_results[target_slot].last_report_time) > 5000000) {
                scan_results[target_slot].last_report_time = now;
                char uuid_str[33];
                for (int i = 0; i < 16; i++) sprintf(uuid_str + (i * 2), "%02x", u[i]);
                json_send_unprov(uuid_str, param->provisioner_recv_unprov_adv_pkt.rssi);
            }

            if (prov_mode_auto) {
                esp_ble_mesh_unprov_dev_add_t add_dev = {0};
                memcpy(add_dev.uuid, scan_results[target_slot].uuid, 16);
                add_dev.addr_type = scan_results[target_slot].addr_type;
                memcpy(add_dev.addr, scan_results[target_slot].addr, 6);
                add_dev.bearer = scan_results[target_slot].bearer;
                esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
                    ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG);
            }
        }
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT: {
        uint16_t addr  = param->provisioner_prov_complete.unicast_addr;
        node_cache_t *node = get_node(addr);
        if (node && node->in_config) return;
        if (node) {
            node->in_config       = true;
            node->config_step     = 0;
            node->config_start_time = esp_timer_get_time();
        }
        json_send_event("NEW_NODE", addr);
        schedule_config_step(node, 200);
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
        esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, 0, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, ESP_BLE_MESH_CID_NVAL);
        esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, 0, ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_CLI, ESP_BLE_MESH_CID_NVAL);
        esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, 0, ESP_BLE_MESH_MODEL_ID_SENSOR_CLI,    ESP_BLE_MESH_CID_NVAL);
        esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, 0, ESP_BLE_MESH_VND_MODEL_ID_CLIENT,    CID_ESP);
        ESP_LOGI(TAG, "Gateway local models bound to AppKey 0");
        break;

    default:
        break;
    }
}

// --- CALLBACK GENERIC CLIENT ---
static void example_ble_mesh_generic_client_cb(
    esp_ble_mesh_generic_client_cb_event_t event,
    esp_ble_mesh_generic_client_cb_param_t *param)
{
    if (param->error_code != 0) return;
    uint16_t addr = param->params->ctx.addr;

    switch (event) {
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
    case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT: {
        uint32_t opcode = param->params->opcode;
        if (opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET  ||
            opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET  ||
            opcode == 0x8204) {
            json_send_data_i(addr, "STATE", (int)param->status_cb.onoff_status.present_onoff);
        }
        if (opcode == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET  ||
            opcode == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET) {
            json_send_data_i(addr, "LEVEL", (int)param->status_cb.level_status.present_level);
        }
        break;
    }
    default:
        break;
    }
}

// =============================================================================
// PROCESARE COMENZI JSON (mesh/in)
// =============================================================================

void process_incoming_command(const char *raw) {
    ESP_LOGI(TAG, "CMD JSON PRIMIT: '%s'", raw);

    cJSON *root = cJSON_Parse(raw);
    if (!root) {
        ESP_LOGE(TAG, "JSON invalid: %s", raw);
        json_send_error("INVALID_JSON", NULL);
        return;
    }

    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (!cmd_item || !cJSON_IsString(cmd_item)) {
        cJSON_Delete(root);
        json_send_error("MISSING_CMD_FIELD", NULL);
        return;
    }

    const char *cmd = cmd_item->valuestring;

    // Helper lambdas simulate prin macro-uri inline
    // Extragem addr (hex string "0x0005" -> uint16_t)
    uint16_t addr = 0;
    cJSON *addr_item = cJSON_GetObjectItem(root, "addr");
    if (addr_item && cJSON_IsString(addr_item)) {
        addr = (uint16_t)strtol(addr_item->valuestring, NULL, 16);
    }

    // Extragem val (int, optional)
    int32_t val = 0;
    cJSON *val_item = cJSON_GetObjectItem(root, "val");
    if (val_item && cJSON_IsNumber(val_item)) {
        val = (int32_t)val_item->valuedouble;
    }

    // Dispatch comenzi
    if (strcmp(cmd, "SET_PROV_MODE") == 0) {
        prov_mode_auto = (val == 1);
        cJSON *extra = cJSON_CreateObject();
        cJSON_AddStringToObject(extra, "val", prov_mode_auto ? "AUTO" : "MANUAL");
        json_send_ack("PROV_MODE", extra);
    }
    else if (strcmp(cmd, "START_PROV") == 0) {
        cJSON *uuid_item = cJSON_GetObjectItem(root, "uuid");
        if (!uuid_item || !cJSON_IsString(uuid_item)) {
            json_send_error("MISSING_UUID_FIELD", NULL);
        } else {
            const char *uuid_str = uuid_item->valuestring;
            uint8_t target_uuid[16];
            hex_to_bytes(uuid_str, target_uuid, 16);

            int found_slot = -1;
            for (int i = 0; i < DB_SIZE; i++) {
                if (scan_results[i].occupied && memcmp(scan_results[i].uuid, target_uuid, 16) == 0) {
                    found_slot = i; break;
                }
            }
            if (found_slot != -1) {
                esp_ble_mesh_unprov_dev_add_t add_dev = {0};
                memcpy(add_dev.uuid, scan_results[found_slot].uuid, 16);
                add_dev.addr_type = scan_results[found_slot].addr_type;
                memcpy(add_dev.addr, scan_results[found_slot].addr, 6);
                add_dev.bearer    = scan_results[found_slot].bearer;

                esp_err_t err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
                    ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG);
                if (err == ESP_OK) {
                    scan_results[found_slot].occupied        = false;
                    scan_results[found_slot].last_report_time = 0;
                    cJSON *extra = cJSON_CreateObject();
                    cJSON_AddStringToObject(extra, "uuid", uuid_str);
                    json_send_ack("PROV_STARTED", extra);
                } else {
                    char err_str[12];
                    snprintf(err_str, sizeof(err_str), "0x%02x", err);
                    json_send_error("PROV_FAILED", err_str);
                }
            } else {
                json_send_error("UUID_NOT_IN_SCAN_LIST", NULL);
            }
        }
    }
    else if (strcmp(cmd, "LIST_NODES")   == 0) { list_node_db(); }
    else if (strcmp(cmd, "DELETE_DB")    == 0) { delete_node_db(); }
    else if (strcmp(cmd, "DELETE_NODE")  == 0) { delete_node_by_addr(addr); }
    else if (strcmp(cmd, "ON")           == 0) { send_mesh_onoff(addr, 1); }
    else if (strcmp(cmd, "OFF")          == 0) { send_mesh_onoff(addr, 0); }
    else if (strcmp(cmd, "LVL")          == 0) { send_mesh_level(addr, (int16_t)val); }
    else if (strcmp(cmd, "GET_ONOFF")    == 0) { request_mesh_onoff_status(addr); }
    else if (strcmp(cmd, "GET_LVL")      == 0) { request_mesh_level_status(addr); }
    else if (strcmp(cmd, "GET_SENSOR")   == 0) { request_mesh_sensor_data(addr); }
    else if (strcmp(cmd, "SET_PUB")      == 0) {
        cJSON *target_item = cJSON_GetObjectItem(root, "target");
        if (target_item && cJSON_IsString(target_item)) {
            uint16_t target = (uint16_t)strtol(target_item->valuestring, NULL, 16);
            set_switch_publication(addr, target);
        } else {
            json_send_error("MISSING_TARGET_FIELD", NULL);
        }
    }
    else if (strcmp(cmd, "IR_LEARN") == 0) {
        cJSON *slot_item = cJSON_GetObjectItem(root, "slot");
        uint8_t slot = slot_item ? (uint8_t)slot_item->valuedouble : 0;
        send_ir_command(addr, 1, slot);
    }
    else if (strcmp(cmd, "IR_SEND") == 0) {
        cJSON *slot_item = cJSON_GetObjectItem(root, "slot");
        uint8_t slot = slot_item ? (uint8_t)slot_item->valuedouble : 0;
        send_ir_command(addr, 2, slot);
    }
    else {
        cJSON *extra = cJSON_CreateObject();
        cJSON_AddStringToObject(extra, "val", cmd);
        json_send_error("UNKNOWN_CMD", cmd);
    }

    cJSON_Delete(root);
}

// =============================================================================
// MQTT EVENT HANDLER
// =============================================================================

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        mqtt_connected = true;
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_IN, 1);
        ESP_LOGI(TAG, "MQTT Conectat. Subscris la %s", TOPIC_IN);
        break;
    case MQTT_EVENT_DISCONNECTED:
        mqtt_connected = false;
        break;
    case MQTT_EVENT_DATA: {
        char *msg = malloc(event->data_len + 1);
        if (msg) {
            memcpy(msg, event->data, event->data_len);
            msg[event->data_len] = '\0';
            process_incoming_command(msg);
            free(msg);
        }
        break;
    }
    default:
        break;
    }
}

// =============================================================================
// INITIALIZARE
// =============================================================================

static bool mesh_started = false;

void start_ble_mesh_logic(void) {
    if (mesh_started) return;
    mesh_started = true;

    bluetooth_init();
    ble_mesh_get_dev_uuid(dev_uuid);
    memset(prov_key.app_key, APP_KEY_OCTET, 16);

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_generic_client_callback(example_ble_mesh_generic_client_cb);
    esp_ble_mesh_register_sensor_client_callback(example_ble_mesh_sensor_client_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    esp_ble_mesh_init(&provision, &composition);
    esp_ble_mesh_client_model_init(&vendor_models[0]);

    esp_ble_mesh_provisioner_set_dev_uuid_match((uint8_t[]){0xdd, 0xdd}, 2, 0x0, false);
    esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, 0, 0);

    esp_mqtt_client_start(mqtt_client);
}

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if      (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    else if (base == IP_EVENT   && id == IP_EVENT_STA_GOT_IP)         start_ble_mesh_logic();
}

static void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID, .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        }
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
}

void reset_node(uint16_t addr) {
    for (int i = 0; i < DB_SIZE; i++) {
        if (node_db[i].addr == addr) {
            json_send_event("NODE_RESET", addr);
            memset(&node_db[i], 0, sizeof(node_cache_t));
            return;
        }
    }
}

void config_watchdog_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        int64_t now = esp_timer_get_time();
        for (int i = 0; i < DB_SIZE; i++) {
            if (node_db[i].addr != 0 && node_db[i].in_config) {
                int64_t elapsed = (now - node_db[i].config_start_time) / 1000;
                if (elapsed > 15000) {
                    json_send_event("NODE_CONFIG_TIMEOUT", node_db[i].addr);
                    reset_node(node_db[i].addr);
                }
            }
        }
    }
}

void reset_button_task(void *arg) {
    gpio_config_t cfg = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .pull_up_en   = 1
    };
    gpio_config(&cfg);
    while (1) {
        if (gpio_get_level(RESET_BUTTON_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (gpio_get_level(RESET_BUTTON_GPIO) == 0) {
                ESP_LOGW(TAG, "Factory reset!");
                delete_node_db();
                esp_ble_mesh_node_local_reset();
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    load_node_db_from_nvs();
    wifi_init();

    const esp_mqtt_client_config_t mqtt_cfg = { .broker.address.uri = MQTT_URL };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    xTaskCreate(config_watchdog_task, "cfg_wdg", 2048, NULL, 2, NULL);
    xTaskCreate(reset_button_task,    "rst_btn", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "Gateway Online!");
}