/**
 * zigbee_heater_cluster.cpp
 *
 * Zigbee endpoint, custom cluster, attribute table, and command
 * dispatcher for the Diesel Heater Controller.
 *
 * All heater telemetry (run state, error, voltage, fan RPM, body
 * temp, actual/desired pump Hz, glow plug, flame, frame valid)
 * is exposed as readable Zigbee attributes and reported to HA
 * every ZIGBEE_REPORT_MS milliseconds.
 */

#include "zigbee_heater_cluster.h"
#include "esp_zigbee_core.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ZB_HEATER";

// ── Zigbee endpoint ──────────────────────────────────────────
#define ZB_ENDPOINT_ID   1
#define ZB_DEVICE_ID     0xFFFF   // custom / unregistered

// ─────────────────────────────────────────────────────────────
//  Attribute mirror  (updated by zigbee_report_attributes())
//  All values stored in the units described in the cluster header
// ─────────────────────────────────────────────────────────────

// Control
static uint8_t  a_heater_mode       = 0;
static uint8_t  a_pump_hz_raw       = 35;    // default 3.5 Hz
static int16_t  a_air_setpoint      = 2000;  // 20.00 °C
static uint8_t  a_servo_position    = 0;

// DS18B20
static int16_t  a_temp_hx           = 0;
static int16_t  a_temp_hw           = 0;

// Heater telemetry
static uint8_t  a_run_state         = 0;
static uint8_t  a_error_code        = 0;
static uint16_t a_supply_voltage    = 0;     // V × 10
static uint16_t a_fan_rpm           = 0;
static int16_t  a_body_temp         = 0;     // °C × 100
static uint8_t  a_pump_hz_actual    = 0;
static uint8_t  a_pump_hz_desired   = 0;
static uint8_t  a_glow_plug_state   = 0;
static uint8_t  a_flame_detected    = 0;
static uint8_t  a_frame_valid       = 0;

// System
static uint16_t a_cooldown_remaining = 0;
static uint8_t  a_emergency_stop     = 0;
// emergency_reason stored as a character array (ZCL octet string style)
static uint8_t  a_emergency_reason[50] = {};  // [0] = length, [1..] = chars

// ─────────────────────────────────────────────────────────────
//  Command handler
// ─────────────────────────────────────────────────────────────
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id,
                                    const void *message)
{
    if (cb_id == ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID) {
        const esp_zb_zcl_custom_cluster_command_message_t *msg =
            (const esp_zb_zcl_custom_cluster_command_message_t *)message;

        if (msg->info.cluster != HEATER_CLUSTER_ID) return ESP_OK;

        const uint8_t *p   = (const uint8_t *)msg->data.value;
        uint8_t        len = msg->data.size;

        ESP_LOGI(TAG, "Custom cluster CMD 0x%02X  payload_len=%u",
                 msg->info.command.id, len);

        switch (msg->info.command.id) {

            case CMD_SET_MODE:
                if (len >= 1) {
                    a_heater_mode = p[0];
                    zb_cmd_set_mode(p[0]);
                }
                break;

            case CMD_SET_PUMP_HZ_RAW:
                if (len >= 1) {
                    a_pump_hz_raw = p[0];
                    zb_cmd_set_pump_hz_raw(p[0]);
                }
                break;

            case CMD_SET_AIR_SETPOINT:
                if (len >= 2) {
                    // Little-endian int16
                    int16_t sp = (int16_t)((p[1] << 8) | p[0]);
                    a_air_setpoint = sp;
                    zb_cmd_set_setpoint(sp / 100.0f);
                }
                break;

            case CMD_SET_SERVO:
                if (len >= 1) {
                    a_servo_position = p[0];
                    zb_cmd_set_servo(p[0]);
                }
                break;

            case CMD_CLEAR_EMERGENCY:
                zb_cmd_clear_emergency();
                break;

            default:
                ESP_LOGW(TAG, "Unknown command 0x%02X", msg->info.command.id);
                break;
        }
    }

    else if (cb_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        const esp_zb_zcl_set_attr_value_message_t *msg =
            (const esp_zb_zcl_set_attr_value_message_t *)message;

        if (msg->info.cluster != HEATER_CLUSTER_ID) return ESP_OK;

        switch (msg->attribute.id) {
            case ATTR_HEATER_MODE:
                a_heater_mode = *(uint8_t *)msg->attribute.data.value;
                zb_cmd_set_mode(a_heater_mode);
                break;
            case ATTR_PUMP_HZ_RAW:
                a_pump_hz_raw = *(uint8_t *)msg->attribute.data.value;
                zb_cmd_set_pump_hz_raw(a_pump_hz_raw);
                break;
            case ATTR_AIR_SETPOINT:
                a_air_setpoint = *(int16_t *)msg->attribute.data.value;
                zb_cmd_set_setpoint(a_air_setpoint / 100.0f);
                break;
            case ATTR_SERVO_POSITION:
                a_servo_position = *(uint8_t *)msg->attribute.data.value;
                zb_cmd_set_servo(a_servo_position);
                break;
            default:
                break;
        }
    }

    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────
//  Build the custom cluster attribute list
// ─────────────────────────────────────────────────────────────
static esp_zb_attribute_list_t *build_heater_attr_list() {
    esp_zb_attribute_list_t *al = esp_zb_zcl_attr_list_create(HEATER_CLUSTER_ID);

    // ── Control ──────────────────────────────────────────────
    esp_zb_custom_cluster_add_custom_attr(al, ATTR_HEATER_MODE,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &a_heater_mode);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_PUMP_HZ_RAW,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &a_pump_hz_raw);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_AIR_SETPOINT,
        ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &a_air_setpoint);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_SERVO_POSITION,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &a_servo_position);

    // ── DS18B20 temperatures ──────────────────────────────────
    esp_zb_custom_cluster_add_custom_attr(al, ATTR_TEMP_HEAT_EXCHANGER,
        ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_temp_hx);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_TEMP_HOT_WATER,
        ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_temp_hw);

    // ── Heater telemetry ──────────────────────────────────────
    esp_zb_custom_cluster_add_custom_attr(al, ATTR_RUN_STATE,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_run_state);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_ERROR_CODE,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_error_code);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_SUPPLY_VOLTAGE,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_supply_voltage);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_FAN_SPEED_RPM,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_fan_rpm);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_BODY_TEMP,
        ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_body_temp);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_PUMP_HZ_ACTUAL,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_pump_hz_actual);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_PUMP_HZ_DESIRED,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_pump_hz_desired);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_GLOW_PLUG_STATE,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_glow_plug_state);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_FLAME_DETECTED,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_flame_detected);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_FRAME_VALID,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_frame_valid);

    // ── System status ─────────────────────────────────────────
    esp_zb_custom_cluster_add_custom_attr(al, ATTR_COOLDOWN_REMAINING,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_cooldown_remaining);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_EMERGENCY_STOP,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_emergency_stop);

    esp_zb_custom_cluster_add_custom_attr(al, ATTR_EMERGENCY_REASON,
        ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &a_emergency_reason);

    return al;
}

// ─────────────────────────────────────────────────────────────
//  Build the full cluster list (basic + identify + heater)
// ─────────────────────────────────────────────────────────────
static esp_zb_cluster_list_t *build_cluster_list() {
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    // Basic cluster
    esp_zb_basic_cluster_cfg_t basic = { .zcl_version = 3, .power_source = 0x03 };
    esp_zb_cluster_list_add_basic_cluster(cl,
        esp_zb_basic_cluster_create(&basic), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Identify cluster
    esp_zb_identify_cluster_cfg_t id = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cl,
        esp_zb_identify_cluster_create(&id), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Custom heater cluster
    esp_zb_cluster_list_add_custom_cluster(cl,
        build_heater_attr_list(), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    return cl;
}

// ─────────────────────────────────────────────────────────────
//  Public:  zigbee_init()
// ─────────────────────────────────────────────────────────────
void zigbee_init() {
    ESP_LOGI(TAG, "Initialising Zigbee stack...");

    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg = { .zczr_cfg = { .max_children = 10 } }
    };
    esp_zb_init(&zb_cfg);

    // Register endpoint
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint          = ZB_ENDPOINT_ID,
        .app_profile_id    = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id     = ZB_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, build_cluster_list(), ep_cfg);
    esp_zb_device_register(ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    esp_zb_start(false);  // false = resume from NVS (won't re-pair if already joined)

    ESP_LOGI(TAG, "Zigbee started.  Endpoint=%u  Cluster=0x%04X",
             ZB_ENDPOINT_ID, HEATER_CLUSTER_ID);
}

// ─────────────────────────────────────────────────────────────
//  Helper: write and mark an attribute dirty (triggers reports)
// ─────────────────────────────────────────────────────────────
#define ZB_SET(attr_id, ptr) \
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT_ID, HEATER_CLUSTER_ID, \
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, (attr_id), (ptr), false)

// ─────────────────────────────────────────────────────────────
//  Public:  zigbee_report_attributes()
//  Called from main loop every ZIGBEE_REPORT_MS.
//  Mirrors all HeaterState fields into attribute storage,
//  then marks them changed so ZHA/HA receives updated values.
// ─────────────────────────────────────────────────────────────
void zigbee_report_attributes(const HeaterState &s) {

    // ── Control ──────────────────────────────────────────────
    a_heater_mode    = (uint8_t)s.mode;
    a_pump_hz_raw    = s.pumpHzTargetRaw;
    a_air_setpoint   = (int16_t)(s.setpointAirC * 100.0f);

    // ── DS18B20 ───────────────────────────────────────────────
    a_temp_hx = (int16_t)(s.tempHeatExchanger * 100.0f);
    a_temp_hw = (int16_t)(s.tempHotWater      * 100.0f);

    // ── Heater telemetry ──────────────────────────────────────
    const HeaterTelemetry &t = s.telem;

    a_run_state       = (uint8_t)t.runState;
    a_error_code      = (uint8_t)t.errorCode;
    a_supply_voltage  = (uint16_t)(t.supplyVoltage * 10.0f);
    a_fan_rpm         = t.fanSpeedRPM;
    a_body_temp       = (int16_t)(t.bodyTempC * 100.0f);
    a_pump_hz_actual  = (uint8_t)(t.pumpHzActual  * 10.0f + 0.5f);
    a_pump_hz_desired = (uint8_t)(t.pumpHzDesired * 10.0f + 0.5f);
    a_glow_plug_state = t.glowPlugState;
    a_flame_detected  = t.flameDetected ? 1 : 0;
    a_frame_valid     = t.frameValid    ? 1 : 0;

    // ── System status ─────────────────────────────────────────
    a_cooldown_remaining = (s.cooldownActive)
        ? (uint16_t)((millis() - s.cooldownStartMs < COOLDOWN_SECONDS * 1000UL)
            ? (COOLDOWN_SECONDS - (millis() - s.cooldownStartMs) / 1000UL)
            : 0)
        : 0;

    a_emergency_stop = s.emergencyStop ? 1 : 0;

    // ZCL char string: [0] = length byte, [1..] = ASCII
    size_t rlen = strlen(s.emergencyReason);
    if (rlen > 48) rlen = 48;
    a_emergency_reason[0] = (uint8_t)rlen;
    memcpy(&a_emergency_reason[1], s.emergencyReason, rlen);

    // ── Notify ZHA of changed values ──────────────────────────
    ZB_SET(ATTR_HEATER_MODE,          &a_heater_mode);
    ZB_SET(ATTR_PUMP_HZ_RAW,          &a_pump_hz_raw);
    ZB_SET(ATTR_AIR_SETPOINT,         &a_air_setpoint);

    ZB_SET(ATTR_TEMP_HEAT_EXCHANGER,  &a_temp_hx);
    ZB_SET(ATTR_TEMP_HOT_WATER,       &a_temp_hw);

    ZB_SET(ATTR_RUN_STATE,            &a_run_state);
    ZB_SET(ATTR_ERROR_CODE,           &a_error_code);
    ZB_SET(ATTR_SUPPLY_VOLTAGE,       &a_supply_voltage);
    ZB_SET(ATTR_FAN_SPEED_RPM,        &a_fan_rpm);
    ZB_SET(ATTR_BODY_TEMP,            &a_body_temp);
    ZB_SET(ATTR_PUMP_HZ_ACTUAL,       &a_pump_hz_actual);
    ZB_SET(ATTR_PUMP_HZ_DESIRED,      &a_pump_hz_desired);
    ZB_SET(ATTR_GLOW_PLUG_STATE,      &a_glow_plug_state);
    ZB_SET(ATTR_FLAME_DETECTED,       &a_flame_detected);
    ZB_SET(ATTR_FRAME_VALID,          &a_frame_valid);

    ZB_SET(ATTR_COOLDOWN_REMAINING,   &a_cooldown_remaining);
    ZB_SET(ATTR_EMERGENCY_STOP,       &a_emergency_stop);
    ZB_SET(ATTR_EMERGENCY_REASON,     &a_emergency_reason);
}
