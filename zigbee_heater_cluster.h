/**
 * zigbee_heater_cluster.h
 *
 * Custom Zigbee cluster for the Diesel Heater Controller.
 *
 * Cluster ID:  0xFC00  (manufacturer-specific private range)
 *
 * ══════════════════════════════════════════════════════════════
 *  ATTRIBUTES  (server = ESP32-C6 device)
 * ══════════════════════════════════════════════════════════════
 *
 *  ── Control ─────────────────────────────────────────────────
 *  0x0000  heater_mode          uint8    R/W   0=off 1=air 2=hotwater
 *  0x0001  pump_hz_raw          uint8    R/W   desired pump Hz × 10
 *                                              (10 = 1.0Hz … 65 = 6.5Hz)
 *  0x0002  air_setpoint         int16    R/W   °C × 100  (2000 = 20.00°C)
 *  0x0003  servo_position       uint8    R/W   degrees 0–180
 *
 *  ── DS18B20 temperatures ────────────────────────────────────
 *  0x0010  temp_heat_exchanger  int16    R     °C × 100
 *  0x0011  temp_hot_water       int16    R     °C × 100
 *
 *  ── Heater telemetry (decoded from serial status frame) ──────
 *  0x0020  run_state            uint8    R     enum RunState (0–9, 0xFF=fault)
 *  0x0021  error_code           uint8    R     enum HeaterError
 *  0x0022  supply_voltage       uint16   R     V × 10  (130 = 13.0V)
 *  0x0023  fan_speed_rpm        uint16   R     RPM
 *  0x0024  body_temp            int16    R     °C × 100  (heater outlet)
 *  0x0025  pump_hz_actual       uint8    R     actual Hz × 10 running now
 *  0x0026  pump_hz_desired      uint8    R     echo of last commanded Hz
 *  0x0027  glow_plug_state      uint8    R     0=off 1=preheat 2=on
 *  0x0028  flame_detected       bool     R     1=flame present
 *  0x0029  frame_valid          bool     R     last RX frame checksum OK
 *
 *  ── System status ───────────────────────────────────────────
 *  0x0030  cooldown_remaining   uint16   R     seconds left in 500s cooldown
 *  0x0031  emergency_stop       bool     R     1 = emergency latch set
 *  0x0032  emergency_reason     string   R     ASCII reason (max 48 chars)
 *
 * ══════════════════════════════════════════════════════════════
 *  COMMANDS  (HA → device)
 * ══════════════════════════════════════════════════════════════
 *  0x00  SetMode          payload: uint8  mode
 *  0x01  SetPumpHzRaw     payload: uint8  Hz×10  (10–65)
 *  0x02  SetAirSetpoint   payload: int16  °C×100
 *  0x03  SetServo         payload: uint8  degrees
 *  0x04  ClearEmergency   no payload
 */

#pragma once
#include "esp_zigbee_core.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── Cluster identity ─────────────────────────────────────────
#define HEATER_CLUSTER_ID           0xFC00
#define HEATER_MANUFACTURER_CODE    0x1234   // replace with your code

// ── Attribute IDs ────────────────────────────────────────────

// Control group
#define ATTR_HEATER_MODE            0x0000
#define ATTR_PUMP_HZ_RAW            0x0001
#define ATTR_AIR_SETPOINT           0x0002
#define ATTR_SERVO_POSITION         0x0003

// DS18B20 group
#define ATTR_TEMP_HEAT_EXCHANGER    0x0010
#define ATTR_TEMP_HOT_WATER         0x0011

// Heater telemetry group
#define ATTR_RUN_STATE              0x0020
#define ATTR_ERROR_CODE             0x0021
#define ATTR_SUPPLY_VOLTAGE         0x0022
#define ATTR_FAN_SPEED_RPM          0x0023
#define ATTR_BODY_TEMP              0x0024
#define ATTR_PUMP_HZ_ACTUAL         0x0025
#define ATTR_PUMP_HZ_DESIRED        0x0026
#define ATTR_GLOW_PLUG_STATE        0x0027
#define ATTR_FLAME_DETECTED         0x0028
#define ATTR_FRAME_VALID            0x0029

// System status group
#define ATTR_COOLDOWN_REMAINING     0x0030
#define ATTR_EMERGENCY_STOP         0x0031
#define ATTR_EMERGENCY_REASON       0x0032

// ── Command IDs ──────────────────────────────────────────────
#define CMD_SET_MODE                0x00
#define CMD_SET_PUMP_HZ_RAW         0x01
#define CMD_SET_AIR_SETPOINT        0x02
#define CMD_SET_SERVO               0x03
#define CMD_CLEAR_EMERGENCY         0x04

// ── Callbacks (implemented in main.cpp) ──────────────────────
void zb_cmd_set_mode(uint8_t mode);
void zb_cmd_set_pump_hz_raw(uint8_t pumpRaw);
void zb_cmd_set_setpoint(float tempC);
void zb_cmd_set_servo(uint8_t deg);
void zb_cmd_clear_emergency(void);

#ifdef __cplusplus
}
#endif
