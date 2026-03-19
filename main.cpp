/**
 * ============================================================
 *  Diesel Heater Zigbee Controller  –  v2.0
 *  Hardware:  ESP32-C6
 *  Control:   Fuel pump frequency (Hz) via serial
 *  Sensors:   2× DS18B20 (heat exchanger + hot water tank)
 *  Actuator:  Servo for hot-air / hot-water diverter
 *  Network:   Zigbee → Home Assistant (ZHA)
 * ============================================================
 *
 *  PINOUT
 *  ─────────────────────────────────────────────────
 *  GPIO 16   Serial TX  (UART1, via 12V LIN transceiver or direct 5V)
 *  GPIO 17   Serial RX  (UART1)
 *  GPIO 4    Bus-Enable  HIGH=TX  LOW=RX  (half-duplex)
 *  GPIO 5    DS18B20 one-wire data  (4k7 pull-up to 3V3 required)
 *  GPIO 6    Servo PWM signal
 *  GPIO 3    Status LED + 220Ω to GND
 *
 *  FUEL PUMP FREQUENCY → HEAT OUTPUT MAPPING
 *  ─────────────────────────────────────────────────
 *  Chinese diesel heaters regulate heat output by varying the
 *  frequency of the solenoid fuel pump:
 *
 *    1.0 Hz  →  ~1.0 kW  (minimum glow / idle tick)
 *    2.0 Hz  →  ~2.0 kW
 *    3.5 Hz  →  ~3.0 kW  (comfortable default)
 *    5.0 Hz  →  ~4.0 kW
 *    6.5 Hz  →  ~5.0 kW  (maximum for most 5 kW units)
 *
 *  The serial frame carries the desired Hz as a fixed-point byte:
 *    encoded_byte = Hz × 10
 *    e.g.  1.5 Hz → 15,   5.0 Hz → 50,   6.5 Hz → 65
 *
 *  Home Assistant slider sends Hz × 10 as an integer attribute.
 *  The firmware clamps to [PUMP_HZ_MIN_RAW … PUMP_HZ_MAX_RAW].
 *
 *  HEATER SERIAL PROTOCOL (common Chinese clone variant)
 *  ─────────────────────────────────────────────────────────
 *  Control frame  →  heater  (9 bytes)
 *  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 *  │ 0x76 │ CMD  │ PMP  │ 0x00 │ 0x00 │ 0x00 │ 0x00 │ 0x00 │  CS  │
 *  └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *   [0] 0x76  header
 *   [1] CMD   0x00=stop  0x01=start  0x02=adjust pump Hz
 *   [2] PMP   pump Hz × 10  (ignored on stop)
 *   [3-7]     reserved / 0x00
 *   [8] CS    XOR of bytes [1..7]
 *
 *  Status frame   ←  heater  (24 bytes)
 *  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬─ ─ ─ ┬──────┬──────┐
 *  │ 0x76 │ STA  │ ERR  │ VLT  │ FNH  │ FNL  │ BTH  │ BTL  │ ACT  │  …   │  CS  │ 0x00 │
 *  └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴─ ─ ─ ┴──────┴──────┘
 *   [0]  0x76  header
 *   [1]  STA   run state (enum RunState)
 *   [2]  ERR   error code (enum ErrorCode)
 *   [3]  VLT   supply voltage × 10  (e.g. 0x82 = 13.0 V)
 *   [4]  FNH   fan RPM high byte
 *   [5]  FNL   fan RPM low  byte
 *   [6]  BTH   body temp × 10 high byte  (signed 16-bit)
 *   [7]  BTL   body temp × 10 low  byte
 *   [8]  ACT   actual pump Hz × 10
 *   [9]  GLW   glow plug state  0=off  1=preheat  2=on
 *   [10] DES   desired pump Hz × 10 (echo of last command)
 *   [11-12]    reserved
 *   [13] FLM   flame detected  0=no  1=yes
 *   [14-21]    reserved / additional diagnostics
 *   [22] CS    XOR of bytes [1..21]
 *   [23] 0x00  tail
 *
 *  NOTE: Some clones differ slightly – capture your own traffic
 *  with a logic analyser (Saleae / PulseView) to verify byte maps.
 * ============================================================
 */

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>

#include "esp_zigbee_core.h"
#include "zigbee_heater_cluster.h"

// ── Pin definitions ─────────────────────────────────────────
#define LIN_TX_PIN           16
#define LIN_RX_PIN           17
#define LIN_BUS_ENABLE_PIN    4
#define ONE_WIRE_PIN          5
#define SERVO_PIN             6
#define STATUS_LED_PIN        3

// ── Serial config ────────────────────────────────────────────
#define HEATER_BAUD           9600     // try 19200 if comms fail
#define HEATER_FRAME_HEADER   0x76
#define HEATER_TX_LEN          9
#define HEATER_RX_LEN         24
#define HEATER_RX_TIMEOUT_MS  200

// ── Fuel pump frequency (Hz × 10) ────────────────────────────
#define PUMP_HZ_MIN_RAW       10       //  1.0 Hz  – minimum heat
#define PUMP_HZ_MAX_RAW       65       //  6.5 Hz  – maximum heat
#define PUMP_HZ_DEFAULT_RAW   35       //  3.5 Hz  – default air heat
#define PUMP_HZ_HOTWATER_RAW  55       //  5.5 Hz  – hot water heating

// ── Timing constants ─────────────────────────────────────────
#define COOLDOWN_SECONDS      500UL    // HARD-CODED – cannot be shortened
#define SENSOR_POLL_MS       5000UL
#define HEATER_POLL_MS       1000UL
#define ZIGBEE_REPORT_MS     5000UL

// ── Servo positions ──────────────────────────────────────────
#define SERVO_HOT_AIR_DEG     0        // diverter → cabin air output
#define SERVO_HOT_WATER_DEG   90       // diverter → heat exchanger

// ── Safety thresholds ────────────────────────────────────────
#define HEAT_EXCHANGER_MAX_C  85.0f
#define HOT_WATER_MAX_C       70.0f
#define SUPPLY_VOLTAGE_MIN_V   9.5f
#define SUPPLY_VOLTAGE_MAX_V  15.5f

// =============================================================
//  Heater run-state enum   (byte [1] of status frame)
// =============================================================
enum RunState : uint8_t {
    RS_OFF              = 0x00,
    RS_STARTUP          = 0x01,   // controller initialising
    RS_PREHEAT          = 0x02,   // glow plug warming up
    RS_IGNITION         = 0x03,   // first ignition attempt
    RS_IGNITION_RETRY   = 0x04,   // second ignition attempt
    RS_STABILISE        = 0x05,   // flame lit, stabilising
    RS_RUNNING          = 0x06,   // normal run
    RS_COOLING_DOWN     = 0x07,   // fuel pump off, fan running
    RS_COOLDOWN_FAN     = 0x08,   // fan only
    RS_SHUTTING_DOWN    = 0x09,   // commanded stop
    RS_FAULT            = 0xFF,
};

// Friendly names for Zigbee / serial debug
static const char *run_state_str(RunState s) {
    switch (s) {
        case RS_OFF:            return "Off";
        case RS_STARTUP:        return "Startup";
        case RS_PREHEAT:        return "Preheat";
        case RS_IGNITION:       return "Ignition";
        case RS_IGNITION_RETRY: return "Ignition retry";
        case RS_STABILISE:      return "Stabilising";
        case RS_RUNNING:        return "Running";
        case RS_COOLING_DOWN:   return "Cooling down";
        case RS_COOLDOWN_FAN:   return "Fan cooldown";
        case RS_SHUTTING_DOWN:  return "Shutting down";
        case RS_FAULT:          return "FAULT";
        default:                return "Unknown";
    }
}

// =============================================================
//  Heater error-code enum   (byte [2] of status frame)
// =============================================================
enum HeaterError : uint8_t {
    ERR_NONE            = 0x00,
    ERR_NO_IGNITION     = 0x01,   // failed to light after retries
    ERR_OVERHEAT        = 0x02,   // internal overheat sensor
    ERR_UNDERVOLTAGE    = 0x03,   // supply too low
    ERR_OVERVOLTAGE     = 0x04,   // supply too high
    ERR_GLOW_PLUG       = 0x05,   // glow plug circuit fault
    ERR_PUMP            = 0x06,   // fuel pump fault
    ERR_FAN             = 0x07,   // fan motor fault
    ERR_SENSOR          = 0x08,   // temp sensor fault
    ERR_FLAME_LOST      = 0x09,   // flame lost during run
    ERR_LOW_VOLTAGE_RUN = 0x0A,   // running at borderline voltage
};

static const char *error_str(HeaterError e) {
    switch (e) {
        case ERR_NONE:            return "None";
        case ERR_NO_IGNITION:     return "No ignition";
        case ERR_OVERHEAT:        return "Overheat";
        case ERR_UNDERVOLTAGE:    return "Under-voltage";
        case ERR_OVERVOLTAGE:     return "Over-voltage";
        case ERR_GLOW_PLUG:       return "Glow plug fault";
        case ERR_PUMP:            return "Pump fault";
        case ERR_FAN:             return "Fan fault";
        case ERR_SENSOR:          return "Sensor fault";
        case ERR_FLAME_LOST:      return "Flame lost";
        case ERR_LOW_VOLTAGE_RUN: return "Low voltage (running)";
        default:                  return "Unknown error";
    }
}

// =============================================================
//  Full heater telemetry  (decoded from status frame)
// =============================================================
struct HeaterTelemetry {
    RunState    runState        = RS_OFF;
    HeaterError errorCode       = ERR_NONE;
    float       supplyVoltage   = 0.0f;   // V
    uint16_t    fanSpeedRPM     = 0;
    float       bodyTempC       = 0.0f;   // heater body outlet temp (°C)
    float       pumpHzActual    = 0.0f;   // pump Hz currently running
    float       pumpHzDesired   = 0.0f;   // pump Hz echo from last command
    uint8_t     glowPlugState   = 0;      // 0=off 1=preheat 2=lit
    bool        flameDetected   = false;
    bool        frameValid      = false;  // did last RX pass checksum?
    uint8_t     rawBytes[24]    = {};     // full raw frame for diagnostics
};

// =============================================================
//  Master state
// =============================================================
enum class HeaterMode : uint8_t {
    OFF       = 0,
    AIR_HEAT  = 1,
    HOT_WATER = 2,
};

struct HeaterState {
    HeaterMode  mode               = HeaterMode::OFF;
    bool        running            = false;

    // Cooldown – enforced in firmware, cannot be bypassed
    bool        cooldownActive     = false;
    uint32_t    cooldownStartMs    = 0;

    // Emergency latch – cleared only by explicit HA command after cooldown
    bool        emergencyStop      = false;
    char        emergencyReason[48]= {};

    // Control
    uint8_t     pumpHzTargetRaw    = PUMP_HZ_DEFAULT_RAW;  // Hz × 10

    // Setpoints
    float       setpointAirC       = 20.0f;

    // External DS18B20
    float       tempHeatExchanger  = 0.0f;
    float       tempHotWater       = 0.0f;

    // Decoded heater telemetry
    HeaterTelemetry telem;
};

HeaterState g_state;

// ── DS18B20 ──────────────────────────────────────────────────
DeviceAddress  g_addrHeatExchanger;
DeviceAddress  g_addrHotWater;
bool           g_sensorsFound = false;

OneWire           oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// ── Servo ────────────────────────────────────────────────────
Servo diverterServo;

// ── Zigbee (implemented in zigbee_heater_cluster.cpp) ────────
extern void zigbee_init();
extern void zigbee_report_attributes(const HeaterState &s);

// =============================================================
//  Serial frame helpers
// =============================================================

/** XOR checksum over bytes [1..len-2] */
static uint8_t frame_cs(const uint8_t *buf, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 1; i < len - 1; i++) cs ^= buf[i];
    return cs;
}

// =============================================================
//  SEND:  issue a command frame to the heater
//  cmd     0x00 = stop   0x01 = start   0x02 = adjust
//  pumpRaw desired pump Hz × 10
// =============================================================
static void heater_send_command(uint8_t cmd, uint8_t pumpRaw) {
    uint8_t f[HEATER_TX_LEN] = {
        HEATER_FRAME_HEADER,
        cmd,
        pumpRaw,
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00  // checksum placeholder
    };
    f[8] = frame_cs(f, HEATER_TX_LEN);

    digitalWrite(LIN_BUS_ENABLE_PIN, HIGH);
    delayMicroseconds(100);
    Serial1.write(f, HEATER_TX_LEN);
    Serial1.flush();
    delayMicroseconds(200);
    digitalWrite(LIN_BUS_ENABLE_PIN, LOW);

    Serial.printf("[TX] cmd=0x%02X  pump=%.1f Hz (raw %u)  cs=0x%02X\n",
                  cmd, pumpRaw / 10.0f, pumpRaw, f[8]);
}

// =============================================================
//  RECEIVE:  poll heater status frame and decode all fields
// =============================================================
static bool heater_poll_status() {
    // Clear RX buffer
    while (Serial1.available()) Serial1.read();

    // Send poll request byte (not all units need this – comment out if yours auto-sends)
    digitalWrite(LIN_BUS_ENABLE_PIN, HIGH);
    Serial1.write(0xA0);
    Serial1.flush();
    delayMicroseconds(100);
    digitalWrite(LIN_BUS_ENABLE_PIN, LOW);

    // Collect HEATER_RX_LEN bytes, re-syncing on header
    uint8_t  buf[HEATER_RX_LEN];
    uint8_t  idx     = 0;
    uint32_t deadline = millis() + HEATER_RX_TIMEOUT_MS;

    while (millis() < deadline && idx < HEATER_RX_LEN) {
        if (!Serial1.available()) continue;
        uint8_t b = Serial1.read();
        if (idx == 0 && b != HEATER_FRAME_HEADER) continue;  // re-sync
        buf[idx++] = b;
    }

    if (idx < HEATER_RX_LEN) {
        Serial.printf("[RX] Incomplete: got %u/%u bytes\n", idx, HEATER_RX_LEN);
        g_state.telem.frameValid = false;
        return false;
    }

    // Checksum:  XOR [1..21] == [22]
    uint8_t cs = 0;
    for (uint8_t i = 1; i <= 21; i++) cs ^= buf[i];
    if (cs != buf[22]) {
        Serial.printf("[RX] Checksum FAIL  calc=0x%02X  got=0x%02X\n", cs, buf[22]);
        g_state.telem.frameValid = false;
        return false;
    }

    // ── Decode all fields ────────────────────────────────────
    HeaterTelemetry &t = g_state.telem;
    memcpy(t.rawBytes, buf, HEATER_RX_LEN);

    t.runState       = (RunState)   buf[1];
    t.errorCode      = (HeaterError)buf[2];
    t.supplyVoltage  = buf[3] / 10.0f;
    t.fanSpeedRPM    = ((uint16_t)buf[4] << 8) | buf[5];
    // Body temp is signed 16-bit × 10
    int16_t rawBodyT = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);
    t.bodyTempC      = rawBodyT / 10.0f;
    t.pumpHzActual   = buf[8]  / 10.0f;
    t.glowPlugState  = buf[9];
    t.pumpHzDesired  = buf[10] / 10.0f;
    t.flameDetected  = (buf[13] != 0);
    t.frameValid     = true;

    Serial.printf("[RX] state=%-16s  err=%-18s  V=%.1fV  fan=%4u rpm  "
                  "body=%.1f°C  pump_act=%.1fHz  pump_des=%.1fHz  glow=%u  flame=%u\n",
                  run_state_str(t.runState), error_str(t.errorCode),
                  t.supplyVoltage, t.fanSpeedRPM, t.bodyTempC,
                  t.pumpHzActual, t.pumpHzDesired, t.glowPlugState, t.flameDetected);

    // ── Voltage safety ───────────────────────────────────────
    if (t.supplyVoltage > 1.0f) {
        if (t.supplyVoltage < SUPPLY_VOLTAGE_MIN_V)
            Serial.printf("[WARN] Low supply voltage: %.1fV\n", t.supplyVoltage);
        if (t.supplyVoltage > SUPPLY_VOLTAGE_MAX_V) {
            char r[48]; snprintf(r, sizeof(r), "Over-voltage %.1fV", t.supplyVoltage);
            heater_emergency_stop(r);
        }
    }

    return true;
}

// =============================================================
//  Heater command logic
// =============================================================
static void heater_start(HeaterMode mode, uint8_t pumpRaw) {
    if (g_state.cooldownActive || g_state.emergencyStop) {
        Serial.println("[HEATER] Start BLOCKED – cooldown or emergency active");
        return;
    }
    pumpRaw = constrain(pumpRaw, (uint8_t)PUMP_HZ_MIN_RAW, (uint8_t)PUMP_HZ_MAX_RAW);

    g_state.mode            = mode;
    g_state.running         = true;
    g_state.pumpHzTargetRaw = pumpRaw;

    // Position diverter servo BEFORE igniting
    uint8_t deg = (mode == HeaterMode::HOT_WATER) ? SERVO_HOT_WATER_DEG : SERVO_HOT_AIR_DEG;
    diverterServo.write(deg);
    Serial.printf("[SERVO] → %u°\n", deg);
    delay(700);   // allow servo to complete movement

    heater_send_command(0x01, pumpRaw);
    Serial.printf("[HEATER] Started  mode=%u  pump=%.1f Hz  servo=%u°\n",
                  (uint8_t)mode, pumpRaw / 10.0f, deg);
}

static void heater_set_pump_hz(uint8_t pumpRaw) {
    if (!g_state.running) {
        Serial.println("[HEATER] set_pump_hz ignored – not running");
        return;
    }
    pumpRaw = constrain(pumpRaw, (uint8_t)PUMP_HZ_MIN_RAW, (uint8_t)PUMP_HZ_MAX_RAW);
    g_state.pumpHzTargetRaw = pumpRaw;
    heater_send_command(0x02, pumpRaw);
}

static void heater_stop() {
    if (!g_state.running && !g_state.cooldownActive) return;

    heater_send_command(0x00, 0x00);

    g_state.running         = false;
    g_state.cooldownActive  = true;
    g_state.cooldownStartMs = millis();

    // Do NOT move servo – cooldown fan needs unrestricted airflow
    Serial.printf("[HEATER] Stop sent.  500s cooldown begins (completes ~%lu)\n",
                  (millis() + COOLDOWN_SECONDS * 1000UL) / 1000UL);
}

// Called from safety checks – sets emergency latch
void heater_emergency_stop(const char *reason) {
    heater_send_command(0x00, 0x00);

    g_state.running         = false;
    g_state.emergencyStop   = true;
    g_state.cooldownActive  = true;
    g_state.cooldownStartMs = millis();
    strncpy(g_state.emergencyReason, reason, sizeof(g_state.emergencyReason) - 1);

    Serial.printf("[SAFETY] *** EMERGENCY STOP: %s ***\n", reason);
}

// =============================================================
//  500-second cooldown  (enforced in firmware, not bypassable)
// =============================================================
static void update_cooldown() {
    if (!g_state.cooldownActive) return;

    uint32_t elapsed = (millis() - g_state.cooldownStartMs) / 1000UL;
    if (elapsed >= COOLDOWN_SECONDS) {
        g_state.cooldownActive = false;
        g_state.mode           = HeaterMode::OFF;
        if (!g_state.emergencyStop) {
            Serial.println("[HEATER] Cooldown complete – system ready.");
        } else {
            Serial.println("[HEATER] Cooldown complete – emergency latch still set. "
                           "Clear via HA before restarting.");
        }
    }
}

static uint16_t cooldown_remaining_sec() {
    if (!g_state.cooldownActive) return 0;
    uint32_t elapsed = (millis() - g_state.cooldownStartMs) / 1000UL;
    return (elapsed < COOLDOWN_SECONDS) ? (uint16_t)(COOLDOWN_SECONDS - elapsed) : 0;
}

// =============================================================
//  DS18B20 temperature sensors
// =============================================================
static void init_temp_sensors() {
    sensors.begin();
    uint8_t n = sensors.getDeviceCount();
    Serial.printf("[SENSORS] %u DS18B20 found\n", n);

    if (n >= 2) {
        sensors.getAddress(g_addrHeatExchanger, 0);
        sensors.getAddress(g_addrHotWater,      1);
        sensors.setResolution(g_addrHeatExchanger, 12);
        sensors.setResolution(g_addrHotWater,      12);
        g_sensorsFound = true;
        Serial.print("[SENSORS] HX: ");
        for (int i=0;i<8;i++) Serial.printf("%02X ",g_addrHeatExchanger[i]);
        Serial.print("\n[SENSORS] HW: ");
        for (int i=0;i<8;i++) Serial.printf("%02X ",g_addrHotWater[i]);
        Serial.println();
    } else if (n == 1) {
        sensors.getAddress(g_addrHeatExchanger, 0);
        sensors.setResolution(g_addrHeatExchanger, 12);
        g_sensorsFound = true;
        Serial.println("[SENSORS] WARNING: Only 1 sensor (need 2 for full safety coverage)");
    } else {
        Serial.println("[SENSORS] ERROR: No DS18B20 found – check wiring + 4k7 pull-up!");
    }
}

static void read_temp_sensors() {
    if (!g_sensorsFound) return;

    sensors.requestTemperatures();

    float hx = sensors.getTempC(g_addrHeatExchanger);
    float hw = sensors.getTempC(g_addrHotWater);

    if (hx != DEVICE_DISCONNECTED_C) g_state.tempHeatExchanger = hx;
    if (hw != DEVICE_DISCONNECTED_C) g_state.tempHotWater       = hw;

    Serial.printf("[SENSORS] HX=%.2f°C  HW=%.2f°C\n",
                  g_state.tempHeatExchanger, g_state.tempHotWater);

    if (g_state.tempHeatExchanger > HEAT_EXCHANGER_MAX_C)
        heater_emergency_stop("Heat exchanger overtemp");

    if (g_state.tempHotWater > HOT_WATER_MAX_C)
        heater_emergency_stop("Hot water tank overtemp");
}

// =============================================================
//  Zigbee command callbacks
//  Called from zigbee_heater_cluster.cpp when HA writes attrs
// =============================================================
extern "C" {

void zb_cmd_set_mode(uint8_t mode) {
    Serial.printf("[ZB] set_mode=%u\n", mode);
    if (g_state.cooldownActive || g_state.emergencyStop) {
        Serial.println("[ZB] BLOCKED – cooldown/emergency");
        return;
    }
    switch (mode) {
        case 0:  heater_stop();                                                break;
        case 1:  heater_start(HeaterMode::AIR_HEAT,  g_state.pumpHzTargetRaw); break;
        case 2:  heater_start(HeaterMode::HOT_WATER, PUMP_HZ_HOTWATER_RAW);   break;
        default: break;
    }
}

/**
 * HA sends pump Hz × 10 as a uint8 (0–255).
 * Range:  10 = 1.0 Hz  …  65 = 6.5 Hz
 */
void zb_cmd_set_pump_hz_raw(uint8_t pumpRaw) {
    Serial.printf("[ZB] set_pump_hz raw=%u  (%.1f Hz)\n", pumpRaw, pumpRaw / 10.0f);
    if (g_state.cooldownActive || g_state.emergencyStop) return;

    g_state.pumpHzTargetRaw = constrain(pumpRaw,
                                        (uint8_t)PUMP_HZ_MIN_RAW,
                                        (uint8_t)PUMP_HZ_MAX_RAW);
    if (g_state.running) heater_set_pump_hz(g_state.pumpHzTargetRaw);
}

void zb_cmd_set_setpoint(float tempC) {
    Serial.printf("[ZB] set_setpoint=%.1f°C\n", tempC);
    g_state.setpointAirC = tempC;
}

void zb_cmd_set_servo(uint8_t deg) {
    Serial.printf("[ZB] set_servo=%u°\n", deg);
    diverterServo.write(constrain(deg, 0, 180));
}

void zb_cmd_clear_emergency() {
    Serial.println("[ZB] clear_emergency");
    g_state.emergencyStop = false;
    memset(g_state.emergencyReason, 0, sizeof(g_state.emergencyReason));
    Serial.println("[HEATER] Emergency cleared."
                   + String(g_state.cooldownActive
                             ? " (cooldown still active)"
                             : " System ready."));
}

} // extern "C"

// =============================================================
//  setup() / loop()
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║  Diesel Heater Zigbee Controller  v2.0  ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.printf( "  Cooldown:   %lu seconds (hard-coded)\n", COOLDOWN_SECONDS);
    Serial.printf( "  Pump Hz:    %.1f–%.1f  default=%.1f Hz\n",
                   PUMP_HZ_MIN_RAW/10.0f, PUMP_HZ_MAX_RAW/10.0f,
                   PUMP_HZ_DEFAULT_RAW/10.0f);

    // GPIO init
    pinMode(STATUS_LED_PIN,     OUTPUT);
    pinMode(LIN_BUS_ENABLE_PIN, OUTPUT);
    digitalWrite(LIN_BUS_ENABLE_PIN, LOW);
    digitalWrite(STATUS_LED_PIN,     LOW);

    // Servo
    ESP32PWM::allocateTimer(0);
    diverterServo.setPeriodHertz(50);
    diverterServo.attach(SERVO_PIN, 500, 2400);
    diverterServo.write(SERVO_HOT_AIR_DEG);
    Serial.printf("[SERVO] Init at %u° (hot-air)\n", SERVO_HOT_AIR_DEG);

    // Temp sensors
    init_temp_sensors();

    // Heater UART
    Serial1.begin(HEATER_BAUD, SERIAL_8N1, LIN_RX_PIN, LIN_TX_PIN);
    Serial.printf("[UART1] %u baud  TX=GPIO%u  RX=GPIO%u\n",
                  HEATER_BAUD, LIN_TX_PIN, LIN_RX_PIN);

    // Zigbee
    zigbee_init();

    digitalWrite(STATUS_LED_PIN, HIGH);
    Serial.println("[BOOT] Ready.\n");
}

void loop() {
    static uint32_t lastSensorMs = 0;
    static uint32_t lastPollMs   = 0;
    static uint32_t lastReportMs = 0;
    static uint32_t ledMs        = 0;

    uint32_t now = millis();

    // 1. Cooldown enforcement
    update_cooldown();

    // 2. DS18B20 reads + safety check
    if (now - lastSensorMs >= SENSOR_POLL_MS) {
        lastSensorMs = now;
        read_temp_sensors();
    }

    // 3. Heater telemetry poll (while running or cooling down)
    if (g_state.running || g_state.cooldownActive) {
        if (now - lastPollMs >= HEATER_POLL_MS) {
            lastPollMs = now;
            heater_poll_status();

            // Trigger emergency stop on heater-reported faults
            if (g_state.telem.frameValid &&
                g_state.telem.errorCode != ERR_NONE &&
                !g_state.emergencyStop) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Heater fault: %s (0x%02X)",
                         error_str(g_state.telem.errorCode),
                         g_state.telem.errorCode);
                heater_emergency_stop(msg);
            }
        }
    }

    // 4. Zigbee attribute reports
    if (now - lastReportMs >= ZIGBEE_REPORT_MS) {
        lastReportMs = now;
        zigbee_report_attributes(g_state);
    }

    // 5. LED blink pattern encodes state
    //    100ms  = emergency stop
    //    250ms  = cooldown active
    //    800ms  = running normally
    //    3000ms = idle / off
    uint32_t blink = g_state.emergencyStop   ?  100 :
                     g_state.cooldownActive   ?  250 :
                     g_state.running          ?  800 : 3000;
    if (now - ledMs > blink) {
        ledMs = now;
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }

    delay(5);
}
