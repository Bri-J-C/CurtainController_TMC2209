// ============================================================================
// CURTAIN CONTROLLER v5.3 - TMC2209 Edition
// Based on original v4.3 with TMC2209 UART control added
// ============================================================================
// Target: ESP32-C3 Super Mini
// Driver: TMC2209 with UART control and StallGuard4
// ============================================================================

#define FW_VERSION "5.3"

#include <esp_netif.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <MycilaWebSerial.h>
#include <TMCStepper.h>
#include <stdarg.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Motor Configuration
int steps_per_revolution = 2000;

// Watchdog timeout (increased to handle slow movements)
const int WDT_TIMEOUT = 180;  // 3 minutes

// Pin definitions for TMC2209 - ESP32-C3 SUPER MINI
// Safe pins: GPIO 0,1,3,4,5,6,7,10,20,21
// Avoid at boot: GPIO 2,8,9 (strapping pins)
const int STEP_PIN = 10;
const int DIR_PIN = 6;
const int ENABLE_PIN = 0;
const int DIAG_PIN = 7;
const int TMC_TX_PIN = 21;
const int TMC_RX_PIN = 20;
const int STATUS_LED = 8;
const int RESET_BUTTON_PIN = 9;

// TMC2209 Configuration
const float R_SENSE = 0.11f;
const uint8_t DRIVER_ADDRESS = 0b00;
HardwareSerial TMCSerial(1);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);
bool tmc_available = false;

// TMC2209 settings
uint16_t motor_current_ma = 800;
uint16_t motor_microsteps = 2;
uint8_t stall_threshold = 50;
bool tmc_verbose = false;  // Verbose mode for TMC diagnostics
bool invert_direction = false;  // Swap open/close direction

// Motor control
int current_position = 0;
int target_position = 0;
bool is_moving = false;
unsigned long last_step_time = 0;
unsigned long movement_start_time = 0;
const unsigned long MOVEMENT_TIMEOUT = 120000;  // 2 minute timeout
int steps_since_last_save = 0;
const int STEPS_BETWEEN_SAVES = 50;
unsigned long last_position_report = 0;
const unsigned long POSITION_REPORT_INTERVAL = 500;

// Motor state
int step_delay_us = 2000;
bool motor_enabled = false;
unsigned long motor_sleep_timeout = 30000;
unsigned long last_motor_activity = 0;

// Calibration state
enum CalibrationState { CAL_IDLE, CAL_FIND_MIN, CAL_FIND_MAX };
CalibrationState cal_state = CAL_IDLE;
unsigned long cal_start_time = 0;
unsigned long cal_last_stall_check = 0;
const unsigned long CAL_TIMEOUT = 240000;
const unsigned long STALL_DEBOUNCE_MS = 50;
volatile bool stall_flag = false;

// Reset button with debouncing
unsigned long button_press_start = 0;
unsigned long last_button_change = 0;
bool button_state = HIGH;
bool last_stable_state = HIGH;
const unsigned long BUTTON_DEBOUNCE_MS = 50;
const unsigned long AP_HOLD_MIN = 3000;
const unsigned long AP_HOLD_MAX = 5000;
const unsigned long RESET_HOLD_MIN = 10000;
const unsigned long RESET_HOLD_MAX = 13000;

// LED control
bool led_manual_control = false;
bool led_desired_state = HIGH;

// MQTT reconnection with exponential backoff
unsigned long last_mqtt_attempt = 0;
int mqtt_retry_delay = 2000;
const int MAX_MQTT_RETRY_DELAY = 60000;
unsigned long mqtt_subscribe_time = 0;
const unsigned long MQTT_IGNORE_RETAINED_MS = 2000;

// WiFi reconnection state machine
enum WiFiReconnectState { WIFI_CONNECTED, WIFI_DISCONNECTED, WIFI_RECONNECTING };
WiFiReconnectState wifi_state = WIFI_CONNECTED;
unsigned long wifi_reconnect_start = 0;
const unsigned long WIFI_RECONNECT_TIMEOUT = 30000;

volatile bool ws_command_pending = false;
const char* last_reset_reason = "Unknown";

// Network
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
AsyncWebServer server(80);
WebSerial WebSerial;
String device_hostname;
String mqtt_server;
int mqtt_port;
String mqtt_user;
String mqtt_password;
String mqtt_command_topic;
String mqtt_stat_topic;
String mqtt_position_topic;
String mqtt_availability_topic;
String mqtt_calibrate_topic;
String mqtt_speed_set_topic;
String mqtt_speed_state_topic;
String mqtt_current_set_topic;
String mqtt_current_state_topic;
String mqtt_stallthreshold_set_topic;
String mqtt_stallthreshold_state_topic;
String mqtt_microsteps_set_topic;
String mqtt_microsteps_state_topic;
String mqtt_invert_set_topic;
String mqtt_invert_state_topic;
String ws_pending_command;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void setup_tmc2209();
void setup_wifi_manager();
bool check_button_hold_at_boot(unsigned long hold_time_ms);
void start_config_portal();
void setup_mqtt();
void setup_ota();
void setup_webserial();
void setup_mdns();
void connect_mqtt();
void handle_movement();
void handle_calibration();
void handle_wifi_reconnection();
void check_reset_button();
void stop_movement(const char* reason);
void save_position();
void publish_status(const char* status);
void publish_position();
void publish_ha_discovery(bool force = false);
void publish_settings_state();
static const char* sensitivity_name(uint8_t thr);
void publish_sensitivity_state();
void process_command(const String& command);
void stop_motor();
void wake_motor();
void sleep_motor();
void step_motor();
void start_movement(int target);
void start_calibration();
void check_tmc_errors();

// ============================================================================
// LOGGING SYSTEM
// ============================================================================

enum LogLevel { LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG };
LogLevel current_log_level = LOG_INFO;

static const char* log_level_name(LogLevel level) {
  switch (level) {
    case LOG_ERROR: return "ERROR";
    case LOG_WARN:  return "WARN ";
    case LOG_INFO:  return "INFO ";
    case LOG_DEBUG: return "DEBUG";
    default:        return "?????";
  }
}

void log_msg(LogLevel level, const char* subsystem, const char* fmt, ...) {
  if (level > current_log_level) return;

  char msg[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  char line[300];
  snprintf(line, sizeof(line), "[%s] [%s] %s", log_level_name(level), subsystem, msg);

  Serial.println(line);
  if (WebSerial.getConnectionCount() > 0)
    WebSerial.println(line);
}

// For structured command output (status, config, help, etc.) — no prefix
void output(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  if (WebSerial.getConnectionCount() > 0)
    WebSerial.print(buf);
}

// Send a complete String as one WebSocket message (instant display, no line-by-line)
void ws_send_bulk(const String& text) {
  Serial.print(text);
  if (WebSerial.getConnectionCount() > 0) {
    auto* buf = WebSerial.makeBuffer(text.length());
    if (buf) {
      memcpy(buf->get(), text.c_str(), text.length());
      WebSerial.send(buf);
    }
  }
}

// Append formatted text to a String buffer (for building bulk output)
void buf_printf(String& out, const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  out += buf;
}

// ============================================================================
// STALLGUARD ISR
// ============================================================================

void IRAM_ATTR stall_isr() {
  stall_flag = true;
}

// ============================================================================
// TMC2209 MOTOR CONTROL
// ============================================================================

void setup_tmc2209() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT_PULLDOWN);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, HIGH);  // Disabled

  // Initialize UART for TMC2209
  log_msg(LOG_INFO, "TMC", "Initializing TMC2209 UART...");
  TMCSerial.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
  TMCSerial.setTimeout(100);
  delay(200);

  driver.begin();
  driver.toff(0);
  delay(50);

  // Test communication
  log_msg(LOG_INFO, "TMC", "Testing TMC2209 communication...");
  uint8_t result = driver.test_connection();

  if (result != 0) {
    log_msg(LOG_DEBUG, "TMC", "Comm test returned: %d", result);

    uint32_t ioin = driver.IOIN();
    log_msg(LOG_DEBUG, "TMC", "IOIN register: 0x%08X", ioin);

    if (ioin == 0 || ioin == 0xFFFFFFFF) {
      log_msg(LOG_ERROR, "TMC", "TMC2209 not responding! Check wiring: GPIO21(TX)--[1K]-->PDN_UART, GPIO20(RX)-->PDN_UART");
      log_msg(LOG_WARN, "TMC", "Motor control will be limited");
      tmc_available = false;
      return;
    }
    log_msg(LOG_INFO, "TMC", "TMC2209 responding, continuing setup...");
  }

  tmc_available = true;

  // Basic configuration
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(motor_current_ma);
  driver.microsteps(motor_microsteps);

  // StealthChop for quiet operation
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);

  // StallGuard configuration (for calibration)
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(0);
  driver.semax(0);
  driver.SGTHRS(stall_threshold);

  // Configure DIAG pin for stall output
  driver.GCONF(driver.GCONF() | (1 << 5));

  // Apply direction inversion (after GCONF setup so it doesn't get overwritten)
  driver.shaft(invert_direction);

  // Attach interrupt for stall detection
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stall_isr, RISING);

  log_msg(LOG_INFO, "TMC", "TMC2209 initialized successfully");
}

void set_motor_current(uint16_t ma) {
  if (!tmc_available) return;
  ma = constrain(ma, 100, 2000);
  motor_current_ma = ma;
  driver.rms_current(ma);
  preferences.putUShort("current_ma", ma);
}

void set_motor_microsteps(uint16_t ms) {
  if (!tmc_available) return;
  // Must be power of 2, 1-256
  if (ms == 0 || (ms & (ms - 1)) != 0 || ms > 256) return;
  motor_microsteps = ms;
  driver.microsteps(ms);
  preferences.putUShort("microsteps", ms);
}

void set_stall_threshold(uint8_t threshold) {
  if (!tmc_available) return;
  stall_threshold = threshold;
  driver.SGTHRS(threshold);
  preferences.putUChar("stall_thr", threshold);
}

// Check and recover from TMC2209 errors
void check_tmc_errors() {
  if (!tmc_available) return;

  uint32_t drv_status = driver.DRV_STATUS();

  // Check for errors
  bool ot = drv_status & (1 << 1);      // Overtemperature
  bool otpw = drv_status & (1 << 0);    // Overtemp pre-warning
  bool s2ga = drv_status & (1 << 2);    // Short to ground A
  bool s2gb = drv_status & (1 << 3);    // Short to ground B
  bool ola = drv_status & (1 << 6);     // Open load A
  bool olb = drv_status & (1 << 7);     // Open load B

  if (otpw && !ot) {
    log_msg(LOG_WARN, "TMC", "Overtemperature pre-warning");
  }
  if (ola || olb) {
    log_msg(LOG_WARN, "TMC", "Open load detected (A:%d B:%d)", ola, olb);
  }

  if (ot || s2ga || s2gb) {
    // Serious error - disable and re-enable
    log_msg(LOG_ERROR, "TMC", "Hardware error: OT:%d S2GA:%d S2GB:%d — resetting", ot, s2ga, s2gb);
    stop_motor();
    delay(50);
    yield();
    driver.toff(0);
    delay(10);
    yield();
    driver.toff(4);
    driver.rms_current(motor_current_ma);
  }
}

void stop_motor() {
  digitalWrite(ENABLE_PIN, HIGH);
  motor_enabled = false;
}

void wake_motor() {
  if (!motor_enabled) {
    digitalWrite(ENABLE_PIN, LOW);
    motor_enabled = true;
    delayMicroseconds(100);
  }
  last_motor_activity = millis();
}

void sleep_motor() {
  if (motor_enabled && !is_moving && cal_state == CAL_IDLE) {
    stop_motor();
  }
}

void step_motor() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);
  last_motor_activity = millis();
}

void save_position() {
  preferences.putInt("position", current_position);
}

void start_movement(int target) {
  if (is_moving || cal_state != CAL_IDLE) return;
  if (!tmc_available) {
    log_msg(LOG_ERROR, "MOTOR", "TMC2209 not available, cannot move");
    return;
  }

  target_position = constrain(target, 0, steps_per_revolution);
  if (current_position == target_position) {
    log_msg(LOG_INFO, "MOTOR", "Already at position %d", current_position);
    return;
  }

  wake_motor();
  stall_flag = false;

  if (target_position > current_position) {
    digitalWrite(DIR_PIN, HIGH);
    log_msg(LOG_INFO, "MOTOR", "Opening: %d -> %d (%d%%)",
            current_position, target_position,
            (target_position * 100) / steps_per_revolution);
    publish_status("opening");
  } else {
    digitalWrite(DIR_PIN, LOW);
    log_msg(LOG_INFO, "MOTOR", "Closing: %d -> %d (%d%%)",
            current_position, target_position,
            (target_position * 100) / steps_per_revolution);
    publish_status("closing");
  }

  delayMicroseconds(10);

  is_moving = true;
  last_step_time = micros();
  movement_start_time = millis();
  steps_since_last_save = 0;
  last_position_report = 0;
  publish_position();
}

void stop_movement(const char* reason) {
  is_moving = false;
  stop_motor();
  save_position();
  publish_position();

  const char* status;
  if (strcmp(reason, "Complete") == 0) {
    status = (current_position <= 0) ? "closed" : "open";
  } else {
    status = "stopped";
  }
  publish_status(status);
}

void handle_movement() {
  if (!is_moving) return;

  // Periodic yield for WebSocket responsiveness
  static unsigned long last_move_yield = 0;
  if (millis() - last_move_yield >= 50) {
    yield();
    last_move_yield = millis();
  }

  // Log stall events in verbose mode (but don't stop - stalls ignored during normal movement)
  if (stall_flag && tmc_verbose && tmc_available) {
    uint16_t sg = driver.SG_RESULT();
    log_msg(LOG_WARN, "MOTOR", "Stall detected SG:%d thr:%d (triggers at SG<%d)",
            sg, stall_threshold, stall_threshold * 2);
  }
  stall_flag = false;

  if (millis() - movement_start_time > MOVEMENT_TIMEOUT) {
    log_msg(LOG_ERROR, "MOTOR", "Movement timeout after %lums", MOVEMENT_TIMEOUT);
    publish_status("error_timeout");
    stop_movement("Timeout");
    return;
  }

  unsigned long now = micros();
  if (now - last_step_time >= (unsigned long)step_delay_us) {
    last_step_time = now;

    if (current_position < target_position) {
      current_position++;
    } else {
      current_position--;
    }

    current_position = constrain(current_position, 0, steps_per_revolution);
    step_motor();

    if (++steps_since_last_save >= STEPS_BETWEEN_SAVES) {
      save_position();
      steps_since_last_save = 0;
    }

    if (current_position == target_position) {
      log_msg(LOG_INFO, "MOTOR", "Movement complete, position %d (%d%%)",
              current_position, (current_position * 100) / steps_per_revolution);
      stop_movement("Complete");
    }
  }

  if (millis() - last_position_report >= POSITION_REPORT_INTERVAL) {
    publish_position();
    last_position_report = millis();

    // Verbose TMC output during movement
    if (tmc_verbose && tmc_available) {
      uint16_t sg = driver.SG_RESULT();
      uint32_t drv = driver.DRV_STATUS();
      uint8_t cs = (drv >> 16) & 0x1F;  // Current scale
      log_msg(LOG_DEBUG, "MOTOR", "SG:%3d CS:%2d/31 DIAG:%d pos:%d",
              sg, cs, digitalRead(DIAG_PIN), current_position);
    }
  }
}

// ============================================================================
// CALIBRATION
// ============================================================================

void start_calibration() {
  if (!tmc_available) {
    log_msg(LOG_ERROR, "CAL", "Cannot calibrate: TMC2209 not available");
    return;
  }
  if (is_moving || cal_state != CAL_IDLE) {
    log_msg(LOG_WARN, "CAL", "Cannot calibrate: motor busy");
    return;
  }

  log_msg(LOG_INFO, "CAL", "Starting sensorless calibration — finding closed position...");

  wake_motor();
  stall_flag = false;
  cal_last_stall_check = millis();

  digitalWrite(DIR_PIN, LOW);  // Close direction
  delayMicroseconds(10);

  cal_state = CAL_FIND_MIN;
  is_moving = true;
  last_step_time = micros();
  cal_start_time = millis();
}

void handle_calibration() {
  if (cal_state == CAL_IDLE) return;

  // Periodic yield for WebSocket responsiveness
  static unsigned long last_cal_yield = 0;
  if (millis() - last_cal_yield >= 50) {
    yield();
    last_cal_yield = millis();
  }

  esp_task_wdt_reset();

  // Timeout
  if (millis() - cal_start_time > CAL_TIMEOUT) {
    log_msg(LOG_ERROR, "CAL", "Calibration timeout after %lums", CAL_TIMEOUT);
    cal_state = CAL_IDLE;
    is_moving = false;
    return;
  }

  // Step timing
  unsigned long now = micros();
  if (now - last_step_time >= (unsigned long)step_delay_us) {
    last_step_time = now;
    step_motor();

    if (cal_state == CAL_FIND_MAX) {
      current_position++;
    }

    // Check stall with debounce
    if (stall_flag) {
      if (millis() - cal_last_stall_check >= STALL_DEBOUNCE_MS) {
        // Log the stall detection
        if (tmc_available) {
          uint16_t sg = driver.SG_RESULT();
          log_msg(LOG_DEBUG, "CAL", "Stall detected SG:%d thr:%d", sg, stall_threshold);
        }

        if (cal_state == CAL_FIND_MIN) {
          log_msg(LOG_INFO, "CAL", "Found closed boundary");

          delay(30);
          yield();
          digitalWrite(DIR_PIN, HIGH);
          delayMicroseconds(10);

          // Back off from close wall — this becomes our safe "position 0"
          const int close_backoff = 30;
          for (int i = 0; i < close_backoff; i++) {
            step_motor();
            delayMicroseconds(step_delay_us);
            if (i % 10 == 0) yield();
          }
          current_position = 0;  // This backed-off spot IS position 0
          save_position();
          log_msg(LOG_INFO, "CAL", "Backed off %d steps, set as position 0", close_backoff);

          stall_flag = false;
          cal_last_stall_check = millis();
          cal_state = CAL_FIND_MAX;
          log_msg(LOG_INFO, "CAL", "Finding open position (max)...");

        } else if (cal_state == CAL_FIND_MAX) {
          int raw_travel = current_position;
          log_msg(LOG_INFO, "CAL", "Found open boundary at %d steps from safe-close", raw_travel);

          delay(30);
          yield();
          digitalWrite(DIR_PIN, LOW);
          delayMicroseconds(10);

          // Back off from open wall — this becomes our max position
          const int open_backoff = 30;
          for (int i = 0; i < open_backoff; i++) {
            step_motor();
            delayMicroseconds(step_delay_us);
            if (i % 10 == 0) yield();
          }

          // Usable range = raw travel minus the open back-off
          // (close back-off already accounted for in position 0)
          steps_per_revolution = raw_travel - open_backoff;
          current_position = steps_per_revolution;

          preferences.putInt("steps_per_rev", steps_per_revolution);
          save_position();

          log_msg(LOG_INFO, "CAL", "Calibration complete! Usable range: %d steps (raw: %d, margins: %d+%d)",
                  steps_per_revolution, raw_travel, 30, open_backoff);

          cal_state = CAL_IDLE;
          is_moving = false;
          publish_position();
          publish_ha_discovery(true);
        }
      }
    } else {
      cal_last_stall_check = millis();
    }
  }
}

// ============================================================================
// MQTT
// ============================================================================

void publish_status(const char* status) {
  if (client.connected()) {
    client.publish(mqtt_stat_topic.c_str(), status, true);
  }
}

void publish_position() {
  if (client.connected()) {
    int percentage = (current_position * 100) / steps_per_revolution;
    char pos_str[8];
    snprintf(pos_str, sizeof(pos_str), "%d", percentage);
    client.publish(mqtt_position_topic.c_str(), pos_str, true);
  }
}

void publish_ha_discovery(bool force) {
  if (!client.connected()) return;

  if (!force) {
    bool already_published = preferences.getBool("ha_disc_done", false);
    if (already_published) return;
  }

  String discovery_topic = "homeassistant/cover/" + device_hostname + "/config";

  StaticJsonDocument<1536> doc;

  doc["name"] = nullptr;
  doc["unique_id"] = "curtain_" + device_hostname;
  doc["object_id"] = device_hostname;

  doc["command_topic"] = mqtt_command_topic;
  doc["state_topic"] = mqtt_stat_topic;
  doc["position_topic"] = mqtt_position_topic;
  doc["set_position_topic"] = mqtt_command_topic;
  doc["availability_topic"] = mqtt_availability_topic;

  doc["payload_open"] = "open";
  doc["payload_close"] = "close";
  doc["payload_stop"] = "stop";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";

  doc["state_open"] = "open";
  doc["state_opening"] = "opening";
  doc["state_closed"] = "closed";
  doc["state_closing"] = "closing";
  doc["state_stopped"] = "stopped";

  doc["position_open"] = 100;
  doc["position_closed"] = 0;

  doc["optimistic"] = false;
  doc["qos"] = 1;
  doc["retain"] = true;
  doc["device_class"] = "curtain";

  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add("curtain_" + WiFi.macAddress());

  device["name"] = device_hostname;
  device["model"] = "CurtainController-TMC2209";
  device["manufacturer"] = "DIY";
  device["sw_version"] = FW_VERSION;
  device["configuration_url"] = "http://" + WiFi.localIP().toString() + "/setup";

  String json;
  serializeJson(doc, json);

  log_msg(LOG_DEBUG, "MQTT", "HA Discovery payload: %d bytes", json.length());

  if (client.publish(discovery_topic.c_str(), json.c_str(), true)) {
    preferences.putBool("ha_disc_done", true);
    log_msg(LOG_INFO, "MQTT", "HA Discovery published");
  } else {
    log_msg(LOG_ERROR, "MQTT", "HA Discovery publish FAILED");
  }

  // Publish calibrate button entity
  String cal_discovery_topic = "homeassistant/button/" + device_hostname + "_calibrate/config";

  StaticJsonDocument<512> cal_doc;
  cal_doc["name"] = "Calibrate";
  cal_doc["unique_id"] = "curtain_" + device_hostname + "_calibrate";
  cal_doc["object_id"] = device_hostname + "_calibrate";
  cal_doc["command_topic"] = mqtt_calibrate_topic;
  cal_doc["payload_press"] = "press";
  cal_doc["availability_topic"] = mqtt_availability_topic;
  cal_doc["payload_available"] = "online";
  cal_doc["payload_not_available"] = "offline";
  cal_doc["icon"] = "mdi:tape-measure";

  JsonObject cal_device = cal_doc.createNestedObject("device");
  JsonArray cal_ids = cal_device.createNestedArray("identifiers");
  cal_ids.add("curtain_" + WiFi.macAddress());
  cal_device["name"] = device_hostname;

  String cal_json;
  serializeJson(cal_doc, cal_json);

  if (client.publish(cal_discovery_topic.c_str(), cal_json.c_str(), true)) {
    log_msg(LOG_INFO, "MQTT", "HA Calibrate button published");
  } else {
    log_msg(LOG_ERROR, "MQTT", "HA Calibrate button publish FAILED");
  }

  // Speed number entity
  {
    String topic = "homeassistant/number/" + device_hostname + "_speed/config";
    StaticJsonDocument<512> doc;
    doc["name"] = "Speed";
    doc["unique_id"] = "curtain_" + device_hostname + "_speed";
    doc["object_id"] = device_hostname + "_speed";
    doc["command_topic"] = mqtt_speed_set_topic;
    doc["state_topic"] = mqtt_speed_state_topic;
    doc["min"] = 100;
    doc["max"] = 10000;
    doc["step"] = 100;
    doc["unit_of_measurement"] = "us";
    doc["icon"] = "mdi:speedometer";
    doc["availability_topic"] = mqtt_availability_topic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids = dev.createNestedArray("identifiers");
    ids.add("curtain_" + WiFi.macAddress());
    dev["name"] = device_hostname;
    String json;
    serializeJson(doc, json);
    client.publish(topic.c_str(), json.c_str(), true);
  }

  // Motor current number entity
  {
    String topic = "homeassistant/number/" + device_hostname + "_current/config";
    StaticJsonDocument<512> doc;
    doc["name"] = "Motor Current";
    doc["unique_id"] = "curtain_" + device_hostname + "_current";
    doc["object_id"] = device_hostname + "_current";
    doc["command_topic"] = mqtt_current_set_topic;
    doc["state_topic"] = mqtt_current_state_topic;
    doc["min"] = 100;
    doc["max"] = 2000;
    doc["step"] = 100;
    doc["unit_of_measurement"] = "mA";
    doc["icon"] = "mdi:current-ac";
    doc["availability_topic"] = mqtt_availability_topic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids = dev.createNestedArray("identifiers");
    ids.add("curtain_" + WiFi.macAddress());
    dev["name"] = device_hostname;
    String json;
    serializeJson(doc, json);
    client.publish(topic.c_str(), json.c_str(), true);
  }

  // Sensitivity select entity
  {
    String topic = "homeassistant/select/" + device_hostname + "_sensitivity/config";
    StaticJsonDocument<512> doc;
    doc["name"] = "Stall Sensitivity";
    doc["unique_id"] = "curtain_" + device_hostname + "_sensitivity";
    doc["object_id"] = device_hostname + "_sensitivity";
    doc["command_topic"] = mqtt_stallthreshold_set_topic;
    doc["state_topic"] = mqtt_stallthreshold_state_topic;
    JsonArray options = doc.createNestedArray("options");
    options.add("extra_low"); options.add("low"); options.add("medium"); options.add("high"); options.add("max");
    doc["icon"] = "mdi:gauge";
    doc["availability_topic"] = mqtt_availability_topic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids = dev.createNestedArray("identifiers");
    ids.add("curtain_" + WiFi.macAddress());
    dev["name"] = device_hostname;
    String json;
    serializeJson(doc, json);
    client.publish(topic.c_str(), json.c_str(), true);
  }

  // Microsteps select entity
  {
    String topic = "homeassistant/select/" + device_hostname + "_microsteps/config";
    StaticJsonDocument<512> doc;
    doc["name"] = "Microsteps";
    doc["unique_id"] = "curtain_" + device_hostname + "_microsteps";
    doc["object_id"] = device_hostname + "_microsteps";
    doc["command_topic"] = mqtt_microsteps_set_topic;
    doc["state_topic"] = mqtt_microsteps_state_topic;
    JsonArray options = doc.createNestedArray("options");
    options.add("1"); options.add("2"); options.add("4"); options.add("8");
    options.add("16"); options.add("32"); options.add("64"); options.add("128"); options.add("256");
    doc["icon"] = "mdi:stairs";
    doc["availability_topic"] = mqtt_availability_topic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids = dev.createNestedArray("identifiers");
    ids.add("curtain_" + WiFi.macAddress());
    dev["name"] = device_hostname;
    String json;
    serializeJson(doc, json);
    client.publish(topic.c_str(), json.c_str(), true);
  }

  // Invert direction switch entity
  {
    String topic = "homeassistant/switch/" + device_hostname + "_invert/config";
    StaticJsonDocument<512> doc;
    doc["name"] = "Invert Direction";
    doc["unique_id"] = "curtain_" + device_hostname + "_invert";
    doc["object_id"] = device_hostname + "_invert";
    doc["command_topic"] = mqtt_invert_set_topic;
    doc["state_topic"] = mqtt_invert_state_topic;
    doc["icon"] = "mdi:swap-horizontal";
    doc["availability_topic"] = mqtt_availability_topic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids = dev.createNestedArray("identifiers");
    ids.add("curtain_" + WiFi.macAddress());
    dev["name"] = device_hostname;
    String json;
    serializeJson(doc, json);
    client.publish(topic.c_str(), json.c_str(), true);
  }

  // Remove old stallthreshold number entity (replaced by sensitivity select)
  String old_topic = "homeassistant/number/" + device_hostname + "_stallthreshold/config";
  client.publish(old_topic.c_str(), "", true);

  log_msg(LOG_INFO, "MQTT", "HA settings entities published");
}

void publish_settings_state() {
  if (!client.connected()) return;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", step_delay_us);
  client.publish(mqtt_speed_state_topic.c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%d", motor_current_ma);
  client.publish(mqtt_current_state_topic.c_str(), buf, true);
  client.publish(mqtt_stallthreshold_state_topic.c_str(), sensitivity_name(stall_threshold), true);
  snprintf(buf, sizeof(buf), "%d", motor_microsteps);
  client.publish(mqtt_microsteps_state_topic.c_str(), buf, true);
  client.publish(mqtt_invert_state_topic.c_str(), invert_direction ? "ON" : "OFF", true);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg((char*)payload, length);
  msg.trim();
  msg.toLowerCase();
  log_msg(LOG_DEBUG, "MQTT", "Received on %s: %s", topic, msg.c_str());

  // Settings topics
  if (strcmp(topic, mqtt_speed_set_topic.c_str()) == 0) {
    int value = msg.toInt();
    if (value >= 100 && value <= 10000) {
      step_delay_us = value;
      preferences.putInt("step_delay", step_delay_us);
      log_msg(LOG_INFO, "MQTT", "Speed set to %d us", step_delay_us);
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", step_delay_us);
      client.publish(mqtt_speed_state_topic.c_str(), buf, true);
    }
    return;
  }
  if (strcmp(topic, mqtt_current_set_topic.c_str()) == 0) {
    int value = msg.toInt();
    if (value >= 100 && value <= 2000) {
      set_motor_current(value);
      log_msg(LOG_INFO, "MQTT", "Current set to %d mA", motor_current_ma);
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", motor_current_ma);
      client.publish(mqtt_current_state_topic.c_str(), buf, true);
    }
    return;
  }
  if (strcmp(topic, mqtt_stallthreshold_set_topic.c_str()) == 0) {
    uint8_t value;
    if (msg == "extra_low") value = 5;
    else if (msg == "low") value = 15;
    else if (msg == "medium") value = 30;
    else if (msg == "high") value = 60;
    else if (msg == "max") value = 100;
    else return;
    set_stall_threshold(value);
    log_msg(LOG_INFO, "MQTT", "Sensitivity set to %s (threshold=%d)", msg.c_str(), stall_threshold);
    publish_sensitivity_state();
    return;
  }
  if (strcmp(topic, mqtt_microsteps_set_topic.c_str()) == 0) {
    int value = msg.toInt();
    if (value == 1 || value == 2 || value == 4 || value == 8 || value == 16 ||
        value == 32 || value == 64 || value == 128 || value == 256) {
      set_motor_microsteps(value);
      log_msg(LOG_INFO, "MQTT", "Microsteps set to %d", motor_microsteps);
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", motor_microsteps);
      client.publish(mqtt_microsteps_state_topic.c_str(), buf, true);
    }
    return;
  }

  // Invert direction switch
  if (strcmp(topic, mqtt_invert_set_topic.c_str()) == 0) {
    bool value = (msg == "on" || msg == "true" || msg == "1");
    invert_direction = value;
    preferences.putBool("invert_dir", invert_direction);
    if (tmc_available) driver.shaft(invert_direction);
    log_msg(LOG_INFO, "MQTT", "Direction invert set to %s", invert_direction ? "ON" : "OFF");
    client.publish(mqtt_invert_state_topic.c_str(), invert_direction ? "ON" : "OFF", true);
    return;
  }

  // Calibrate button topic
  if (strcmp(topic, mqtt_calibrate_topic.c_str()) == 0) {
    if (msg == "press") {
      cmd_calibrate("");
    }
    return;
  }

  // Ignore retained messages delivered right after subscribing
  if (millis() - mqtt_subscribe_time < MQTT_IGNORE_RETAINED_MS) {
    log_msg(LOG_DEBUG, "MQTT", "Ignoring retained command: %s", msg.c_str());
    return;
  }

  // Only allow safe commands via MQTT: open/close/stop and bare percentages
  if (msg == "open" || msg == "close" || msg == "stop") {
    process_command(msg);
    return;
  }

  // Allow bare percentage numbers (HA position control)
  if (msg.length() > 0) {
    bool all_digits = true;
    for (size_t i = 0; i < msg.length(); i++) {
      if (!isDigit(msg.charAt(i))) { all_digits = false; break; }
    }
    if (all_digits) {
      process_command(msg);
      return;
    }
  }

  log_msg(LOG_WARN, "MQTT", "Ignored unknown command: %s", msg.c_str());
}

void connect_mqtt() {
  if (client.connected()) return;

  if (millis() - last_mqtt_attempt < mqtt_retry_delay) return;

  last_mqtt_attempt = millis();

  char client_id[80];
  {
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    snprintf(client_id, sizeof(client_id), "%s_%s", device_hostname.c_str(), mac.c_str());
  }

  bool connected;
  if (mqtt_user.length() > 0) {
    connected = client.connect(client_id, mqtt_user.c_str(), mqtt_password.c_str(),
                              mqtt_availability_topic.c_str(), 1, true, "offline");
  } else {
    connected = client.connect(client_id, mqtt_availability_topic.c_str(), 1, true, "offline");
  }

  if (connected) {
    log_msg(LOG_INFO, "MQTT", "Connected to %s:%d", mqtt_server.c_str(), mqtt_port);
    mqtt_subscribe_time = millis();
    client.subscribe(mqtt_command_topic.c_str());
    client.subscribe(mqtt_calibrate_topic.c_str());
    client.subscribe(mqtt_speed_set_topic.c_str());
    client.subscribe(mqtt_current_set_topic.c_str());
    client.subscribe(mqtt_stallthreshold_set_topic.c_str());
    client.subscribe(mqtt_microsteps_set_topic.c_str());
    client.subscribe(mqtt_invert_set_topic.c_str());
    client.publish(mqtt_availability_topic.c_str(), "online", true);
    publish_position();
    publish_status(current_position >= steps_per_revolution ? "open" :
                   current_position <= 0 ? "closed" : "open");
    publish_ha_discovery();
    publish_settings_state();
    mqtt_retry_delay = 2000;
  } else {
    log_msg(LOG_WARN, "MQTT", "Connection failed (rc=%d), retry in %dms", client.state(), mqtt_retry_delay);
    mqtt_retry_delay = min(mqtt_retry_delay * 2, MAX_MQTT_RETRY_DELAY);
  }
}

void setup_mqtt() {
  mqtt_server = preferences.getString("mqtt_server", "192.168.1.100");
  mqtt_port = preferences.getInt("mqtt_port", 1883);
  mqtt_user = preferences.getString("mqtt_user", "your_mqtt_user");
  mqtt_password = preferences.getString("mqtt_pass", "your_mqtt_password");

  String mqtt_root_topic = preferences.getString("mqtt_root_topic", "home/room/curtains");
  mqtt_command_topic = mqtt_root_topic + "/cmd";
  mqtt_stat_topic = mqtt_root_topic + "/status";
  mqtt_position_topic = mqtt_root_topic + "/position";
  mqtt_availability_topic = mqtt_root_topic + "/availability";
  mqtt_calibrate_topic = mqtt_root_topic + "/calibrate";
  mqtt_speed_set_topic = mqtt_root_topic + "/speed/set";
  mqtt_speed_state_topic = mqtt_root_topic + "/speed/state";
  mqtt_current_set_topic = mqtt_root_topic + "/current/set";
  mqtt_current_state_topic = mqtt_root_topic + "/current/state";
  mqtt_stallthreshold_set_topic = mqtt_root_topic + "/stallthreshold/set";
  mqtt_stallthreshold_state_topic = mqtt_root_topic + "/stallthreshold/state";
  mqtt_microsteps_set_topic = mqtt_root_topic + "/microsteps/set";
  mqtt_microsteps_state_topic = mqtt_root_topic + "/microsteps/state";
  mqtt_invert_set_topic = mqtt_root_topic + "/invert/set";
  mqtt_invert_state_topic = mqtt_root_topic + "/invert/state";

  client.setBufferSize(2048);
  client.setServer(mqtt_server.c_str(), mqtt_port);
  client.setCallback(mqtt_callback);

  connect_mqtt();
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void cmd_open(const String& param) {
  log_msg(LOG_INFO, "CMD", "open");
  start_movement(steps_per_revolution);
}

void cmd_close(const String& param) {
  log_msg(LOG_INFO, "CMD", "close");
  start_movement(0);
}

void cmd_stop(const String& param) {
  log_msg(LOG_INFO, "CMD", "stop");
  if (cal_state != CAL_IDLE) {
    cal_state = CAL_IDLE;
    log_msg(LOG_INFO, "CAL", "Calibration cancelled by user");
  }
  stop_movement("User command");
}

void cmd_speed(const String& param) {
  int value = param.toInt();
  if (value >= 100 && value <= 10000) {
    step_delay_us = value;
    preferences.putInt("step_delay", step_delay_us);
    log_msg(LOG_INFO, "NVS", "Speed set to %d us/step", step_delay_us);
    if (client.connected()) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", step_delay_us);
      client.publish(mqtt_speed_state_topic.c_str(), buf, true);
    }
  } else {
    log_msg(LOG_ERROR, "CMD", "speed: value must be 100-10000 us (got %d)", value);
  }
}

void cmd_microsteps(const String& param) {
  int value = param.toInt();
  if (value == 1 || value == 2 || value == 4 || value == 8 ||
      value == 16 || value == 32 || value == 64 || value == 128 || value == 256) {
    set_motor_microsteps(value);
    log_msg(LOG_INFO, "NVS", "Microsteps set to %d", motor_microsteps);
    if (client.connected()) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", motor_microsteps);
      client.publish(mqtt_microsteps_state_topic.c_str(), buf, true);
    }
  } else {
    log_msg(LOG_ERROR, "CMD", "microsteps: must be 1,2,4,8,16,32,64,128,256 (got %d)", value);
  }
}

void cmd_current(const String& param) {
  int value = param.toInt();
  if (value >= 100 && value <= 2000) {
    set_motor_current(value);
    log_msg(LOG_INFO, "NVS", "Current set to %d mA", motor_current_ma);
    if (client.connected()) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", motor_current_ma);
      client.publish(mqtt_current_state_topic.c_str(), buf, true);
    }
  } else {
    log_msg(LOG_ERROR, "CMD", "current: must be 100-2000 mA (got %d)", value);
  }
}

// Map sensitivity name to SGTHRS value
static const char* sensitivity_name(uint8_t thr) {
  if (thr <= 8) return "extra_low";
  if (thr <= 20) return "low";
  if (thr <= 45) return "medium";
  if (thr <= 80) return "high";
  if (thr <= 120) return "max";
  return "custom";
}

void publish_sensitivity_state() {
  if (!client.connected()) return;
  client.publish(mqtt_stallthreshold_state_topic.c_str(), sensitivity_name(stall_threshold), true);
}

void apply_sensitivity(uint8_t value, const char* label) {
  set_stall_threshold(value);
  log_msg(LOG_INFO, "NVS", "Sensitivity set to %s (threshold=%d)", label, stall_threshold);
  publish_sensitivity_state();
}

void cmd_sensitivity(const String& param) {
  String p = param;
  p.trim();
  p.toLowerCase();

  if (p == "extra_low" || p == "extralow" || p == "xlow") {
    apply_sensitivity(5, "extra_low");
  } else if (p == "low") {
    apply_sensitivity(15, "low");
  } else if (p == "medium" || p == "med") {
    apply_sensitivity(30, "medium");
  } else if (p == "high") {
    apply_sensitivity(60, "high");
  } else if (p == "max") {
    apply_sensitivity(100, "max");
  } else if (p.startsWith("custom ")) {
    int value = p.substring(7).toInt();
    if (value >= 0 && value <= 255) {
      apply_sensitivity(value, "custom");
    } else {
      log_msg(LOG_ERROR, "CMD", "sensitivity custom: must be 0-255 (got %d)", value);
    }
  } else if (p.length() == 0) {
    // No argument — show current
    output("Sensitivity: %s (threshold=%d, stalls when load drops below %d)\n",
           sensitivity_name(stall_threshold), stall_threshold, stall_threshold * 2);
  } else {
    log_msg(LOG_ERROR, "CMD", "sensitivity: use extra_low, low, medium, high, max, or custom <0-255>");
  }
}

void cmd_calibrate(const String& param) {
  log_msg(LOG_INFO, "CMD", "calibrate — ensure curtains can move freely");
  start_calibration();
}

void cmd_travelsteps(const String& param) {
  int value = param.toInt();
  if (value > 0 && value <= 500000) {
    steps_per_revolution = value;
    preferences.putInt("steps_per_rev", steps_per_revolution);
    log_msg(LOG_INFO, "NVS", "Total travel steps set to %d", steps_per_revolution);
  } else {
    log_msg(LOG_ERROR, "CMD", "travelsteps: must be 1-500000 (got %d)", value);
  }
}

void cmd_setposition(const String& param) {
  int value = param.toInt();
  if (value >= 0 && value <= steps_per_revolution) {
    current_position = value;
    save_position();
    publish_position();
    log_msg(LOG_INFO, "NVS", "Position reset to %d", current_position);
  } else {
    log_msg(LOG_ERROR, "CMD", "setposition: must be 0-%d (got %d)", steps_per_revolution, value);
  }
}

void cmd_sleep(const String& param) {
  int value = param.toInt();
  if (value >= 0 && value <= 300000) {
    motor_sleep_timeout = value;
    preferences.putULong("sleep_timeout", motor_sleep_timeout);
    log_msg(LOG_INFO, "NVS", "Sleep timeout set to %lu ms", motor_sleep_timeout);
  } else {
    log_msg(LOG_ERROR, "CMD", "sleep: must be 0-300000 ms (got %d)", value);
  }
}

void cmd_hadiscovery(const String& param) {
  log_msg(LOG_INFO, "CMD", "hadiscovery — forcing republish");
  preferences.putBool("ha_disc_done", false);
  publish_ha_discovery(true);
}

void cmd_config(const String& param) {
  String mqtt_topic = preferences.getString("mqtt_root_topic", "home/room/curtains");
  String out;
  out.reserve(700);

  buf_printf(out, "\n=== Configuration ===\n");
  buf_printf(out, "Hostname: %s\n", device_hostname.c_str());
  buf_printf(out, "IP: %s\n", WiFi.localIP().toString().c_str());
  buf_printf(out, "SSID: %s\n", WiFi.SSID().c_str());
  buf_printf(out, "RSSI: %d dBm\n", WiFi.RSSI());
  buf_printf(out, "MAC: %s\n", WiFi.macAddress().c_str());
  buf_printf(out, "MQTT: %s:%d\n", mqtt_server.c_str(), mqtt_port);
  buf_printf(out, "MQTT User: %s\n", mqtt_user.length() > 0 ? mqtt_user.c_str() : "(none)");
  buf_printf(out, "MQTT Topic: %s\n", mqtt_topic.c_str());
  buf_printf(out, "Speed: %d us/step (lower=faster)\n", step_delay_us);
  buf_printf(out, "Microsteps: %d\n", motor_microsteps);
  buf_printf(out, "Current: %d mA\n", motor_current_ma);
  buf_printf(out, "Sensitivity: %s (threshold=%d)\n", sensitivity_name(stall_threshold), stall_threshold);
  buf_printf(out, "Travel Steps: %d\n", steps_per_revolution);
  buf_printf(out, "Invert Direction: %s\n", invert_direction ? "YES" : "NO");
  buf_printf(out, "Sleep Timeout: %lu ms\n", motor_sleep_timeout);
  buf_printf(out, "TMC2209: %s\n", tmc_available ? "OK" : "NOT CONNECTED");
  buf_printf(out, "Log level: %s\n", log_level_name(current_log_level));
  buf_printf(out, "Setup: http://%s/setup\n", WiFi.localIP().toString().c_str());
  buf_printf(out, "====================\n");
  ws_send_bulk(out);
}

void cmd_invert(const String& param) {
  invert_direction = !invert_direction;
  preferences.putBool("invert_dir", invert_direction);
  if (tmc_available) driver.shaft(invert_direction);
  log_msg(LOG_INFO, "CMD", "Direction invert: %s", invert_direction ? "ON" : "OFF");
  if (client.connected())
    client.publish(mqtt_invert_state_topic.c_str(), invert_direction ? "ON" : "OFF", true);
}

void cmd_restart(const String& param) {
  log_msg(LOG_INFO, "SYS", "Restarting by command...");
  delay(1000);
  ESP.restart();
}

void cmd_help(const String& param) {
  static const char help_text[] PROGMEM =
    "\n=== Movement ===\n"
    "open              Open curtain\n"
    "close             Close curtain\n"
    "stop              Stop movement / cancel calibrate\n"
    "<0-100>           Move to percentage\n"
    "\n=== Settings ===\n"
    "speed <us>        Step delay (100-10000, lower=faster)\n"
    "current <mA>      Motor current (100-2000)\n"
    "microsteps <n>    Microsteps (1,2,4,8,16,32,64,128,256)\n"
    "sensitivity <lvl> Stall sensitivity (extra_low|low|medium|high|max|custom N)\n"
    "invert            Toggle open/close direction\n"
    "sleep <ms>        Motor sleep timeout (0=never)\n"
    "travelsteps <n>   Total travel range in steps\n"
    "\n=== Calibration ===\n"
    "calibrate         Find curtain travel range automatically\n"
    "motortest [sec]   Test motor and check sensitivity (default 5s)\n"
    "\n=== Diagnostics ===\n"
    "status            Position, motor, MQTT, TMC status\n"
    "config            Full configuration dump\n"
    "verbose           Toggle SG debug during movement\n"
    "loglevel <level>  Set log level (error|warn|info|debug)\n"
    "ledon / ledoff    Manual LED control\n"
    "\n=== System ===\n"
    "setposition <n>   Override position counter (use with care)\n"
    "hadiscovery       Republish HA discovery\n"
    "restart           Reboot device\n";
  ws_send_bulk(String(help_text));
}

void cmd_status(const String& param) {
  String out;
  out.reserve(600);

  buf_printf(out, "\n=== Status ===\n");
  buf_printf(out, "Position: %d (%d%%)\n", current_position,
         (current_position * 100) / steps_per_revolution);
  buf_printf(out, "Moving: %s\n", is_moving ? "Yes" : "No");
  if (is_moving) {
    buf_printf(out, "Target: %d\n", target_position);
  }
  buf_printf(out, "Motor: %s\n", motor_enabled ? "Enabled" : "Disabled");
  buf_printf(out, "MQTT: %s\n", client.connected() ? "Connected" : "Disconnected");

  if (tmc_available) {
    uint32_t drv_status = driver.DRV_STATUS();
    uint16_t sg = driver.SG_RESULT();
    buf_printf(out, "-- TMC2209 --\n");
    buf_printf(out, "SG_RESULT: %d\n", sg);
    buf_printf(out, "Current scale: %d/31\n", (drv_status >> 16) & 0x1F);
    buf_printf(out, "Standstill: %s\n", (drv_status >> 31) & 1 ? "Yes" : "No");
    buf_printf(out, "OT warning: %s\n", (drv_status >> 0) & 1 ? "Yes" : "No");
    buf_printf(out, "DIAG pin: %s\n", digitalRead(DIAG_PIN) ? "HIGH" : "LOW");
  } else {
    buf_printf(out, "TMC2209: NOT CONNECTED\n");
  }

  buf_printf(out, "-- System --\n");
  buf_printf(out, "Heap: %u bytes\n", ESP.getFreeHeap());
  unsigned long uptime = millis() / 1000;
  buf_printf(out, "Uptime: %lud %luh %lum %lus\n", uptime / 86400, (uptime % 86400) / 3600, (uptime % 3600) / 60, uptime % 60);
  buf_printf(out, "Last reset: %s\n", last_reset_reason);
  buf_printf(out, "Log level: %s\n", log_level_name(current_log_level));
  buf_printf(out, "==============\n");
  ws_send_bulk(out);
}

void cmd_verbose(const String& param) {
  tmc_verbose = !tmc_verbose;
  log_msg(LOG_INFO, "TMC", "Verbose mode %s", tmc_verbose ? "ON" : "OFF");
  if (tmc_verbose) {
    log_msg(LOG_INFO, "TMC", "Motor load readings will appear during movement (set loglevel debug)");
  }
}

void cmd_motortest(const String& param) {
  if (!tmc_available) {
    output("TMC2209 not available\n");
    return;
  }

  int duration = param.length() > 0 ? param.toInt() : 5;
  duration = constrain(duration, 1, 60);

  unsigned long test_step_delay = step_delay_us;
  int steps_between_reads = constrain(motor_microsteps, 4, 32);

  output("\n=== Motor Test ===\n");
  output("Duration: %d seconds\n", duration);
  output("Sensitivity: %s (threshold=%d)\n", sensitivity_name(stall_threshold), stall_threshold);
  output("Apply resistance to the shaft to test stall detection.\n\n");

  wake_motor();
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(100);
  stall_flag = false;

  // Ramp up (500ms)
  output("Ramping up...\n");
  unsigned long ramp_start = millis();
  unsigned long last_step = micros();
  unsigned long current_delay = test_step_delay * 3;
  unsigned long last_yield = millis();

  while (millis() - ramp_start < 500) {
    unsigned long now = micros();
    if (now - last_step >= current_delay) {
      step_motor();
      last_step = now;
      if (current_delay > test_step_delay) {
        current_delay -= 2;
        if (current_delay < test_step_delay) current_delay = test_step_delay;
      }
    }
    if (millis() - last_yield >= 50) {
      yield();
      esp_task_wdt_reset();
      last_yield = millis();
    }
  }

  output("Running...\n\n");

  // Measurement phase
  unsigned long test_start = millis();
  uint16_t min_sg = 1023, max_sg = 0;
  uint32_t sg_sum = 0, sg_count = 0;
  int step_count = 0, stall_events = 0;
  unsigned long last_report = 0;
  uint16_t recent_sg[8] = {0};
  int recent_idx = 0;

  while (millis() - test_start < (unsigned long)(duration * 1000)) {
    unsigned long now = micros();

    if (now - last_step >= test_step_delay) {
      step_motor();
      last_step = now;
      step_count++;

      if (step_count >= steps_between_reads) {
        step_count = 0;
        uint16_t sg = driver.SG_RESULT();

        if (sg < min_sg) min_sg = sg;
        if (sg > max_sg) max_sg = sg;
        sg_sum += sg;
        sg_count++;

        recent_sg[recent_idx] = sg;
        recent_idx = (recent_idx + 1) % 8;

        if (sg < (uint16_t)(stall_threshold * 2)) {
          stall_events++;
        }
      }
    }

    unsigned long now_ms = millis();
    if (now_ms - last_report >= 500) {
      last_report = now_ms;

      uint32_t recent_sum = 0;
      for (int i = 0; i < 8; i++) recent_sum += recent_sg[i];
      uint16_t avg = recent_sum / 8;
      uint16_t latest = recent_sg[(recent_idx + 7) % 8];

      // Visual load bar: map SG to 0-16 blocks (higher SG = more blocks = lighter load)
      int blocks = (avg > 0) ? constrain(avg / 40, 0, 16) : 0;
      char bar[17];
      for (int i = 0; i < 16; i++) bar[i] = (i < blocks) ? '#' : '.';
      bar[16] = 0;

      const char* load_label;
      if (latest < (uint16_t)(stall_threshold * 2)) load_label = "STALL!";
      else if (avg < 100) load_label = "Heavy";
      else if (avg < 300) load_label = "Medium";
      else load_label = "Light";

      output("  Load: %-6s [%s] SG:%d\n", load_label, bar, latest);
      esp_task_wdt_reset();
    }

    static unsigned long last_loop_yield = 0;
    unsigned long now_ms2 = millis();
    if (now_ms2 - last_loop_yield >= 50) {
      yield();
      last_loop_yield = now_ms2;
    }
  }

  stop_motor();

  // Results
  uint16_t avg_sg = sg_count > 0 ? (sg_sum / sg_count) : 0;

  output("\n=== Results ===\n");
  output("Average load: %s (SG avg: %d, range: %d-%d)\n",
         avg_sg < 100 ? "Heavy" : avg_sg < 300 ? "Medium" : "Light",
         avg_sg, min_sg, max_sg);
  output("Sensitivity: %s (threshold=%d)\n", sensitivity_name(stall_threshold), stall_threshold);
  output("False stalls during test: %d\n", stall_events);

  if (stall_events > 5) {
    output("\nProblem: Too many false stalls detected.\n");
    output("Try:  sensitivity low   (less sensitive, ignores light friction)\n");
  } else if (stall_events > 0) {
    output("\nSome false stalls detected. This might cause calibration issues.\n");
    output("Try:  sensitivity low   if calibration stops too early\n");
  } else {
    output("\nNo false stalls. Current sensitivity looks good for calibration.\n");
  }

  if (avg_sg < 50) {
    output("\nWarning: Motor is under heavy load even when free.\n");
    output("Check: Is something blocking the curtain?\n");
    output("Try:   current %d   (increase motor power)\n", constrain(motor_current_ma + 200, 100, 2000));
  }

  output("===============\n");
}

void cmd_loglevel(const String& param) {
  LogLevel new_level;
  if (param == "error") {
    new_level = LOG_ERROR;
  } else if (param == "warn") {
    new_level = LOG_WARN;
  } else if (param == "info") {
    new_level = LOG_INFO;
  } else if (param == "debug") {
    new_level = LOG_DEBUG;
  } else {
    log_msg(LOG_ERROR, "CMD", "loglevel: unknown level '%s' (use error|warn|info|debug)", param.c_str());
    return;
  }
  current_log_level = new_level;
  preferences.putUChar("log_level", (uint8_t)new_level);
  log_msg(LOG_INFO, "NVS", "Log level set to %s", log_level_name(new_level));
}

void cmd_ledon(const String& param) {
  led_manual_control = true;
  led_desired_state = LOW;
  digitalWrite(STATUS_LED, led_desired_state);
  log_msg(LOG_INFO, "CMD", "LED ON");
}

void cmd_ledoff(const String& param) {
  led_manual_control = true;
  led_desired_state = HIGH;
  digitalWrite(STATUS_LED, led_desired_state);
  log_msg(LOG_INFO, "CMD", "LED OFF");
}

struct Command {
  const char* name;
  void (*handler)(const String& param);
};

const Command commands[] = {
  {"open", cmd_open},
  {"close", cmd_close},
  {"stop", cmd_stop},
  {"speed ", cmd_speed},
  {"microsteps ", cmd_microsteps},
  {"current ", cmd_current},
  {"sensitivity ", cmd_sensitivity},
  {"sensitivity", cmd_sensitivity},
  {"calibrate", cmd_calibrate},
  {"travelsteps ", cmd_travelsteps},
  {"setposition ", cmd_setposition},
  {"invert", cmd_invert},
  {"sleep ", cmd_sleep},
  {"hadiscovery", cmd_hadiscovery},
  {"config", cmd_config},
  {"status", cmd_status},
  {"verbose", cmd_verbose},
  {"motortest ", cmd_motortest},
  {"motortest", cmd_motortest},
  {"loglevel ", cmd_loglevel},
  {"restart", cmd_restart},
  {"help", cmd_help},
  {"ledon", cmd_ledon},
  {"ledoff", cmd_ledoff},
  {nullptr, nullptr}
};

void process_command(const String& cmd) {
  String command = cmd;
  command.trim();
  command.toLowerCase();

  if (command.length() == 0) return;

  const char* input = command.c_str();
  size_t input_len = command.length();

  for (int i = 0; commands[i].name != nullptr; i++) {
    const char* name = commands[i].name;
    size_t name_len = strlen(name);

    if (name[name_len - 1] == ' ') {
      if (input_len >= name_len && strncmp(input, name, name_len) == 0) {
        commands[i].handler(command.substring(name_len));
        return;
      }
    } else {
      if (input_len == name_len && strcmp(input, name) == 0) {
        commands[i].handler("");
        return;
      }
    }
  }

  // Check if it's a plain number (HA sends percentage directly)
  bool is_numeric = true;
  for (size_t i = 0; i < command.length(); i++) {
    if (!isDigit(command.charAt(i))) {
      is_numeric = false;
      break;
    }
  }

  if (is_numeric && command.length() > 0) {
    int percentage = command.toInt();
    if (percentage >= 0 && percentage <= 100) {
      int target_steps = (percentage * steps_per_revolution) / 100;
      log_msg(LOG_INFO, "CMD", "HA position: %d%% -> step %d", percentage, target_steps);
      start_movement(target_steps);
      return;
    }
  }

  log_msg(LOG_ERROR, "CMD", "Unknown command: %s", command.c_str());
}

// ============================================================================
// WEBSERIAL & WEB SERVER
// ============================================================================

const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1.0,user-scalable=no">
  <title>Curtain Controller</title>
  <style>
    *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
    :root{--cyan:#00D4FF;--purple:#6366F1;--gradient:linear-gradient(135deg,var(--cyan),var(--purple));--bg:#0a0a1a;--card:rgba(255,255,255,0.03);--border:rgba(255,255,255,0.08);--text:#fff;--dim:rgba(255,255,255,0.5);--success:#10B981}
    body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;margin:0;padding:20px;background:var(--bg);background-image:radial-gradient(ellipse at top left,rgba(0,212,255,0.1) 0%,transparent 50%),radial-gradient(ellipse at bottom right,rgba(99,102,241,0.1) 0%,transparent 50%);color:var(--text);min-height:100vh;display:flex;flex-direction:column;align-items:center}
    .container{max-width:420px;width:100%}
    .header{display:flex;align-items:center;justify-content:center;gap:14px;margin-bottom:8px}
    .header svg{width:48px;height:48px;filter:drop-shadow(0 4px 12px rgba(0,212,255,0.3))}
    h1{font-size:26px;font-weight:700;margin:0;background:var(--gradient);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}
    .subtitle{text-align:center;color:var(--dim);font-size:13px;margin-bottom:24px}
    .card{background:var(--card);backdrop-filter:blur(20px);-webkit-backdrop-filter:blur(20px);border:1px solid var(--border);border-radius:16px;padding:20px;margin-bottom:16px}
    .card h2{font-size:15px;font-weight:600;margin:0 0 14px 0;background:var(--gradient);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}
    label{display:block;color:var(--dim);font-size:13px;font-weight:500;margin-bottom:4px;margin-top:12px}
    label:first-of-type{margin-top:0}
    input,select{width:100%;padding:10px 14px;border-radius:10px;border:1px solid var(--border);background:var(--card);color:var(--text);font-size:14px;font-weight:500;transition:border-color 0.2s}
    input:focus,select:focus{outline:none;border-color:var(--cyan);box-shadow:0 0 0 3px rgba(0,212,255,0.1)}
    .hint{color:var(--dim);font-size:11px;margin-top:2px}
    .btn-row{display:flex;gap:12px;margin-top:20px}
    .btn{flex:1;padding:14px;font-size:15px;border-radius:12px;border:none;cursor:pointer;font-weight:600;transition:all 0.2s;text-align:center;text-decoration:none}
    .btn-primary{background:var(--gradient);color:#fff;box-shadow:0 4px 16px rgba(0,212,255,0.3)}
    .btn-primary:hover{transform:translateY(-1px);box-shadow:0 6px 20px rgba(0,212,255,0.4)}
    .btn-secondary{background:var(--card);color:var(--dim);border:1px solid var(--border)}
    .btn-secondary:hover{border-color:var(--cyan);color:var(--text)}
    .version{text-align:center;color:rgba(255,255,255,0.15);font-size:10px;margin-top:16px}
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <svg viewBox="0 0 512 512" fill="none">
        <defs><linearGradient id="g" x1="0%" y1="0%" x2="100%" y2="100%"><stop offset="0%" stop-color="#00D4FF"/><stop offset="100%" stop-color="#6366F1"/></linearGradient></defs>
        <rect x="108" y="100" width="296" height="28" rx="4" fill="url(#g)"/>
        <rect x="108" y="86" width="32" height="56" rx="4" fill="url(#g)"/>
        <rect x="372" y="86" width="32" height="56" rx="4" fill="url(#g)"/>
        <rect x="140" y="140" width="26" height="300" rx="3" fill="url(#g)"/>
        <rect x="176" y="140" width="26" height="300" rx="3" fill="url(#g)"/>
        <rect x="212" y="140" width="26" height="300" rx="3" fill="url(#g)"/>
        <rect x="274" y="140" width="26" height="300" rx="3" fill="url(#g)"/>
        <rect x="310" y="140" width="26" height="300" rx="3" fill="url(#g)"/>
        <rect x="346" y="140" width="26" height="300" rx="3" fill="url(#g)"/>
      </svg>
      <h1>Curtain Controller</h1>
    </div>
    <div class="subtitle">TMC2209 Edition &middot; %HOSTNAME%</div>

    <form action="/save" method="POST">
      <div class="card">
        <h2>Network</h2>
        <label>Hostname</label>
        <input name="hostname" value="%HOSTNAME%">
        <label>MQTT Server</label>
        <input name="mqtt_server" value="%MQTT_SERVER%">
        <label>MQTT Port</label>
        <input name="mqtt_port" type="number" value="%MQTT_PORT%">
        <label>MQTT Username</label>
        <input name="mqtt_user" value="%MQTT_USER%">
        <label>MQTT Password</label>
        <input name="mqtt_pass" type="password" value="%MQTT_PASS%">
        <label>MQTT Root Topic</label>
        <input name="mqtt_topic" value="%MQTT_TOPIC%">
        <div class="hint">Creates: /cmd, /status, /position, /availability</div>
      </div>

      <div class="card">
        <h2>Motor</h2>
        <label>Travel Steps</label>
        <input name="steps" type="number" value="%STEPS%">
        <label>Current (mA)</label>
        <input name="current" type="number" value="%CURRENT%" min="100" max="2000">
        <label>Microsteps</label>
        <select name="microsteps">
          <option value="1" %MS1%>1</option><option value="2" %MS2%>2</option>
          <option value="4" %MS4%>4</option><option value="8" %MS8%>8</option>
          <option value="16" %MS16%>16</option><option value="32" %MS32%>32</option>
          <option value="64" %MS64%>64</option><option value="128" %MS128%>128</option>
          <option value="256" %MS256%>256</option>
        </select>
        <label>Stall Sensitivity</label>
        <select name="stallthreshold">
          <option value="5" %SEN_XL%>Extra Low (5)</option>
          <option value="15" %SEN_LO%>Low (15)</option>
          <option value="30" %SEN_MD%>Medium (30)</option>
          <option value="60" %SEN_HI%>High (60)</option>
          <option value="100" %SEN_MX%>Max (100)</option>
        </select>
      </div>

      <div class="card">
        <h2>System</h2>
        <label>OTA Password</label>
        <input name="ota_pass" type="password" placeholder="Leave blank to keep current">
        <div class="hint">Device will reboot after saving.</div>
      </div>

      <div class="btn-row">
        <button type="submit" class="btn btn-primary">Save &amp; Reboot</button>
        <a href="/webserial" class="btn btn-secondary">Console</a>
      </div>
    </form>

    <div class="version">v%VERSION%</div>
  </div>
</body>
</html>
)rawliteral";

void setup_webserial() {
  // Root redirects to setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/setup");
  });

  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = FPSTR(SETUP_HTML);
    String hostname = preferences.getString("hostname", "CurtainController");
    html.replace("%HOSTNAME%", hostname);
    html.replace("%VERSION%", FW_VERSION);
    html.replace("%MQTT_SERVER%", preferences.getString("mqtt_server", "192.168.1.100"));
    html.replace("%MQTT_PORT%", String(preferences.getInt("mqtt_port", 1883)));
    html.replace("%MQTT_USER%", preferences.getString("mqtt_user", "your_mqtt_user"));
    html.replace("%MQTT_PASS%", preferences.getString("mqtt_pass", "your_mqtt_password"));
    html.replace("%MQTT_TOPIC%", preferences.getString("mqtt_root_topic", "home/room/curtains"));
    html.replace("%STEPS%", String(preferences.getInt("steps_per_rev", 2000)));
    html.replace("%CURRENT%", String(preferences.getUShort("current_ma", 800)));

    // Microsteps dropdown selected state
    uint16_t ms = preferences.getUShort("microsteps", 2);
    html.replace("%MS1%", ms == 1 ? "selected" : "");
    html.replace("%MS2%", ms == 2 ? "selected" : "");
    html.replace("%MS4%", ms == 4 ? "selected" : "");
    html.replace("%MS8%", ms == 8 ? "selected" : "");
    html.replace("%MS16%", ms == 16 ? "selected" : "");
    html.replace("%MS32%", ms == 32 ? "selected" : "");
    html.replace("%MS64%", ms == 64 ? "selected" : "");
    html.replace("%MS128%", ms == 128 ? "selected" : "");
    html.replace("%MS256%", ms == 256 ? "selected" : "");

    // Sensitivity dropdown selected state
    uint8_t st = preferences.getUChar("stall_thr", 50);
    html.replace("%SEN_XL%", st <= 8 ? "selected" : "");
    html.replace("%SEN_LO%", (st > 8 && st <= 20) ? "selected" : "");
    html.replace("%SEN_MD%", (st > 20 && st <= 45) ? "selected" : "");
    html.replace("%SEN_HI%", (st > 45 && st <= 80) ? "selected" : "");
    html.replace("%SEN_MX%", st > 80 ? "selected" : "");

    request->send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("hostname", true)) {
      preferences.putString("hostname", request->getParam("hostname", true)->value());
    }
    if (request->hasParam("mqtt_server", true)) {
      preferences.putString("mqtt_server", request->getParam("mqtt_server", true)->value());
    }
    if (request->hasParam("mqtt_port", true)) {
      preferences.putInt("mqtt_port", request->getParam("mqtt_port", true)->value().toInt());
    }
    if (request->hasParam("mqtt_user", true)) {
      preferences.putString("mqtt_user", request->getParam("mqtt_user", true)->value());
    }
    if (request->hasParam("mqtt_pass", true)) {
      preferences.putString("mqtt_pass", request->getParam("mqtt_pass", true)->value());
    }
    if (request->hasParam("mqtt_topic", true)) {
      preferences.putString("mqtt_root_topic", request->getParam("mqtt_topic", true)->value());
    }
    if (request->hasParam("steps", true)) {
      int steps = request->getParam("steps", true)->value().toInt();
      if (steps > 0) preferences.putInt("steps_per_rev", steps);
    }
    if (request->hasParam("current", true)) {
      int val = request->getParam("current", true)->value().toInt();
      if (val >= 100 && val <= 2000) preferences.putUShort("current_ma", val);
    }
    if (request->hasParam("microsteps", true)) {
      int val = request->getParam("microsteps", true)->value().toInt();
      preferences.putUShort("microsteps", val);
    }
    if (request->hasParam("stallthreshold", true)) {
      int val = request->getParam("stallthreshold", true)->value().toInt();
      if (val >= 0 && val <= 255) preferences.putUChar("stall_thr", val);
    }
    if (request->hasParam("ota_pass", true)) {
      String ota = request->getParam("ota_pass", true)->value();
      if (ota.length() > 0) preferences.putString("ota_pass", ota);
    }
    preferences.putBool("ha_disc_done", false);

    request->send(200, "text/html", "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'><style>body{font-family:-apple-system,sans-serif;background:#0a0a1a;color:#fff;display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}div{text-align:center}h1{background:linear-gradient(135deg,#00D4FF,#6366F1);-webkit-background-clip:text;-webkit-text-fill-color:transparent}</style></head><body><div><h1>Saved!</h1><p style='color:rgba(255,255,255,0.5)'>Rebooting...</p></div></body></html>");
    delay(1000);
    ESP.restart();
  });

  WebSerial.onMessage([](uint8_t *data, size_t len) {
    String command;
    command.reserve(len + 1);

    for (size_t i = 0; i < len; i++) {
      char c = (char)data[i];
      if (c >= 32 && c <= 126) command += c;
    }

    command.trim();
    if (command.length() == 0) return;

    output("> %s\n", command.c_str());

    ws_pending_command = command;
    ws_command_pending = true;
  });

  WebSerial.begin(&server);
  WebSerial.setBuffer(256);
  server.begin();
}

// ============================================================================
// WIFI & NETWORK
// ============================================================================

bool check_button_hold_at_boot(unsigned long hold_time_ms) {
  unsigned long start = millis();
  while (digitalRead(RESET_BUTTON_PIN) == LOW) {
    if (millis() - start >= hold_time_ms) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED, LOW);
        delay(100);
        digitalWrite(STATUS_LED, HIGH);
        delay(100);
      }
      return true;
    }
    delay(10);
  }
  return false;
}

void start_config_portal() {
  String mqtt_topic = preferences.getString("mqtt_root_topic", "home/room/curtains");
  String ota_pass = preferences.getString("ota_pass", "");

  WiFiManager wm;
  wm.setConfigPortalTimeout(300);
  wm.setConnectTimeout(60);

  bool shouldSave = false;
  wm.setSaveConfigCallback([&shouldSave]() { shouldSave = true; });

  WiFiManagerParameter p_hostname("hostname", "Device Hostname", device_hostname.c_str(), 40);
  WiFiManagerParameter p_server("server", "MQTT Server IP", mqtt_server.c_str(), 40);
  WiFiManagerParameter p_port("port", "MQTT Port", String(mqtt_port).c_str(), 6);
  WiFiManagerParameter p_user("user", "MQTT Username", mqtt_user.c_str(), 40);
  WiFiManagerParameter p_pass("password", "MQTT Password", mqtt_password.c_str(), 40);
  WiFiManagerParameter p_topic("mqtt_root_topic", "MQTT Root Topic", mqtt_topic.c_str(), 80);
  WiFiManagerParameter p_ota("ota_pass", "OTA Password", ota_pass.c_str(), 40);
  WiFiManagerParameter p_steps("steps_per_rev", "Steps per Revolution", String(steps_per_revolution).c_str(), 8);

  wm.addParameter(&p_hostname);
  wm.addParameter(&p_server);
  wm.addParameter(&p_port);
  wm.addParameter(&p_user);
  wm.addParameter(&p_pass);
  wm.addParameter(&p_topic);
  wm.addParameter(&p_ota);
  wm.addParameter(&p_steps);

  log_msg(LOG_INFO, "WIFI", "Starting config portal AP: CurtainSetup");
  if (!wm.autoConnect("CurtainSetup", "12345678")) {
    log_msg(LOG_ERROR, "WIFI", "Config portal timeout, restarting...");
    delay(1000);
    ESP.restart();
  }
  log_msg(LOG_INFO, "WIFI", "WiFi connected via config portal");

  if (shouldSave) {
    preferences.putString("hostname", p_hostname.getValue());
    preferences.putString("mqtt_server", p_server.getValue());
    preferences.putInt("mqtt_port", atoi(p_port.getValue()));
    preferences.putString("mqtt_user", p_user.getValue());
    preferences.putString("mqtt_pass", p_pass.getValue());
    preferences.putString("mqtt_root_topic", p_topic.getValue());
    int steps_val = atoi(p_steps.getValue());
    if (steps_val > 0) preferences.putInt("steps_per_rev", steps_val);
    if (strlen(p_ota.getValue()) > 0) preferences.putString("ota_pass", p_ota.getValue());
    preferences.putBool("ha_disc_done", false);
    delay(1000);
    ESP.restart();
  }
}

void setup_wifi_manager() {
  device_hostname = preferences.getString("hostname", "CurtainController");
  mqtt_server = preferences.getString("mqtt_server", "192.168.1.100");
  mqtt_port = preferences.getInt("mqtt_port", 1883);
  mqtt_user = preferences.getString("mqtt_user", "your_mqtt_user");
  mqtt_password = preferences.getString("mqtt_pass", "your_mqtt_password");

  log_msg(LOG_INFO, "WIFI", "Starting WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(device_hostname.c_str());

  bool force_portal = check_button_hold_at_boot(3000);
  bool has_wifi_config = WiFi.SSID().length() > 0;

  if (force_portal || !has_wifi_config) {
    if (force_portal) {
      log_msg(LOG_INFO, "BTN", "Button held at boot — starting config portal");
    } else {
      log_msg(LOG_INFO, "WIFI", "No WiFi config saved — starting config portal");
    }
    start_config_portal();
  } else {
    log_msg(LOG_INFO, "WIFI", "Connecting to saved WiFi...");
    WiFi.begin();

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      attempts++;

      digitalWrite(STATUS_LED, (attempts % 2) ? LOW : HIGH);

      if (attempts >= 60) {
        Serial.println();
        log_msg(LOG_WARN, "WIFI", "Retrying WiFi connection...");
        WiFi.disconnect();
        delay(1000);
        WiFi.begin();
        attempts = 0;
      }
    }
    digitalWrite(STATUS_LED, HIGH);
    Serial.println();
    log_msg(LOG_INFO, "WIFI", "WiFi connected: %s", WiFi.localIP().toString().c_str());
  }

  // Set DHCP hostname so router shows correct device name
  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif) {
    esp_netif_set_hostname(netif, device_hostname.c_str());
    log_msg(LOG_INFO, "WIFI", "DHCP hostname set to: %s", device_hostname.c_str());
  }

  wifi_state = WIFI_CONNECTED;
}

void handle_wifi_reconnection() {
  switch (wifi_state) {
    case WIFI_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        wifi_state = WIFI_DISCONNECTED;
      }
      break;

    case WIFI_DISCONNECTED:
      log_msg(LOG_WARN, "WIFI", "Disconnected — attempting reconnection");
      WiFi.disconnect();
      WiFi.setHostname(device_hostname.c_str());
      WiFi.begin();
      wifi_reconnect_start = millis();
      wifi_state = WIFI_RECONNECTING;
      break;

    case WIFI_RECONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        log_msg(LOG_INFO, "WIFI", "Reconnected: %s", WiFi.localIP().toString().c_str());
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) esp_netif_set_hostname(netif, device_hostname.c_str());
        wifi_state = WIFI_CONNECTED;
        MDNS.end();
        setup_mdns();
      } else if (millis() - wifi_reconnect_start > WIFI_RECONNECT_TIMEOUT) {
        log_msg(LOG_ERROR, "WIFI", "Reconnect timeout after %lums — restarting", WIFI_RECONNECT_TIMEOUT);
        ESP.restart();
      }
      break;
  }
}

void setup_mdns() {
  if (!MDNS.begin(device_hostname.c_str())) return;
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("arduino", "tcp", 3232);
}

// ============================================================================
// OTA
// ============================================================================

void setup_ota() {
  ArduinoOTA.setHostname(device_hostname.c_str());
  ArduinoOTA.setPort(3232);

  String ota_password = preferences.getString("ota_pass", "");
  if (ota_password.length() > 0) {
    ArduinoOTA.setPassword(ota_password.c_str());
  }

  ArduinoOTA.onStart([]() {
    log_msg(LOG_INFO, "OTA", "OTA update starting — stopping motor and disconnecting");
    if (is_moving) stop_movement("OTA");
    if (cal_state != CAL_IDLE) {
      cal_state = CAL_IDLE;
      is_moving = false;
    }
    sleep_motor();
    if (client.connected()) client.disconnect();
    esp_task_wdt_delete(NULL);
  });

  ArduinoOTA.begin();
}

// ============================================================================
// RESET BUTTON
// ============================================================================

void check_reset_button() {
  bool current_reading = digitalRead(RESET_BUTTON_PIN);

  if (current_reading != button_state) {
    last_button_change = millis();
    button_state = current_reading;
  }

  if (millis() - last_button_change >= BUTTON_DEBOUNCE_MS) {
    bool new_stable_state = button_state;

    if (new_stable_state == LOW && last_stable_state == HIGH) {
      if (is_moving) {
        if (!led_manual_control) {
          digitalWrite(STATUS_LED, LOW);
          delay(100);
          digitalWrite(STATUS_LED, HIGH);
        }
        last_stable_state = new_stable_state;
        return;
      }
      button_press_start = millis();
    }
    else if (new_stable_state == LOW && last_stable_state == LOW) {
      unsigned long hold_time = millis() - button_press_start;

      if (!led_manual_control) {
        if (hold_time >= AP_HOLD_MIN && hold_time < AP_HOLD_MAX) {
          digitalWrite(STATUS_LED, LOW);
        } else if (hold_time >= RESET_HOLD_MIN && hold_time < RESET_HOLD_MAX) {
          digitalWrite(STATUS_LED, (millis() / 100) % 2);
        } else {
          digitalWrite(STATUS_LED, HIGH);
        }
      }
    }
    else if (new_stable_state == HIGH && last_stable_state == LOW) {
      unsigned long hold_time = millis() - button_press_start;

      if (hold_time >= AP_HOLD_MIN && hold_time < AP_HOLD_MAX) {
        log_msg(LOG_INFO, "BTN", "Config portal triggered by button hold (%lums)", hold_time);
        if (client.connected()) client.disconnect();

        WiFi.disconnect(true, true);
        WiFi.mode(WIFI_OFF);
        delay(100);
        WiFi.mode(WIFI_AP);
        delay(100);

        WiFiManager wm;
        wm.setConfigPortalTimeout(300);
        wm.setBreakAfterConfig(true);
        wm.startConfigPortal("CurtainSetup", "12345678");

        delay(1000);
        ESP.restart();
      }

      if (hold_time >= RESET_HOLD_MIN && hold_time < RESET_HOLD_MAX) {
        log_msg(LOG_INFO, "BTN", "Factory reset triggered by button hold (%lums)", hold_time);
        if (client.connected()) client.disconnect();
        WiFi.disconnect(true);

        WiFiManager wm;
        wm.resetSettings();
        preferences.clear();

        delay(1000);
        ESP.restart();
      }

      if (!led_manual_control) {
        digitalWrite(STATUS_LED, HIGH);
      }
    }

    last_stable_state = new_stable_state;
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.printf("\n=== Curtain Controller v%s (TMC2209) ===\n", FW_VERSION);

  // Log reset reason before anything else
  esp_reset_reason_t reason = esp_reset_reason();
  const char* reason_str;
  switch (reason) {
    case ESP_RST_POWERON:  reason_str = "Power-on"; break;
    case ESP_RST_SW:       reason_str = "Software restart"; break;
    case ESP_RST_PANIC:    reason_str = "Crash (panic)"; break;
    case ESP_RST_INT_WDT:  reason_str = "Interrupt watchdog"; break;
    case ESP_RST_TASK_WDT: reason_str = "Task watchdog"; break;
    case ESP_RST_WDT:      reason_str = "Other watchdog"; break;
    case ESP_RST_BROWNOUT: reason_str = "Brownout"; break;
    default:               reason_str = "Unknown"; break;
  }
  last_reset_reason = reason_str;
  Serial.printf("Reset reason: %s\n", reason_str);

  preferences.begin("curtains", false);

  // Load log level before any log_msg calls
  current_log_level = (LogLevel)preferences.getUChar("log_level", (uint8_t)LOG_INFO);

  log_msg(LOG_INFO, "BOOT", "Reset reason: %s", reason_str);

  current_position = preferences.getInt("position", 0);
  step_delay_us = preferences.getInt("step_delay", 2000);
  motor_sleep_timeout = preferences.getULong("sleep_timeout", 30000);
  steps_per_revolution = preferences.getInt("steps_per_rev", 2000);
  motor_current_ma = preferences.getUShort("current_ma", 800);
  motor_microsteps = preferences.getUShort("microsteps", 2);
  stall_threshold = preferences.getUChar("stall_thr", 50);
  invert_direction = preferences.getBool("invert_dir", false);

  pinMode(STATUS_LED, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(STATUS_LED, HIGH);

  setup_tmc2209();
  setup_wifi_manager();
  setup_mdns();
  setup_webserial();
  setup_mqtt();
  setup_ota();

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  log_msg(LOG_INFO, "BOOT", "Ready! IP:%s TMC2209:%s LogLevel:%s",
          WiFi.localIP().toString().c_str(),
          tmc_available ? "OK" : "NOT CONNECTED",
          log_level_name(current_log_level));
  log_msg(LOG_INFO, "BOOT", "WebSerial: http://%s/webserial", WiFi.localIP().toString().c_str());
  log_msg(LOG_INFO, "BOOT", "Setup: http://%s/setup", WiFi.localIP().toString().c_str());
}

void loop() {
  esp_task_wdt_reset();

  if (led_manual_control) {
    digitalWrite(STATUS_LED, led_desired_state);
  }

  ArduinoOTA.handle();
  check_reset_button();

  if (cal_state != CAL_IDLE) {
    handle_calibration();
  } else {
    handle_movement();
  }

  if (!client.connected()) {
    connect_mqtt();
  }
  client.loop();

  if (ws_command_pending) {
    ws_command_pending = false;
    process_command(ws_pending_command);
  }

  // Auto-sleep motor after inactivity
  if (!is_moving && cal_state == CAL_IDLE && motor_enabled) {
    if (millis() - last_motor_activity > motor_sleep_timeout) {
      sleep_motor();
    }
  }

  // Periodic TMC error check (every 5 seconds when idle)
  static unsigned long last_tmc_check = 0;
  if (!is_moving && millis() - last_tmc_check > 5000) {
    check_tmc_errors();
    last_tmc_check = millis();
  }

  handle_wifi_reconnection();

  yield();  // Let WiFi/WebSocket process
}
