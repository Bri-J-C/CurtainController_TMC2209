// ============================================================================
// CURTAIN CONTROLLER v5.2 - TMC2209 Edition
// Based on original v4.3 with TMC2209 UART control added
// ============================================================================
// Target: ESP32-C3 Super Mini
// Driver: TMC2209 with UART control and StallGuard4
// ============================================================================

#include <esp_netif.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
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
uint16_t motor_microsteps = 16;
uint8_t stall_threshold = 50;
bool tmc_verbose = false;  // Verbose mode for TMC diagnostics

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
int step_delay_us = 800;
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
bool reset_in_progress = false;
bool ap_triggered = false;
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

// WiFi reconnection state machine
enum WiFiReconnectState { WIFI_CONNECTED, WIFI_DISCONNECTED, WIFI_RECONNECTING };
WiFiReconnectState wifi_state = WIFI_CONNECTED;
unsigned long wifi_reconnect_start = 0;
const unsigned long WIFI_RECONNECT_TIMEOUT = 30000;

volatile bool ws_command_pending = false;

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
void process_command(const String& command);
void stop_motor();
void wake_motor();
void sleep_motor();
void step_motor();
void start_movement(int target);
void start_calibration();
void check_tmc_errors();

// ============================================================================
// WEBSERIAL HELPERS
// ============================================================================

void ws_println(const String& s) {
  WebSerial.println(s);
  yield();  // Let WiFi/WebSocket process
}

void ws_print(const String& s) {
  WebSerial.print(s);
  yield();
}

void ws_printf(const char* fmt, ...) {
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  WebSerial.print(buffer);
  yield();
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
  Serial.println(F("Initializing TMC2209 UART..."));
  TMCSerial.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
  TMCSerial.setTimeout(100);
  delay(200);

  driver.begin();
  driver.toff(0);
  delay(50);

  // Test communication
  Serial.println(F("Testing TMC2209 communication..."));
  uint8_t result = driver.test_connection();

  if (result != 0) {
    Serial.printf("TMC2209 comm test returned: %d\n", result);

    uint32_t ioin = driver.IOIN();
    Serial.printf("IOIN register: 0x%08X\n", ioin);

    if (ioin == 0 || ioin == 0xFFFFFFFF) {
      Serial.println(F("TMC2209 not responding!"));
      Serial.println(F("Wiring: GPIO21(TX)--[1K]-->PDN_UART, GPIO20(RX)-->PDN_UART"));
      Serial.println(F("Motor control will be limited."));
      tmc_available = false;
      return;
    }
    Serial.println(F("TMC2209 responding, continuing setup..."));
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

  // Attach interrupt for stall detection
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stall_isr, RISING);

  Serial.println(F("TMC2209 initialized successfully"));
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

  if (ot || s2ga || s2gb) {
    // Serious error - disable and re-enable
    Serial.println(F("TMC2209 error detected, resetting..."));
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
    Serial.println(F("TMC2209 not available"));
    ws_println(F("TMC2209 not available"));
    return;
  }

  target_position = constrain(target, 0, steps_per_revolution);
  if (current_position == target_position) {
    Serial.println(F("Already at position"));
    ws_println(F("Already at position"));
    return;
  }

  wake_motor();
  stall_flag = false;

  if (target_position > current_position) {
    digitalWrite(DIR_PIN, HIGH);
    publish_status("opening");
  } else {
    digitalWrite(DIR_PIN, LOW);
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
    if (current_position >= steps_per_revolution) {
      status = "open";
    } else if (current_position <= 0) {
      status = "closed";
    } else {
      status = "open";
    }
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
    Serial.printf("[STALL DETECTED] SG:%d threshold:%d (triggers at SG<%d)\n", sg, stall_threshold, stall_threshold * 2);
    ws_printf("[STALL DETECTED] SG:%d thr:%d\n", sg, stall_threshold);
  }
  stall_flag = false;

  if (millis() - movement_start_time > MOVEMENT_TIMEOUT) {
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
      Serial.printf("[TMC] SG:%3d CS:%2d/31 DIAG:%d\n", sg, cs, digitalRead(DIAG_PIN));
      ws_printf("[TMC] SG:%3d CS:%2d/31 DIAG:%d\n", sg, cs, digitalRead(DIAG_PIN));
    }
  }
}

// ============================================================================
// CALIBRATION
// ============================================================================

void start_calibration() {
  if (!tmc_available) {
    Serial.println(F("Cannot calibrate: TMC2209 not available"));
    ws_println(F("Cannot calibrate: TMC2209 not available"));
    return;
  }
  if (is_moving || cal_state != CAL_IDLE) {
    Serial.println(F("Cannot calibrate: busy"));
    ws_println(F("Cannot calibrate: busy"));
    return;
  }

  Serial.println(F("Starting calibration - finding closed position..."));
  ws_println(F("Starting calibration - finding closed position..."));

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
    Serial.println(F("Calibration timeout!"));
    ws_println(F("Calibration timeout!"));
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
          Serial.printf("[CAL STALL] SG:%d threshold:%d\n", sg, stall_threshold);
          ws_printf("[CAL STALL] SG:%d thr:%d\n", sg, stall_threshold);
        }

        if (cal_state == CAL_FIND_MIN) {
          current_position = 0;
          save_position();
          Serial.println(F("Found closed position"));
          ws_println(F("Found closed position"));

          delay(30);
          yield();
          digitalWrite(DIR_PIN, HIGH);
          delayMicroseconds(10);

          // Back off a bit
          for (int i = 0; i < 50; i++) {
            step_motor();
            delayMicroseconds(step_delay_us);
            if (i % 10 == 0) yield();  // Yield every 10 steps
          }
          current_position = 50;

          stall_flag = false;
          cal_last_stall_check = millis();
          cal_state = CAL_FIND_MAX;
          Serial.println(F("Finding open position..."));
          ws_println(F("Finding open position..."));

        } else if (cal_state == CAL_FIND_MAX) {
          int total = current_position;
          Serial.printf("Found open position at %d steps\n", total);
          ws_printf("Found open position at %d steps\n", total);

          delay(30);
          yield();
          digitalWrite(DIR_PIN, LOW);
          delayMicroseconds(10);

          // Back off
          for (int i = 0; i < 30; i++) {
            step_motor();
            delayMicroseconds(step_delay_us);
            current_position--;
            if (i % 10 == 0) yield();  // Yield every 10 steps
          }

          // Apply 2% margin each side
          int margin = total / 50;
          steps_per_revolution = total - (margin * 2);
          current_position = steps_per_revolution;

          preferences.putInt("steps_per_rev", steps_per_revolution);
          save_position();

          Serial.printf("Calibration complete! Range: %d steps\n", steps_per_revolution);
          ws_printf("Calibration complete! Range: %d steps\n", steps_per_revolution);

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
  device["sw_version"] = "5.2";
  device["configuration_url"] = "http://" + WiFi.localIP().toString() + "/setup";

  String json;
  serializeJson(doc, json);

  Serial.printf("HA Discovery payload size: %d bytes\n", json.length());
  ws_printf("HA Discovery payload size: %d bytes\n", json.length());

  if (client.publish(discovery_topic.c_str(), json.c_str(), true)) {
    preferences.putBool("ha_disc_done", true);
    Serial.println("HA Discovery published successfully");
    ws_println("HA Discovery published successfully");
  } else {
    Serial.println("HA Discovery publish FAILED");
    ws_println("HA Discovery publish FAILED");
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();
  process_command(msg);
}

void connect_mqtt() {
  if (client.connected()) return;

  if (millis() - last_mqtt_attempt < mqtt_retry_delay) return;

  last_mqtt_attempt = millis();

  String client_id = device_hostname + "_" + WiFi.macAddress();
  client_id.replace(":", "");

  bool connected;
  if (mqtt_user.length() > 0) {
    connected = client.connect(client_id.c_str(), mqtt_user.c_str(), mqtt_password.c_str(),
                              mqtt_availability_topic.c_str(), 1, true, "offline");
  } else {
    connected = client.connect(client_id.c_str(), mqtt_availability_topic.c_str(), 1, true, "offline");
  }

  if (connected) {
    client.subscribe(mqtt_command_topic.c_str());
    client.publish(mqtt_availability_topic.c_str(), "online", true);
    publish_position();
    publish_status(current_position >= steps_per_revolution ? "open" :
                   current_position <= 0 ? "closed" : "open");
    publish_ha_discovery();
    mqtt_retry_delay = 2000;
  } else {
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

  client.setBufferSize(1536);
  client.setServer(mqtt_server.c_str(), mqtt_port);
  client.setCallback(mqtt_callback);

  connect_mqtt();
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void cmd_open(const String& param) {
  start_movement(steps_per_revolution);
  String msg = "Opening curtain...";
  Serial.println(msg);
  ws_println(msg);
}

void cmd_close(const String& param) {
  start_movement(0);
  String msg = "Closing curtain...";
  Serial.println(msg);
  ws_println(msg);
}

void cmd_stop(const String& param) {
  if (cal_state != CAL_IDLE) {
    cal_state = CAL_IDLE;
    Serial.println(F("Calibration cancelled"));
    ws_println(F("Calibration cancelled"));
  }
  stop_movement("User command");
  String msg = "Stopped";
  Serial.println(msg);
  ws_println(msg);
}

void cmd_position(const String& param) {
  int pos = param.toInt();
  if (pos >= 0 && pos <= steps_per_revolution) {
    start_movement(pos);
    Serial.printf("Moving to position %d\n", pos);
    ws_printf("Moving to position %d\n", pos);
  } else {
    Serial.printf("[ERROR] Position must be 0-%d\n", steps_per_revolution);
    ws_printf("[ERROR] Position must be 0-%d\n", steps_per_revolution);
  }
}

void cmd_speed(const String& param) {
  int value = param.toInt();
  if (value >= 100 && value <= 10000) {
    step_delay_us = value;
    preferences.putInt("step_delay", step_delay_us);
    Serial.printf("Speed: %d us/step\n", step_delay_us);
    ws_printf("Speed: %d us/step\n", step_delay_us);
  } else {
    String msg = "[ERROR] Speed must be 100-10000 us";
    Serial.println(msg);
    ws_println(msg);
  }
}

void cmd_microsteps(const String& param) {
  int value = param.toInt();
  if (value == 1 || value == 2 || value == 4 || value == 8 ||
      value == 16 || value == 32 || value == 64 || value == 128 || value == 256) {
    set_motor_microsteps(value);
    Serial.printf("Microsteps: %d\n", motor_microsteps);
    ws_printf("Microsteps: %d\n", motor_microsteps);
  } else {
    String msg = "[ERROR] Microsteps must be 1,2,4,8,16,32,64,128,256";
    Serial.println(msg);
    ws_println(msg);
  }
}

void cmd_current(const String& param) {
  int value = param.toInt();
  if (value >= 100 && value <= 2000) {
    set_motor_current(value);
    Serial.printf("Current: %d mA\n", motor_current_ma);
    ws_printf("Current: %d mA\n", motor_current_ma);
  } else {
    String msg = "[ERROR] Current must be 100-2000 mA";
    Serial.println(msg);
    ws_println(msg);
  }
}

void cmd_stallthreshold(const String& param) {
  int value = param.toInt();
  if (value >= 0 && value <= 255) {
    set_stall_threshold(value);
    Serial.printf("Stall threshold: %d\n", stall_threshold);
    ws_printf("Stall threshold: %d\n", stall_threshold);
  } else {
    String msg = "[ERROR] Threshold must be 0-255";
    Serial.println(msg);
    ws_println(msg);
  }
}

void cmd_calibrate(const String& param) {
  Serial.println(F("Starting sensorless calibration..."));
  ws_println(F("Starting sensorless calibration..."));
  Serial.println(F("Ensure curtains can move freely!"));
  ws_println(F("Ensure curtains can move freely!"));
  start_calibration();
}

void cmd_steps(const String& param) {
  int value = param.toInt();
  if (value > 0 && value <= 500000) {
    steps_per_revolution = value;
    preferences.putInt("steps_per_rev", steps_per_revolution);
    Serial.printf("Steps/rev: %d\n", steps_per_revolution);
    ws_printf("Steps/rev: %d\n", steps_per_revolution);
  } else {
    String msg = "[ERROR] Steps must be 1-500000";
    Serial.println(msg);
    ws_println(msg);
  }
}

void cmd_setposition(const String& param) {
  int value = param.toInt();
  if (value >= 0 && value <= steps_per_revolution) {
    current_position = value;
    save_position();
    publish_position();
    Serial.printf("Position reset to %d\n", current_position);
    ws_printf("Position reset to %d\n", current_position);
  } else {
    Serial.printf("[ERROR] Position must be 0-%d\n", steps_per_revolution);
    ws_printf("[ERROR] Position must be 0-%d\n", steps_per_revolution);
  }
}

void cmd_sleep(const String& param) {
  int value = param.toInt();
  if (value >= 0 && value <= 300000) {
    motor_sleep_timeout = value;
    preferences.putULong("sleep_timeout", motor_sleep_timeout);
    Serial.printf("Sleep timeout: %lu ms\n", motor_sleep_timeout);
    ws_printf("Sleep timeout: %lu ms\n", motor_sleep_timeout);
  } else {
    String msg = "[ERROR] Sleep must be 0-300000 ms";
    Serial.println(msg);
    ws_println(msg);
  }
}

void cmd_hadiscovery(const String& param) {
  preferences.putBool("ha_disc_done", false);
  publish_ha_discovery(true);
  String msg = "HA discovery republished";
  Serial.println(msg);
  ws_println(msg);
}

void cmd_config(const String& param) {
  String mqtt_topic = preferences.getString("mqtt_root_topic", "home/room/curtains");

  String output = "\n=== Configuration ===\n";
  output += "Hostname: " + device_hostname + "\n";
  output += "IP: " + WiFi.localIP().toString() + "\n";
  output += "SSID: " + WiFi.SSID() + "\n";
  output += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
  output += "MAC: " + WiFi.macAddress() + "\n";
  output += "MQTT: " + mqtt_server + ":" + String(mqtt_port) + "\n";
  output += "MQTT User: " + (mqtt_user.length() > 0 ? mqtt_user : "(none)") + "\n";
  output += "MQTT Topic: " + mqtt_topic + "\n";
  output += "Speed: " + String(step_delay_us) + " us/step\n";
  output += "Microsteps: " + String(motor_microsteps) + "\n";
  output += "Current: " + String(motor_current_ma) + " mA\n";
  output += "Stall threshold: " + String(stall_threshold) + "\n";
  output += "Steps/Rev: " + String(steps_per_revolution) + "\n";
  output += "Sleep Timeout: " + String(motor_sleep_timeout) + " ms\n";
  output += "TMC2209: " + String(tmc_available ? "OK" : "NOT CONNECTED") + "\n";
  output += "Setup: http://" + WiFi.localIP().toString() + "/setup\n";
  output += "====================\n";

  Serial.print(output);
  ws_print(output);
}

void cmd_restart(const String& param) {
  String msg = "Restarting...";
  Serial.println(msg);
  ws_println(msg);
  delay(1000);
  ESP.restart();
}

void cmd_help(const String& param) {
  String help = "\n=== Commands ===\n";
  help += "open          - Open curtain\n";
  help += "close         - Close curtain\n";
  help += "stop          - Stop movement\n";
  help += "position <n>  - Move to step position\n";
  help += "speed <us>    - Set step delay (100-10000)\n";
  help += "steps <n>     - Set steps per revolution\n";
  help += "setposition <n> - Reset position counter\n";
  help += "sleep <ms>    - Set sleep timeout\n";
  help += "\n-- TMC2209 --\n";
  help += "microsteps <n> - Set microsteps (1-256)\n";
  help += "current <mA>  - Set motor current (100-2000)\n";
  help += "stallthreshold <n> - Stall sensitivity (0-255)\n";
  help += "calibrate     - Auto-calibrate using stallguard\n";
  help += "verbose       - Toggle SG output during movement\n";
  help += "sgtest [sec]  - Monitor SG values (default 5s)\n";
  help += "\n-- System --\n";
  help += "status        - Show current status\n";
  help += "config        - Show configuration\n";
  help += "hadiscovery   - Republish HA discovery\n";
  help += "restart       - Reboot device\n";
  help += "================\n";

  Serial.print(help);
  ws_print(help);
}

void cmd_status(const String& param) {
  String output = "\n=== Status ===\n";
  output += "Position: " + String(current_position) + " (" + String((current_position * 100) / steps_per_revolution) + "%)\n";
  output += "Moving: " + String(is_moving ? "Yes" : "No") + "\n";
  if (is_moving) {
    output += "Target: " + String(target_position) + "\n";
  }
  output += "Motor: " + String(motor_enabled ? "Enabled" : "Disabled") + "\n";
  output += "MQTT: " + String(client.connected() ? "Connected" : "Disconnected") + "\n";

  // TMC2209 live status
  if (tmc_available) {
    uint32_t drv_status = driver.DRV_STATUS();
    uint16_t sg = driver.SG_RESULT();
    output += "-- TMC2209 --\n";
    output += "SG_RESULT: " + String(sg) + "\n";
    output += "Current scale: " + String((drv_status >> 16) & 0x1F) + "/31\n";
    output += "Standstill: " + String((drv_status >> 31) & 1 ? "Yes" : "No") + "\n";
    output += "OT warning: " + String((drv_status >> 0) & 1 ? "Yes" : "No") + "\n";
    output += "DIAG pin: " + String(digitalRead(DIAG_PIN) ? "HIGH" : "LOW") + "\n";
  } else {
    output += "TMC2209: NOT CONNECTED\n";
  }

  output += "-- System --\n";
  output += "Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
  output += "Uptime: " + String(millis() / 1000) + "s\n";
  output += "==============\n";

  Serial.print(output);
  ws_print(output);
}

void cmd_tmcstatus(const String& param) {
  if (!tmc_available) {
    String msg = "TMC2209 not available";
    Serial.println(msg);
    ws_println(msg);
    return;
  }

  uint32_t drv_status = driver.DRV_STATUS();
  uint16_t sg = driver.SG_RESULT();
  uint32_t ioin = driver.IOIN();

  String output = "\n=== TMC2209 Status ===\n";
  output += "Connection: " + String(driver.test_connection() == 0 ? "OK" : "ERROR") + "\n";
  output += "IOIN: 0x" + String(ioin, HEX) + "\n";
  output += "DRV_STATUS: 0x" + String(drv_status, HEX) + "\n";
  output += "SG_RESULT: " + String(sg) + "\n";
  output += "Stall threshold: " + String(stall_threshold) + "\n";
  output += "DIAG pin: " + String(digitalRead(DIAG_PIN) ? "HIGH" : "LOW") + "\n";
  output += "Standstill: " + String((drv_status >> 31) & 1 ? "Yes" : "No") + "\n";
  output += "OT warning: " + String((drv_status >> 0) & 1 ? "Yes" : "No") + "\n";
  output += "Current scale: " + String((drv_status >> 16) & 0x1F) + "/31\n";
  output += "======================\n";

  Serial.print(output);
  ws_print(output);
}

void cmd_verbose(const String& param) {
  tmc_verbose = !tmc_verbose;
  String msg = "Verbose mode: " + String(tmc_verbose ? "ON" : "OFF");
  Serial.println(msg);
  ws_println(msg);
  if (tmc_verbose) {
    Serial.println(F("SG values will show during movement (every 500ms)"));
    ws_println(F("SG values will show during movement (every 500ms)"));
    Serial.println(F("SG = StallGuard load value (0-1023)"));
    ws_println(F("SG = StallGuard load value (0-1023)"));
    Serial.println(F("When SG < SGTHRS*2, stall is detected"));
    ws_println(F("When SG < SGTHRS*2, stall is detected"));
  }
}

void cmd_sgtest(const String& param) {
  if (!tmc_available) {
    ws_println(F("TMC2209 not available"));
    return;
  }

  // Parse duration (default 5 seconds)
  int duration = param.length() > 0 ? param.toInt() : 5;
  duration = constrain(duration, 1, 60);

  // Use actual motor speed settings for realistic test
  unsigned long test_step_delay = step_delay_us;

  // Calculate full steps per SG update (SG updates once per full step = microsteps steps)
  // We'll read SG every N steps to get fresh values without blocking too much
  int steps_between_reads = motor_microsteps;  // Read once per full step
  if (steps_between_reads < 4) steps_between_reads = 4;
  if (steps_between_reads > 32) steps_between_reads = 32;

  ws_println(F("\n========== StallGuard Test =========="));
  ws_printf("Duration: %d seconds\n", duration);
  ws_printf("Step delay: %lu us (your speed setting)\n", test_step_delay);
  ws_printf("Microsteps: %d\n", motor_microsteps);
  ws_printf("Current: %d mA\n", motor_current_ma);
  ws_printf("Current threshold: %d (stall when SG < %d)\n", stall_threshold, stall_threshold * 2);
  ws_println(F(""));
  ws_println(F(">> Apply resistance to shaft to see SG drop <<"));
  ws_println(F(">> SG updates once per FULL motor step <<"));
  ws_println(F("==========================================\n"));

  wake_motor();
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(100);

  // Clear stall flag
  stall_flag = false;

  // --- RAMP-UP PHASE (500ms) ---
  // Start slower and accelerate to target speed for stable readings
  ws_println(F("Ramping up to speed..."));
  unsigned long ramp_start = millis();
  unsigned long last_step = micros();
  unsigned long current_delay = test_step_delay * 3;  // Start at 1/3 speed

  unsigned long last_yield = millis();
  while (millis() - ramp_start < 500) {
    unsigned long now = micros();
    if (now - last_step >= current_delay) {
      step_motor();
      last_step = now;

      // Gradually speed up
      if (current_delay > test_step_delay) {
        current_delay -= 2;
        if (current_delay < test_step_delay) current_delay = test_step_delay;
      }
    }

    // Yield every 50ms to keep WiFi alive
    if (millis() - last_yield >= 50) {
      yield();
      esp_task_wdt_reset();
      last_yield = millis();
    }
  }

  ws_println(F("At speed - measuring...\n"));

  // --- MAIN MEASUREMENT PHASE ---
  unsigned long test_start = millis();
  uint16_t min_sg = 1023, max_sg = 0;
  uint32_t sg_sum = 0;
  uint32_t sg_count = 0;
  int step_count = 0;
  unsigned long last_report = 0;
  const unsigned long REPORT_INTERVAL_MS = 250;  // Report 4x per second

  // For tracking recent values (simple moving average)
  uint16_t recent_sg[8] = {0};
  int recent_idx = 0;

  // Stall event counter
  int stall_events = 0;

  while (millis() - test_start < (unsigned long)(duration * 1000)) {
    unsigned long now = micros();

    // Step at configured rate - this is the CRITICAL part
    // Motor MUST keep stepping for valid SG readings
    if (now - last_step >= test_step_delay) {
      step_motor();
      last_step = now;
      step_count++;

      // Read SG after enough steps for it to have updated
      // SG_RESULT updates once per full step (= microsteps worth of step pulses)
      if (step_count >= steps_between_reads) {
        step_count = 0;

        // Quick UART read between steps
        uint16_t sg = driver.SG_RESULT();

        // Update statistics
        if (sg < min_sg) min_sg = sg;
        if (sg > max_sg) max_sg = sg;
        sg_sum += sg;
        sg_count++;

        // Track recent values for moving average
        recent_sg[recent_idx] = sg;
        recent_idx = (recent_idx + 1) % 8;

        // Check for stall events
        if (sg < (uint16_t)(stall_threshold * 2)) {
          stall_events++;
        }
      }
    }

    // Periodic reporting (non-blocking)
    unsigned long now_ms = millis();
    if (now_ms - last_report >= REPORT_INTERVAL_MS) {
      last_report = now_ms;

      // Calculate moving average from recent samples
      uint32_t recent_sum = 0;
      for (int i = 0; i < 8; i++) recent_sum += recent_sg[i];
      uint16_t recent_avg = recent_sum / 8;

      // Get current DIAG state
      bool diag = digitalRead(DIAG_PIN);

      // Output - keep it compact for readability
      Serial.printf("SG:%4d  recent:%4d  [%4d-%4d]  DIAG:%d  stalls:%d\n",
                    recent_sg[(recent_idx + 7) % 8],  // Most recent
                    recent_avg, min_sg, max_sg, diag, stall_events);
      ws_printf("SG:%4d avg:%4d [%d-%d] stalls:%d\n",
                recent_sg[(recent_idx + 7) % 8], recent_avg, min_sg, max_sg, stall_events);

      esp_task_wdt_reset();
    }

    // Yield frequently to keep WebSocket alive (every ~50ms)
    static unsigned long last_loop_yield = 0;
    if (now_ms - last_loop_yield >= 50) {
      yield();
      last_loop_yield = now_ms;
    }
  }

  stop_motor();

  // --- RESULTS SUMMARY ---
  uint16_t avg_sg = sg_count > 0 ? (sg_sum / sg_count) : 0;
  uint16_t sg_range = max_sg - min_sg;

  ws_println(F("\n============ RESULTS ============"));
  ws_printf("Samples collected: %lu\n", sg_count);
  ws_printf("SG range: %d to %d (span: %d)\n", min_sg, max_sg, sg_range);
  ws_printf("Average SG: %d\n", avg_sg);
  ws_printf("Stall events (SG < %d): %d\n", stall_threshold * 2, stall_events);

  ws_println(F("\n------ Threshold Recommendations ------"));
  ws_println(F("(Higher threshold = MORE sensitive to stalls)"));
  ws_println(F("(Lower threshold = LESS sensitive to stalls)"));

  // Calculate suggestions based on actual data
  // The stall triggers when SG_RESULT < SGTHRS * 2
  // So SGTHRS = (trigger_point) / 2

  if (sg_count > 10 && min_sg > 10) {
    // Conservative: Won't false-trigger during normal operation
    // Set to half of minimum observed (with margin)
    uint8_t conservative = (min_sg > 20) ? ((min_sg - 10) / 2) : 1;

    // Moderate: Good balance, set at 1/3 of average
    uint8_t moderate = avg_sg / 6;

    // Sensitive: Will catch lighter stalls, may have some false triggers
    // Set based on span - closer to normal operating point
    uint8_t sensitive = avg_sg / 4;

    ws_printf("\nConservative: %3d  (stalls when SG < %d)\n", conservative, conservative * 2);
    ws_printf("Moderate:     %3d  (stalls when SG < %d)\n", moderate, moderate * 2);
    ws_printf("Sensitive:    %3d  (stalls when SG < %d)\n", sensitive, sensitive * 2);

    ws_printf("\nYour current: %3d  (stalls when SG < %d)\n", stall_threshold, stall_threshold * 2);

    // Give specific advice
    if (stall_events > 5) {
      ws_println(F("\n! Many stall events detected during test"));
      ws_println(F("! Consider LOWERING threshold or checking mechanics"));
    } else if (stall_events == 0 && stall_threshold > sensitive) {
      ws_println(F("\n* No stalls during test - threshold may be too high"));
      ws_printf("* Try threshold around %d for calibration\n", moderate);
    }
  } else {
    ws_println(F("\nInsufficient data for recommendations."));
    ws_println(F("Try running longer or check TMC2209 connection."));
  }

  ws_println(F("\nTip: Run test while manually resisting shaft"));
  ws_println(F("     to see SG drop when load increases."));
  ws_println(F("====================================="));
}

void cmd_ledon(const String& param) {
  led_manual_control = true;
  led_desired_state = LOW;
  digitalWrite(STATUS_LED, led_desired_state);
  String msg = "LED ON";
  Serial.println(msg);
  ws_println(msg);
}

void cmd_ledoff(const String& param) {
  led_manual_control = true;
  led_desired_state = HIGH;
  digitalWrite(STATUS_LED, led_desired_state);
  String msg = "LED OFF";
  Serial.println(msg);
  ws_println(msg);
}

struct Command {
  const char* name;
  void (*handler)(const String& param);
};

const Command commands[] = {
  {"open", cmd_open},
  {"close", cmd_close},
  {"stop", cmd_stop},
  {"position ", cmd_position},
  {"speed ", cmd_speed},
  {"microsteps ", cmd_microsteps},
  {"current ", cmd_current},
  {"stallthreshold ", cmd_stallthreshold},
  {"calibrate", cmd_calibrate},
  {"steps ", cmd_steps},
  {"setposition ", cmd_setposition},
  {"sleep ", cmd_sleep},
  {"hadiscovery", cmd_hadiscovery},
  {"config", cmd_config},
  {"status", cmd_status},
  {"verbose", cmd_verbose},
  {"sgtest ", cmd_sgtest},
  {"sgtest", cmd_sgtest},
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
      start_movement(target_steps);
      Serial.printf("HA position: %d%% -> step %d\n", percentage, target_steps);
      ws_printf("HA position: %d%% -> step %d\n", percentage, target_steps);
      return;
    }
  }

  String msg = "[ERROR] Unknown: " + command;
  Serial.println(msg);
  ws_println(msg);
}

// ============================================================================
// WEBSERIAL & WEB SERVER
// ============================================================================

const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Curtain Controller Setup</title>
  <style>
    body{font-family:Arial,sans-serif;margin:20px;background:#1a1a1a;color:#fff}
    h1{color:#4CAF50}
    form{max-width:400px}
    label{display:block;margin-top:10px;font-weight:bold}
    input{width:100%;padding:8px;margin-top:4px;box-sizing:border-box;border-radius:4px;border:1px solid #444;background:#333;color:#fff}
    button{margin-top:20px;padding:12px 24px;background:#4CAF50;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:16px}
    button:hover{background:#45a049}
    .info{color:#888;font-size:12px}
    h2{color:#4CAF50;margin-top:20px;border-top:1px solid #333;padding-top:15px}
  </style>
</head>
<body>
  <h1>Curtain Controller Setup</h1>
  <p>TMC2209 Edition v5.2</p>
  <form action="/save" method="POST">
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
    <p class="info">Creates: /cmd, /status, /position, /availability</p>

    <h2>Motor</h2>
    <label>Steps per Revolution</label>
    <input name="steps" type="number" value="%STEPS%">
    <label>Current (mA)</label>
    <input name="current" type="number" value="%CURRENT%" min="100" max="2000">
    <label>Microsteps</label>
    <input name="microsteps" type="number" value="%MICROSTEPS%">
    <p class="info">Valid: 1, 2, 4, 8, 16, 32, 64, 128, 256</p>
    <label>Stall Threshold</label>
    <input name="stallthreshold" type="number" value="%STALLTHRESHOLD%" min="0" max="255">
    <p class="info">Higher = less sensitive (0-255)</p>

    <h2>System</h2>
    <label>OTA Password</label>
    <input name="ota_pass" type="password" placeholder="Leave blank to keep current">
    <p class="info">Device will reboot after saving.</p>
    <button type="submit">Save &amp; Reboot</button>
  </form>
</body>
</html>
)rawliteral";

void setup_webserial() {
  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = FPSTR(SETUP_HTML);
    html.replace("%HOSTNAME%", preferences.getString("hostname", "CurtainController"));
    html.replace("%MQTT_SERVER%", preferences.getString("mqtt_server", "192.168.1.100"));
    html.replace("%MQTT_PORT%", String(preferences.getInt("mqtt_port", 1883)));
    html.replace("%MQTT_USER%", preferences.getString("mqtt_user", "your_mqtt_user"));
    html.replace("%MQTT_PASS%", preferences.getString("mqtt_pass", "your_mqtt_password"));
    html.replace("%MQTT_TOPIC%", preferences.getString("mqtt_root_topic", "home/room/curtains"));
    html.replace("%STEPS%", String(preferences.getInt("steps_per_rev", 2000)));
    html.replace("%CURRENT%", String(preferences.getUShort("current_ma", 800)));
    html.replace("%MICROSTEPS%", String(preferences.getUShort("microsteps", 16)));
    html.replace("%STALLTHRESHOLD%", String(preferences.getUChar("stall_thr", 50)));
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

    request->send(200, "text/html", "<html><body><h1>Saved!</h1><p>Rebooting...</p></body></html>");
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

    ws_println("> " + command);

    ws_pending_command = command;
    ws_command_pending = true;
  });

  WebSerial.begin(&server);
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

  Serial.println("Starting AP: CurtainSetup");
  if (!wm.autoConnect("CurtainSetup", "12345678")) {
    Serial.println("Config portal timeout, restarting...");
    delay(1000);
    ESP.restart();
  }
  Serial.println("WiFi connected via portal");

  if (shouldSave) {
    preferences.putString("hostname", p_hostname.getValue());
    preferences.putString("mqtt_server", p_server.getValue());
    preferences.putInt("mqtt_port", String(p_port.getValue()).toInt());
    preferences.putString("mqtt_user", p_user.getValue());
    preferences.putString("mqtt_pass", p_pass.getValue());
    preferences.putString("mqtt_root_topic", p_topic.getValue());
    int steps_val = String(p_steps.getValue()).toInt();
    if (steps_val > 0) preferences.putInt("steps_per_rev", steps_val);
    String ota_new = String(p_ota.getValue());
    if (ota_new.length() > 0) preferences.putString("ota_pass", ota_new);
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

  Serial.println("Starting WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(device_hostname.c_str());

  bool force_portal = check_button_hold_at_boot(3000);
  bool has_wifi_config = WiFi.SSID().length() > 0;

  if (force_portal || !has_wifi_config) {
    if (force_portal) {
      Serial.println("Button held - starting config portal");
    } else {
      Serial.println("No WiFi config - starting config portal");
    }
    start_config_portal();
  } else {
    Serial.println("Connecting to saved WiFi...");
    WiFi.begin();

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      attempts++;

      digitalWrite(STATUS_LED, (attempts % 2) ? LOW : HIGH);

      if (attempts >= 60) {
        Serial.println("\nRetrying WiFi connection...");
        WiFi.disconnect();
        delay(1000);
        WiFi.begin();
        attempts = 0;
      }
    }
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("\nWiFi connected");
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
      WiFi.disconnect();
      WiFi.setHostname(device_hostname.c_str());
      WiFi.begin();
      wifi_reconnect_start = millis();
      wifi_state = WIFI_RECONNECTING;
      break;

    case WIFI_RECONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        wifi_state = WIFI_CONNECTED;
        MDNS.end();
        setup_mdns();
      } else if (millis() - wifi_reconnect_start > WIFI_RECONNECT_TIMEOUT) {
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
        Serial.println("Starting config portal...");
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
        Serial.println("Factory reset...");
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

  Serial.println("\n=== Curtain Controller v5.2 (TMC2209) ===");

  preferences.begin("curtains", false);

  current_position = preferences.getInt("position", 0);
  step_delay_us = preferences.getInt("step_delay", 800);
  motor_sleep_timeout = preferences.getULong("sleep_timeout", 30000);
  steps_per_revolution = preferences.getInt("steps_per_rev", 2000);
  motor_current_ma = preferences.getUShort("current_ma", 800);
  motor_microsteps = preferences.getUShort("microsteps", 16);
  stall_threshold = preferences.getUChar("stall_thr", 50);

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

  Serial.println("Ready!");
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("WebSerial: http://%s/webserial\n", WiFi.localIP().toString().c_str());
  Serial.printf("Setup: http://%s/setup\n", WiFi.localIP().toString().c_str());
  Serial.printf("TMC2209: %s\n", tmc_available ? "OK" : "NOT CONNECTED");
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
