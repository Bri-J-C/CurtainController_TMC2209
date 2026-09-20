// ============================================================================
// CURTAIN CONTROLLER v5.5 - TMC2209 Edition
// Based on original v4.3 with TMC2209 UART control added
// ============================================================================
// Target: ESP32-C3 Super Mini
// Driver: TMC2209 with UART control and StallGuard4
// ============================================================================

#define FW_VERSION "5.5"

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
#include <TMCStepper.h>
#include <stdarg.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Motor Configuration
int travel_steps = 2000;  // total calibrated travel, in microsteps

// Watchdog timeout (increased to handle slow movements)
const int WDT_TIMEOUT = 180;  // 3 minutes

// Pin definitions for TMC2209 - ESP32-C3 SUPER MINI
// Safe pins: GPIO 0,1,3,4,5,6,7,10,20,21
// Avoid at boot: GPIO 2,8,9 (strapping pins)
// Two wiring generations exist. Boards built to the original layout keep
// working by selecting the legacy profile; the pins are read at boot.
struct PinProfile {
  const char* name;
  int step, dir, enable, diag, tx, rx;
};
const PinProfile PIN_PROFILES[] = {
  {"current", 6, 5, 20, 21, 7, 10},
  {"legacy", 10, 6, 0, 7, 21, 20},
};
const int PIN_PROFILE_COUNT = sizeof(PIN_PROFILES) / sizeof(PIN_PROFILES[0]);

const uint8_t PIN_PROFILE_CUSTOM = 250;
uint8_t pin_profile = 0;
int STEP_PIN = 6;
int DIR_PIN = 5;
int ENABLE_PIN = 20;
int DIAG_PIN = 21;
int TMC_TX_PIN = 7;
int TMC_RX_PIN = 10;

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
volatile int current_position = 0;  // written by step ISR while moving
int target_position = 0;
bool is_moving = false;
unsigned long movement_start_time = 0;
const unsigned long MOVEMENT_TIMEOUT = 120000;  // 2 minute timeout
unsigned long last_position_save = 0;
const unsigned long POSITION_SAVE_INTERVAL = 1000;
unsigned long last_position_report = 0;
const unsigned long POSITION_REPORT_INTERVAL = 500;

// Motor state
int motor_rpm = 75;                          // shaft speed setting
int step_delay_us = 2000;                    // derived from motor_rpm and microsteps
const int MOTOR_FULL_STEPS_PER_REV = 200;    // 1.8 degree motor
const int MIN_STEP_PERIOD_US = 50;           // step ISR floor
bool motor_enabled = false;
unsigned long motor_sleep_timeout = 30000;
unsigned long last_motor_activity = 0;

bool motor_test_active = false;  // motortest drives the stepper directly

// Calibration state
enum CalibrationState { CAL_IDLE, CAL_FIND_MIN, CAL_BACKOFF_MIN, CAL_FIND_MAX, CAL_BACKOFF_MAX };
CalibrationState cal_state = CAL_IDLE;
unsigned long cal_start_time = 0;
const unsigned long CAL_TIMEOUT = 240000;

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
char ws_pending_command[80];
const char* last_reset_reason = "Unknown";

// Network
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
AsyncWebServer server(80);
AsyncWebSocket console_ws("/webserialws");
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
String mqtt_backoff_close_set_topic;
String mqtt_backoff_close_state_topic;
String mqtt_backoff_open_set_topic;
String mqtt_backoff_open_state_topic;
String mqtt_stallthreshold_set_topic;
String mqtt_stallthreshold_state_topic;
String mqtt_microsteps_set_topic;
String mqtt_microsteps_state_topic;
String mqtt_invert_set_topic;
String mqtt_invert_state_topic;

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

static const char CONSOLE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Curtain Console</title><style>
:root{--bg:#0a0a1a;--panel:#12122a;--line:#23234a;--txt:#d8d8ea;--dim:#7d7d9c;--accent:#00D4FF;--err:#ff6b6b;--warn:#ffc46b}
*{box-sizing:border-box}
body{margin:0;height:100vh;display:flex;flex-direction:column;background:var(--bg);color:var(--txt);
font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
header{display:flex;align-items:center;gap:.6rem;padding:.6rem .9rem;border-bottom:1px solid var(--line);background:var(--panel)}
h1{margin:0;font:600 14px system-ui,sans-serif;background:linear-gradient(135deg,#00D4FF,#6366F1);
-webkit-background-clip:text;-webkit-text-fill-color:transparent}
#dot{width:9px;height:9px;border-radius:50%;background:var(--err);flex:none}
#dot.on{background:#3ddc84}
#state{color:var(--dim);font-size:11px;margin-right:auto}
button,a.btn{background:#1c1c38;color:var(--txt);border:1px solid var(--line);border-radius:6px;padding:.3rem .6rem;
font:inherit;font-size:11px;cursor:pointer;text-decoration:none;display:inline-block}
button:hover,a.btn:hover{border-color:var(--accent)}
#log{flex:1;overflow:auto;padding:.7rem .9rem;white-space:pre-wrap;word-break:break-word}
#log .e{color:var(--err)}#log .w{color:var(--warn)}#log .d{color:var(--dim)}#log .cmd{color:var(--accent)}
form{display:flex;gap:.5rem;padding:.6rem .9rem;border-top:1px solid var(--line);background:var(--panel)}
input{flex:1;background:#0d0d20;border:1px solid var(--line);border-radius:6px;color:var(--txt);
padding:.5rem .6rem;font:inherit}input:focus{outline:none;border-color:var(--accent)}
</style></head><body>
<header><span id="dot"></span><h1>Curtain Console</h1><span id="state">connecting</span>
<a class="btn" href="/setup">setup</a><button id="clear">clear</button></header>
<div id="log"></div>
<form id="f"><input id="c" placeholder="type a command, or help" autocomplete="off" autofocus></form>
<script>
var log=document.getElementById('log'),dot=document.getElementById('dot'),state=document.getElementById('state');
var hist=[],hpos=0,ws;
function add(t){
  var atEnd=log.scrollHeight-log.scrollTop-log.clientHeight<40;
  t.split('\n').forEach(function(line){
    if(!line&&!t.trim())return;
    var d=document.createElement('div');
    if(/^\[ERROR\]/.test(line))d.className='e';
    else if(/^\[WARN/.test(line))d.className='w';
    else if(/^\[DEBUG\]/.test(line))d.className='d';
    else if(/^>/.test(line))d.className='cmd';
    d.textContent=line;log.appendChild(d);
  });
  while(log.childElementCount>800)log.removeChild(log.firstChild);
  if(atEnd)log.scrollTop=log.scrollHeight;
}
function connect(){
  ws=new WebSocket('ws://'+location.host+'/webserialws');
  ws.onopen=function(){dot.className='on';state.textContent='connected'};
  ws.onclose=function(){dot.className='';state.textContent='reconnecting';setTimeout(connect,2000)};
  ws.onerror=function(){ws.close()};
  ws.onmessage=function(e){if(e.data)add(e.data)};
}
connect();
document.getElementById('f').onsubmit=function(e){
  e.preventDefault();var i=document.getElementById('c'),v=i.value.trim();
  if(!v||!ws||ws.readyState!=1)return;
  ws.send(v);hist.push(v);hpos=hist.length;i.value='';
};
document.getElementById('c').onkeydown=function(e){
  if(e.key=='ArrowUp'&&hpos>0){hpos--;this.value=hist[hpos];e.preventDefault()}
  else if(e.key=='ArrowDown'){hpos=Math.min(hpos+1,hist.length);this.value=hist[hpos]||'';e.preventDefault()}
};
document.getElementById('clear').onclick=function(){log.innerHTML=''};
</script></body></html>)HTML";

// Browser console transport. Line buffering keeps partial output() calls together
// until a newline, so the page receives whole lines.
String console_buf;

void console_send(const char* text, bool add_newline) {
  if (console_ws.count() == 0) {
    console_buf = "";
    return;
  }
  console_buf += text;
  if (add_newline) console_buf += "\n";
  int cut = console_buf.lastIndexOf('\n');
  if (cut < 0) return;
  String chunk = console_buf.substring(0, cut);
  console_buf.remove(0, cut + 1);
  console_ws.textAll(chunk);
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
  console_send(line, true);
}

// For structured command output (status, config, help, etc.) — no prefix
void output(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  console_send(buf, false);
}

// Send a complete String as one WebSocket message (instant display, no line-by-line)
void ws_send_bulk(const String& text) {
  Serial.print(text);
  console_send(text.c_str(), false);
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

// GPIO 8 and 9 drive the LED and button; 11-17 are wired to flash
bool valid_motor_gpio(int pin) {
  if (pin < 0 || pin > 21) return false;
  if (pin == 8 || pin == 9) return false;
  if (pin >= 11 && pin <= 17) return false;
  return true;
}

const char* pin_profile_name() {
  return pin_profile == PIN_PROFILE_CUSTOM ? "custom" : PIN_PROFILES[pin_profile].name;
}

void apply_pin_profile(uint8_t index) {
  if (index == PIN_PROFILE_CUSTOM) {
    pin_profile = index;
    STEP_PIN = preferences.getInt("pin_step", PIN_PROFILES[0].step);
    DIR_PIN = preferences.getInt("pin_dir", PIN_PROFILES[0].dir);
    ENABLE_PIN = preferences.getInt("pin_en", PIN_PROFILES[0].enable);
    DIAG_PIN = preferences.getInt("pin_diag", PIN_PROFILES[0].diag);
    TMC_TX_PIN = preferences.getInt("pin_tx", PIN_PROFILES[0].tx);
    TMC_RX_PIN = preferences.getInt("pin_rx", PIN_PROFILES[0].rx);
    return;
  }

  if (index >= PIN_PROFILE_COUNT) index = 0;
  pin_profile = index;
  const PinProfile& p = PIN_PROFILES[index];
  STEP_PIN = p.step;
  DIR_PIN = p.dir;
  ENABLE_PIN = p.enable;
  DIAG_PIN = p.diag;
  TMC_TX_PIN = p.tx;
  TMC_RX_PIN = p.rx;
}

// ============================================================================
// STEP GENERATOR + STALLGUARD
// ============================================================================
// Step pulses are generated by a hardware timer ISR, which also ramps the
// step rate and evaluates StallGuard. Step timing must stay steady: the
// driver's MicroPlyer interpolation and StallGuard load measurement both
// depend on a constant step interval.

const uint32_t STEPS_UNLIMITED = 0xFFFFFFFF;
const uint32_t RAMP_TIME_US = 300000;    // accel/decel ramp duration
const uint32_t STALL_BLANK_US = 200000;  // StallGuard ignored this long after the ramp completes
const uint8_t STALL_CONFIRM = 6;         // score confirming a stall
const uint8_t STALL_HIT_WEIGHT = 2;      // score added per stalled full step, 1 subtracted per clean one
int cal_backoff_close = 15;              // margin kept clear of the closed end
int cal_backoff_open = 15;               // margin kept clear of the open end

hw_timer_t* step_timer = nullptr;
bool step_timer_on = false;
volatile bool step_running = false;
volatile int8_t step_dir = 1;
volatile uint32_t steps_left = 0;
volatile uint32_t steps_done = 0;
uint32_t step_period_us = 0;
volatile uint32_t step_cruise_us = 2000;
volatile uint32_t step_ramp_steps = 1;

// Stall detection state, evaluated once per full step in the step ISR
volatile uint32_t diag_edges = 0;
uint32_t diag_edges_seen = 0;
uint32_t stall_blank_steps = 0;
volatile bool stall_stop_enabled = false;
volatile uint8_t stall_score = 0;
volatile uint8_t stall_score_max = 0;
volatile uint32_t stall_fullsteps = 0;

void IRAM_ATTR diag_isr() {
  diag_edges = diag_edges + 1;
}

void IRAM_ATTR step_isr() {
  if (!step_running) return;

  REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << STEP_PIN);
  current_position = current_position + step_dir;
  steps_done = steps_done + 1;
  if (steps_left != STEPS_UNLIMITED) steps_left = steps_left - 1;

  // StallGuard updates once per full step. A full step counts as stalled if
  // DIAG pulsed since the last sample or is still high. Stalled steps add more
  // than clean steps subtract, so a slipping motor that alternates between
  // stalled and clean still accumulates, while isolated spikes decay away.
  if (steps_done >= stall_blank_steps && steps_done % motor_microsteps == 0) {
    uint32_t edges = diag_edges;
    bool hit = (edges != diag_edges_seen) || (REG_READ(GPIO_IN_REG) & (1UL << DIAG_PIN));
    diag_edges_seen = edges;
    if (hit) {
      stall_fullsteps = stall_fullsteps + 1;
      if (stall_score < 255 - STALL_HIT_WEIGHT) stall_score = stall_score + STALL_HIT_WEIGHT;
      if (stall_score > stall_score_max) stall_score_max = stall_score;
    } else if (stall_score > 0) {
      stall_score = stall_score - 1;
    }
    if (stall_stop_enabled && stall_score >= STALL_CONFIRM) {
      step_running = false;
    }
  }
  if (steps_left == 0) step_running = false;

  REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << STEP_PIN);  // preceding work covers the 100ns min pulse width

  // Velocity ramps linearly with distance from 25% to 100% of cruise; symmetric for decel
  if (step_running) {
    uint32_t s = steps_done < steps_left ? steps_done : steps_left;
    uint32_t p = step_cruise_us;
    if (s < step_ramp_steps) p = step_cruise_us * 4 * step_ramp_steps / (step_ramp_steps + 3 * s);
    if (p != step_period_us) {
      step_period_us = p;
      timerAlarm(step_timer, p, true, 0);
    }
  }
}

// Cruise period and ramp length, applied live so a speed change mid-move takes effect
void step_set_speed(uint32_t cruise_us) {
  step_cruise_us = cruise_us;
  step_ramp_steps = RAMP_TIME_US * 100 / 185 / cruise_us + 1;
}

void step_stop() {
  step_running = false;
  if (step_timer_on) {
    timerStop(step_timer);
    step_timer_on = false;
  }
}

// Start stepping in dir (+1 = open). count may be STEPS_UNLIMITED.
void step_start(int8_t dir, uint32_t count, bool stop_on_stall) {
  step_stop();
  step_dir = dir;
  digitalWrite(DIR_PIN, dir > 0 ? HIGH : LOW);
  steps_left = count;
  steps_done = 0;
  step_set_speed(step_delay_us);  // ramp duration ~= 1.85 * ramp_steps * cruise period
  stall_blank_steps = step_ramp_steps + STALL_BLANK_US / step_cruise_us;
  stall_stop_enabled = stop_on_stall;
  stall_score = 0;
  stall_score_max = 0;
  stall_fullsteps = 0;
  diag_edges_seen = diag_edges;
  step_period_us = step_cruise_us * 4;
  delayMicroseconds(10);  // DIR setup time

  timerWrite(step_timer, 0);
  timerAlarm(step_timer, step_period_us, true, 0);
  step_running = true;
  timerStart(step_timer);
  step_timer_on = true;
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

  step_timer = timerBegin(1000000);  // 1 MHz = 1us resolution
  timerAttachInterrupt(step_timer, &step_isr);
  timerStop(step_timer);

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
  tmc_apply_microsteps(motor_microsteps);

  // Current comes from the UART setting alone; the module's VREF trimpot
  // otherwise scales it and makes rms_current() advisory
  driver.I_scale_analog(false);

  // StealthChop for quiet operation
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);

  // StallGuard configuration (for calibration)
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(0);
  driver.semax(0);
  driver.SGTHRS(stall_threshold);

  // Apply direction inversion (after GCONF setup so it doesn't get overwritten)
  driver.shaft(invert_direction);

  // DIAG pulse counter, sampled per full step by the step ISR
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), diag_isr, RISING);

  log_msg(LOG_INFO, "TMC", "TMC2209 initialized successfully");
}

// TMCStepper's microsteps() takes 0 for full step; 1 is not a case in its switch
// and would be silently ignored, leaving the driver at its 256 power-on default.
static void tmc_apply_microsteps(uint16_t ms) {
  driver.microsteps(ms == 1 ? 0 : ms);
  uint16_t reported = driver.microsteps();
  uint16_t actual = reported == 0 ? 1 : reported;
  if (actual != ms) {
    log_msg(LOG_ERROR, "TMC", "Microsteps not applied: asked %d, driver reports %d", ms, actual);
  }
}

// Step interval for the requested shaft speed at the current resolution, so
// changing microsteps alters smoothness only, not how fast the curtain moves
void apply_motor_rpm() {
  uint32_t period = 60000000UL /
                    ((uint32_t)motor_rpm * MOTOR_FULL_STEPS_PER_REV * motor_microsteps);
  if (period < MIN_STEP_PERIOD_US) {
    log_msg(LOG_WARN, "TMC", "%d RPM at %d microsteps needs %luus/step, floor is %dus",
            motor_rpm, motor_microsteps, (unsigned long)period, MIN_STEP_PERIOD_US);
    period = MIN_STEP_PERIOD_US;
  }
  step_delay_us = (int)period;
  if (step_running) step_set_speed(step_delay_us);
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
  if (step_running) {
    log_msg(LOG_WARN, "TMC", "Cannot change microsteps while moving");
    return;
  }

  // Position, travel range and step delay are all counted in microsteps, so they
  // have to be rescaled or the curtain's stored geometry and speed change with
  // the resolution
  if (ms != motor_microsteps && motor_microsteps > 0) {
    current_position = (int)((int64_t)current_position * ms / motor_microsteps);
    travel_steps = (int)((int64_t)travel_steps * ms / motor_microsteps);

    preferences.putInt("position", current_position);
    preferences.putInt("steps_per_rev", travel_steps);
    log_msg(LOG_INFO, "TMC", "Rescaled for %d microsteps: position %d, travel %d",
            ms, current_position, travel_steps);
  }

  motor_microsteps = ms;
  tmc_apply_microsteps(ms);
  preferences.putUShort("microsteps", ms);
  apply_motor_rpm();
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
  step_stop();
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

void save_position() {
  preferences.putInt("position", current_position);
}

void start_movement(int target) {
  if (is_moving || cal_state != CAL_IDLE || motor_test_active) return;
  if (!tmc_available) {
    log_msg(LOG_ERROR, "MOTOR", "TMC2209 not available, cannot move");
    return;
  }

  target_position = constrain(target, 0, travel_steps);
  if (current_position == target_position) {
    log_msg(LOG_INFO, "MOTOR", "Already at position %d", current_position);
    return;
  }

  wake_motor();

  int8_t dir;
  if (target_position > current_position) {
    dir = 1;
    log_msg(LOG_INFO, "MOTOR", "Opening: %d -> %d (%d%%)",
            current_position, target_position,
            (target_position * 100) / travel_steps);
    publish_status("opening");
  } else {
    dir = -1;
    log_msg(LOG_INFO, "MOTOR", "Closing: %d -> %d (%d%%)",
            current_position, target_position,
            (target_position * 100) / travel_steps);
    publish_status("closing");
  }

  is_moving = true;
  movement_start_time = millis();
  last_position_save = millis();
  last_position_report = 0;
  publish_position();

  // Normal moves count stalls for logging but do not stop on them
  step_start(dir, abs(target_position - current_position), false);
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

  if (millis() - movement_start_time > MOVEMENT_TIMEOUT) {
    log_msg(LOG_ERROR, "MOTOR", "Movement timeout after %lums", MOVEMENT_TIMEOUT);
    publish_status("error_timeout");
    stop_movement("Timeout");
    return;
  }

  if (!step_running) {
    log_msg(LOG_INFO, "MOTOR", "Movement complete, position %d (%d%%)",
            current_position, (current_position * 100) / travel_steps);
    stop_movement("Complete");
    return;
  }

  last_motor_activity = millis();

  // Interval-based: each NVS write masks the step ISR while flash is busy
  if (millis() - last_position_save >= POSITION_SAVE_INTERVAL) {
    save_position();
    last_position_save = millis();
  }

  if (millis() - last_position_report >= POSITION_REPORT_INTERVAL) {
    publish_position();
    last_position_report = millis();

    // Verbose TMC output during movement
    if (tmc_verbose && tmc_available) {
      uint16_t sg = driver.SG_RESULT();
      uint32_t drv = driver.DRV_STATUS();
      uint8_t cs = (drv >> 16) & 0x1F;  // Current scale
      log_msg(LOG_DEBUG, "MOTOR", "SG:%3d (stall<=%d) CS:%2d/31 stalled_fs:%lu score:%d/%d pos:%d",
              sg, stall_threshold * 2, cs, stall_fullsteps, stall_score, STALL_CONFIRM,
              current_position);
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
  if (is_moving || cal_state != CAL_IDLE || motor_test_active) {
    log_msg(LOG_WARN, "CAL", "Cannot calibrate: motor busy");
    return;
  }

  log_msg(LOG_INFO, "CAL", "Starting sensorless calibration — finding closed position...");

  wake_motor();
  cal_state = CAL_FIND_MIN;
  is_moving = true;
  cal_start_time = millis();
  step_start(-1, STEPS_UNLIMITED, true);
}

void abort_calibration(const char* why) {
  log_msg(LOG_ERROR, "CAL", "Calibration failed: %s", why);
  cal_state = CAL_IDLE;
  is_moving = false;
  stop_motor();
  publish_status("stopped");
}

void handle_calibration() {
  if (cal_state == CAL_IDLE) return;

  if (millis() - cal_start_time > CAL_TIMEOUT) {
    abort_calibration("timeout, no stall detected (try higher sensitivity)");
    return;
  }

  last_motor_activity = millis();
  if (step_running) return;

  const int backoff_close = cal_backoff_close * motor_microsteps;
  const int backoff_open = cal_backoff_open * motor_microsteps;

  switch (cal_state) {
    case CAL_FIND_MIN:
      log_msg(LOG_INFO, "CAL", "Found closed boundary: stall confirmed after %lu steps (score %d/%d; %lu flagged full steps this run)",
              steps_done, STALL_CONFIRM, STALL_CONFIRM, stall_fullsteps);
      step_start(1, backoff_close, false);
      cal_state = CAL_BACKOFF_MIN;
      break;

    case CAL_BACKOFF_MIN:
      current_position = 0;  // This backed-off spot IS position 0
      save_position();
      log_msg(LOG_INFO, "CAL", "Backed off %d steps, set as position 0. Finding open position...", backoff_close);
      step_start(1, STEPS_UNLIMITED, true);
      cal_state = CAL_FIND_MAX;
      break;

    case CAL_FIND_MAX:
      log_msg(LOG_INFO, "CAL", "Found open boundary at %d steps from safe-close: stall confirmed (score %d/%d; %lu flagged full steps this run)",
              current_position, STALL_CONFIRM, STALL_CONFIRM, stall_fullsteps);
      if (current_position < 4 * backoff_close) {
        abort_calibration("travel too short, likely a false stall (try lower sensitivity, run motortest)");
        return;
      }
      step_start(-1, backoff_open, false);
      cal_state = CAL_BACKOFF_MAX;
      break;

    case CAL_BACKOFF_MAX:
      // Close back-off is already accounted for in position 0
      travel_steps = current_position;
      preferences.putInt("steps_per_rev", travel_steps);
      save_position();
      log_msg(LOG_INFO, "CAL", "Calibration complete! Usable range: %d steps, clear by %d closed / %d open full steps",
              travel_steps, cal_backoff_close, cal_backoff_open);
      cal_state = CAL_IDLE;
      is_moving = false;
      publish_position();
      publish_ha_discovery(true);
      break;

    default:
      break;
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
    int percentage = travel_steps > 0 ? (current_position * 100) / travel_steps : 0;
    percentage = constrain(percentage, 0, 100);
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
  cal_doc["entity_category"] = "config";
    doc["availability_topic"] = mqtt_availability_topic;
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
    String topic = "homeassistant/number/" + device_hostname + "_speed_rpm/config";
    StaticJsonDocument<512> doc;
    doc["name"] = "Speed";
    doc["unique_id"] = "curtain_" + device_hostname + "_speed_rpm";
    doc["object_id"] = device_hostname + "_speed_rpm";
    doc["command_topic"] = mqtt_speed_set_topic;
    doc["state_topic"] = mqtt_speed_state_topic;
    doc["min"] = 10;
    doc["max"] = 300;
    doc["step"] = 5;
    doc["unit_of_measurement"] = "RPM";
    doc["icon"] = "mdi:speedometer";
    doc["entity_category"] = "config";
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
    doc["entity_category"] = "config";
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
    doc["entity_category"] = "config";
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
    doc["entity_category"] = "config";
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
    doc["entity_category"] = "config";
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

  // Calibration back-off number entities, one per end
  for (int end = 0; end < 2; end++) {
    bool is_open = end == 1;
    String key = is_open ? "backoff_open" : "backoff_close";
    String topic = "homeassistant/number/" + device_hostname + "_" + key + "/config";
    StaticJsonDocument<512> doc;
    doc["name"] = is_open ? "Back-off Open" : "Back-off Close";
    doc["unique_id"] = "curtain_" + device_hostname + "_" + key;
    doc["object_id"] = device_hostname + "_" + key;
    doc["command_topic"] = is_open ? mqtt_backoff_open_set_topic : mqtt_backoff_close_set_topic;
    doc["state_topic"] = is_open ? mqtt_backoff_open_state_topic : mqtt_backoff_close_state_topic;
    doc["min"] = 1;
    doc["max"] = 2000;
    doc["step"] = 5;
    doc["unit_of_measurement"] = "full steps";
    doc["icon"] = "mdi:arrow-collapse-horizontal";
    doc["entity_category"] = "config";
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

  // Remove entities replaced by newer ones
  String old_topic = "homeassistant/number/" + device_hostname + "_stallthreshold/config";
  client.publish(old_topic.c_str(), "", true);
  old_topic = "homeassistant/number/" + device_hostname + "_speed/config";
  client.publish(old_topic.c_str(), "", true);
  old_topic = "homeassistant/number/" + device_hostname + "_backoff/config";
  client.publish(old_topic.c_str(), "", true);

  log_msg(LOG_INFO, "MQTT", "HA settings entities published");
}

void publish_settings_state() {
  if (!client.connected()) return;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", motor_rpm);
  client.publish(mqtt_speed_state_topic.c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%d", motor_current_ma);
  client.publish(mqtt_current_state_topic.c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%d", cal_backoff_close);
  client.publish(mqtt_backoff_close_state_topic.c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%d", cal_backoff_open);
  client.publish(mqtt_backoff_open_state_topic.c_str(), buf, true);
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
    if (value >= 10 && value <= 300) {
      motor_rpm = value;
      preferences.putInt("rpm", motor_rpm);
      apply_motor_rpm();
      log_msg(LOG_INFO, "MQTT", "Speed set to %d RPM", motor_rpm);
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", motor_rpm);
      client.publish(mqtt_speed_state_topic.c_str(), buf, true);
    }
    return;
  }
  if (strcmp(topic, mqtt_backoff_close_set_topic.c_str()) == 0 ||
      strcmp(topic, mqtt_backoff_open_set_topic.c_str()) == 0) {
    bool is_open = strcmp(topic, mqtt_backoff_open_set_topic.c_str()) == 0;
    int value = msg.toInt();
    if (value >= 1 && value <= 2000) {
      if (is_open) {
        cal_backoff_open = value;
        preferences.putInt("backoff_open", value);
      } else {
        cal_backoff_close = value;
        preferences.putInt("backoff_close", value);
      }
      log_msg(LOG_INFO, "MQTT", "%s back-off set to %d full steps", is_open ? "Open" : "Close", value);
      publish_backoff_state();
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
    client.subscribe(mqtt_backoff_close_set_topic.c_str());
    client.subscribe(mqtt_backoff_open_set_topic.c_str());
    client.subscribe(mqtt_stallthreshold_set_topic.c_str());
    client.subscribe(mqtt_microsteps_set_topic.c_str());
    client.subscribe(mqtt_invert_set_topic.c_str());
    client.publish(mqtt_availability_topic.c_str(), "online", true);
    publish_position();
    publish_status(current_position >= travel_steps ? "open" :
                   current_position <= 0 ? "closed" : "open");
    publish_ha_discovery(true);
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
  mqtt_speed_set_topic = mqtt_root_topic + "/speed_rpm/set";
  mqtt_speed_state_topic = mqtt_root_topic + "/speed_rpm/state";
  mqtt_current_set_topic = mqtt_root_topic + "/current/set";
  mqtt_current_state_topic = mqtt_root_topic + "/current/state";
  mqtt_backoff_close_set_topic = mqtt_root_topic + "/backoff_close/set";
  mqtt_backoff_close_state_topic = mqtt_root_topic + "/backoff_close/state";
  mqtt_backoff_open_set_topic = mqtt_root_topic + "/backoff_open/set";
  mqtt_backoff_open_state_topic = mqtt_root_topic + "/backoff_open/state";
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
  start_movement(travel_steps);
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
  if (value >= 10 && value <= 300) {
    motor_rpm = value;
    preferences.putInt("rpm", motor_rpm);
    apply_motor_rpm();
    log_msg(LOG_INFO, "NVS", "Speed set to %d RPM (%dus/step at %d microsteps)",
            motor_rpm, step_delay_us, motor_microsteps);
    if (client.connected()) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d", motor_rpm);
      client.publish(mqtt_speed_state_topic.c_str(), buf, true);
    }
  } else {
    log_msg(LOG_ERROR, "CMD", "speed: value must be 10-300 RPM (got %d)", value);
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

void publish_backoff_state() {
  if (!client.connected()) return;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", cal_backoff_close);
  client.publish(mqtt_backoff_close_state_topic.c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%d", cal_backoff_open);
  client.publish(mqtt_backoff_open_state_topic.c_str(), buf, true);
}

void cmd_backoff(const String& param) {
  String p = param;
  p.trim();
  p.toLowerCase();

  int space = p.indexOf(' ');
  if (space > 0) {
    String end = p.substring(0, space);
    int value = p.substring(space + 1).toInt();
    if (value >= 1 && value <= 2000 && (end == "open" || end == "close")) {
      if (end == "open") {
        cal_backoff_open = value;
        preferences.putInt("backoff_open", value);
      } else {
        cal_backoff_close = value;
        preferences.putInt("backoff_close", value);
      }
      log_msg(LOG_INFO, "NVS", "%s back-off set to %d full steps (%d microsteps)",
              end.c_str(), value, value * motor_microsteps);
      publish_backoff_state();
      return;
    }
    log_msg(LOG_ERROR, "CMD", "backoff: use 'backoff open <1-2000>' or 'backoff close <1-2000>'");
    return;
  }

  output("Back-off: closed end %d, open end %d full steps\n", cal_backoff_close, cal_backoff_open);
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
    travel_steps = value;
    preferences.putInt("steps_per_rev", travel_steps);
    log_msg(LOG_INFO, "NVS", "Total travel steps set to %d", travel_steps);
  } else {
    log_msg(LOG_ERROR, "CMD", "travelsteps: must be 1-500000 (got %d)", value);
  }
}

void cmd_setposition(const String& param) {
  int value = param.toInt();
  if (value >= 0 && value <= travel_steps) {
    current_position = value;
    save_position();
    publish_position();
    log_msg(LOG_INFO, "NVS", "Position reset to %d", current_position);
  } else {
    log_msg(LOG_ERROR, "CMD", "setposition: must be 0-%d (got %d)", travel_steps, value);
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
  buf_printf(out, "Speed: %d RPM (%dus/step at %d microsteps)\n", motor_rpm, step_delay_us, motor_microsteps);
  buf_printf(out, "Microsteps: %d\n", motor_microsteps);
  buf_printf(out, "Current: %d mA\n", motor_current_ma);
  buf_printf(out, "Sensitivity: %s (threshold=%d)\n", sensitivity_name(stall_threshold), stall_threshold);
  buf_printf(out, "Pin profile: %s (STEP:%d DIR:%d EN:%d DIAG:%d TX:%d RX:%d)\n",
             pin_profile_name(), STEP_PIN, DIR_PIN, ENABLE_PIN, DIAG_PIN,
             TMC_TX_PIN, TMC_RX_PIN);
  buf_printf(out, "End back-off: closed %d, open %d full steps\n", cal_backoff_close, cal_backoff_open);
  buf_printf(out, "Travel Steps: %d\n", travel_steps);
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
    "speed <rpm>       Shaft speed (10-300 RPM)\n"
    "current <mA>      Motor current (100-2000)\n"
    "microsteps <n>    Microsteps (1,2,4,8,16,32,64,128,256)\n"
    "sensitivity <lvl> Stall sensitivity (extra_low|low|medium|high|max|custom N)\n"
    "backoff [end n]   Show, or set 'open'/'close' clearance in full steps\n"
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
    "tmcdiag           Probe the TMC2209 UART link and report what came back\n"
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
         (current_position * 100) / travel_steps);
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
  if (is_moving || cal_state != CAL_IDLE) {
    output("Motor busy\n");
    return;
  }

  int duration = param.length() > 0 ? param.toInt() : 5;
  duration = constrain(duration, 1, 60);
  uint16_t stall_line = stall_threshold * 2;

  output("\n=== Motor Test ===\n");
  output("Duration: %d s, speed: %d us/step, microsteps: %d\n", duration, step_delay_us, motor_microsteps);
  output("Sensitivity: %s (threshold=%d, stall when SG<=%d)\n",
         sensitivity_name(stall_threshold), stall_threshold, stall_line);
  output("Uses the same detector as calibration. Apply resistance to the shaft to test.\n\n");

  wake_motor();
  motor_test_active = true;
  int position_before = current_position;  // a test must not move the coordinate system
  step_start(1, STEPS_UNLIMITED, false);   // count stalls without stopping

  unsigned long test_start = millis();
  unsigned long last_sample = 0, last_report = 0;
  uint16_t free_min = 1023, free_max = 0, latest = 0;
  uint32_t free_sum = 0, free_count = 0, read_errors = 0;
  uint8_t peak_score = 0, interval_peak = 0;
  uint32_t interval_mark = 0;
  int stall_events = 0;

  while (millis() - test_start < (unsigned long)duration * 1000) {
    unsigned long now = millis();
    bool settled = steps_done >= stall_blank_steps;

    // Sampled every pass: a short stall can start and end between report lines
    uint8_t score_now = stall_score;
    if (score_now > interval_peak) interval_peak = score_now;
    if (score_now > peak_score) peak_score = score_now;

    if (settled && now - last_sample >= 50) {
      last_sample = now;
      uint16_t sg = driver.SG_RESULT();
      if (driver.CRCerror) {  // a failed read returns 0, which is not a load value
        read_errors++;
        continue;
      }
      latest = sg;
      // Free-running baseline: samples taken while nothing is flagged as stalled
      if (score_now == 0) {
        if (sg < free_min) free_min = sg;
        if (sg > free_max) free_max = sg;
        free_sum += sg;
        free_count++;
      }
    }

    if (now - last_report >= 500) {
      last_report = now;
      if (!settled) {
        output("  Ramping up...\n");
      } else {
        uint32_t interval_steps = stall_fullsteps - interval_mark;
        interval_mark = stall_fullsteps;
        bool tripped = interval_peak >= STALL_CONFIRM;
        if (tripped) stall_events++;

        // Visual load bar: higher SG = more blocks = lighter load
        int blocks = constrain(latest / 40, 0, 16);
        char bar[17];
        for (int i = 0; i < 16; i++) bar[i] = (i < blocks) ? '#' : '.';
        bar[16] = 0;

        output("  SG:%4d [%s] stalled:%lu peak:%d/%d%s\n", latest, bar,
               (unsigned long)interval_steps, interval_peak, STALL_CONFIRM,
               tripped ? "  STALL!" : "");
        interval_peak = 0;
      }
      esp_task_wdt_reset();
    }

    // Keep the network alive: this loop blocks the main loop for the whole test,
    // and PubSubClient's keepalive is 15s
    client.loop();
    ArduinoOTA.handle();

    if (ws_command_pending) {
      bool abort = strcmp(ws_pending_command, "stop") == 0;
      ws_command_pending = false;  // consume it either way, or the console latches
      if (abort) {
        output("  Aborted\n");
        break;
      }
      output("  Ignored during test: %s\n", ws_pending_command);
    }
    delay(1);
  }

  stop_motor();
  motor_test_active = false;
  current_position = position_before;
  save_position();
  publish_position();

  output("\n=== Results ===\n");
  if (free_count == 0) {
    output("No free-running samples: the motor was loaded the whole time.\n");
  } else {
    output("Free-running SG: avg %lu, min %d, max %d (stall line: %d)\n",
           (unsigned long)(free_sum / free_count), free_min, free_max, stall_line);
  }
  output("Stall events: %d (peak score %d, confirm at %d)\n", stall_events, peak_score, STALL_CONFIRM);
  output("Stalled full steps total: %lu\n", (unsigned long)stall_fullsteps);
  if (read_errors > 0) {
    output("UART read errors: %lu (skipped; check wiring if frequent)\n", (unsigned long)read_errors);
  }

  if (stall_events > 0) {
    output("\nStalls were detected. If you applied resistance, the detector is working.\n");
    output("If the shaft was free the whole run, lower the sensitivity.\n");
  } else if (stall_fullsteps > 0) {
    output("\nSome stalled full steps, none sustained enough to confirm.\n");
  } else {
    output("\nNo stall pulses at all this run.\n");
  }

  if (free_count > 0) {
    // Stall line at ~60% of the lowest free-running load leaves margin both ways
    int suggested = constrain(free_min * 3 / 10, 1, 255);
    output("Suggested sensitivity for this speed and current: custom %d\n", suggested);
    if (free_min < stall_line) {
      output("Warning: free-running load already dips below the stall line.\n");
    }
  }

  uint32_t avg = free_count ? free_sum / free_count : 0;
  if (free_count > 0 && avg < 50) {
    output("\nWarning: heavy load even when free. Check for binding.\n");
    output("Try:   current %d\n", constrain(motor_current_ma + 200, 100, 2000));
  }

  output("===============\n");
}

// TMC2209 UART datagram CRC (datasheet section 5.2)
static uint8_t tmc_crc(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc >> 7) ^ (byte & 0x01)) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
      byte >>= 1;
    }
  }
  return crc;
}

// Send a read request and capture everything that lands on RX: on a single-wire
// bus that is the 4-byte echo of our own request, then the driver's 8-byte reply.
static int tmc_probe(uint8_t address, uint8_t reg, uint8_t* buf, int max_len) {
  while (TMCSerial.available()) TMCSerial.read();

  uint8_t req[4] = { 0x05, address, reg, 0 };
  req[3] = tmc_crc(req, 3);
  TMCSerial.write(req, 4);
  TMCSerial.flush();

  int len = 0;
  unsigned long deadline = millis() + 100;
  while (len < max_len && millis() < deadline) {
    if (TMCSerial.available()) buf[len++] = TMCSerial.read();
  }
  return len;
}

// A valid capture is echo + reply with a good CRC; returns the register value
static bool tmc_parse(const uint8_t* buf, int len, uint32_t* value) {
  if (len < 12) return false;
  const uint8_t* reply = buf + 4;
  if (reply[0] != 0x05 || reply[1] != 0xFF) return false;
  if (tmc_crc(reply, 7) != reply[7]) return false;
  *value = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16) |
           ((uint32_t)reply[5] << 8) | reply[6];
  return true;
}

static bool tmc_read_reg(uint8_t address, uint8_t reg, uint32_t* value) {
  uint8_t buf[16];
  int len = tmc_probe(address, reg, buf, sizeof(buf));
  return tmc_parse(buf, len, value);
}

void cmd_tmcdiag(const String& param) {
  String out;
  out.reserve(1500);

  buf_printf(out, "\n=== TMC2209 Diagnostics ===\n");
  buf_printf(out, "Firmware pins: STEP:%d DIR:%d EN:%d DIAG:%d TX:%d RX:%d\n",
             STEP_PIN, DIR_PIN, ENABLE_PIN, DIAG_PIN, TMC_TX_PIN, TMC_RX_PIN);
  buf_printf(out, "Detected at boot: %s\n", tmc_available ? "yes" : "no");
  buf_printf(out, "DIAG pin now: %s\n", digitalRead(DIAG_PIN) ? "HIGH" : "LOW");

  uint8_t buf[16];
  int found_addr = -1;
  bool any_echo = false, any_bytes = false;

  for (uint8_t addr = 0; addr < 4; addr++) {
    int len = tmc_probe(addr, 0x06, buf, sizeof(buf));  // IOIN
    if (len > 0) any_bytes = true;

    bool echo = (len >= 4 && buf[0] == 0x05 && buf[1] == addr && buf[2] == 0x06);
    if (echo) any_echo = true;

    uint32_t ioin = 0;
    bool reply = tmc_parse(buf, len, &ioin);

    buf_printf(out, "addr %d: %d bytes", addr, len);
    if (len > 0) {
      buf_printf(out, " [");
      for (int i = 0; i < len && i < 12; i++) buf_printf(out, "%02X ", buf[i]);
      buf_printf(out, "]");
    }
    buf_printf(out, " echo:%s reply:%s", echo ? "yes" : "no", reply ? "yes" : "no");
    if (reply) {
      buf_printf(out, " IOIN:0x%08X version:0x%02X", ioin, (uint8_t)(ioin >> 24));
      if (found_addr < 0) found_addr = addr;
    }
    buf_printf(out, "\n");
    yield();
  }

  if (found_addr >= 0) {
    uint32_t before = 0, after = 0;
    int len = tmc_probe(found_addr, 0x02, buf, sizeof(buf));  // IFCNT
    bool got_before = tmc_parse(buf, len, &before);
    driver.SGTHRS(stall_threshold);  // a write the driver should count
    len = tmc_probe(found_addr, 0x02, buf, sizeof(buf));
    bool got_after = tmc_parse(buf, len, &after);
    if (got_before && got_after) {
      buf_printf(out, "IFCNT: %u -> %u (writes %s)\n", before, after,
                 after != before ? "accepted" : "NOT accepted");
    }
  }

  if (found_addr >= 0) {
    uint32_t gconf = 0, chop = 0, drv = 0, tstep = 0, sg = 0, pwm = 0, sgthrs = 0, ihold = 0;
    bool has_gconf = tmc_read_reg(found_addr, 0x00, &gconf);
    bool has_chop = tmc_read_reg(found_addr, 0x6C, &chop);
    bool has_drv = tmc_read_reg(found_addr, 0x6F, &drv);
    tmc_read_reg(found_addr, 0x12, &tstep);
    tmc_read_reg(found_addr, 0x41, &sg);
    tmc_read_reg(found_addr, 0x71, &pwm);
    bool has_sgthrs = tmc_read_reg(found_addr, 0x40, &sgthrs);
    bool has_ihold = tmc_read_reg(found_addr, 0x10, &ihold);

    buf_printf(out, "-- Live registers --\n");

    if (has_gconf) {
      buf_printf(out, "GCONF     0x%08X  %s inv:%s uart:%s analog_iref:%s index_step:%s\n", gconf,
                 (gconf & (1 << 2)) ? "SpreadCycle" : "StealthChop",
                 (gconf & (1 << 3)) ? "yes" : "no",
                 (gconf & (1 << 6)) ? "on" : "off",
                 (gconf & (1 << 0)) ? "yes" : "no",
                 (gconf & (1 << 5)) ? "yes" : "no");
    }

    if (has_chop) {
      uint8_t mres = (chop >> 24) & 0x0F;
      uint8_t toff = chop & 0x0F;
      bool vsense = chop & (1UL << 17);
      buf_printf(out, "CHOPCONF  0x%08X  microsteps:%d interpolate:%s toff:%d vsense:%s\n", chop,
                 256 >> mres, (chop & (1UL << 28)) ? "yes" : "no", toff, vsense ? "high" : "low");
      if (toff == 0) buf_printf(out, "          toff=0: driver output is off\n");
    }

    if (has_drv) {
      uint8_t cs = (drv >> 16) & 0x1F;
      float vfs = (has_chop && (chop & (1UL << 17))) ? 0.180f : 0.325f;
      int ma = (int)(((cs + 1) / 32.0f) * (vfs / (R_SENSE + 0.02f)) / 1.41421f * 1000.0f);
      bool standstill = drv & (1UL << 31);
      buf_printf(out, "DRV_STATUS 0x%08X CS:%d/31 (~%dmA rms %s) %s %s\n", drv, cs, ma,
                 standstill ? "hold" : "run",
                 (drv & (1UL << 30)) ? "stealth" : "spread",
                 standstill ? "standstill" : "running");
      if (standstill) {
        buf_printf(out, "          hold current is a fraction of run current, so this reads below the setting\n");
      }
      if (drv & 0x3F) {
        buf_printf(out, "          flags: otpw:%d ot:%d s2g:%d%d s2vs:%d%d\n", (int)(drv & 1),
                   (int)((drv >> 1) & 1), (int)((drv >> 2) & 1), (int)((drv >> 3) & 1),
                   (int)((drv >> 4) & 1), (int)((drv >> 5) & 1));
      }
    }

    buf_printf(out, "TSTEP:%u SG_RESULT:%u PWM_SCALE:0x%04X\n", tstep & 0xFFFFF, sg & 0x3FF,
               (unsigned)(pwm & 0xFFFF));

    // These are write-only on the TMC2209; a successful read of 0 means no readback
    buf_printf(out, "SGTHRS read:%s firmware:%d | IHOLD_IRUN read:%s firmware:%dmA\n",
               has_sgthrs ? String(sgthrs).c_str() : "no reply", stall_threshold,
               has_ihold ? String(ihold).c_str() : "no reply", motor_current_ma);
  }

  buf_printf(out, "--\n");
  if (found_addr >= 0) {
    buf_printf(out, "Link OK at address %d.\n", found_addr);
    if (found_addr != DRIVER_ADDRESS) {
      buf_printf(out, "Firmware expects address %d — check MS1/MS2.\n", DRIVER_ADDRESS);
    }
  } else if (any_echo) {
    buf_printf(out, "Echo but no reply: wiring to the ESP32 is fine, the driver is not answering.\n");
    buf_printf(out, "Check VM (motor supply) and VDD, and that MS1/MS2 set the expected address.\n");
  } else if (any_bytes) {
    buf_printf(out, "Garbled bytes: baud mismatch or bus contention.\n");
  } else {
    buf_printf(out, "Nothing on RX, not even our own echo.\n");
    buf_printf(out, "TX is not reaching RX — check both go to the driver's UART pad, and the pins above.\n");
  }
  buf_printf(out, "===========================\n");
  ws_send_bulk(out);
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
  {"backoff ", cmd_backoff},
  {"backoff", cmd_backoff},
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
  {"tmcdiag", cmd_tmcdiag},
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
      int target_steps = (percentage * travel_steps) / 100;
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
    select{appearance:none;-webkit-appearance:none;background-image:linear-gradient(45deg,transparent 50%,var(--dim) 50%),linear-gradient(135deg,var(--dim) 50%,transparent 50%);background-position:calc(100% - 18px) 50%,calc(100% - 13px) 50%;background-size:5px 5px,5px 5px;background-repeat:no-repeat;padding-right:36px}
    select option{background:#12122a;color:var(--text)}
    .hint{color:var(--dim);font-size:11px;margin-top:2px}
    .btn-row{display:flex;gap:12px;margin-top:20px}
    .btn{flex:1;padding:14px;font-size:15px;border-radius:12px;border:none;cursor:pointer;font-weight:600;transition:all 0.2s;text-align:center;text-decoration:none}
    .btn-primary{background:var(--gradient);color:#fff;box-shadow:0 4px 16px rgba(0,212,255,0.3)}
    .btn-primary:hover{transform:translateY(-1px);box-shadow:0 6px 20px rgba(0,212,255,0.4)}
    .btn-secondary{background:var(--card);color:var(--dim);border:1px solid var(--border)}
    .btn-secondary:hover{border-color:var(--cyan);color:var(--text)}
    .version{text-align:center;color:rgba(255,255,255,0.15);font-size:10px;margin-top:16px}
    .tabs{display:flex;gap:6px;margin-bottom:16px}
    .tab{flex:1;padding:10px 6px;font-size:13px;font-weight:600;border-radius:10px;border:1px solid var(--border);background:var(--card);color:var(--dim);cursor:pointer;transition:all 0.2s}
    .tab:hover{color:var(--text)}
    .tab.on{background:var(--gradient);color:#fff;border-color:transparent}
    .panel{display:none}
    .panel.on{display:block}
    .pins{display:grid;grid-template-columns:1fr 1fr;gap:0 12px;margin-top:8px}
    .pins label{margin-top:10px}
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

    <div class="tabs">
      <button type="button" class="tab on" data-panel="network">Network</button>
      <button type="button" class="tab" data-panel="motor">Motor</button>
      <button type="button" class="tab" data-panel="system">System</button>
    </div>

    <form action="/save" method="POST">
      <div class="panel on" id="network">
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

      </div>

      <div class="panel" id="motor">
      <div class="card">
        <h2>Motor</h2>
        <label>Speed (RPM)</label>
        <input name="rpm" type="number" value="%RPM%" min="10" max="300">
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
        <label>Close Back-off (full steps)</label>
        <input name="backoff_close" type="number" value="%BACKOFF_CLOSE%" min="1" max="2000">
        <label>Open Back-off (full steps)</label>
        <input name="backoff_open" type="number" value="%BACKOFF_OPEN%" min="1" max="2000">
        <div class="hint">Clearance kept at each end. The ends often need different values.</div>
      </div>

      </div>

      <div class="panel" id="system">
      <div class="card">
        <h2>System</h2>
        <label>OTA Password</label>
        <input name="ota_pass" type="password" placeholder="Leave blank to keep current">
        <label>Wiring Profile</label>
        <select name="pinprofile" id="pinprofile">
          <option value="0" %PIN0%>Current</option>
          <option value="1" %PIN1%>Legacy</option>
          <option value="250" %PINC%>Custom</option>
        </select>
        <div class="hint">Must match how this board is wired. The wrong profile leaves the motor unresponsive.</div>
        <div class="pins">
          <div><label>STEP</label><input name="pin_step" id="pin_step" type="number" value="%PIN_STEP%" min="0" max="21"></div>
          <div><label>DIR</label><input name="pin_dir" id="pin_dir" type="number" value="%PIN_DIR%" min="0" max="21"></div>
          <div><label>EN</label><input name="pin_en" id="pin_en" type="number" value="%PIN_EN%" min="0" max="21"></div>
          <div><label>DIAG</label><input name="pin_diag" id="pin_diag" type="number" value="%PIN_DIAG%" min="0" max="21"></div>
          <div><label>UART TX</label><input name="pin_tx" id="pin_tx" type="number" value="%PIN_TX%" min="0" max="21"></div>
          <div><label>UART RX</label><input name="pin_rx" id="pin_rx" type="number" value="%PIN_RX%" min="0" max="21"></div>
        </div>
        <div class="hint">GPIO 0-7, 10, 18-21. Editing a pin switches the profile to Custom.</div>
        <div class="hint">Device will reboot after saving.</div>
      </div>
      </div>

      <div class="btn-row">
        <button type="submit" class="btn btn-primary">Save &amp; Reboot</button>
        <a href="/webserial" class="btn btn-secondary">Console</a>
      </div>
    </form>

    <script>
      var PRESETS = {0:[6,5,20,21,7,10], 1:[10,6,0,7,21,20]};
      var PIN_IDS = ['pin_step','pin_dir','pin_en','pin_diag','pin_tx','pin_rx'];
      var profileSelect = document.getElementById('pinprofile');
      profileSelect.onchange = function() {
        var preset = PRESETS[this.value];
        if (!preset) return;
        for (var i = 0; i < PIN_IDS.length; i++) {
          document.getElementById(PIN_IDS[i]).value = preset[i];
        }
      };
      for (var p = 0; p < PIN_IDS.length; p++) {
        document.getElementById(PIN_IDS[p]).oninput = function() {
          profileSelect.value = '250';
        };
      }

      var tabs = document.querySelectorAll('.tab');
      for (var i = 0; i < tabs.length; i++) {
        tabs[i].onclick = function() {
          var target = this.getAttribute('data-panel');
          for (var j = 0; j < tabs.length; j++) {
            tabs[j].className = tabs[j] === this ? 'tab on' : 'tab';
          }
          var panels = document.querySelectorAll('.panel');
          for (var k = 0; k < panels.length; k++) {
            panels[k].className = panels[k].id === target ? 'panel on' : 'panel';
          }
        };
      }
    </script>

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
    html.replace("%RPM%", String(motor_rpm));
    html.replace("%BACKOFF_CLOSE%", String(cal_backoff_close));
    html.replace("%BACKOFF_OPEN%", String(cal_backoff_open));
    html.replace("%PIN0%", pin_profile == 0 ? "selected" : "");
    html.replace("%PIN1%", pin_profile == 1 ? "selected" : "");
    html.replace("%PINC%", pin_profile == PIN_PROFILE_CUSTOM ? "selected" : "");
    html.replace("%PIN_STEP%", String(STEP_PIN));
    html.replace("%PIN_DIR%", String(DIR_PIN));
    html.replace("%PIN_EN%", String(ENABLE_PIN));
    html.replace("%PIN_DIAG%", String(DIAG_PIN));
    html.replace("%PIN_TX%", String(TMC_TX_PIN));
    html.replace("%PIN_RX%", String(TMC_RX_PIN));
    html.replace("%STEPS%", String(preferences.getInt("steps_per_rev", 2000)));
    html.replace("%CURRENT%", String(preferences.getUShort("current_ma", 800)));

    // Microsteps dropdown selected state
    uint16_t ms = motor_microsteps;
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
    // Microsteps first: position and travel are counted in microsteps, so they
    // are rescaled here, and an explicit travel value below still wins. Only NVS
    // is touched — the driver is configured from these on the reboot that follows.
    if (request->hasParam("microsteps", true)) {
      int val = request->getParam("microsteps", true)->value().toInt();
      uint16_t previous = preferences.getUShort("microsteps", motor_microsteps);
      bool valid = val > 0 && val <= 256 && (val & (val - 1)) == 0;
      if (valid && (uint16_t)val != previous) {
        int pos = preferences.getInt("position", 0);
        int travel = preferences.getInt("steps_per_rev", 2000);
        preferences.putInt("position", (int)((int64_t)pos * val / previous));
        preferences.putInt("steps_per_rev", (int)((int64_t)travel * val / previous));
        preferences.putUShort("microsteps", val);
      }
    }
    if (request->hasParam("steps", true)) {
      int steps = request->getParam("steps", true)->value().toInt();
      if (steps > 0) preferences.putInt("steps_per_rev", steps);
    }
    if (request->hasParam("rpm", true)) {
      int val = request->getParam("rpm", true)->value().toInt();
      if (val >= 10 && val <= 300) preferences.putInt("rpm", val);
    }
    if (request->hasParam("backoff_close", true)) {
      int val = request->getParam("backoff_close", true)->value().toInt();
      if (val >= 1 && val <= 2000) preferences.putInt("backoff_close", val);
    }
    if (request->hasParam("backoff_open", true)) {
      int val = request->getParam("backoff_open", true)->value().toInt();
      if (val >= 1 && val <= 2000) preferences.putInt("backoff_open", val);
    }
    if (request->hasParam("pinprofile", true)) {
      int val = request->getParam("pinprofile", true)->value().toInt();
      if (val >= 0 && val < PIN_PROFILE_COUNT) {
        preferences.putUChar("pin_profile", val);
      } else if (val == PIN_PROFILE_CUSTOM) {
        const char* fields[] = {"pin_step", "pin_dir", "pin_en", "pin_diag", "pin_tx", "pin_rx"};
        int pins[6];
        bool ok = true;
        for (int i = 0; i < 6 && ok; i++) {
          if (!request->hasParam(fields[i], true)) { ok = false; break; }
          pins[i] = request->getParam(fields[i], true)->value().toInt();
          if (!valid_motor_gpio(pins[i])) ok = false;
          for (int j = 0; j < i; j++) {
            if (pins[j] == pins[i]) ok = false;  // two functions on one pin
          }
        }
        if (ok) {
          for (int i = 0; i < 6; i++) preferences.putInt(fields[i], pins[i]);
          preferences.putUChar("pin_profile", PIN_PROFILE_CUSTOM);
        } else {
          log_msg(LOG_ERROR, "SYS", "Rejected custom pins: invalid GPIO or duplicate");
        }
      }
    }
    if (request->hasParam("current", true)) {
      int val = request->getParam("current", true)->value().toInt();
      if (val >= 100 && val <= 2000) preferences.putUShort("current_ma", val);
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

  console_ws.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
                        void* arg, uint8_t* data, size_t len) {
    if (type != WS_EVT_DATA) return;

    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) return;

    // Runs on the AsyncTCP task: fill the buffer, then raise the flag. Echoing
    // and processing happen in the main loop, which owns the console buffer.
    if (ws_command_pending) return;

    size_t out = 0;
    for (size_t i = 0; i < len && out < sizeof(ws_pending_command) - 1; i++) {
      char c = (char)data[i];
      if (c >= 32 && c <= 126) ws_pending_command[out++] = c;
    }
    while (out > 0 && ws_pending_command[out - 1] == ' ') out--;
    ws_pending_command[out] = 0;
    if (out == 0) return;
    ws_command_pending = true;
  });
  server.addHandler(&console_ws);

  server.on("/webserial", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", CONSOLE_HTML);
  });
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
  WiFiManagerParameter p_steps("steps_per_rev", "Steps per Revolution", String(travel_steps).c_str(), 8);

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
  // Modem sleep adds 100-200ms latency spikes, which bulk transfers like OTA
  // tolerate badly on a weak signal
  WiFi.setSleep(false);
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
    esp_task_wdt_delete(NULL);  // a flash write outlasts the watchdog period
  });

  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static unsigned int last_decile = 0;
    unsigned int decile = total ? (done * 10) / total : 0;
    if (decile != last_decile) {
      last_decile = decile;
      log_msg(LOG_INFO, "OTA", "%u%% (%u/%u bytes)", decile * 10, done, total);
    }
  });

  ArduinoOTA.onEnd([]() {
    log_msg(LOG_INFO, "OTA", "Update received, rebooting");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    const char* reason;
    switch (error) {
      case OTA_AUTH_ERROR:    reason = "auth failed"; break;
      case OTA_BEGIN_ERROR:   reason = "begin failed (partition too small?)"; break;
      case OTA_CONNECT_ERROR: reason = "connect failed"; break;
      case OTA_RECEIVE_ERROR: reason = "receive failed (link dropped)"; break;
      case OTA_END_ERROR:     reason = "end failed (flash verify)"; break;
      default:                reason = "unknown"; break;
    }
    log_msg(LOG_ERROR, "OTA", "Update failed: %s (code %d, update error %d)",
            reason, (int)error, (int)Update.getError());

    // The update is over either way, so put the watchdog back
    esp_task_wdt_add(NULL);
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

  apply_pin_profile(preferences.getUChar("pin_profile", 0));

  current_position = preferences.getInt("position", 0);
  motor_rpm = preferences.getInt("rpm", 0);
  motor_sleep_timeout = preferences.getULong("sleep_timeout", 30000);
  travel_steps = preferences.getInt("steps_per_rev", 2000);
  motor_current_ma = preferences.getUShort("current_ma", 800);
  // The default changed from 2 to 16. An install that predates the key was
  // calibrated at 2, and travel_steps counts microsteps, so pin it to 2 there
  // and give only genuinely new devices the new default.
  if (!preferences.isKey("microsteps")) {
    bool existing_install = preferences.isKey("steps_per_rev") ||
                            preferences.isKey("position") ||
                            preferences.isKey("step_delay");
    uint16_t pinned = existing_install ? 2 : 16;
    preferences.putUShort("microsteps", pinned);
    log_msg(LOG_INFO, "BOOT", "No stored microsteps, using %d (%s install)",
            pinned, existing_install ? "existing" : "new");
  }
  motor_microsteps = preferences.getUShort("microsteps", 16);
  stall_threshold = preferences.getUChar("stall_thr", 50);
  int legacy_backoff = preferences.getInt("cal_backoff", 15);
  cal_backoff_close = preferences.getInt("backoff_close", legacy_backoff);
  cal_backoff_open = preferences.getInt("backoff_open", legacy_backoff);
  invert_direction = preferences.getBool("invert_dir", false);

  // Speed used to be stored as a step interval; convert it once so existing
  // tuning carries over
  if (motor_rpm <= 0) {
    int legacy_us = preferences.getInt("step_delay", 0);
    if (legacy_us > 0) {
      motor_rpm = 60000000L / ((long)legacy_us * MOTOR_FULL_STEPS_PER_REV * motor_microsteps);
      log_msg(LOG_INFO, "BOOT", "Converted stored %dus/step to %d RPM", legacy_us, motor_rpm);
    } else {
      motor_rpm = 75;
    }
    motor_rpm = constrain(motor_rpm, 10, 300);
    preferences.putInt("rpm", motor_rpm);
  }
  apply_motor_rpm();

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
    String command(ws_pending_command);
    ws_command_pending = false;
    output("> %s\n", command.c_str());
    process_command(command);
  }

  // Auto-sleep motor after inactivity
  if (!is_moving && cal_state == CAL_IDLE && motor_enabled) {
    if (millis() - last_motor_activity > motor_sleep_timeout) {
      sleep_motor();
    }
  }

  // Reap dead WebSocket clients: a browser that vanished (sleep, dropped WiFi)
  // holds its slot until cleaned up, and the limit is 8
  static unsigned long last_ws_cleanup = 0;
  if (millis() - last_ws_cleanup > 5000) {
    last_ws_cleanup = millis();
    console_ws.cleanupClients();
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
