#include <Arduino.h>
#include <Adafruit_SHT31.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <IRac.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRutils.h>
#include <LittleFS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "AppConfig.h"

using namespace appcfg;

struct CurvePoint {
  uint16_t minute;
  float temp;
};

struct DeviceConfig {
  String staSsid = kDefaultWifiSsid;
  String staPassword = kDefaultWifiPassword;
  decode_type_t acProtocol = decode_type_t::UNKNOWN;
  int16_t acModel = 1;
  bool autoEnabled = false;
  String autoMode = "cool";
  bool remotePower = true;
  String remoteMode = "cool";
  String remoteFan = "auto";
  float remoteDegrees = 26.0f;
  bool remoteTurbo = false;
  bool remoteQuiet = false;
  bool remoteSleep = false;
  bool remoteSwingV = false;
  bool remoteSwingH = false;
  bool remoteFilter = false;
  String curveControlMode = "staged";
  String curveEndAction = "hold";
  uint16_t quietSwitchMinute = 90;
  uint16_t sleepStartMinute = 23 * 60;
  uint16_t sleepDurationMinute = 8 * 60;
  uint16_t controlIntervalSec = 600;
  float deadband = 0.4f;
  float autoSendDelta = 0.5f;
  float minSetpoint = 16.0f;
  float maxSetpoint = 32.0f;
  float sensorTempOffset = 0.0f;
  float sensorHumidityOffset = 0.0f;
  bool humidityControlEnabled = false;
  bool curveHumidityEnabled = false;
  float targetHumidity = 58.0f;
  float humidityDeadband = 3.0f;
  float humidityTargetTemp = 26.0f;
  bool predictiveSkipEnabled = true;
  bool adaptiveControlEnabled = false;
  float learnedFastRate = 0.0f;
  float learnedQuietRate = 0.0f;
  float closedLoopFastGain = 2.6f;
  float closedLoopQuietGain = 0.9f;
  bool capTurbo = true;
  bool capQuiet = true;
  bool capSleep = true;
  bool capSwingV = true;
  bool capSwingH = true;
  bool capFilter = true;
  uint8_t curveCount = 4;
  CurvePoint curve[kMaxCurvePoints] = {
      {0, 27.0f},
      {90, 26.5f},
      {300, 25.5f},
      {480, 26.5f},
  };
};

struct LearnedCommand {
  uint8_t id = 0;
  String name;
  decode_type_t protocol = decode_type_t::UNKNOWN;
  uint16_t bits = 0;
  uint64_t value = 0;
  bool hasAcMeta = false;
  bool power = true;
  float degrees = 26.0f;
  String mode = "cool";
  String fan = "auto";
  uint16_t freqKhz = 38;
  uint16_t rawLen = 0;
  uint16_t raw[kMaxRawPulses] = {};
  String acDescription;
};

struct PresetCommand {
  uint8_t id = 0;
  String name;
  bool power = true;
  float degrees = 26.0f;
  String mode = "cool";
  String fan = "auto";
  bool turbo = false;
  bool quiet = false;
  bool sleep = false;
  bool swingV = false;
  bool swingH = false;
  bool filter = false;
  bool scheduleEnabled = false;
  uint16_t scheduleMinute = 7 * 60;
  String scheduleMode = "daily";
  uint8_t dayMask = 0b0111110;
  uint32_t lastScheduleKey = 0;
};

struct CaptureSnapshot {
  bool available = false;
  decode_type_t protocol = decode_type_t::UNKNOWN;
  uint16_t bits = 0;
  uint64_t value = 0;
  uint16_t rawLen = 0;
  uint16_t raw[kMaxRawPulses] = {};
  String summary;
  String acDescription;
  uint32_t capturedAtMs = 0;
};

struct TempHistorySample {
  uint32_t minute = 0;
  int16_t temp10 = 0;
  int16_t humidity10 = INT16_MIN;
};

struct ControlEvent {
  uint32_t minute = 0;
  int16_t room10 = INT16_MIN;
  int16_t target10 = INT16_MIN;
  int16_t setpoint10 = INT16_MIN;
  char source[12] = "";
  char action[16] = "";
  char mode[8] = "";
  char fan[8] = "";
  bool power = true;
  bool turbo = false;
  bool quiet = false;
  bool sleep = false;
};

struct DecisionLogEntry {
  uint32_t minute = 0;
  int16_t room10 = INT16_MIN;
  int16_t target10 = INT16_MIN;
  int16_t setpoint10 = INT16_MIN;
  char action[24] = "";
  char stage[12] = "";
  char note[160] = "";
};

struct AcRequest {
  bool power = true;
  float degrees = 26.0f;
  String mode = "cool";
  String fan = "auto";
  bool turbo = false;
  bool quiet = false;
  bool sleep = false;
  bool swingV = false;
  bool swingH = false;
  bool filter = false;
};

enum class PendingAction : uint8_t {
  None,
  SendAc,
  SendLearned,
};

DeviceConfig config;
LearnedCommand learned[kMaxLearnedCommands];
uint8_t learnedCount = 0;
constexpr uint8_t kMaxPresetCommands = 50;
constexpr char kPresetPath[] = "/presets.json";
PresetCommand presets[kMaxPresetCommands];
uint8_t presetCount = 0;
CaptureSnapshot lastCapture;

Adafruit_SHT31 sht31;
bool sht31Ready = false;
float roomTempC = NAN;
float roomHumidity = NAN;
uint32_t lastSensorMs = 0;
constexpr uint16_t kTempHistoryPoints = 72 * 60;
constexpr uint16_t kTempHistoryCompactSamples = 60;
constexpr uint32_t kTempHistoryCompactIntervalMs = 60UL * 60UL * 1000UL;
constexpr char kTempHistoryPath[] = "/temp_history.csv";
TempHistorySample tempHistory[kTempHistoryPoints];
uint16_t tempHistoryHead = 0;
uint16_t tempHistoryCount = 0;
uint32_t lastHistoryMinute = UINT32_MAX;
bool tempHistoryUsesEpoch = false;
bool tempHistoryDirty = false;
uint16_t unsavedTempHistorySamples = 0;
uint32_t lastTempHistorySaveMs = 0;
constexpr uint16_t kControlEventPoints = 160;
constexpr char kControlEventsPath[] = "/control_events.csv";
ControlEvent controlEvents[kControlEventPoints];
uint16_t controlEventHead = 0;
uint16_t controlEventCount = 0;
bool controlEventsUseEpoch = false;
constexpr uint16_t kDecisionLogPoints = 192;
DecisionLogEntry decisionLog[kDecisionLogPoints];
uint16_t decisionLogHead = 0;
uint16_t decisionLogCount = 0;
bool decisionLogUsesEpoch = false;

WebServer server(80);
DNSServer dnsServer;
IRrecv irrecv(kIrRxPin, kIrCaptureBufferSize, kIrTimeoutMs, true);
decode_results irResults;
IRsend rawSender(kIrTxPin);
IRac ac(kIrTxPin);

PendingAction pendingAction = PendingAction::None;
AcRequest pendingAc;
uint8_t pendingLearnedId = 0;
bool irBusy = false;
String lastActionResult = "boot";
float lastSentSetpoint = NAN;
float lastAutoSentSetpoint = NAN;
String lastAutoSentMode = "";
String lastAutoSentFan = "";
bool lastAutoSentTurbo = false;
bool lastAutoSentQuiet = false;
bool lastAutoSentSleep = false;
char pendingAcSource[12] = "manual";
char pendingAcAction[16] = "send";
float pendingAcTarget = NAN;
float pendingAcSetpoint = NAN;
uint32_t lastControlMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t wifiConnectStartMs = 0;
bool ntpConfigured = false;
bool apStarted = false;
bool sleepCurveWasActive = false;
uint32_t lastAdaptiveSaveMs = 0;

const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>红外空调控制器</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #e8decc;
      --panel: #eee6d8;
      --panel-strong: #f7efe4;
      --ink: #1f1a14;
      --muted: #776d5c;
      --line: rgba(81, 68, 45, 0.18);
      --accent: #006a70;
      --accent-deep: #004e54;
      --accent-soft: rgba(0, 106, 112, 0.10);
      --ok: #0a6d5e;
      --warn: #9a651c;
      --danger: #a93f32;
      --control: rgba(118, 105, 82, 0.13);
      --shadow: 8px 8px 18px rgba(111, 91, 58, 0.18), -8px -8px 18px rgba(255, 255, 255, 0.58);
      --shadow-sm: 4px 4px 10px rgba(111, 91, 58, 0.16), -4px -4px 10px rgba(255, 255, 255, 0.56);
      --inset: inset 4px 4px 9px rgba(111, 91, 58, 0.18), inset -4px -4px 9px rgba(255, 255, 255, 0.58);
      --pressed: inset 3px 3px 8px rgba(31, 24, 12, 0.28), inset -3px -3px 7px rgba(255, 255, 255, 0.42);
    }
    body[data-skin="bluehome"] {
      --bg: #cfdaec;
      --panel: #f7faff;
      --panel-strong: #ffffff;
      --ink: #12213d;
      --muted: #7a89a3;
      --line: rgba(83, 112, 158, 0.13);
      --accent: #2f80ed;
      --accent-deep: #2667d8;
      --accent-soft: rgba(47, 128, 237, 0.13);
      --ok: #2f80ed;
      --warn: #7b61ff;
      --danger: #ec5f67;
      --control: rgba(47, 128, 237, 0.08);
      --shadow: 0 22px 46px rgba(78, 101, 142, 0.20), 0 2px 7px rgba(255, 255, 255, 0.80);
      --shadow-sm: 0 12px 28px rgba(78, 101, 142, 0.16), 0 1px 5px rgba(255, 255, 255, 0.82);
      --inset: inset 0 1px 0 rgba(255, 255, 255, 0.92), inset 0 -10px 22px rgba(73, 112, 178, 0.05);
      --pressed: inset 0 3px 10px rgba(50, 80, 130, 0.20);
    }
    * { box-sizing: border-box; }
    html { width: 100%; overflow-x: hidden; }
    body {
      width: 100%;
      margin: 0;
      overflow-x: hidden;
      background: var(--bg);
      color: var(--ink);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    body[data-skin="bluehome"] {
      background: linear-gradient(180deg, #d8e3f5 0%, #cfd9eb 48%, #c4d0e5 100%);
    }
    main {
      width: min(1160px, 100%);
      margin: 22px auto 42px;
      padding: 0 18px;
      display: grid;
      gap: 18px;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: end;
      gap: 16px;
      padding: 10px 2px 6px;
    }
    h1 { margin: 0; font-size: 27px; line-height: 1.15; letter-spacing: 0; text-transform: uppercase; }
    h2 { margin: 0 0 14px; font-size: 18px; letter-spacing: .01em; }
    h3 { margin: 0; font-size: 13px; color: var(--ink); font-weight: 760; text-transform: uppercase; }
    section {
      min-width: 0;
      max-width: 100%;
      position: relative;
      overflow: hidden;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: var(--shadow);
    }
    body[data-skin="bluehome"] section {
      border: 0;
      border-radius: 24px;
      background: rgba(248, 251, 255, 0.88);
      backdrop-filter: blur(10px);
      padding: 22px;
    }
    body[data-skin="bluehome"] main {
      width: min(1320px, 100%);
      grid-template-columns: minmax(0, 1.06fr) minmax(360px, 0.94fr);
      align-items: start;
      gap: 22px;
      padding: 0 26px;
    }
    body[data-skin="bluehome"] header {
      grid-column: 1 / -1;
      align-items: center;
      padding: 18px 4px 8px;
    }
    body[data-skin="bluehome"] h1 {
      color: #10203b;
      font-size: 28px;
      text-transform: none;
    }
    body[data-skin="bluehome"] h2 {
      color: #142440;
      font-size: 17px;
    }
    body[data-skin="bluehome"] h3 {
      color: #172845;
      text-transform: none;
    }
    body[data-skin="bluehome"] main > section:nth-of-type(1),
    body[data-skin="bluehome"] main > section:nth-of-type(4),
    body[data-skin="bluehome"] main > section:nth-of-type(5),
    body[data-skin="bluehome"] main > section:nth-of-type(8) {
      grid-column: 1 / -1;
    }
    body[data-skin="bluehome"] main > section:nth-of-type(2),
    body[data-skin="bluehome"] main > section:nth-of-type(6) {
      grid-column: 1 / 2;
    }
    body[data-skin="bluehome"] main > section:nth-of-type(3),
    body[data-skin="bluehome"] main > section:nth-of-type(7) {
      grid-column: 2 / 3;
    }
    body[data-skin="bluehome"] .status-grid {
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 18px;
    }
    label { display: grid; gap: 7px; color: var(--muted); font-size: 13px; font-weight: 650; }
    input, select, textarea, button {
      width: 100%;
      min-width: 0;
      max-width: 100%;
      min-height: 42px;
      border-radius: 12px;
      border: 1px solid var(--line);
      background: var(--panel-strong);
      color: var(--ink);
      font: inherit;
      padding: 9px 10px;
      box-shadow: var(--inset);
    }
    select { cursor: pointer; }
    select option {
      background: #ffffff;
      color: #172033;
    }
    select option:checked {
      background: #bfd6cf;
      color: #172033;
    }
    textarea {
      min-height: 116px;
      resize: vertical;
      font-family: ui-monospace, SFMono-Regular, Consolas, monospace;
      line-height: 1.5;
    }
    button {
      cursor: pointer;
      border-color: rgba(0, 78, 84, 0.22);
      background: linear-gradient(180deg, #08777d, var(--accent-deep));
      color: #fff;
      font-weight: 650;
      box-shadow: var(--shadow-sm);
    }
    button:active { box-shadow: var(--pressed); transform: translateY(1px); }
    button.secondary { background: var(--panel); color: var(--accent); border-color: rgba(0, 106, 112, 0.14); box-shadow: var(--shadow-sm); }
    button.danger { background: #efe1d8; color: var(--danger); border-color: rgba(169, 63, 50, 0.18); box-shadow: var(--shadow-sm); }
    button:disabled { cursor: wait; opacity: 0.68; }
    body[data-skin="bluehome"] button {
      border-color: rgba(47, 128, 237, 0.16);
      background: linear-gradient(180deg, #3d8bff, var(--accent-deep));
      box-shadow: 0 14px 26px rgba(47, 128, 237, 0.30);
    }
    body[data-skin="bluehome"] button.secondary,
    body[data-skin="bluehome"] .badge {
      background: rgba(255, 255, 255, 0.76);
      border-color: rgba(47, 128, 237, 0.10);
      box-shadow: var(--shadow-sm);
    }
    body[data-skin="bluehome"] button.danger {
      background: #fff1f2;
      border-color: rgba(236, 95, 103, 0.18);
      box-shadow: var(--shadow-sm);
    }
    .collapse-toggle {
      position: absolute;
      top: 12px;
      right: 12px;
      width: 28px;
      min-width: 28px;
      height: 28px;
      min-height: 28px;
      padding: 0;
      border-radius: 999px;
      display: grid;
      place-items: center;
      color: var(--muted);
      background: var(--panel);
      box-shadow: var(--shadow-sm);
    }
    .collapse-toggle::before {
      content: "";
      width: 7px;
      height: 7px;
      border-right: 2px solid currentColor;
      border-bottom: 2px solid currentColor;
      transform: rotate(-135deg) translate(-1px, -1px);
      transition: transform 0.16s ease;
    }
    section.collapsed .collapse-toggle::before { transform: rotate(45deg) translate(-1px, -1px); }
    section.collapsed .collapsible-body { display: none; }
    section.collapsed .section-head { margin-bottom: 0; }
    .subhead { color: var(--muted); font-size: 13px; line-height: 1.55; margin: -6px 0 14px; }
    body[data-skin="bluehome"] .subhead {
      max-width: 68ch;
      line-height: 1.7;
    }
    .message { min-height: 22px; color: var(--ok); font-size: 14px; line-height: 1.45; text-align: right; }
    .message.warn { color: var(--warn); }
    .status-grid, .form-grid {
      min-width: 0;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(190px, 1fr));
      gap: 14px;
    }
    body[data-skin="bluehome"] .form-grid {
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 16px;
    }
    .metric {
      min-width: 0;
      min-height: 112px;
      position: relative;
      display: grid;
      gap: 4px;
      padding: 12px;
      border: 1px solid rgba(81, 68, 45, 0.12);
      border-radius: 14px;
      background: var(--panel);
      box-shadow: var(--inset);
    }
    body[data-skin="bluehome"] .metric {
      border: 0;
      border-radius: 20px;
      background: rgba(255, 255, 255, 0.82);
      box-shadow: var(--shadow-sm);
    }
    .metric:last-child { border-right: 1px solid rgba(81, 68, 45, 0.12); }
    .label { color: var(--muted); font-size: 13px; }
    .value {
      min-height: 32px;
      font-size: 24px;
      line-height: 1.25;
      font-weight: 720;
      overflow-wrap: anywhere;
    }
    .status-grid .metric:nth-child(-n+2) {
      min-height: 168px;
      align-content: space-between;
      overflow: hidden;
      border-color: rgba(0, 106, 112, 0.20);
      background: linear-gradient(145deg, #f5eddf, #e8decc);
    }
    body[data-skin="bluehome"] .status-grid .metric:nth-child(-n+2) {
      background: linear-gradient(180deg, rgba(255, 255, 255, 0.96), rgba(241, 247, 255, 0.92));
      border: 1px solid rgba(47, 128, 237, 0.12);
    }
    .status-grid .metric:nth-child(-n+2) .label {
      color: var(--accent);
      font-weight: 780;
    }
    .status-grid .metric:nth-child(-n+2) .value {
      width: 100%;
      min-height: 92px;
      display: block;
      align-self: end;
      color: var(--accent-deep);
      font-size: 88px;
      line-height: 0.88;
      font-weight: 860;
      font-variant-numeric: tabular-nums;
      letter-spacing: 0;
      white-space: nowrap;
      overflow: hidden;
      overflow-wrap: normal;
      text-shadow: 1px 1px 0 rgba(255,255,255,0.55);
    }
    body[data-skin="bluehome"] .status-grid .metric:nth-child(-n+2) .value {
      color: #2f80ed;
      text-shadow: 0 8px 22px rgba(47, 128, 237, 0.18);
    }
    body[data-skin="bluehome"] .status-grid .metric:nth-child(n+3) {
      min-height: 132px;
      align-content: space-between;
    }
    body[data-skin="bluehome"] .status-grid .metric:nth-child(n+3) .value {
      font-size: clamp(17px, 1.7vw, 23px);
      line-height: 1.34;
      font-weight: 760;
    }
    body[data-skin="bluehome"] #last {
      font-size: 16px;
      line-height: 1.35;
      font-weight: 720;
    }
    .badge {
      display: inline-flex;
      align-items: center;
      width: fit-content;
      max-width: 100%;
      min-height: 28px;
      padding: 4px 9px;
      border-radius: 999px;
      border: 1px solid rgba(0, 106, 112, 0.10);
      background: var(--panel);
      color: var(--accent);
      font-size: 13px;
      font-weight: 650;
      overflow-wrap: anywhere;
      box-shadow: var(--shadow-sm);
    }
    a.badge { text-decoration: none; }
    .header-links { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 10px; }
    .skin-switch {
      display: inline-grid;
      grid-auto-flow: column;
      gap: 4px;
      padding: 4px;
      border-radius: 999px;
      background: var(--panel);
      border: 1px solid var(--line);
      box-shadow: var(--shadow-sm);
    }
    .skin-switch button {
      width: auto;
      min-width: 58px;
      min-height: 28px;
      padding: 4px 10px;
      border: 0;
      border-radius: 999px;
      background: transparent;
      color: var(--muted);
      box-shadow: none;
      font-size: 12px;
      font-weight: 720;
    }
    .skin-switch button.selected {
      background: linear-gradient(180deg, #08777d, var(--accent-deep));
      color: #fff;
      box-shadow: var(--shadow-sm);
    }
    body[data-skin="bluehome"] .skin-switch {
      background: rgba(255, 255, 255, 0.62);
      border-color: rgba(47, 128, 237, 0.08);
    }
    body[data-skin="bluehome"] .skin-switch button.selected {
      background: linear-gradient(180deg, #3d8bff, var(--accent-deep));
      box-shadow: 0 10px 18px rgba(47, 128, 237, 0.28);
    }
    .actions {
      min-width: 0;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(132px, 1fr));
      gap: 10px;
      margin-top: 14px;
    }
    .section-head {
      min-width: 0;
      display: flex;
      justify-content: space-between;
      align-items: start;
      gap: 12px;
      margin-bottom: 14px;
      padding-right: 34px;
    }
    .section-head > * { min-width: 0; }
    .section-head h2 { margin: 0 0 4px; }
    .section-head .subhead { margin: 0; }
    body[data-skin="bluehome"] .section-head {
      gap: 16px;
      margin-bottom: 18px;
    }
    body[data-skin="bluehome"] .section-head .header-links,
    body[data-skin="bluehome"] .section-head > .badge {
      flex-shrink: 0;
    }
    .segmented {
      width: 100%;
      display: grid;
      grid-auto-flow: column;
      grid-auto-columns: minmax(0, 1fr);
      gap: 4px;
      padding: 5px;
      border-radius: 14px;
      background: var(--panel);
      border: 1px solid rgba(81, 68, 45, 0.10);
      box-shadow: var(--inset);
    }
    .segmented.wrap {
      grid-auto-flow: row;
      grid-template-columns: repeat(auto-fit, minmax(76px, 1fr));
    }
    body[data-skin="bluehome"] .segmented.wrap {
      grid-template-columns: repeat(auto-fit, minmax(92px, 1fr));
    }
    .segmented button {
      min-height: 34px;
      padding: 6px 8px;
      border: 0;
      border-radius: 10px;
      background: transparent;
      color: var(--ink);
      box-shadow: none;
      font-weight: 650;
      line-height: 1.15;
      white-space: normal;
      overflow-wrap: anywhere;
    }
    .segmented button.selected {
      background: linear-gradient(180deg, #08777d, var(--accent-deep));
      color: #fff;
      box-shadow: var(--shadow-sm);
    }
    body[data-skin="bluehome"] .segmented {
      background: rgba(237, 243, 252, 0.86);
      border-color: rgba(47, 128, 237, 0.08);
    }
    body[data-skin="bluehome"] .segmented button.selected,
    body[data-skin="bluehome"] button.curve-chip.selected {
      background: linear-gradient(180deg, #3d8bff, var(--accent-deep));
      color: #fff;
      box-shadow: 0 10px 18px rgba(47, 128, 237, 0.24);
    }
    .segmented-source {
      position: absolute;
      inline-size: 1px;
      block-size: 1px;
      opacity: 0;
      pointer-events: none;
    }
    .two-col {
      min-width: 0;
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 14px;
    }
    pre {
      margin: 0;
      min-height: 114px;
      max-height: 220px;
      overflow: auto;
      white-space: pre-wrap;
      word-break: break-word;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: var(--panel);
      padding: 12px;
      font-family: ui-monospace, SFMono-Regular, Consolas, monospace;
      font-size: 12px;
      line-height: 1.55;
    }
    .table-wrap { max-width: 100%; overflow-x: auto; border: 1px solid var(--line); border-radius: 14px; box-shadow: var(--inset); }
    body[data-skin="bluehome"] pre,
    body[data-skin="bluehome"] .table-wrap,
    body[data-skin="bluehome"] .preset-card,
    body[data-skin="bluehome"] .preset-schedule,
    body[data-skin="bluehome"] .curve-editor,
    body[data-skin="bluehome"] .settings-panel,
    body[data-skin="bluehome"] .special-panel,
    body[data-skin="bluehome"] .log-item,
    body[data-skin="bluehome"] .wifi-network {
      border-color: rgba(47, 128, 237, 0.08);
      background: rgba(255, 255, 255, 0.72);
      box-shadow: var(--shadow-sm);
    }
    body[data-skin="bluehome"] th {
      background: rgba(47, 128, 237, 0.08);
    }
    table { width: 100%; border-collapse: collapse; font-size: 13px; min-width: 680px; }
    th, td { border-bottom: 1px solid var(--line); padding: 9px 10px; text-align: left; vertical-align: top; }
    th { color: var(--muted); font-weight: 650; background: rgba(0, 106, 112, 0.08); }
    tr:last-child td { border-bottom: 0; }
    td.actions-cell { width: 172px; }
    .inline-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .empty { color: var(--muted); text-align: center; padding: 18px 10px; }
    .preset-panel {
      margin-top: 14px;
      padding-top: 14px;
      border-top: 1px solid var(--line);
      display: grid;
      gap: 10px;
    }
    .preset-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
    }
    body[data-skin="bluehome"] .preset-head,
    body[data-skin="bluehome"] .special-title,
    body[data-skin="bluehome"] .curve-toolbar,
    body[data-skin="bluehome"] .curve-adjuster-head {
      align-items: start;
      gap: 14px;
    }
    .preset-head button { width: auto; min-width: 132px; }
    .preset-list {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(170px, 1fr));
      gap: 8px;
    }
    .preset-card {
      border: 1px solid var(--line);
      border-radius: 16px;
      background: var(--panel);
      padding: 10px;
      display: grid;
      gap: 8px;
      box-shadow: var(--shadow-sm);
    }
    .preset-title { font-weight: 720; color: var(--ink); overflow-wrap: anywhere; }
    .preset-meta { color: var(--muted); font-size: 12px; line-height: 1.35; }
    .preset-actions { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px; }
    .preset-actions button { min-height: 34px; padding: 6px; font-size: 12px; }
    .preset-schedule-summary {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      align-items: center;
      gap: 8px;
      padding-top: 8px;
      border-top: 1px solid var(--line);
    }
    .preset-schedule-summary button {
      width: auto;
      min-width: 78px;
      min-height: 32px;
      padding: 5px 9px;
      font-size: 12px;
    }
    .preset-schedule {
      display: grid;
      gap: 8px;
      padding: 8px;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: var(--panel);
      box-shadow: var(--inset);
    }
    .preset-schedule-grid {
      display: grid;
      grid-template-columns: 92px minmax(0, 1fr);
      gap: 8px;
    }
    .preset-toggle {
      display: flex;
      grid-template-columns: none;
      align-items: center;
      gap: 7px;
      color: var(--ink);
    }
    .preset-toggle input { width: auto; min-height: 0; }
    .preset-days {
      display: grid;
      grid-template-columns: repeat(7, minmax(0, 1fr));
      gap: 4px;
    }
    .preset-days button {
      min-height: 30px;
      padding: 4px;
      border-color: transparent;
      background: var(--panel);
      color: var(--muted);
      box-shadow: var(--shadow-sm);
      font-size: 12px;
    }
    .preset-days button.selected {
      background: var(--accent);
      color: #fff;
      box-shadow: var(--pressed);
    }
    .special-panel {
      margin-top: 12px;
      padding-top: 12px;
      border-top: 1px solid var(--line);
      display: grid;
      gap: 10px;
    }
    .special-title {
      display: flex;
      align-items: end;
      justify-content: space-between;
      gap: 10px;
    }
    .sleep-presets, .mini-actions {
      min-width: 0;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(116px, 1fr));
      gap: 8px;
      margin: 10px 0 0;
    }
    .sleep-presets button, .mini-actions button {
      min-height: 38px;
      padding: 7px 9px;
      font-size: 13px;
    }
    .settings-panel {
      margin-top: 14px;
      padding-top: 14px;
      border-top: 1px solid var(--line);
      display: grid;
      gap: 12px;
    }
    .wifi-scan-list {
      display: grid;
      gap: 8px;
      margin-top: 12px;
    }
    .wifi-network {
      min-height: 0;
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      align-items: center;
      gap: 10px;
      padding: 10px 12px;
      border-radius: 14px;
      border-color: var(--line);
      background: var(--panel);
      color: var(--ink);
      text-align: left;
      box-shadow: var(--shadow-sm);
    }
    .wifi-network strong { display: block; overflow-wrap: anywhere; }
    .wifi-network span { color: var(--muted); font-size: 12px; }
    .wifi-rssi { color: var(--accent); font-weight: 760; white-space: nowrap; }
    .control-log {
      display: grid;
      gap: 8px;
      max-height: 280px;
      overflow: auto;
      padding-right: 4px;
    }
    .log-item {
      display: grid;
      grid-template-columns: minmax(72px, auto) minmax(0, 1fr);
      gap: 8px;
      padding: 8px 10px;
      border: 1px solid var(--line);
      border-radius: 12px;
      background: var(--panel);
      box-shadow: var(--shadow-sm);
      font-size: 12px;
      line-height: 1.4;
    }
    .log-item strong { color: var(--accent); }
    .history-event-line { stroke: var(--warn); stroke-width: 1.2; stroke-dasharray: 3 4; pointer-events: none; }
    .history-event-dot { fill: var(--warn); stroke: var(--panel-strong); stroke-width: 1.5; pointer-events: none; }
    .segmented button:disabled {
      opacity: .42;
      cursor: not-allowed;
      box-shadow: none;
    }
    .curve-editor {
      margin-top: 14px;
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 14px;
      background: var(--panel);
      box-shadow: var(--inset);
    }
    .curve-toolbar {
      min-width: 0;
      display: flex;
      justify-content: space-between;
      align-items: start;
      gap: 12px;
      margin-bottom: 12px;
    }
    .curve-buttons {
      min-width: 0;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
      width: min(260px, 100%);
    }
    .curve-stage {
      max-width: 100%;
      border: 1px solid var(--line);
      border-radius: 16px;
      background: #e6dccb;
      overflow: hidden;
      box-shadow: var(--inset);
      touch-action: none;
    }
    body[data-skin="bluehome"] .curve-stage {
      border: 0;
      border-radius: 24px;
      background: linear-gradient(180deg, #eef5ff, #e6effc);
      box-shadow: inset 0 1px 0 rgba(255,255,255,.85), 0 16px 34px rgba(72, 102, 150, 0.16);
    }
    #curveSvg {
      display: block;
      width: 100%;
      height: 280px;
      user-select: none;
      touch-action: none;
    }
    .curve-grid { stroke: var(--line); stroke-width: 1; }
    .curve-axis { stroke: var(--muted); stroke-width: 1.2; }
    .curve-line { fill: none; stroke: var(--accent); stroke-width: 3; stroke-linecap: round; stroke-linejoin: round; }
    .curve-fill { fill: rgba(23, 105, 224, 0.12); }
    .history-line { fill: none; stroke: var(--ok); stroke-width: 2.6; stroke-linecap: round; stroke-linejoin: round; }
    .history-humidity-line { fill: none; stroke: var(--warn); stroke-width: 2.3; stroke-linecap: round; stroke-linejoin: round; }
    .history-fill { fill: rgba(11, 143, 85, 0.12); }
    .history-stage { position: relative; cursor: grab; }
    .history-stage.dragging { cursor: grabbing; }
    #tempHistorySvg {
      display: block;
      width: 100%;
      height: 260px;
      user-select: none;
      touch-action: none;
    }
    .history-hover-line { stroke: var(--muted); stroke-width: 1; stroke-dasharray: 4 4; pointer-events: none; }
    .history-hover-dot { fill: var(--panel-strong); stroke: var(--ok); stroke-width: 2; pointer-events: none; }
    .history-hover-dot.humidity { stroke: var(--warn); }
    .history-tooltip {
      position: absolute;
      z-index: 3;
      display: none;
      min-width: 126px;
      padding: 7px 9px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel-strong);
      color: var(--ink);
      box-shadow: 0 8px 24px rgba(0, 0, 0, 0.14);
      font-size: 12px;
      line-height: 1.45;
      pointer-events: none;
    }
    .history-tooltip strong { display: block; font-size: 14px; }
    .curve-node circle {
      fill: var(--panel);
      stroke: var(--accent);
      stroke-width: 3;
      cursor: grab;
    }
    .curve-node.selected circle {
      fill: var(--accent);
      stroke: var(--ink);
    }
    .curve-node text, .curve-label {
      fill: var(--muted);
      font-size: 12px;
      pointer-events: none;
    }
    .curve-temp-label { fill: var(--ink); font-weight: 650; }
    .curve-list {
      min-width: 0;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(145px, 1fr));
      gap: 8px;
      margin-top: 10px;
    }
    button.curve-chip {
      min-width: 0;
      min-height: 58px;
      display: grid;
      gap: 2px;
      text-align: left;
      background: transparent;
      color: var(--ink);
      border-color: var(--line);
      font-weight: 500;
    }
    button.curve-chip.selected {
      border-color: var(--accent);
      background: var(--accent-soft);
    }
    .curve-chip strong { font-size: 17px; }
    .curve-adjuster {
      min-width: 0;
      margin-top: 12px;
      padding-top: 12px;
      border-top: 1px solid var(--line);
      display: grid;
      gap: 10px;
    }
    .curve-adjuster-head {
      display: flex;
      align-items: end;
      justify-content: space-between;
      gap: 10px;
    }
    .curve-step-grid {
      min-width: 0;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .curve-stepper {
      min-width: 0;
      display: grid;
      grid-template-columns: 54px minmax(0, 1fr) 54px;
      gap: 8px;
      align-items: center;
    }
    .curve-stepper button {
      min-height: 48px;
      padding: 8px 4px;
      font-size: 18px;
    }
    .curve-stepper output {
      min-height: 48px;
      display: grid;
      place-items: center;
      border: 1px solid var(--line);
      border-radius: 12px;
      color: var(--ink);
      font-weight: 720;
      overflow-wrap: anywhere;
      background: var(--panel);
      box-shadow: var(--inset);
    }
    small { display: block; margin-top: 8px; color: var(--muted); line-height: 1.45; }
    @media (max-width: 1180px) {
      body[data-skin="bluehome"] main {
        grid-template-columns: 1fr;
        width: 100%;
        padding: 0 18px;
      }
      body[data-skin="bluehome"] main > section {
        grid-column: 1 / -1 !important;
      }
      body[data-skin="bluehome"] .status-grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }
    @media (max-width: 900px) {
      body[data-skin="bluehome"] .section-head,
      body[data-skin="bluehome"] .preset-head,
      body[data-skin="bluehome"] .special-title,
      body[data-skin="bluehome"] .curve-toolbar,
      body[data-skin="bluehome"] .curve-adjuster-head {
        display: grid;
        grid-template-columns: minmax(0, 1fr);
      }
      body[data-skin="bluehome"] .section-head .header-links,
      body[data-skin="bluehome"] .section-head > .badge {
        width: 100%;
      }
      body[data-skin="bluehome"] .two-col {
        grid-template-columns: 1fr;
      }
    }
    @media (max-width: 760px) {
      main { width: 100%; padding: 0 10px; margin-top: 12px; gap: 12px; }
      header { display: grid; align-items: start; }
      h1 { font-size: 24px; }
      section { padding: 14px; border-radius: 16px; }
      .section-head {
        display: grid;
        grid-template-columns: minmax(0, 1fr);
        gap: 8px;
        padding-right: 32px;
        margin-bottom: 12px;
      }
      .section-head h2 { font-size: 17px; }
      .collapse-toggle { top: 10px; right: 10px; width: 26px; min-width: 26px; height: 26px; min-height: 26px; }
      .header-links { gap: 6px; margin-top: 6px; }
      .badge { min-height: 26px; padding: 3px 8px; font-size: 12px; }
      .message { text-align: left; }
      .status-grid, .form-grid { grid-template-columns: 1fr; gap: 10px; }
      body[data-skin="bluehome"] .status-grid { grid-template-columns: 1fr; }
      body[data-skin="bluehome"] main { padding: 0 10px; gap: 14px; }
      body[data-skin="bluehome"] section { padding: 16px; border-radius: 20px; }
      body[data-skin="bluehome"] .form-grid { grid-template-columns: 1fr; gap: 12px; }
      body[data-skin="bluehome"] .subhead { display: none; }
      body[data-skin="bluehome"] .status-grid { gap: 12px; }
      body[data-skin="bluehome"] .header-links { width: 100%; }
      body[data-skin="bluehome"] .skin-switch { width: 100%; grid-auto-columns: minmax(0, 1fr); }
      .segmented.wrap { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .segmented button { min-height: 36px; padding: 6px 5px; font-size: 13px; }
      .preset-schedule-summary { grid-template-columns: 1fr; }
      .preset-schedule-summary button { width: 100%; }
      .two-col { grid-template-columns: 1fr; }
      .curve-toolbar { display: grid; }
      .curve-buttons { min-width: 0; width: 100%; }
      #curveSvg { height: 240px; }
      #tempHistorySvg { height: 220px; }
      .curve-node circle { r: 14px; }
      .curve-step-grid { grid-template-columns: 1fr; }
      .curve-stepper { grid-template-columns: 64px minmax(0, 1fr) 64px; }
      .curve-stepper button, .curve-stepper output { min-height: 52px; }
      .metric { border-right: 1px solid rgba(81, 68, 45, 0.12); border-bottom: 1px solid rgba(81, 68, 45, 0.12); padding: 12px; }
      .metric:last-child { border-bottom: 1px solid rgba(81, 68, 45, 0.12); }
    }
  </style>
</head>
<body>
<script>try{document.body.dataset.skin=localStorage.getItem('ir-ac-skin')==='bluehome'?'bluehome':'cream'}catch(e){}</script>
<main>
  <header>
    <div>
      <h1>红外空调控制器</h1>
      <div class="label">ESP32-S3 · SHT31 · 红外学习与睡眠曲线</div>
      <div class="header-links">
        <a class="badge" href="/remote">打开手机遥控器</a>
        <a class="badge" href="/match">打开对码检查</a>
        <div class="skin-switch" aria-label="皮肤选择">
          <button type="button" data-skin-choice="cream" onclick="setSkin('cream')">奶油</button>
          <button type="button" data-skin-choice="bluehome" onclick="setSkin('bluehome')">蓝家居</button>
        </div>
      </div>
    </div>
    <div class="message" id="msg">正在读取状态...</div>
  </header>

  <section>
    <div class="status-grid">
      <div class="metric"><div class="label">室温</div><div id="temp" class="value">--</div></div>
      <div class="metric"><div class="label">湿度</div><div id="hum" class="value">--</div></div>
      <div class="metric"><div class="label">网络</div><div id="wifi" class="value">--</div></div>
      <div class="metric"><div class="label">最近动作</div><div id="last" class="value">--</div></div>
    </div>
  </section>

  <section>
    <div class="section-head">
      <div>
        <h2>空调控制</h2>
        <div class="subhead">优先使用内置协议库发送；如果协议不匹配，可以在下方保存原始红外命令作为自建库。</div>
      </div>
      <div class="badge" id="protocolBadge">协议读取中</div>
    </div>
    <div class="form-grid">
      <label>协议<select id="protocol"></select></label>
      <label>型号<input id="model" type="number" min="1" value="1"></label>
      <label>电源<select id="power"><option value="true">开机</option><option value="false">关机</option></select></label>
      <label>模式<select id="mode"><option value="cool">制冷</option><option value="auto">自动</option><option value="dry">除湿</option><option value="heat">制热</option><option value="fan">送风</option></select></label>
      <label>设定温度<input id="degrees" type="number" step="0.5" min="16" max="32" value="26"></label>
      <label>风量<select id="fan"><option value="auto">自动</option><option value="min">最小</option><option value="low">低风</option><option value="medium">中风</option><option value="high">高风</option><option value="max">最大</option></select></label>
    </div>
    <div class="special-panel">
      <div class="special-title">
        <h3>特殊功能</h3>
        <div class="label">随立即发送、遥控状态记忆和快捷指令一起保存。</div>
      </div>
      <div class="form-grid">
        <label>运行特殊模式<select id="specialMode"><option value="none">标准</option><option value="turbo">强劲</option><option value="quiet">静音</option><option value="sleep">睡眠</option></select></label>
        <label>上下摆风<select id="swingV"><option value="false">关</option><option value="true">开</option></select></label>
        <label>左右摆风<select id="swingH"><option value="false">关</option><option value="true">开</option></select></label>
        <label>滤网/出风口<select id="filterFlag"><option value="false">关</option><option value="true">开</option></select></label>
      </div>
    </div>
    <div class="special-panel">
      <div class="special-title">
        <h3>湿度控制</h3>
        <div class="label">独立湿度控制不受睡眠开始时间限制；睡眠曲线内的除湿使用同一目标湿度。</div>
      </div>
      <div class="form-grid">
        <label>独立湿度控制<select id="humidityControlEnabled"><option value="false">关闭</option><option value="true">开启</option></select></label>
        <label>目标湿度 %<input id="targetHumidity" type="number" step="1" min="35" max="85" value="58"></label>
        <label>湿度死区 %<input id="humidityDeadband" type="number" step="1" min="1" max="15" value="3"></label>
        <label>湿控目标温度 ℃<input id="humidityTargetTemp" type="number" step="0.5" min="16" max="32" value="26"></label>
      </div>
      <div class="mini-actions">
        <button class="secondary" type="button" onclick="saveSettings('湿度控制已保存')">保存湿度控制</button>
      </div>
    </div>
    <div class="actions">
      <button onclick="sendAc()">立即发送</button>
      <button class="secondary" onclick="saveAcConfig()">保存协议配置</button>
    </div>
    <div class="preset-panel">
      <div class="preset-head">
        <div>
          <h3>快捷指令</h3>
          <div class="label">把当前电源、模式、温度、风量保存成一个可点击发送的指令，最多 50 个。</div>
        </div>
        <button class="secondary" type="button" onclick="createPresetFromCurrent()">添加当前</button>
      </div>
      <div class="preset-list" id="presetList"><div class="empty">暂无快捷指令</div></div>
    </div>
  </section>

  <section>
    <div class="section-head">
      <div>
        <h2>红外学习库</h2>
        <div class="subhead">对准接收头按实体遥控器按键，页面会显示最近捕获；保存时可附加温度、模式、风量元数据。</div>
      </div>
      <div class="badge" id="captureBadge">等待信号</div>
    </div>
    <div class="two-col">
      <div>
        <h3>最近捕获</h3>
        <pre id="capture">暂无红外捕获。</pre>
      </div>
      <div>
        <div class="form-grid">
          <label>命令名称<input id="learnName" placeholder="制冷 26 自动风"></label>
          <label>载波频率 kHz<input id="learnFreq" type="number" value="38"></label>
          <label>电源<select id="learnPower"><option value="true">开机</option><option value="false">关机</option></select></label>
          <label>模式<select id="learnMode"><option value="cool">制冷</option><option value="auto">自动</option><option value="dry">除湿</option><option value="heat">制热</option><option value="fan">送风</option></select></label>
          <label>温度<input id="learnDegrees" type="number" step="0.5" value="26"></label>
          <label>风量<select id="learnFan"><option value="auto">自动</option><option value="min">最小</option><option value="low">低风</option><option value="medium">中风</option><option value="high">高风</option><option value="max">最大</option></select></label>
        </div>
        <div class="actions">
          <button onclick="saveLearned()">保存最近捕获</button>
        </div>
      </div>
    </div>
    <div class="table-wrap" style="margin-top:14px">
      <table>
        <thead><tr><th>ID</th><th>名称</th><th>协议</th><th>元数据</th><th>Raw 长度</th><th>操作</th></tr></thead>
        <tbody id="learned"><tr><td colspan="6" class="empty">正在读取...</td></tr></tbody>
      </table>
    </div>
  </section>

  <section>
    <div class="section-head">
      <div>
        <h2>睡眠温度曲线</h2>
        <div class="subhead">曲线按入睡后的分钟数计算目标室温，用于夜间自动微调空调设定。</div>
      </div>
      <div class="header-links"><div class="badge" id="curveTargetBadge">目标 --</div><div class="badge" id="curveBadge">未启用</div></div>
    </div>
    <div class="form-grid">
      <label>自动控制<select id="autoEnabled"><option value="false">关闭</option><option value="true">开启</option></select></label>
      <label>自动模式<select id="autoMode"><option value="cool">制冷</option><option value="auto">自动</option><option value="dry">除湿</option><option value="heat">制热</option><option value="fan">送风</option></select></label>
      <label>曲线除湿<select id="curveHumidityEnabled"><option value="false">关闭</option><option value="true">开启</option></select></label>
      <label>开始时间 HH:MM<input id="sleepStart" value="23:00" inputmode="numeric"></label>
      <label>持续分钟<input id="sleepDuration" type="number" value="480"></label>
      <label>控制间隔秒<input id="controlInterval" type="number" min="5" value="600"></label>
      <label>温度死区 ℃<input id="deadband" type="number" step="0.1" value="0.4"></label>
    </div>
    <div class="curve-editor">
      <div class="curve-toolbar">
        <div>
          <h3>曲线图</h3>
          <div class="label" id="curvePointInfo">拖动控制点调整目标室温，双击图表添加控制点。</div>
        </div>
        <div class="curve-buttons">
          <button class="secondary" type="button" onclick="addCurvePoint()">添加点</button>
          <button class="danger" type="button" onclick="removeSelectedCurvePoint()">删除选中</button>
        </div>
      </div>
      <div class="curve-stage">
        <svg id="curveSvg" viewBox="0 0 720 280" role="img" aria-label="睡眠温度曲线图"></svg>
      </div>
      <div class="curve-list" id="curveList"></div>
      <div class="curve-adjuster">
        <div class="curve-adjuster-head">
          <div>
            <h3>选中点微调</h3>
            <div class="label" id="curveAdjustInfo">先点选一个控制点。</div>
          </div>
        </div>
        <div class="curve-step-grid">
          <div>
            <div class="label">温度</div>
            <div class="curve-stepper">
              <button class="secondary" type="button" onclick="adjustSelectedCurveTemp(-0.5)">-0.5</button>
              <output id="curveTempOut">-- ℃</output>
              <button class="secondary" type="button" onclick="adjustSelectedCurveTemp(0.5)">+0.5</button>
            </div>
          </div>
          <div>
            <div class="label">时间</div>
            <div class="curve-stepper">
              <button class="secondary" type="button" data-curve-minute-step onclick="adjustSelectedCurveMinute(-15)">-15</button>
              <output id="curveMinuteOut">-- 分</output>
              <button class="secondary" type="button" data-curve-minute-step onclick="adjustSelectedCurveMinute(15)">+15</button>
            </div>
          </div>
        </div>
      </div>
      <small>首尾点固定在入睡开始和结束时间，只拖动温度；中间点可拖动时间和温度。</small>
    </div>
    <label style="margin-top:14px">曲线 JSON<textarea id="curve" rows="5">[{"minute":0,"temp":27},{"minute":90,"temp":26.5},{"minute":300,"temp":25.5},{"minute":480,"temp":26.5}]</textarea></label>
  </section>

  <section>
    <div class="section-head">
      <div>
        <h2>维护与闭环设置</h2>
        <div class="subhead">校准自身传感器、设置库支持能力、调整闭环策略，并支持配置备份和浏览器 OTA。</div>
      </div>
      <div class="badge" id="settingsBadge">待保存</div>
    </div>
    <div class="settings-panel">
      <h3>传感器校准</h3>
      <div class="form-grid">
        <label>室温修正 ℃<input id="sensorTempOffset" type="number" step="0.1" min="-5" max="5" value="0"></label>
        <label>湿度修正 %<input id="sensorHumidityOffset" type="number" step="1" min="-20" max="20" value="0"></label>
      </div>
      <h3>闭环增强</h3>
      <div class="form-grid">
        <label>预测跳过<select id="predictiveSkipEnabled"><option value="true">开启</option><option value="false">关闭</option></select></label>
        <label>自学习闭环<select id="adaptiveControlEnabled"><option value="false">关闭</option><option value="true">开启</option></select></label>
        <label>急速增益<input id="closedLoopFastGain" type="number" step="0.1" min="0.5" max="5" value="2.6"></label>
        <label>安静增益<input id="closedLoopQuietGain" type="number" step="0.1" min="0.2" max="3" value="0.9"></label>
      </div>
      <h3>空调库能力</h3>
      <div class="form-grid">
        <label>强劲<select id="capTurbo"><option value="true">支持</option><option value="false">不支持</option></select></label>
        <label>静音<select id="capQuiet"><option value="true">支持</option><option value="false">不支持</option></select></label>
        <label>睡眠<select id="capSleep"><option value="true">支持</option><option value="false">不支持</option></select></label>
        <label>上下摆风<select id="capSwingV"><option value="true">支持</option><option value="false">不支持</option></select></label>
        <label>左右摆风<select id="capSwingH"><option value="true">支持</option><option value="false">不支持</option></select></label>
        <label>滤网/出风口<select id="capFilter"><option value="true">支持</option><option value="false">不支持</option></select></label>
      </div>
      <div class="mini-actions">
        <button type="button" onclick="saveSettings()">保存设置</button>
        <button class="secondary" type="button" onclick="exportConfig()">导出配置</button>
        <button class="secondary" type="button" onclick="$('configImportFile').click()">导入配置</button>
        <button class="secondary" type="button" onclick="$('otaFile').click()">OTA 升级</button>
      </div>
      <input id="configImportFile" type="file" accept="application/json,.json" style="display:none">
      <input id="otaFile" type="file" accept=".bin,application/octet-stream" style="display:none">
    </div>
  </section>

  <section>
    <div class="section-head">
      <div>
        <h2>控制事件日志</h2>
        <div class="subhead">记录自动曲线、湿度控制、手动、快捷和定时指令，可按来源和类型筛选。</div>
      </div>
      <div class="header-links"><div class="badge" id="controlLogBadge">等待数据</div><button class="secondary" type="button" onclick="refreshControlLog()">刷新</button></div>
    </div>
    <div class="form-grid compact">
      <label>事件来源<select id="controlLogSourceFilter" onchange="renderControlLog(); renderTempHistory()">
        <option value="all">全部</option>
        <option value="auto">自动曲线</option>
        <option value="humidity">湿度</option>
        <option value="manual">手动</option>
        <option value="preset_schedule">快捷/定时</option>
        <option value="system">结束/系统</option>
      </select></label>
      <label>事件类型<select id="controlLogTypeFilter" onchange="renderControlLog(); renderTempHistory()">
        <option value="all">全部</option>
        <option value="sent">发送</option>
        <option value="queue">排队</option>
        <option value="skip">跳过</option>
        <option value="end">结束</option>
        <option value="failed">失败</option>
      </select></label>
    </div>
    <div class="control-log" id="controlLog"><div class="empty">暂无决策日志</div></div>
  </section>

  <section>
    <div class="section-head">
      <div>
        <h2>WiFi 配网</h2>
        <div class="subhead">保存后设备会优先连接路由器；连接失败时保留热点模式。</div>
      </div>
      <div class="badge" id="clock">时间未同步</div>
    </div>
    <div class="form-grid">
      <label>WiFi 名称<input id="staSsid" autocomplete="off"></label>
      <label>WiFi 密码<input id="staPassword" type="password" autocomplete="new-password" placeholder="不修改密码可留空"></label>
    </div>
    <div class="wifi-scan-list" id="wifiScanList"></div>
    <div class="actions">
      <button onclick="saveWifi()">保存并连接</button>
      <button class="secondary" id="wifiScanButton" onclick="scanWifiNetworks()">扫描热点</button>
      <button class="secondary" onclick="forgetWifi()">忘记 WiFi</button>
    </div>
    <small>设备热点：IR-AC-S3，密码：12345678。当前主机名：ir-ac-s3。</small>
  </section>
</main>
<script>
let state = {};
let refreshInFlight = false;
let liveRefreshInFlight = false;
const $ = id => document.getElementById(id);
const dirty = new Set();
const guardedIds = ['staSsid','staPassword','protocol','model','power','mode','degrees','fan','specialMode','swingV','swingH','filterFlag','learnName','learnFreq','learnPower','learnMode','learnDegrees','learnFan','autoEnabled','autoMode','curveControlMode','curveEndAction','curveHumidityEnabled','quietSwitchMinute','sleepStart','sleepDuration','controlInterval','deadband','autoSendDelta','curve','sensorTempOffset','sensorHumidityOffset','humidityControlEnabled','targetHumidity','humidityDeadband','humidityTargetTemp','predictiveSkipEnabled','adaptiveControlEnabled','closedLoopFastGain','closedLoopQuietGain','capTurbo','capQuiet','capSleep','capSwingV','capSwingH','capFilter'];
const curveView = {w:720, h:280, l:44, r:14, t:10, b:30};
const historyView = {w:720, h:260, l:50, r:52, t:18, b:38};
let tempHistoryRefreshInFlight = false;
let controlLogRefreshInFlight = false;
const tempHistoryState = {samples:[], events:[], usesEpoch:false, eventsUseEpoch:false, start:null, end:null, minTemp:0, maxTemp:0, followLatest:true, hover:null};
const controlLogState = {logs:[], usesEpoch:false};
const tempHistoryPinch = {active:false, startDistance:0, startSpan:0, anchor:0, ratio:0.5};
const tempHistoryDrag = {active:false, pointerId:null, startClientX:0, startSvgX:0, startStart:0, startEnd:0, moved:false};
let curveRenderRange = null;
let curveAutoSaveTimer = 0;
let curveAutoSaveInFlight = false;
let curveAutoSavePending = false;
let remoteStateSaveTimer = 0;
const openPresetSchedules = new Set();
function setSkin(name){
  const skin = name === 'bluehome' ? 'bluehome' : 'cream';
  document.body.dataset.skin = skin;
  document.querySelectorAll('[data-skin-choice]').forEach(btn => {
    btn.classList.toggle('selected', btn.dataset.skinChoice === skin);
  });
  try { localStorage.setItem('ir-ac-skin', skin); } catch(e) {}
}
function initSkin(){
  let skin = 'cream';
  try { skin = localStorage.getItem('ir-ac-skin') || 'cream'; } catch(e) {}
  setSkin(skin);
}
function ensureCurveStrategyControls(){
  if ($('curveControlMode') || !$('autoMode')) return;
  if ($('sleepDuration')) {
    $('sleepDuration').type = 'time';
    $('sleepDuration').step = 300;
    const durationLabel = $('sleepDuration').closest('label');
    if (durationLabel && durationLabel.firstChild) durationLabel.firstChild.textContent = '结束时间';
  }
  if ($('sleepStart')) {
    $('sleepStart').type = 'time';
    $('sleepStart').step = 300;
  }
  if ($('deadband') && !$('autoSendDelta')) {
    $('deadband').closest('label').insertAdjacentHTML('afterend',
      '<label>发送过滤阈值 ℃<input id="autoSendDelta" type="number" min="0" step="0.1" value="0.5"></label>');
  }
  $('autoMode').closest('label').insertAdjacentHTML('afterend', `
    <label>控制策略<select id="curveControlMode">
      <option value="staged">先急速后安静</option>
      <option value="fast">急速直到安静时间</option>
      <option value="quiet">始终安静</option>
    </select></label>
    <label>切换安静时间<input id="quietSwitchMinute" type="time" step="300" value="00:30"></label>
    <label>结束模式<select id="curveEndAction">
      <option value="hold">保持状态（不发射）</option>
      <option value="poweroff">结束时关机</option>
    </select></label>
  `);
}
const segmentedSelectIds = [
  'power','mode','fan','specialMode','swingV','swingH','filterFlag',
  'learnPower','learnMode','learnFan',
  'autoEnabled','autoMode','curveControlMode','curveEndAction','curveHumidityEnabled',
  'humidityControlEnabled','predictiveSkipEnabled','adaptiveControlEnabled','capTurbo','capQuiet','capSleep','capSwingV','capSwingH','capFilter',
  'controlLogSourceFilter','controlLogTypeFilter'
];
function syncSegmentedControl(id){
  const select = $(id);
  const group = document.querySelector(`[data-segmented-for="${id}"]`);
  if (!select || !group) return;
  group.querySelectorAll('button').forEach(btn => {
    const opt = Array.from(select.options).find(option => option.value === btn.dataset.value);
    btn.disabled = !!(select.disabled || opt?.disabled);
    btn.classList.toggle('selected', btn.dataset.value === select.value);
  });
}
function enhanceSegmentedControls(){
  segmentedSelectIds.forEach(id => {
    const select = $(id);
    if (!select || select.dataset.segmentedReady) return;
    const options = Array.from(select.options).map(opt => ({value: opt.value, label: opt.textContent.trim()}));
    if (options.length < 2 || options.length > 6) return;

    const group = document.createElement('div');
    group.className = 'segmented' + (options.length > 3 ? ' wrap' : '');
    group.dataset.segmentedFor = id;
    options.forEach(opt => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.dataset.value = opt.value;
      btn.textContent = opt.label;
      btn.addEventListener('click', () => {
        select.value = opt.value;
        if (!id.startsWith('controlLog')) dirty.add(id);
        select.dispatchEvent(new Event('input', {bubbles:true}));
        select.dispatchEvent(new Event('change', {bubbles:true}));
        syncSegmentedControl(id);
        if (id === 'autoMode' || id === 'curveControlMode') renderCurve();
        if (id.startsWith('controlLog')) {
          renderControlLog();
          renderTempHistory();
        }
      });
      group.appendChild(btn);
    });
    select.classList.add('segmented-source');
    select.setAttribute('tabindex', '-1');
    select.closest('label')?.appendChild(group);
    select.dataset.segmentedReady = '1';
    syncSegmentedControl(id);
  });
}
function ensureTempHistorySection(){
  if ($('tempHistorySvg')) return;
  const curveSection = $('curveSvg')?.closest('section');
  if (!curveSection) return;
  curveSection.insertAdjacentHTML('afterend', `
    <section>
      <div class="section-head">
        <div>
          <h2>72小时温湿度曲线</h2>
          <div class="subhead">每分钟记录一次自身传感器温湿度，用来检查实际环境是否贴近睡眠曲线。</div>
        </div>
        <div class="header-links"><div class="badge" id="tempHistoryBadge">等待数据</div><button class="secondary" type="button" onclick="refreshTempHistory()">刷新</button></div>
      </div>
      <div class="curve-stage history-stage" id="tempHistoryStage">
        <svg id="tempHistorySvg" viewBox="0 0 720 260" role="img" aria-label="72小时温湿度曲线"></svg>
        <div class="history-tooltip" id="tempHistoryTip"></div>
      </div>
      <div class="label" id="tempHistoryInfo" style="margin-top:10px">暂无记录</div>
    </section>
  `);
}
function enableCollapsibleSections(){
  document.querySelectorAll('main > section').forEach((section, index) => {
    if (section.dataset.collapsibleReady) return;
    let head = section.querySelector(':scope > .section-head');
    if (!head) {
      head = document.createElement('div');
      head.className = 'section-head';
      head.innerHTML = '<div><h2>当前状态</h2></div>';
      section.insertBefore(head, section.firstChild);
    }
    const body = document.createElement('div');
    body.className = 'collapsible-body';
    while (head.nextSibling) body.appendChild(head.nextSibling);
    section.appendChild(body);

    const title = (head.querySelector('h2')?.textContent || `板块${index + 1}`).trim();
    const key = `config-section-collapsed-v1:${index}:${title}`;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'secondary collapse-toggle';
    const apply = collapsed => {
      section.classList.toggle('collapsed', collapsed);
      btn.textContent = '';
      btn.title = collapsed ? '展开' : '折叠';
      btn.setAttribute('aria-label', collapsed ? '展开' : '折叠');
      btn.setAttribute('aria-expanded', collapsed ? 'false' : 'true');
      try { localStorage.setItem(key, collapsed ? '1' : '0'); } catch(e) {}
    };
    btn.addEventListener('click', () => apply(!section.classList.contains('collapsed')));
    head.appendChild(btn);
    section.dataset.collapsibleReady = '1';
    let collapsed = false;
    try { collapsed = localStorage.getItem(key) === '1'; } catch(e) {}
    apply(collapsed);
  });
}
const maxCurvePoints = 8;
let selectedCurveIndex = 0;
let draggingCurveIndex = -1;
const modeText = {cool:'制冷', auto:'自动', dry:'除湿', heat:'制热', fan:'送风'};
const fanText = {auto:'自动', min:'最小', low:'低风', medium:'中风', high:'高风', max:'最大'};

function hhmmToMin(s){
  const p = String(s || '').split(':').map(Number);
  return ((p[0] || 0) * 60 + (p[1] || 0)) % 1440;
}
function minToHhmm(m){
  m = (Number(m || 0) + 1440) % 1440;
  return String(Math.floor(m / 60)).padStart(2,'0') + ':' + String(m % 60).padStart(2,'0');
}
function minutesBetween(startMinute, endMinute, fullDayIfSame=false){
  const diff = (Number(endMinute || 0) + 1440 - Number(startMinute || 0)) % 1440;
  return diff === 0 && fullDayIfSame ? 1439 : diff;
}
function curveDurationFromTimes(){
  return minutesBetween(hhmmToMin($('sleepStart').value), hhmmToMin($('sleepDuration').value), true);
}
function quietSwitchFromTimes(){
  return minutesBetween(hhmmToMin($('sleepStart').value), hhmmToMin($('quietSwitchMinute').value), false);
}
function esc(value){
  return String(value ?? '').replace(/[&<>"']/g, ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
}
function clamp(value, min, max){ return Math.min(max, Math.max(min, value)); }
function roundTemp(value){ return Math.round(Number(value) * 2) / 2; }
function roundMinute(value){ return Math.round(Number(value) / 5) * 5; }
function historyX(minute, startMinute, endMinute){
  const plotW = historyView.w - historyView.l - historyView.r;
  return historyView.l + (minute - startMinute) / Math.max(1, endMinute - startMinute) * plotW;
}
function historyY(temp, minTemp, maxTemp){
  const plotH = historyView.h - historyView.t - historyView.b;
  return historyView.t + (maxTemp - temp) / Math.max(1, maxTemp - minTemp) * plotH;
}
function logSource(item){
  const stage = String(item?.source || item?.stage || '').toLowerCase();
  if (['auto','curve','fast','quiet'].includes(stage)) return 'auto';
  if (stage === 'humidity') return 'humidity';
  if (stage === 'manual') return 'manual';
  if (stage === 'preset' || stage === 'schedule') return 'preset_schedule';
  if (stage === 'end' || stage === 'system') return 'system';
  return stage || 'system';
}
function logType(item){
  const action = String(item?.action || '').toLowerCase();
  if (action.startsWith('skip')) return 'skip';
  if (action.startsWith('queue') || action === 'humidity_off') return 'queue';
  if (action.startsWith('sent')) return 'sent';
  if (action.includes('failed')) return 'failed';
  if (action.startsWith('end')) return 'end';
  return 'sent';
}
function logSourceText(source){
  return {auto:'自动曲线', humidity:'湿度', manual:'手动', preset_schedule:'快捷/定时', system:'结束/系统'}[source] || source || '事件';
}
function logTypeText(type){
  return {sent:'发送', queue:'排队', skip:'跳过', end:'结束', failed:'失败'}[type] || type || '事件';
}
function filteredControlLogs(logs=controlLogState.logs){
  const source = $('controlLogSourceFilter')?.value || 'all';
  const type = $('controlLogTypeFilter')?.value || 'all';
  return (logs || []).filter(item => {
    if (source !== 'all' && logSource(item) !== source) return false;
    if (type !== 'all' && logType(item) !== type) return false;
    return true;
  });
}
function linePath(points, valueKey, minValue, maxValue){
  let path = '';
  let open = false;
  points.forEach(p => {
    const value = Number(p[valueKey]);
    if (!Number.isFinite(value)) {
      open = false;
      return;
    }
    path += `${open ? 'L' : 'M'} ${historyX(p.minute, tempHistoryState.start, tempHistoryState.end).toFixed(1)} ${historyY(value, minValue, maxValue).toFixed(1)} `;
    open = true;
  });
  return path.trim();
}
function historyTicks(min, max, count=5){
  if (!Number.isFinite(min) || !Number.isFinite(max) || max <= min) return [];
  const ticks = [];
  for (let i = 0; i < count; i++) ticks.push(min + (max - min) * i / Math.max(1, count - 1));
  return ticks;
}
function historySvgPointFromClient(clientX, clientY=0){
  const svg = $('tempHistorySvg');
  if (!svg) return {x: historyView.l + (historyView.w - historyView.l - historyView.r) / 2, y: historyView.t};
  if (svg.createSVGPoint && svg.getScreenCTM()) {
    const point = svg.createSVGPoint();
    point.x = clientX;
    point.y = clientY;
    const matrix = svg.getScreenCTM();
    if (matrix) return point.matrixTransform(matrix.inverse());
  }
  const rect = svg.getBoundingClientRect();
  return {
    x: (clientX - rect.left) * historyView.w / Math.max(1, rect.width),
    y: (clientY - rect.top) * historyView.h / Math.max(1, rect.height)
  };
}
function historySvgXFromClient(clientX){
  return historySvgPointFromClient(clientX, 0).x;
}
function formatHistoryLabel(minute, usesEpoch, latestMinute){
  if (usesEpoch) {
    const d = new Date(minute * 60000);
    return `${String(d.getMonth() + 1).padStart(2,'0')}/${String(d.getDate()).padStart(2,'0')} ${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}`;
  }
  const d = new Date(Date.now() - Math.max(0, latestMinute - minute) * 60000);
  return `${String(d.getMonth() + 1).padStart(2,'0')}/${String(d.getDate()).padStart(2,'0')} ${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}`;
}
function renderTempHistory(data=null){
  const svg = $('tempHistorySvg');
  if (!svg) return;
  if (data) {
    tempHistoryState.samples = (Array.isArray(data.samples) ? data.samples : [])
      .map(p => {
        const humidity = Number(p.humidity);
        return {minute:Number(p.minute), temp:Number(p.temp), humidity:Number.isFinite(humidity) ? humidity : null};
      })
      .filter(p => Number.isFinite(p.minute) && Number.isFinite(p.temp));
    tempHistoryState.usesEpoch = !!data.usesEpoch;
    if (Array.isArray(data.events)) {
      tempHistoryState.events = data.events
        .map(e => ({minute:Number(e.minute), source:e.source, stage:e.stage, action:e.action, note:e.note, mode:e.mode, fan:e.fan, setpoint:Number(e.setpoint), target:Number(e.target), room:Number(e.room), power:!!e.power, turbo:!!e.turbo, quiet:!!e.quiet, sleep:!!e.sleep}))
        .filter(e => Number.isFinite(e.minute));
      tempHistoryState.eventsUseEpoch = !!data.eventsUseEpoch;
    }
  }
  const samples = tempHistoryState.samples;
  if (!samples.length) {
    svg.innerHTML = '<text x="360" y="130" text-anchor="middle" class="curve-label">暂无温湿度记录，运行满1分钟后会出现第一个点</text>';
    if ($('tempHistoryBadge')) $('tempHistoryBadge').textContent = '0 / 4320 点';
    if ($('tempHistoryInfo')) $('tempHistoryInfo').textContent = '历史记录已持久化保存；滚轮缩放横轴，拖动可左右平移。';
    return;
  }

  const latest = samples[samples.length - 1].minute;
  const oldest = samples[0].minute;
  const fullStart = Math.max(oldest, latest - 72 * 60 + 1);
  const fullEnd = latest;
  if (tempHistoryState.start == null || tempHistoryState.end == null || tempHistoryState.followLatest) {
    const span = tempHistoryState.start == null ? (fullEnd - fullStart) : (tempHistoryState.end - tempHistoryState.start);
    tempHistoryState.end = fullEnd;
    tempHistoryState.start = Math.max(fullStart, fullEnd - Math.max(60, span));
  }
  tempHistoryState.start = clamp(tempHistoryState.start, fullStart, fullEnd);
  tempHistoryState.end = clamp(tempHistoryState.end, tempHistoryState.start + 1, fullEnd);
  const start = tempHistoryState.start;
  const end = tempHistoryState.end;
  const visible = samples.filter(p => p.minute >= start && p.minute <= end);
  const viewSamples = visible.length ? visible : samples.slice(-1);
  const temps = viewSamples.map(p => p.temp);
  const humidities = viewSamples.map(p => Number(p.humidity)).filter(Number.isFinite);
  let minTemp = Math.floor(Math.min(...temps) - 1);
  let maxTemp = Math.ceil(Math.max(...temps) + 1);
  minTemp = clamp(minTemp, 10, 40);
  maxTemp = clamp(maxTemp, minTemp + 2, 45);
  let minHumidity = humidities.length ? Math.floor(Math.min(...humidities) - 5) : 0;
  let maxHumidity = humidities.length ? Math.ceil(Math.max(...humidities) + 5) : 100;
  minHumidity = clamp(minHumidity, 0, 95);
  maxHumidity = clamp(maxHumidity, minHumidity + 10, 100);
  tempHistoryState.minTemp = minTemp;
  tempHistoryState.maxTemp = maxTemp;

  const baseY = historyView.h - historyView.b;
  const path = linePath(visible, 'temp', minTemp, maxTemp);
  const humidityPath = linePath(visible, 'humidity', minHumidity, maxHumidity);
  const fill = path ? `${path} L ${historyX(visible[visible.length - 1].minute, start, end).toFixed(1)} ${baseY} L ${historyX(visible[0].minute, start, end).toFixed(1)} ${baseY} Z` : '';
  const xTicks = [0, 0.25, 0.5, 0.75, 1].map(v => Math.round(start + (end - start) * v));
  const yTicks = [];
  for (let t = Math.ceil(minTemp); t <= Math.floor(maxTemp); t++) yTicks.push(t);
  const humidityTicks = historyTicks(minHumidity, maxHumidity, 5);
  const grid = [
    ...xTicks.map(m => `<line class="curve-grid" x1="${historyX(m,start,end).toFixed(1)}" y1="${historyView.t}" x2="${historyX(m,start,end).toFixed(1)}" y2="${baseY}"></line><text class="curve-label" x="${historyX(m,start,end).toFixed(1)}" y="${historyView.h - 12}" text-anchor="middle">${formatHistoryLabel(m, tempHistoryState.usesEpoch, latest)}</text>`),
    ...yTicks.map(t => `<line class="curve-grid" x1="${historyView.l}" y1="${historyY(t,minTemp,maxTemp).toFixed(1)}" x2="${historyView.w - historyView.r}" y2="${historyY(t,minTemp,maxTemp).toFixed(1)}"></line><text class="curve-label" x="8" y="${(historyY(t,minTemp,maxTemp) + 4).toFixed(1)}">${t}℃</text>`),
    ...humidityTicks.map(h => `<text class="curve-label" x="${historyView.w - 8}" y="${(historyY(h,minHumidity,maxHumidity) + 4).toFixed(1)}" text-anchor="end">${Math.round(h)}%</text>`)
  ].join('');
  const last = samples[samples.length - 1];
  let hover = '';
  if (tempHistoryState.hover && tempHistoryState.hover.minute >= start && tempHistoryState.hover.minute <= end) {
    const hx = historyX(tempHistoryState.hover.minute, start, end).toFixed(1);
    const hy = historyY(tempHistoryState.hover.temp, minTemp, maxTemp).toFixed(1);
    const hh = Number(tempHistoryState.hover.humidity);
    const humidityDot = Number.isFinite(hh)
      ? `<circle class="history-hover-dot humidity" cx="${hx}" cy="${historyY(hh, minHumidity, maxHumidity).toFixed(1)}" r="4.5"></circle>`
      : '';
    hover = `<line class="history-hover-line" x1="${hx}" y1="${historyView.t}" x2="${hx}" y2="${baseY}"></line><circle class="history-hover-dot" cx="${hx}" cy="${hy}" r="5"></circle>${humidityDot}`;
  }
  let lastNode = '';
  if (last.minute >= start && last.minute <= end) {
    const lx = historyX(last.minute,start,end).toFixed(1);
    lastNode = `<circle cx="${lx}" cy="${historyY(last.temp,minTemp,maxTemp).toFixed(1)}" r="5" fill="var(--ok)"></circle>`;
    if (Number.isFinite(Number(last.humidity))) lastNode += `<circle cx="${lx}" cy="${historyY(Number(last.humidity),minHumidity,maxHumidity).toFixed(1)}" r="4.5" fill="var(--warn)"></circle>`;
  }
  const eventMarkers = tempHistoryState.eventsUseEpoch === tempHistoryState.usesEpoch
    ? filteredControlLogs(tempHistoryState.events).filter(e => e.minute >= start && e.minute <= end).map(e => {
      const x = historyX(e.minute, start, end).toFixed(1);
      return `<line class="history-event-line" x1="${x}" y1="${historyView.t}" x2="${x}" y2="${baseY}"></line><circle class="history-event-dot" cx="${x}" cy="${historyView.t + 10}" r="4"></circle>`;
    }).join('')
    : '';
  const legend = `<text class="curve-label" x="${historyView.l}" y="12" style="fill:var(--ok)">室温 ℃</text><text class="curve-label" x="${historyView.w - historyView.r}" y="12" text-anchor="end" style="fill:var(--warn)">湿度 %</text>`;
  svg.innerHTML = `${grid}${legend}<line class="curve-axis" x1="${historyView.l}" y1="${baseY}" x2="${historyView.w - historyView.r}" y2="${baseY}"></line><line class="curve-axis" x1="${historyView.l}" y1="${historyView.t}" x2="${historyView.l}" y2="${baseY}"></line><line class="curve-axis" x1="${historyView.w - historyView.r}" y1="${historyView.t}" x2="${historyView.w - historyView.r}" y2="${baseY}"></line>${fill ? `<path class="history-fill" d="${fill}"></path><path class="history-line" d="${path}"></path>` : ''}${humidityPath ? `<path class="history-humidity-line" d="${humidityPath}"></path>` : ''}${eventMarkers}${lastNode}${hover}`;
  if ($('tempHistoryBadge')) $('tempHistoryBadge').textContent = `${samples.length} / 4320 点`;
  if ($('tempHistoryInfo')) {
    const spanHours = Math.max(1, Math.round((end - start) / 60));
    const lastHumidity = Number(last.humidity);
    const humidityText = Number.isFinite(lastHumidity) ? ` / ${lastHumidity.toFixed(0)}%` : '';
    $('tempHistoryInfo').textContent = `最近 ${last.temp.toFixed(1)} ℃${humidityText}，当前视窗约 ${spanHours} 小时；滚轮缩放，拖动平移，悬停查看时间、温度和湿度。`;
  }
}
async function refreshTempHistory(){
  if (tempHistoryRefreshInFlight || !$('tempHistorySvg')) return;
  tempHistoryRefreshInFlight = true;
  try {
    const [historyRes, eventsRes] = await Promise.all([fetch('/api/temp-history'), fetch('/api/control-log')]);
    if (!historyRes.ok) throw new Error(await historyRes.text());
    const history = await historyRes.json();
    if (eventsRes.ok) {
      const eventData = await eventsRes.json();
      history.events = eventData.logs || [];
      history.eventsUseEpoch = !!eventData.usesEpoch;
      controlLogState.logs = eventData.logs || [];
      controlLogState.usesEpoch = !!eventData.usesEpoch;
    }
    renderTempHistory(history);
  } catch(e) {
    if ($('tempHistoryInfo')) $('tempHistoryInfo').textContent = e.message || '室温历史读取失败';
  } finally {
    tempHistoryRefreshInFlight = false;
  }
}
function historyPointFromClient(clientX, clientY=0){
  const svg = $('tempHistorySvg');
  if (!svg || tempHistoryState.start == null || tempHistoryState.end == null) return null;
  const x = historySvgPointFromClient(clientX, clientY).x;
  const ratio = clamp((x - historyView.l) / (historyView.w - historyView.l - historyView.r), 0, 1);
  const minute = tempHistoryState.start + ratio * (tempHistoryState.end - tempHistoryState.start);
  let best = null;
  let bestDist = Infinity;
  tempHistoryState.samples.forEach(p => {
    if (p.minute < tempHistoryState.start || p.minute > tempHistoryState.end) return;
    const dist = Math.abs(p.minute - minute);
    if (dist < bestDist) { bestDist = dist; best = p; }
  });
  return best;
}
function historyPointFromEvent(evt){
  return historyPointFromClient(evt.clientX, evt.clientY);
}
function nearestHistoryEvent(minute){
  if (tempHistoryState.eventsUseEpoch !== tempHistoryState.usesEpoch) return null;
  const span = Math.max(1, (tempHistoryState.end || minute) - (tempHistoryState.start || minute));
  const tolerance = Math.max(2, span / 120);
  let best = null;
  let bestDist = Infinity;
  filteredControlLogs(tempHistoryState.events).forEach(e => {
    if (e.minute < tempHistoryState.start || e.minute > tempHistoryState.end) return;
    const dist = Math.abs(e.minute - minute);
    if (dist < bestDist) { bestDist = dist; best = e; }
  });
  return bestDist <= tolerance ? best : null;
}
function historyEventText(event){
  if (!event) return '';
  const sourceText = logSourceText(logSource(event));
  const action = actionText(event.action);
  const setpoint = Number.isFinite(event.setpoint) ? ` · 设定 ${event.setpoint.toFixed(1)}℃` : '';
  const mode = modeText[event.mode] || event.mode || '--';
  const fan = fanText[event.fan] || event.fan || '--';
  return `<span>${sourceText} · ${action} · ${mode}/${fan}${setpoint}</span>`;
}
function showTempHistoryTooltipAt(clientX, clientY){
  const tip = $('tempHistoryTip');
  const stage = $('tempHistoryStage');
  const point = historyPointFromClient(clientX, clientY);
  if (!tip || !stage || !point) return;
  tempHistoryState.hover = point;
  renderTempHistory();
  const rect = stage.getBoundingClientRect();
  const x = clamp(clientX - rect.left + 12, 8, rect.width - 150);
  const y = clamp(clientY - rect.top - 54, 8, rect.height - 62);
  tip.style.left = x + 'px';
  tip.style.top = y + 'px';
  tip.style.display = 'block';
  const event = nearestHistoryEvent(point.minute);
  const humidity = Number(point.humidity);
  const humidityText = Number.isFinite(humidity) ? ` · ${humidity.toFixed(0)}%` : '';
  tip.innerHTML = `<strong>${point.temp.toFixed(1)} ℃${humidityText}</strong><span>${formatHistoryLabel(point.minute, tempHistoryState.usesEpoch, tempHistoryState.samples[tempHistoryState.samples.length - 1]?.minute || point.minute)}</span>${historyEventText(event)}`;
}
function showTempHistoryTooltip(evt){
  if (tempHistoryDrag.active) return;
  showTempHistoryTooltipAt(evt.clientX, evt.clientY);
}
function hideTempHistoryTooltip(){
  tempHistoryState.hover = null;
  if ($('tempHistoryTip')) $('tempHistoryTip').style.display = 'none';
  renderTempHistory();
}
function historyRatioFromClientX(clientX){
  return clamp((historySvgXFromClient(clientX) - historyView.l) / (historyView.w - historyView.l - historyView.r), 0, 1);
}
function tempHistoryBounds(){
  if (!tempHistoryState.samples.length) return null;
  const latest = tempHistoryState.samples[tempHistoryState.samples.length - 1].minute;
  const oldest = Math.max(tempHistoryState.samples[0].minute, latest - 72 * 60 + 1);
  return {oldest, latest, maxSpan: Math.max(1, latest - oldest)};
}
function setTempHistoryWindow(start, end){
  const bounds = tempHistoryBounds();
  if (!bounds) return;
  let span = clamp(end - start, 1, bounds.maxSpan);
  if (!Number.isFinite(span)) span = bounds.maxSpan;
  if (start < bounds.oldest) { start = bounds.oldest; end = start + span; }
  if (end > bounds.latest) { end = bounds.latest; start = end - span; }
  if (start < bounds.oldest) { start = bounds.oldest; end = Math.min(bounds.latest, start + span); }
  tempHistoryState.start = clamp(start, bounds.oldest, bounds.latest);
  tempHistoryState.end = clamp(end, tempHistoryState.start, bounds.latest);
  tempHistoryState.followLatest = Math.abs(tempHistoryState.end - bounds.latest) < 1;
  renderTempHistory();
}
function applyTempHistoryZoom(nextSpan, ratio, anchor=null){
  if (!tempHistoryState.samples.length || tempHistoryState.start == null || tempHistoryState.end == null) return;
  const bounds = tempHistoryBounds();
  if (!bounds) return;
  const minSpan = Math.min(10, bounds.maxSpan);
  const currentSpan = Math.max(1, tempHistoryState.end - tempHistoryState.start);
  nextSpan = clamp(nextSpan, minSpan, bounds.maxSpan);
  anchor = anchor == null ? tempHistoryState.start + ratio * currentSpan : anchor;
  let start = anchor - ratio * nextSpan;
  let end = start + nextSpan;
  setTempHistoryWindow(start, end);
}
function zoomTempHistory(evt){
  if (!tempHistoryState.samples.length || tempHistoryState.start == null || tempHistoryState.end == null) return;
  evt.preventDefault();
  const currentSpan = Math.max(1, tempHistoryState.end - tempHistoryState.start);
  const factor = evt.deltaY < 0 ? 0.75 : 1.33;
  applyTempHistoryZoom(currentSpan * factor, historyRatioFromClientX(evt.clientX));
}
function tempHistoryPinchDistance(touches){
  return Math.max(1, Math.abs(touches[0].clientX - touches[1].clientX));
}
function tempHistoryStartDrag(clientX, clientY=0, pointerId=null){
  if (!tempHistoryState.samples.length || tempHistoryState.start == null || tempHistoryState.end == null) return false;
  tempHistoryDrag.active = true;
  tempHistoryDrag.pointerId = pointerId;
  tempHistoryDrag.startClientX = clientX;
  tempHistoryDrag.startSvgX = historySvgXFromClient(clientX);
  tempHistoryDrag.startStart = tempHistoryState.start;
  tempHistoryDrag.startEnd = tempHistoryState.end;
  tempHistoryDrag.moved = false;
  ($('tempHistoryStage') || $('tempHistorySvg'))?.classList.add('dragging');
  showTempHistoryTooltipAt(clientX, clientY);
  return true;
}
function tempHistoryMoveDrag(clientX, clientY=0){
  if (!tempHistoryDrag.active) return false;
  const plotW = historyView.w - historyView.l - historyView.r;
  const span = Math.max(1, tempHistoryDrag.startEnd - tempHistoryDrag.startStart);
  const dx = historySvgXFromClient(clientX) - tempHistoryDrag.startSvgX;
  if (Math.abs(dx) > 2) tempHistoryDrag.moved = true;
  const deltaMinute = -dx / Math.max(1, plotW) * span;
  setTempHistoryWindow(tempHistoryDrag.startStart + deltaMinute, tempHistoryDrag.startEnd + deltaMinute);
  showTempHistoryTooltipAt(clientX, clientY);
  return true;
}
function tempHistoryEndDrag(){
  if (!tempHistoryDrag.active) return false;
  ($('tempHistoryStage') || $('tempHistorySvg'))?.classList.remove('dragging');
  tempHistoryDrag.active = false;
  tempHistoryDrag.pointerId = null;
  return true;
}
function tempHistoryTouchStart(evt){
  if (!tempHistoryState.samples.length || tempHistoryState.start == null || tempHistoryState.end == null) return;
  if (evt.touches.length === 1) {
    tempHistoryPinch.active = false;
    if (tempHistoryStartDrag(evt.touches[0].clientX, evt.touches[0].clientY, 'touch')) evt.preventDefault();
    return;
  }
  if (evt.touches.length !== 2) return;
  tempHistoryEndDrag();
  const centerX = (evt.touches[0].clientX + evt.touches[1].clientX) / 2;
  tempHistoryPinch.active = true;
  tempHistoryPinch.startDistance = tempHistoryPinchDistance(evt.touches);
  tempHistoryPinch.startSpan = Math.max(1, tempHistoryState.end - tempHistoryState.start);
  tempHistoryPinch.ratio = historyRatioFromClientX(centerX);
  tempHistoryPinch.anchor = tempHistoryState.start + tempHistoryPinch.ratio * tempHistoryPinch.startSpan;
  hideTempHistoryTooltip();
  evt.preventDefault();
}
function tempHistoryTouchMove(evt){
  if (tempHistoryPinch.active && evt.touches.length === 2) {
    const scale = tempHistoryPinchDistance(evt.touches) / tempHistoryPinch.startDistance;
    applyTempHistoryZoom(tempHistoryPinch.startSpan / Math.max(0.2, scale), tempHistoryPinch.ratio, tempHistoryPinch.anchor);
    evt.preventDefault();
    return;
  }
  if (evt.touches.length === 1 && tempHistoryMoveDrag(evt.touches[0].clientX, evt.touches[0].clientY)) {
    evt.preventDefault();
    return;
  }
}
function tempHistoryTouchEnd(evt){
  if (evt.touches.length < 2) tempHistoryPinch.active = false;
  if (evt.touches.length === 0) {
    tempHistoryEndDrag();
  } else if (!tempHistoryPinch.active && evt.touches.length === 1) {
    tempHistoryStartDrag(evt.touches[0].clientX, evt.touches[0].clientY, 'touch');
  }
}
function tempHistoryPointerDown(evt){
  if (evt.pointerType === 'touch' || evt.button !== 0) return;
  if (!tempHistoryStartDrag(evt.clientX, evt.clientY, evt.pointerId)) return;
  const stage = $('tempHistoryStage') || $('tempHistorySvg');
  stage?.setPointerCapture?.(evt.pointerId);
  evt.preventDefault();
}
function tempHistoryPointerMove(evt){
  if (!tempHistoryDrag.active || evt.pointerId !== tempHistoryDrag.pointerId) return;
  tempHistoryMoveDrag(evt.clientX, evt.clientY);
  evt.preventDefault();
}
function tempHistoryPointerEnd(evt){
  if (!tempHistoryDrag.active || evt.pointerId !== tempHistoryDrag.pointerId) return;
  const stage = $('tempHistoryStage') || $('tempHistorySvg');
  stage?.releasePointerCapture?.(evt.pointerId);
  tempHistoryEndDrag();
  evt.preventDefault();
}
function bindTempHistoryInteractions(){
  const svg = $('tempHistorySvg');
  if (!svg || svg.dataset.historyBound) return;
  const stage = $('tempHistoryStage') || svg;
  svg.dataset.historyBound = '1';
  svg.addEventListener('mousemove', showTempHistoryTooltip);
  svg.addEventListener('mouseleave', hideTempHistoryTooltip);
  stage.addEventListener('wheel', zoomTempHistory, {passive:false});
  stage.addEventListener('touchstart', tempHistoryTouchStart, {passive:false});
  stage.addEventListener('touchmove', tempHistoryTouchMove, {passive:false});
  stage.addEventListener('touchend', tempHistoryTouchEnd);
  stage.addEventListener('touchcancel', tempHistoryTouchEnd);
  stage.addEventListener('pointerdown', tempHistoryPointerDown);
  stage.addEventListener('pointermove', tempHistoryPointerMove);
  stage.addEventListener('pointerup', tempHistoryPointerEnd);
  stage.addEventListener('pointercancel', tempHistoryPointerEnd);
  stage.addEventListener('lostpointercapture', tempHistoryPointerEnd);
}
function curveDuration(){ return Math.max(30, curveDurationFromTimes() || 480); }
function curveRange(){
  const cfg = state.config || {};
  let min = Number(cfg.minSetpoint ?? 16);
  let max = Number(cfg.maxSetpoint ?? 32);
  if (!Number.isFinite(min)) min = 16;
  if (!Number.isFinite(max)) max = 32;
  if (max - min < 2) max = min + 2;
  return {min, max};
}
function curveDisplayRange(points){
  const hard = curveRange();
  const temps = (points || []).map(p => Number(p.temp)).filter(Number.isFinite);
  if (!temps.length) return hard;
  const lo = Math.min(...temps);
  const hi = Math.max(...temps);
  const minSpan = 4;
  let min = lo - 1;
  let max = hi + 1;
  if (max - min < minSpan) {
    const mid = (lo + hi) / 2;
    min = mid - minSpan / 2;
    max = mid + minSpan / 2;
  }
  min = Math.max(hard.min, Math.floor(min * 2) / 2);
  max = Math.min(hard.max, Math.ceil(max * 2) / 2);
  if (max - min < minSpan) {
    if (min <= hard.min) max = Math.min(hard.max, min + minSpan);
    else if (max >= hard.max) min = Math.max(hard.min, max - minSpan);
  }
  return {min, max};
}
function normalizeCurve(raw){
  const duration = curveDuration();
  const range = curveRange();
  let points = Array.isArray(raw) ? raw.map(p => ({
    minute: roundMinute(p.minute ?? 0),
    temp: roundTemp(p.temp ?? 26)
  })).filter(p => Number.isFinite(p.minute) && Number.isFinite(p.temp)) : [];
  if (!points.length) points = [{minute:0, temp:26}, {minute:duration, temp:26}];
  if (points.length === 1) points = [{minute:0, temp:points[0].temp}, {minute:duration, temp:points[0].temp}];
  points.sort((a, b) => a.minute - b.minute);
  if (points.length > maxCurvePoints) {
    points = [points[0], ...points.slice(1, maxCurvePoints - 1), points[points.length - 1]];
  }
  points = points.map(p => ({
    minute: clamp(roundMinute(p.minute), 0, duration),
    temp: clamp(roundTemp(p.temp), range.min, range.max)
  }));
  points[0].minute = 0;
  points[points.length - 1].minute = duration;
  let previous = 0;
  for (let i = 1; i < points.length - 1; i++) {
    const remaining = points.length - 1 - i;
    const minMinute = previous + 5;
    const maxMinute = duration - remaining * 5;
    points[i].minute = clamp(points[i].minute, minMinute, Math.max(minMinute, maxMinute));
    previous = points[i].minute;
  }
  return points.map(p => ({minute: Math.round(p.minute), temp: Number(p.temp.toFixed(1))}));
}
function readCurvePoints(){
  try { return normalizeCurve(JSON.parse($('curve').value)); }
  catch(e) { return null; }
}
function syncCurveTextarea(points, mark=true){
  $('curve').value = JSON.stringify(points);
  if (mark) dirty.add('curve');
  renderCurve(points);
  if (mark) scheduleCurveAutoSave();
}
function curveX(minute){
  const plotW = curveView.w - curveView.l - curveView.r;
  return curveView.l + clamp(minute, 0, curveDuration()) / curveDuration() * plotW;
}
function curveY(temp){
  const range = curveRenderRange || curveRange();
  const plotH = curveView.h - curveView.t - curveView.b;
  return curveView.t + (range.max - temp) / (range.max - range.min) * plotH;
}
function curveTimeLabel(minute){
  return minToHhmm(hhmmToMin($('sleepStart')?.value || '00:00') + minute);
}
function curvePointFromEvent(evt){
  const svg = $('curveSvg');
  const rect = svg.getBoundingClientRect();
  const x = (evt.clientX - rect.left) * curveView.w / rect.width;
  const y = (evt.clientY - rect.top) * curveView.h / rect.height;
  const range = curveRenderRange || curveRange();
  const plotW = curveView.w - curveView.l - curveView.r;
  const plotH = curveView.h - curveView.t - curveView.b;
  const minute = roundMinute((x - curveView.l) / plotW * curveDuration());
  const temp = roundTemp(range.max - (y - curveView.t) / plotH * (range.max - range.min));
  const hard = curveRange();
  return {
    minute: clamp(minute, 0, curveDuration()),
    temp: clamp(temp, hard.min, hard.max)
  };
}
function renderCurve(points=null){
  const svg = $('curveSvg');
  if (!svg) return;
  points = points || readCurvePoints();
  if (!points) {
    svg.innerHTML = '<text x="360" y="140" text-anchor="middle" class="curve-label">曲线 JSON 无法解析</text>';
    $('curveList').innerHTML = '';
    $('curvePointInfo').textContent = '曲线 JSON 无法解析，修正后图表会恢复。';
    $('curveAdjustInfo').textContent = '曲线 JSON 无法解析。';
    $('curveTempOut').textContent = '-- ℃';
    $('curveMinuteOut').textContent = '-- 分';
    return;
  }
  selectedCurveIndex = clamp(selectedCurveIndex, 0, points.length - 1);
  const range = curveDisplayRange(points);
  curveRenderRange = range;
  const duration = curveDuration();
  const baseY = curveView.h - curveView.b;
  const timeTicks = [0, 0.25, 0.5, 0.75, 1].map(v => Math.round(duration * v));
  const tempTicks = [];
  for (let t = Math.ceil(range.min); t <= Math.floor(range.max); t++) tempTicks.push(t);
  const path = points.map((p, i) => `${i ? 'L' : 'M'} ${curveX(p.minute).toFixed(1)} ${curveY(p.temp).toFixed(1)}`).join(' ');
  const fill = `${path} L ${curveX(points[points.length - 1].minute).toFixed(1)} ${baseY} L ${curveX(points[0].minute).toFixed(1)} ${baseY} Z`;
  const grid = [
    ...timeTicks.map(m => `<line class="curve-grid" x1="${curveX(m).toFixed(1)}" y1="${curveView.t}" x2="${curveX(m).toFixed(1)}" y2="${baseY}"></line><text class="curve-label" x="${curveX(m).toFixed(1)}" y="${curveView.h - 12}" text-anchor="middle">${curveTimeLabel(m)}</text>`),
    ...tempTicks.map(t => `<line class="curve-grid" x1="${curveView.l}" y1="${curveY(t).toFixed(1)}" x2="${curveView.w - curveView.r}" y2="${curveY(t).toFixed(1)}"></line><text class="curve-label" x="8" y="${(curveY(t) + 4).toFixed(1)}">${t}℃</text>`)
  ].join('');
  const nodes = points.map((p, i) => {
    const x = curveX(p.minute).toFixed(1);
    const y = curveY(p.temp).toFixed(1);
    const selected = i === selectedCurveIndex ? ' selected' : '';
    return `<g class="curve-node${selected}" data-curve-index="${i}"><circle cx="${x}" cy="${y}" r="13"></circle><text class="curve-temp-label" x="${x}" y="${(Number(y) - 18).toFixed(1)}" text-anchor="middle">${p.temp.toFixed(1)}℃</text></g>`;
  }).join('');
  svg.innerHTML = `${grid}<line class="curve-axis" x1="${curveView.l}" y1="${baseY}" x2="${curveView.w - curveView.r}" y2="${baseY}"></line><line class="curve-axis" x1="${curveView.l}" y1="${curveView.t}" x2="${curveView.l}" y2="${baseY}"></line><path class="curve-fill" d="${fill}"></path><path class="curve-line" d="${path}"></path>${nodes}`;
  renderCurveList(points);
  updateCurveAdjuster(points);
  const point = points[selectedCurveIndex];
  $('curvePointInfo').textContent = `选中：入睡后 ${point.minute} 分钟，目标 ${point.temp.toFixed(1)} ℃`;
}
function renderCurveList(points){
  const start = hhmmToMin($('sleepStart').value);
  $('curveList').innerHTML = points.map((p, i) => `
    <button class="curve-chip ${i === selectedCurveIndex ? 'selected' : ''}" type="button" onclick="selectCurvePoint(${i})">
      <span>${minToHhmm(start + p.minute)} · +${p.minute} 分</span>
      <strong>${p.temp.toFixed(1)} ℃</strong>
    </button>`).join('');
}
function updateCurveAdjuster(points){
  const point = points[selectedCurveIndex];
  const endpoint = selectedCurveIndex === 0 || selectedCurveIndex === points.length - 1;
  $('curveTempOut').textContent = point.temp.toFixed(1) + ' ℃';
  $('curveMinuteOut').textContent = '+' + point.minute + ' 分';
  $('curveAdjustInfo').textContent = endpoint ? '首尾点固定时间，只调温度。' : '用按钮细调，手机上比拖动更稳。';
  document.querySelectorAll('[data-curve-minute-step]').forEach(btn => btn.disabled = endpoint);
}
function selectCurvePoint(index){
  const points = readCurvePoints();
  if (!points) return;
  selectedCurveIndex = clamp(Number(index) || 0, 0, points.length - 1);
  renderCurve(points);
}
function selectNearestCurvePoint(point){
  const points = readCurvePoints();
  if (!points) return;
  let best = 0;
  let bestDist = Infinity;
  points.forEach((p, i) => {
    const dx = curveX(p.minute) - curveX(point.minute);
    const dy = curveY(p.temp) - curveY(point.temp);
    const dist = dx * dx + dy * dy;
    if (dist < bestDist) {
      bestDist = dist;
      best = i;
    }
  });
  selectedCurveIndex = best;
  renderCurve(points);
}
function adjustSelectedCurveTemp(delta){
  const points = readCurvePoints();
  if (!points) return msg('曲线 JSON 无法解析', true);
  selectedCurveIndex = clamp(selectedCurveIndex, 0, points.length - 1);
  const range = curveRange();
  points[selectedCurveIndex].temp = clamp(roundTemp(points[selectedCurveIndex].temp + delta), range.min, range.max);
  syncCurveTextarea(normalizeCurve(points));
}
function adjustSelectedCurveMinute(delta){
  const points = readCurvePoints();
  if (!points) return msg('曲线 JSON 无法解析', true);
  selectedCurveIndex = clamp(selectedCurveIndex, 0, points.length - 1);
  if (selectedCurveIndex === 0 || selectedCurveIndex === points.length - 1) {
    msg('首尾点时间固定，只能调温度', true);
    return;
  }
  const prev = points[selectedCurveIndex - 1].minute + 5;
  const next = points[selectedCurveIndex + 1].minute - 5;
  points[selectedCurveIndex].minute = clamp(roundMinute(points[selectedCurveIndex].minute + delta), prev, next);
  syncCurveTextarea(normalizeCurve(points));
}
function insertCurvePoint(point){
  const points = readCurvePoints();
  if (!points) return msg('曲线 JSON 无法解析', true);
  if (points.length >= maxCurvePoints) return msg('最多 8 个控制点', true);
  point.minute = clamp(roundMinute(point.minute), 5, curveDuration() - 5);
  point.temp = clamp(roundTemp(point.temp), curveRange().min, curveRange().max);
  points.push(point);
  points.sort((a, b) => a.minute - b.minute);
  selectedCurveIndex = points.findIndex(p => p.minute === point.minute && p.temp === point.temp);
  syncCurveTextarea(normalizeCurve(points));
}
function addCurvePoint(){
  const points = readCurvePoints();
  if (!points) return msg('曲线 JSON 无法解析', true);
  if (points.length >= maxCurvePoints) return msg('最多 8 个控制点', true);
  const i = clamp(selectedCurveIndex, 0, points.length - 2);
  const a = points[i];
  const b = points[i + 1];
  if (b.minute - a.minute < 10) return msg('两个点之间至少需要 10 分钟间隔', true);
  selectedCurveIndex = i + 1;
  insertCurvePoint({minute: roundMinute((a.minute + b.minute) / 2), temp: roundTemp((a.temp + b.temp) / 2)});
}
function removeSelectedCurvePoint(){
  const points = readCurvePoints();
  if (!points) return msg('曲线 JSON 无法解析', true);
  if (selectedCurveIndex <= 0 || selectedCurveIndex >= points.length - 1) return msg('首尾点保留，只能调整温度', true);
  points.splice(selectedCurveIndex, 1);
  selectedCurveIndex = clamp(selectedCurveIndex - 1, 0, points.length - 1);
  syncCurveTextarea(normalizeCurve(points));
}
function curvePointerDown(evt){
  const node = evt.target.closest('[data-curve-index]');
  if (!node) {
    selectNearestCurvePoint(curvePointFromEvent(evt));
    evt.preventDefault();
    return;
  }
  selectedCurveIndex = Number(node.dataset.curveIndex);
  draggingCurveIndex = selectedCurveIndex;
  $('curveSvg').setPointerCapture?.(evt.pointerId);
  renderCurve();
  evt.preventDefault();
}
function curvePointerMove(evt){
  if (draggingCurveIndex < 0) return;
  const points = readCurvePoints();
  if (!points) return;
  const idx = draggingCurveIndex;
  const p = curvePointFromEvent(evt);
  if (idx === 0) p.minute = 0;
  else if (idx === points.length - 1) p.minute = curveDuration();
  else p.minute = clamp(p.minute, points[idx - 1].minute + 5, points[idx + 1].minute - 5);
  points[idx] = p;
  selectedCurveIndex = idx;
  syncCurveTextarea(normalizeCurve(points));
  evt.preventDefault();
}
function curvePointerUp(evt){
  draggingCurveIndex = -1;
  $('curveSvg')?.releasePointerCapture?.(evt.pointerId);
}
function bindCurveEditor(){
  const svg = $('curveSvg');
  if (!svg) return;
  svg.addEventListener('pointerdown', curvePointerDown);
  svg.addEventListener('pointermove', curvePointerMove);
  svg.addEventListener('pointerup', curvePointerUp);
  svg.addEventListener('pointercancel', curvePointerUp);
  svg.addEventListener('dblclick', evt => {
    if (evt.target.closest('[data-curve-index]')) return;
    insertCurvePoint(curvePointFromEvent(evt));
  });
  $('curve').addEventListener('input', () => renderCurve());
  $('curve').addEventListener('change', () => scheduleCurveAutoSave());
  $('sleepDuration').addEventListener('input', () => {
    const points = readCurvePoints();
    if (points) syncCurveTextarea(points);
    else renderCurve();
  });
  $('sleepStart').addEventListener('input', () => { renderCurve(); scheduleCurveAutoSave(); });
  $('quietSwitchMinute').addEventListener('input', () => { renderCurve(); scheduleCurveAutoSave(); });
  ['autoEnabled','autoMode','curveControlMode','curveEndAction','curveHumidityEnabled','controlInterval','deadband','autoSendDelta'].forEach(id => {
    const el = $(id);
    if (el) el.addEventListener('input', () => scheduleCurveAutoSave());
  });
}
function msg(text, warn=false){
  $('msg').textContent = text || '';
  $('msg').classList.toggle('warn', warn);
  if (text && !warn) setTimeout(() => {
    if ($('msg').textContent === text) $('msg').textContent = '';
  }, 3500);
}
function setValue(id, value, force=false){
  const el = $(id);
  if (!el || (!force && dirty.has(id))) return;
  el.value = value ?? '';
  syncSegmentedControl(id);
}
function boolSelectValue(id){
  return $(id)?.value === 'true';
}
function specialModeFromFlags(item){
  if (item?.turbo || item?.remoteTurbo) return 'turbo';
  if (item?.sleep || item?.remoteSleep) return 'sleep';
  if (item?.quiet || item?.remoteQuiet) return 'quiet';
  return 'none';
}
function remoteCommandPayload(){
  const special = $('specialMode')?.value || 'none';
  return {
    power: $('power').value === 'true',
    mode: $('mode').value,
    degrees: Number($('degrees').value),
    fan: $('fan').value,
    turbo: special === 'turbo',
    quiet: special === 'quiet',
    sleep: special === 'sleep',
    swingV: boolSelectValue('swingV'),
    swingH: boolSelectValue('swingH'),
    filter: boolSelectValue('filterFlag')
  };
}
function clearDirty(ids){ ids.forEach(id => dirty.delete(id)); }
function bindDirty(){
  guardedIds.forEach(id => {
    const el = $(id);
    if (el) el.addEventListener('input', () => dirty.add(id));
  });
}
async function post(url, body, okText, clearIds=[]){
  msg('正在发送...');
  const r = await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  const t = await r.text();
  if (!r.ok) throw new Error(t || '操作失败');
  clearDirty(clearIds);
  msg(okText || '已完成');
  await refresh(true);
}
function scheduleRemoteStateSave(){
  clearTimeout(remoteStateSaveTimer);
  remoteStateSaveTimer = setTimeout(async () => {
    try {
      await fetch('/api/remote-state', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(remoteCommandPayload())});
    } catch(e) {}
  }, 500);
}
function bindRemoteStatePersistence(){
  ['power','mode','degrees','fan','specialMode','swingV','swingH','filterFlag'].forEach(id => {
    const el = $(id);
    if (el) el.addEventListener('input', () => scheduleRemoteStateSave());
  });
}
function wifiText(wifi){
  if (!wifi) return '--';
  if (wifi.connected) return wifi.ip || '已连接';
  if (wifi.apActive) return '热点 ' + (wifi.apIp || '192.168.4.1');
  return '连接中';
}
function captureText(capture){
  if (!capture || !capture.available) return '暂无红外捕获。';
  const lines = [
    '协议：' + (capture.protocol || '未知'),
    '位数：' + (capture.bits ?? '--'),
    '码值：' + (capture.value || '--'),
    'Raw 长度：' + (capture.rawLen ?? '--'),
    '空调解析：' + (capture.acDescription || '未识别为空调协议'),
    '摘要：' + (capture.summary || '--')
  ];
  return lines.join('\n');
}
function renderLearned(items){
  if (!items || !items.length) {
    $('learned').innerHTML = '<tr><td colspan="6" class="empty">暂无保存的红外命令</td></tr>';
    return;
  }
  $('learned').innerHTML = items.map(c => `
    <tr>
      <td>${esc(c.id)}</td>
      <td>${esc(c.name)}</td>
      <td>${esc(c.protocol)}</td>
      <td>${esc(c.meta || '')}</td>
      <td>${esc(c.rawLen)}</td>
      <td class="actions-cell"><div class="inline-actions"><button class="secondary" onclick="sendLearned(${Number(c.id)})">发送</button><button class="danger" onclick="deleteLearned(${Number(c.id)})">删除</button></div></td>
    </tr>`).join('');
}
function presetMeta(item){
  const power = item.power ? '开机' : '关机';
  const mode = modeText[item.mode] || item.mode || '--';
  const fan = fanText[item.fan] || item.fan || '--';
  const specials = enabledSpecialNames(item);
  return `${power} / ${mode} / ${Number(item.degrees).toFixed(1)} ℃ / ${fan}${specials.length ? ' / ' + specials.join('、') : ''}`;
}
function enabledSpecialNames(item){
  const names = [];
  const special = specialModeFromFlags(item);
  const specialLabel = {turbo:'强劲', quiet:'静音', sleep:'睡眠'}[special];
  if (specialLabel) names.push(specialLabel);
  if (item?.swingV) names.push('上下摆风');
  if (item?.swingH) names.push('左右摆风');
  if (item?.filter) names.push('滤网/出风口');
  return names;
}
function presetScheduleText(item){
  if (!item.scheduleEnabled) return '未启用定时';
  const modeText = {daily:'每天', workday:'工作日', holiday:'节假日', custom:'自定义'}[item.scheduleMode] || '每天';
  return `${modeText} ${minToHhmm(item.scheduleMinute || 0)} 定时发送`;
}
function presetDayButtons(item){
  const labels = ['日','一','二','三','四','五','六'];
  const mask = Number(item.dayMask ?? 62);
  return labels.map((label, day) => `<button type="button" class="${mask & (1 << day) ? 'selected' : ''}" data-preset-day="${day}" onclick="togglePresetDay(${Number(item.id)}, ${day})">${label}</button>`).join('');
}
function togglePresetScheduleEditor(id){
  id = Number(id);
  if (openPresetSchedules.has(id)) openPresetSchedules.delete(id);
  else openPresetSchedules.add(id);
  renderPresets(state.presets || []);
}
function renderPresets(items){
  const box = $('presetList');
  if (!box) return;
  if (!items || !items.length) {
    box.innerHTML = '<div class="empty">暂无快捷指令</div>';
    return;
  }
  box.innerHTML = items.map(item => {
    const id = Number(item.id);
    const scheduleOpen = openPresetSchedules.has(id);
    return `
    <div class="preset-card">
      <div>
        <div class="preset-title">${esc(item.name || ('指令 ' + item.id))}</div>
        <div class="preset-meta">${esc(presetMeta(item))}</div>
      </div>
      <div class="preset-schedule-summary">
        <div class="preset-meta">${esc(presetScheduleText(item))}</div>
        <button class="secondary" type="button" onclick="togglePresetScheduleEditor(${id})">${scheduleOpen ? '收起' : '定时设置'}</button>
      </div>
      ${scheduleOpen ? `<div class="preset-schedule" data-preset-schedule="${id}">
        <label class="preset-toggle"><input id="presetEnabled${id}" type="checkbox" ${item.scheduleEnabled ? 'checked' : ''} onchange="updatePresetSchedule(${id})">启用定时发送</label>
        <div class="preset-schedule-grid">
          <input id="presetTime${id}" type="time" value="${minToHhmm(item.scheduleMinute || 420)}" onchange="updatePresetSchedule(${id})">
          <select id="presetMode${id}" onchange="updatePresetSchedule(${id})">
            <option value="daily" ${item.scheduleMode === 'daily' ? 'selected' : ''}>每天</option>
            <option value="workday" ${item.scheduleMode === 'workday' ? 'selected' : ''}>工作日</option>
            <option value="holiday" ${item.scheduleMode === 'holiday' ? 'selected' : ''}>节假日</option>
            <option value="custom" ${item.scheduleMode === 'custom' ? 'selected' : ''}>自定义星期</option>
          </select>
        </div>
        <div class="preset-days">${presetDayButtons(item)}</div>
      </div>` : ''}
      <div class="preset-actions">
        <button type="button" onclick="sendPreset(${id})">发送</button>
        <button class="secondary" type="button" onclick="renamePreset(${id}, '${esc(item.name || '')}')">改名</button>
        <button class="danger" type="button" onclick="deletePreset(${id})">删除</button>
      </div>
    </div>`;
  }).join('');
}
function ensureSleepPresets(){
  if ($('sleepPresetPanel') || !$('curveControlMode')) return;
  $('curveControlMode').closest('.form-grid')?.insertAdjacentHTML('afterend', `
    <div class="sleep-presets" id="sleepPresetPanel">
      <button class="secondary" type="button" onclick="applySleepPreset('fast')">快速入睡</button>
      <button class="secondary" type="button" onclick="applySleepPreset('stable')">整夜平稳</button>
      <button class="secondary" type="button" onclick="applySleepPreset('warm')">怕冷防醒</button>
      <button class="secondary" type="button" onclick="applySleepPreset('quiet')">省电安静</button>
    </div>
  `);
}
function applySleepPreset(kind){
  const start = hhmmToMin($('sleepStart').value || '23:00');
  const presets = {
    fast: {mode:'staged', quiet:70, duration:510, interval:60, deadband:0.35, delta:0.5, points:[[0,27],[45,25],[210,25.5],[510,26.5]]},
    stable: {mode:'staged', quiet:100, duration:540, interval:120, deadband:0.4, delta:0.5, points:[[0,26.5],[120,25.5],[360,25.8],[540,26.5]]},
    warm: {mode:'staged', quiet:60, duration:540, interval:120, deadband:0.45, delta:0.5, points:[[0,27],[120,26.5],[360,26.8],[540,27.2]]},
    quiet: {mode:'quiet', quiet:0, duration:540, interval:180, deadband:0.5, delta:0.6, points:[[0,26.5],[180,26],[420,26.3],[540,26.8]]}
  };
  const p = presets[kind] || presets.stable;
  setValue('curveControlMode', p.mode, true);
  setValue('quietSwitchMinute', minToHhmm(start + p.quiet), true);
  setValue('sleepDuration', minToHhmm(start + p.duration), true);
  setValue('controlInterval', p.interval, true);
  setValue('deadband', p.deadband, true);
  setValue('autoSendDelta', p.delta, true);
  syncCurveTextarea(p.points.map(item => ({minute:item[0], temp:item[1]})));
  ['curveControlMode','quietSwitchMinute','sleepDuration','controlInterval','deadband','autoSendDelta','curve'].forEach(id => dirty.add(id));
  scheduleCurveAutoSave();
}
function formatLogMinute(minute){
  minute = Number(minute || 0);
  if (minute > 1000000) {
    const d = new Date(minute * 60000);
    return `${String(d.getMonth()+1).padStart(2,'0')}/${String(d.getDate()).padStart(2,'0')} ${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}`;
  }
  return `+${minute}分`;
}
function actionText(action){
  return {
    sent:'已发送',
    sent_raw:'已发送学习码',
    failed:'发送失败',
    queue_send:'已排队发送',
    queue_dehumidify:'已排队除湿',
    humidity_off:'湿控关闭关机',
    skip_sensor:'等待传感器',
    skip_humidity_sensor:'等待湿度',
    skip_dehumidify_cold:'暂停除湿',
    skip_humidity_ok:'湿度已稳定',
    skip_deadband:'死区内跳过',
    skip_predict:'趋势预测跳过',
    skip_delta:'发送过滤跳过',
    end_poweroff:'结束关机',
    end_hold:'结束保持'
  }[action] || action || '事件';
}
function stageText(stage){
  return {fast:'急速', quiet:'安静', curve:'曲线', humidity:'湿度', auto:'自动', manual:'手动', preset:'快捷', schedule:'定时', end:'结束'}[stage] || stage || '--';
}
function renderControlLog(logs=null){
  const box = $('controlLog');
  if (!box) return;
  if (Array.isArray(logs)) controlLogState.logs = logs;
  const allLogs = controlLogState.logs || [];
  const filtered = filteredControlLogs(allLogs);
  if (!allLogs.length) {
    box.innerHTML = '<div class="empty">暂无控制事件</div>';
    if ($('controlLogBadge')) $('controlLogBadge').textContent = '0 条';
    return;
  }
  if (!filtered.length) {
    box.innerHTML = '<div class="empty">当前筛选下暂无事件</div>';
    if ($('controlLogBadge')) $('controlLogBadge').textContent = `0 / ${allLogs.length} 条`;
    return;
  }
  box.innerHTML = filtered.slice().reverse().map(item => {
    const temps = [
      Number.isFinite(Number(item.room)) ? `室温 ${Number(item.room).toFixed(1)}℃` : '',
      Number.isFinite(Number(item.target)) ? `目标 ${Number(item.target).toFixed(1)}℃` : '',
      Number.isFinite(Number(item.setpoint)) ? `设定 ${Number(item.setpoint).toFixed(1)}℃` : ''
    ].filter(Boolean).join(' · ');
    const source = logSource(item);
    const type = logType(item);
    const head = `${logSourceText(source)} · ${logTypeText(type)} · ${actionText(item.action)}`;
    const stage = stageText(item.stage || item.source);
    return `<div class="log-item"><strong>${formatLogMinute(item.minute)}</strong><div>${esc(head)}<br><span class="label">${esc(stage)} · ${esc(temps || '--')}</span><br>${esc(item.note || '')}</div></div>`;
  }).join('');
  if ($('controlLogBadge')) $('controlLogBadge').textContent = `${filtered.length} / ${allLogs.length} 条`;
}
async function refreshControlLog(){
  if (controlLogRefreshInFlight || !$('controlLog')) return;
  controlLogRefreshInFlight = true;
  try {
    const r = await fetch('/api/control-log');
    if (!r.ok) throw new Error(await r.text());
    const data = await r.json();
    controlLogState.usesEpoch = !!data.usesEpoch;
    tempHistoryState.events = data.logs || [];
    tempHistoryState.eventsUseEpoch = !!data.usesEpoch;
    renderControlLog(data.logs || []);
    renderTempHistory();
  } catch(e) {
    if ($('controlLog')) $('controlLog').innerHTML = `<div class="empty">${esc(e.message || '日志读取失败')}</div>`;
  } finally {
    controlLogRefreshInFlight = false;
  }
}
function settingsPayload(){
  return {
    sensorTempOffset:Number($('sensorTempOffset').value || 0),
    sensorHumidityOffset:Number($('sensorHumidityOffset').value || 0),
    humidityControlEnabled:$('humidityControlEnabled').value === 'true',
    targetHumidity:Number($('targetHumidity').value || 58),
    humidityDeadband:Number($('humidityDeadband').value || 3),
    humidityTargetTemp:Number($('humidityTargetTemp').value || 26),
    predictiveSkipEnabled:$('predictiveSkipEnabled').value === 'true',
    adaptiveControlEnabled:$('adaptiveControlEnabled').value === 'true',
    closedLoopFastGain:Number($('closedLoopFastGain').value || 2.6),
    closedLoopQuietGain:Number($('closedLoopQuietGain').value || 0.9),
    capTurbo:$('capTurbo').value === 'true',
    capQuiet:$('capQuiet').value === 'true',
    capSleep:$('capSleep').value === 'true',
    capSwingV:$('capSwingV').value === 'true',
    capSwingH:$('capSwingH').value === 'true',
    capFilter:$('capFilter').value === 'true'
  };
}
async function saveSettings(okText='维护与闭环设置已保存'){
  try {
    await post('/api/settings', settingsPayload(), okText, ['sensorTempOffset','sensorHumidityOffset','humidityControlEnabled','targetHumidity','humidityDeadband','humidityTargetTemp','predictiveSkipEnabled','adaptiveControlEnabled','closedLoopFastGain','closedLoopQuietGain','capTurbo','capQuiet','capSleep','capSwingV','capSwingH','capFilter']);
  } catch(e) { msg(e.message || '设置保存失败', true); }
}
function exportConfig(){
  window.location.href = '/api/config-export';
}
async function importConfigFile(file){
  if (!file) return;
  try {
    const text = await file.text();
    const parsed = JSON.parse(text);
    await post('/api/config-import', parsed, '配置已导入');
  } catch(e) { msg(e.message || '配置导入失败', true); }
}
async function uploadOtaFile(file){
  if (!file) return;
  try {
    msg('正在上传 OTA，请不要断电...');
    const form = new FormData();
    form.append('firmware', file, file.name || 'firmware.bin');
    const r = await fetch('/api/ota', {method:'POST', body:form});
    const t = await r.text();
    if (!r.ok) throw new Error(t || 'OTA 上传失败');
    msg('OTA 已上传，设备正在重启');
  } catch(e) { msg(e.message || 'OTA 失败', true); }
}
function applyCapabilities(){
  const cfg = state.config || {};
  const caps = {
    turbo: cfg.capTurbo !== false,
    quiet: cfg.capQuiet !== false,
    sleep: cfg.capSleep !== false,
    swingV: cfg.capSwingV !== false,
    swingH: cfg.capSwingH !== false,
    filter: cfg.capFilter !== false
  };
  const special = $('specialMode');
  if (special) {
    [...special.options].forEach(opt => {
      if (opt.value === 'turbo') opt.disabled = !caps.turbo;
      if (opt.value === 'quiet') opt.disabled = !caps.quiet;
      if (opt.value === 'sleep') opt.disabled = !caps.sleep;
    });
    if (special.selectedOptions[0]?.disabled) special.value = 'none';
  }
  [['swingV', caps.swingV], ['swingH', caps.swingH], ['filterFlag', caps.filter]].forEach(([id, enabled]) => {
    const el = $(id);
    if (el) {
      el.disabled = !enabled;
      if (!enabled) el.value = 'false';
    }
  });
  enhanceSegmentedControls();
  ['specialMode','swingV','swingH','filterFlag'].forEach(syncSegmentedControl);
}
function fitStatusReadout(id){
  const el = $(id);
  const card = el?.closest('.metric');
  if (!el || !card || !el.textContent.trim()) return;
  const availableW = Math.max(80, card.clientWidth - 28);
  const label = card.querySelector('.label');
  const availableH = Math.max(52, card.clientHeight - (label?.offsetHeight || 0) - 36);
  const maxByHeight = availableH / 0.88;
  const maxSize = Math.max(60, Math.min(220, maxByHeight));
  const measurer = fitStatusReadout.measurer || (fitStatusReadout.measurer = document.createElement('span'));
  if (!measurer.parentNode) document.body.appendChild(measurer);
  const style = getComputedStyle(el);
  measurer.textContent = el.textContent;
  measurer.style.cssText = `
    position:absolute; left:-9999px; top:-9999px; visibility:hidden;
    white-space:nowrap; font-family:${style.fontFamily};
    font-weight:${style.fontWeight}; letter-spacing:${style.letterSpacing};
    line-height:${style.lineHeight}; padding:0; margin:0; border:0;
  `;
  el.style.whiteSpace = 'nowrap';
  let low = 28;
  let high = maxSize;
  for (let i = 0; i < 10; i++) {
    const mid = (low + high) / 2;
    measurer.style.fontSize = mid + 'px';
    if (measurer.offsetWidth <= availableW) low = mid;
    else high = mid;
  }
  el.style.fontSize = Math.floor(low) + 'px';
}
function fitStatusReadouts(){
  fitStatusReadout('temp');
  fitStatusReadout('hum');
}
function applyLive(data){
  if (!data) return;
  state = Object.assign(state || {}, data);
  $('temp').textContent = Number.isFinite(data.temperatureC) ? data.temperatureC.toFixed(1) + '℃' : '--';
  $('hum').textContent = Number.isFinite(data.humidity) ? data.humidity.toFixed(0) + '%' : '--';
  requestAnimationFrame(fitStatusReadouts);
  $('wifi').textContent = wifiText(data.wifi);
  $('last').textContent = data.lastAction || '--';
  $('clock').textContent = data.clock && data.clock.synced ? ('时间 ' + data.clock.local) : '时间未同步';
  if (data.autoTarget) {
    const mode = modeText[data.autoTarget.mode] || data.autoTarget.mode || '--';
    const room = Number.isFinite(data.temperatureC) ? `${Number(data.temperatureC).toFixed(1)} ℃` : '--';
    const humidityPart = data.autoTarget.humidityActive && Number.isFinite(Number(data.humidity))
      ? ` · 湿度 ${Number(data.humidity).toFixed(0)}% / 目标 ${Number(data.autoTarget.targetHumidity).toFixed(0)}%`
      : '';
    const elapsedPart = data.autoTarget.curveActive ? ` / +${data.autoTarget.elapsedMinute} 分` : ' / 湿度控制';
    $('curveTargetBadge').textContent = data.autoTarget.active
      ? `室温 ${room} · 目标 ${Number(data.autoTarget.temperatureC).toFixed(1)} ℃ · 设定 ${Number(data.autoTarget.setpointC).toFixed(1)} ℃ / ${mode}${humidityPart}${elapsedPart}`
      : '目标 -- / 曲线未生效';
  }
  $('captureBadge').textContent = data.capture && data.capture.available ? '已捕获' : '等待信号';
  $('capture').textContent = captureText(data.capture);
}
async function refreshLive(){
  if (liveRefreshInFlight || refreshInFlight) return;
  liveRefreshInFlight = true;
  try {
    const r = await fetch('/api/live');
    const live = await r.json();
    applyLive(live);
  } catch (e) {
    msg(e.message || '实时状态读取失败', true);
  } finally {
    liveRefreshInFlight = false;
  }
}
async function refresh(force=false){
  if (refreshInFlight) return;
  refreshInFlight = true;
  try {
    const r = await fetch('/api/status');
    state = await r.json();
    applyLive(state);
    $('protocolBadge').textContent = state.config && state.config.acProtocol ? ('当前 ' + state.config.acProtocol) : '协议未配置';
    $('curveBadge').textContent = state.config && state.config.autoEnabled ? '自动控制已开启' : '自动控制关闭';

    setValue('staSsid', state.config.staSsid || '', force);
    const protocols = state.supportedProtocols || [];
    if (force || !dirty.has('protocol')) {
      $('protocol').innerHTML = protocols.map(p => `<option value="${esc(p)}" ${p == state.config.acProtocol ? 'selected' : ''}>${esc(p)}</option>`).join('');
    }
    setValue('model', state.config.acModel, force);
    setValue('power', String(state.config.remotePower ?? true), force);
    setValue('mode', state.config.remoteMode || 'cool', force);
    setValue('degrees', state.config.remoteDegrees ?? 26, force);
    setValue('fan', state.config.remoteFan || 'auto', force);
    setValue('specialMode', specialModeFromFlags(state.config), force);
    setValue('swingV', String(!!state.config.remoteSwingV), force);
    setValue('swingH', String(!!state.config.remoteSwingH), force);
    setValue('filterFlag', String(!!state.config.remoteFilter), force);
    setValue('autoEnabled', String(!!state.config.autoEnabled), force);
    setValue('autoMode', state.config.autoMode || 'cool', force);
    setValue('curveControlMode', state.config.curveControlMode || 'staged', force);
    setValue('curveEndAction', state.config.curveEndAction || 'hold', force);
    setValue('curveHumidityEnabled', String(!!state.config.curveHumidityEnabled), force);
    setValue('sleepStart', minToHhmm(state.config.sleepStartMinute), force);
    setValue('quietSwitchMinute', minToHhmm(Number(state.config.sleepStartMinute || 0) + Number(state.config.quietSwitchMinute ?? 90)), force);
    setValue('sleepDuration', minToHhmm(Number(state.config.sleepStartMinute || 0) + Number(state.config.sleepDurationMinute || 480)), force);
    setValue('controlInterval', state.config.controlIntervalSec, force);
    setValue('deadband', state.config.deadband, force);
    setValue('autoSendDelta', state.config.autoSendDelta ?? 0.5, force);
    setValue('sensorTempOffset', state.config.sensorTempOffset ?? 0, force);
    setValue('sensorHumidityOffset', state.config.sensorHumidityOffset ?? 0, force);
    setValue('humidityControlEnabled', String(!!state.config.humidityControlEnabled), force);
    setValue('targetHumidity', state.config.targetHumidity ?? 58, force);
    setValue('humidityDeadband', state.config.humidityDeadband ?? 3, force);
    setValue('humidityTargetTemp', state.config.humidityTargetTemp ?? 26, force);
    setValue('predictiveSkipEnabled', String(state.config.predictiveSkipEnabled !== false), force);
    setValue('adaptiveControlEnabled', String(!!state.config.adaptiveControlEnabled), force);
    setValue('closedLoopFastGain', state.config.closedLoopFastGain ?? 2.6, force);
    setValue('closedLoopQuietGain', state.config.closedLoopQuietGain ?? 0.9, force);
    setValue('capTurbo', String(state.config.capTurbo !== false), force);
    setValue('capQuiet', String(state.config.capQuiet !== false), force);
    setValue('capSleep', String(state.config.capSleep !== false), force);
    setValue('capSwingV', String(state.config.capSwingV !== false), force);
    setValue('capSwingH', String(state.config.capSwingH !== false), force);
    setValue('capFilter', String(state.config.capFilter !== false), force);
    if ($('settingsBadge')) $('settingsBadge').textContent = `急速学习 ${Number(state.config.learnedFastRate || 0).toFixed(3)} · 安静学习 ${Number(state.config.learnedQuietRate || 0).toFixed(3)}`;
    applyCapabilities();
    setValue('curve', JSON.stringify(state.config.curve), force);
    renderCurve();
    renderLearned(state.learned);
    renderPresets(state.presets || []);
    if (!$('msg').textContent) msg('状态已更新');
  } catch (e) {
    msg(e.message || '状态读取失败', true);
  } finally {
    refreshInFlight = false;
  }
}
async function saveWifi(){
  try {
    await post('/api/wifi', {ssid:$('staSsid').value, password:$('staPassword').value}, 'WiFi 已保存，正在连接...', ['staSsid','staPassword']);
    $('staPassword').value = '';
  } catch(e) { msg(e.message || 'WiFi 保存失败', true); }
}
function wifiSignalText(rssi){
  rssi = Number(rssi);
  if (!Number.isFinite(rssi)) return '--';
  if (rssi >= -55) return '很强';
  if (rssi >= -67) return '较强';
  if (rssi >= -75) return '一般';
  return '较弱';
}
function renderWifiScan(networks){
  const box = $('wifiScanList');
  if (!box) return;
  if (!networks || !networks.length) {
    box.innerHTML = '<div class="empty">没有扫描到热点</div>';
    return;
  }
  box.innerHTML = networks.map((n, idx) => `
    <button class="wifi-network" type="button" data-wifi-index="${idx}">
      <div><strong>${esc(n.ssid || '隐藏网络')}</strong><span>${esc(n.auth || (n.open ? '开放' : '加密'))} · CH ${Number(n.channel || 0)} · ${Number(n.rssi || 0)} dBm</span></div>
      <div class="wifi-rssi">${wifiSignalText(n.rssi)}</div>
    </button>
  `).join('');
  box.querySelectorAll('[data-wifi-index]').forEach(btn => {
    btn.addEventListener('click', () => {
      const item = networks[Number(btn.dataset.wifiIndex)];
      if (!item || !item.ssid) return;
      $('staSsid').value = item.ssid;
      dirty.add('staSsid');
      $('staSsid').dispatchEvent(new Event('input', {bubbles:true}));
      $('staPassword')?.focus();
      msg(`已选择热点：${item.ssid}`);
    });
  });
}
async function scanWifiNetworks(){
  const btn = $('wifiScanButton');
  const box = $('wifiScanList');
  try {
    if (btn) btn.disabled = true;
    if (box) box.innerHTML = '<div class="empty">正在扫描附近热点...</div>';
    const r = await fetch('/api/wifi-scan');
    const text = await r.text();
    if (!r.ok) throw new Error(text || '扫描失败');
    const data = JSON.parse(text);
    renderWifiScan(data.networks || []);
    msg(`扫描完成：${data.count || 0} 个热点`);
  } catch(e) {
    if (box) box.innerHTML = `<div class="empty">${esc(e.message || '热点扫描失败')}</div>`;
    msg(e.message || '热点扫描失败', true);
  } finally {
    if (btn) btn.disabled = false;
  }
}
async function forgetWifi(){
  try {
    await post('/api/wifi/forget', {}, 'WiFi 已清除，已切换到热点模式', ['staSsid','staPassword']);
    $('staPassword').value = '';
  } catch(e) { msg(e.message || 'WiFi 清除失败', true); }
}
async function saveAcConfig(){
  try {
    await post('/api/ac-config', {protocol:$('protocol').value, model:Number($('model').value)}, '空调协议配置已保存', ['protocol','model']);
  } catch(e) { msg(e.message || '配置保存失败', true); }
}
async function sendAc(){
  try {
    await post('/api/send-ac', remoteCommandPayload(), '红外命令已发送');
  } catch(e) { msg(e.message || '发送失败', true); }
}
async function createPresetFromCurrent(){
  const current = remoteCommandPayload();
  const specials = enabledSpecialNames(current);
  const defaultName = `${current.power ? '开机' : '关机'} ${modeText[current.mode] || current.mode} ${Number(current.degrees).toFixed(1)}℃ ${fanText[current.fan] || current.fan}${specials.length ? ' ' + specials.join(' ') : ''}`;
  const name = prompt('快捷指令名称', defaultName);
  if (name === null) return;
  try {
    await post('/api/create-preset', Object.assign(current, {name:name.trim() || defaultName}), '快捷指令已保存');
  } catch(e) { msg(e.message || '快捷指令保存失败', true); }
}
async function sendPreset(id){
  try { await post('/api/send-preset', {id}, '快捷指令已发送'); }
  catch(e) { msg(e.message || '发送失败', true); }
}
async function renamePreset(id, oldName){
  const name = prompt('快捷指令名称', oldName || '');
  if (name === null) return;
  try { await post('/api/update-preset', {id, name:name.trim()}, '快捷指令已重命名'); }
  catch(e) { msg(e.message || '重命名失败', true); }
}
function presetDayMask(id){
  let mask = 0;
  document.querySelectorAll(`[data-preset-schedule="${id}"] [data-preset-day]`).forEach(btn => {
    if (btn.classList.contains('selected')) mask |= (1 << Number(btn.dataset.presetDay));
  });
  return mask || 0b0111110;
}
function togglePresetDay(id, day){
  const btn = document.querySelector(`[data-preset-schedule="${id}"] [data-preset-day="${day}"]`);
  if (!btn) return;
  btn.classList.toggle('selected');
  const mode = $('presetMode' + id);
  if (mode) mode.value = 'custom';
  updatePresetSchedule(id);
}
async function updatePresetSchedule(id){
  const enabled = $('presetEnabled' + id)?.checked || false;
  const time = hhmmToMin($('presetTime' + id)?.value || '07:00');
  const mode = $('presetMode' + id)?.value || 'daily';
  try {
    const r = await fetch('/api/update-preset', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({
      id,
      scheduleEnabled:enabled,
      scheduleMinute:time,
      scheduleMode:mode,
      dayMask:presetDayMask(id)
    })});
    const t = await r.text();
    if (!r.ok) throw new Error(t || '定时保存失败');
    msg('快捷指令定时已保存');
    await refresh(false);
  } catch(e) {
    msg(e.message || '定时保存失败', true);
  }
}
async function deletePreset(id){
  if (!confirm('删除这个快捷指令？')) return;
  try { await post('/api/delete-preset', {id}, '快捷指令已删除'); }
  catch(e) { msg(e.message || '删除失败', true); }
}
async function saveLearned(){
  try {
    await post('/api/learn', {name:$('learnName').value, freqKhz:Number($('learnFreq').value), power:$('learnPower').value==='true', mode:$('learnMode').value, degrees:Number($('learnDegrees').value), fan:$('learnFan').value}, '最近捕获已保存', ['learnName','learnFreq','learnPower','learnMode','learnDegrees','learnFan']);
  } catch(e) { msg(e.message || '保存失败', true); }
}
async function sendLearned(id){
  try { await post('/api/send-learned', {id}, '已发送学习库命令'); }
  catch(e) { msg(e.message || '发送失败', true); }
}
async function deleteLearned(id){
  try { await post('/api/delete-learned', {id}, '命令已删除'); }
  catch(e) { msg(e.message || '删除失败', true); }
}
function scheduleCurveAutoSave(){
  clearTimeout(curveAutoSaveTimer);
  curveAutoSaveTimer = setTimeout(() => saveCurve(true), 800);
}
async function saveCurve(auto=false){
  if (auto && curveAutoSaveInFlight) {
    curveAutoSavePending = true;
    return;
  }
  curveAutoSaveInFlight = true;
  try {
    const curve = readCurvePoints();
    if (!curve) throw new Error('曲线 JSON 无效');
    syncCurveTextarea(curve, false);
    if (!auto) msg('正在发送...');
    const r = await fetch('/api/curve', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({
      autoEnabled:$('autoEnabled').value==='true',
      autoMode:$('autoMode').value,
      curveControlMode:$('curveControlMode').value,
      curveEndAction:$('curveEndAction').value,
      curveHumidityEnabled:$('curveHumidityEnabled').value === 'true',
      quietSwitchMinute:quietSwitchFromTimes(),
      sleepStartMinute:hhmmToMin($('sleepStart').value),
      sleepDurationMinute:curveDurationFromTimes(),
      controlIntervalSec:Number($('controlInterval').value),
      deadband:Number($('deadband').value),
      autoSendDelta:Number($('autoSendDelta').value),
      curve
    })});
    const t = await r.text();
    if (!r.ok) throw new Error(t || '保存失败');
    clearDirty(['autoEnabled','autoMode','curveControlMode','curveEndAction','curveHumidityEnabled','quietSwitchMinute','sleepStart','sleepDuration','controlInterval','deadband','autoSendDelta','curve']);
    msg(auto ? '睡眠曲线已自动保存' : '睡眠曲线已保存');
  } catch(e) {
    msg(e.message || '曲线 JSON 无效', true);
  } finally {
    curveAutoSaveInFlight = false;
    if (curveAutoSavePending) {
      curveAutoSavePending = false;
      scheduleCurveAutoSave();
    }
  }
}
initSkin();
ensureCurveStrategyControls();
enhanceSegmentedControls();
ensureSleepPresets();
ensureTempHistorySection();
bindTempHistoryInteractions();
enableCollapsibleSections();
bindDirty();
bindRemoteStatePersistence();
bindCurveEditor();
$('configImportFile')?.addEventListener('change', evt => {
  const file = evt.target.files && evt.target.files[0];
  evt.target.value = '';
  importConfigFile(file);
});
$('otaFile')?.addEventListener('change', evt => {
  const file = evt.target.files && evt.target.files[0];
  evt.target.value = '';
  uploadOtaFile(file);
});
refresh(true);
refreshTempHistory();
refreshControlLog();
window.addEventListener('resize', () => requestAnimationFrame(fitStatusReadouts));
setInterval(refreshLive, 1000);
setInterval(() => refresh(false), 15000);
setInterval(refreshTempHistory, 60000);
setInterval(refreshControlLog, 15000);
</script>
</body>
</html>
)HTML";

const char kMatchHtml[] PROGMEM = R"MATCH(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>空调对码检查</title>
  <style>
    :root { color-scheme:light; --bg:#e8decc; --panel:#eee6d8; --panel-strong:#f7efe4; --ink:#1f1a14; --muted:#776d5c; --line:rgba(81,68,45,.18); --accent:#006a70; --accent-deep:#004e54; --ok:#0a6d5e; --warn:#9a651c; --control:rgba(118,105,82,.13); --shadow:8px 8px 18px rgba(111,91,58,.18), -8px -8px 18px rgba(255,255,255,.58); --shadow-sm:4px 4px 10px rgba(111,91,58,.16), -4px -4px 10px rgba(255,255,255,.56); --inset:inset 4px 4px 9px rgba(111,91,58,.18), inset -4px -4px 9px rgba(255,255,255,.58); --pressed:inset 3px 3px 8px rgba(31,24,12,.28), inset -3px -3px 7px rgba(255,255,255,.42); }
    * { box-sizing:border-box; }
    body { margin:0; background:var(--bg); color:var(--ink); font-family:system-ui,-apple-system,"Segoe UI",sans-serif; }
    body[data-skin="bluehome"] { --bg:#cfdaec; --panel:#f7faff; --panel-strong:#fff; --ink:#12213d; --muted:#7a89a3; --line:rgba(83,112,158,.13); --accent:#2f80ed; --accent-deep:#2667d8; --ok:#2f80ed; --warn:#7b61ff; --control:rgba(47,128,237,.08); --shadow:0 22px 46px rgba(78,101,142,.20), 0 2px 7px rgba(255,255,255,.80); --shadow-sm:0 12px 28px rgba(78,101,142,.16), 0 1px 5px rgba(255,255,255,.82); --inset:inset 0 1px 0 rgba(255,255,255,.92), inset 0 -10px 22px rgba(73,112,178,.05); --pressed:inset 0 3px 10px rgba(50,80,130,.20); background:linear-gradient(180deg,#d8e3f5 0%,#cfd9eb 48%,#c4d0e5 100%); }
    main { width:min(1180px,100%); margin:0 auto; padding:18px; display:grid; gap:18px; }
    header { display:flex; justify-content:space-between; gap:12px; align-items:flex-start; }
    h1 { margin:0; font-size:24px; }
    .muted { color:var(--muted); font-size:13px; line-height:1.5; }
    .badge { display:inline-flex; align-items:center; min-height:30px; padding:5px 10px; border-radius:999px; color:var(--accent); background:var(--panel); border:1px solid rgba(0,106,112,.1); box-shadow:var(--shadow-sm); font-weight:650; text-decoration:none; }
    section { background:var(--panel); border:1px solid var(--line); border-radius:18px; padding:16px; box-shadow:var(--shadow); }
    .toolbar { display:grid; grid-template-columns:repeat(auto-fit,minmax(160px,1fr)); gap:10px; align-items:end; }
    label { display:grid; gap:6px; color:var(--muted); font-size:13px; }
    select, button { min-height:42px; border-radius:12px; border:1px solid var(--line); background:var(--panel-strong); color:var(--ink); font:inherit; padding:8px 10px; box-shadow:var(--inset); }
    select option { background:#fff; color:#172033; }
    select option:checked { background:#bfd6cf; color:#172033; }
    button { cursor:pointer; border-color:rgba(0,78,84,.22); background:linear-gradient(180deg,#08777d,var(--accent-deep)); color:#fff; font-weight:650; box-shadow:var(--shadow-sm); }
    body[data-skin="bluehome"] button { border-color:rgba(47,128,237,.16); background:linear-gradient(180deg,#3d8bff,var(--accent-deep)); box-shadow:0 14px 26px rgba(47,128,237,.30); }
    button:active { box-shadow:var(--pressed); transform:translateY(1px); }
    button.secondary { background:var(--panel); color:var(--accent); }
    .stats { display:flex; flex-wrap:wrap; gap:8px; }
    .capture { white-space:pre-wrap; overflow-wrap:anywhere; color:var(--muted); font-family:ui-monospace,Consolas,monospace; font-size:12px; line-height:1.45; }
    .grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(210px,1fr)); gap:10px; }
    .card { border:1px solid var(--line); border-radius:16px; padding:10px; display:grid; gap:6px; min-height:112px; background:var(--panel); box-shadow:var(--shadow-sm); }
    body[data-skin="bluehome"] section,
    body[data-skin="bluehome"] .card,
    body[data-skin="bluehome"] .unknown-card,
    body[data-skin="bluehome"] .badge {
      border-color:rgba(47,128,237,.08);
      border-radius:22px;
      background:rgba(255,255,255,.78);
      box-shadow:var(--shadow-sm);
    }
    body[data-skin="bluehome"] main { max-width:1260px; }
    body[data-skin="bluehome"] .grid { grid-template-columns:repeat(auto-fill,minmax(220px,1fr)); }
    body[data-skin="bluehome"] .unknown-grid { grid-template-columns:repeat(auto-fill,minmax(280px,1fr)); }
    .card.matched { border-color:rgba(10,109,94,.65); background:rgba(0,106,112,.10); box-shadow:var(--pressed); }
    .unknown-head { display:flex; justify-content:space-between; gap:10px; align-items:center; margin-bottom:10px; }
    .unknown-head h2 { margin:0; font-size:18px; }
    .unknown-grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(260px,1fr)); gap:10px; }
    .unknown-card { border:1px solid rgba(154,101,28,.35); border-radius:16px; padding:10px; display:grid; gap:7px; background:#efe4d3; box-shadow:var(--shadow-sm); }
    .unknown-reason { color:var(--warn); font-weight:750; }
    .title { font-weight:750; }
    .code { color:var(--muted); font-family:ui-monospace,Consolas,monospace; font-size:12px; overflow-wrap:anywhere; }
    .ok { color:var(--ok); font-weight:700; }
    .warn { color:var(--warn); }
    @media (max-width:760px){ header{display:grid} main{padding:10px} }
  </style>
</head>
<body>
<script>try{document.body.dataset.skin=localStorage.getItem('ir-ac-skin')==='bluehome'?'bluehome':'cream'}catch(e){}</script>
<main>
  <header>
    <div>
      <h1>空调对码检查</h1>
      <div class="muted">按当前协议/型号生成待测状态。用实体遥控器按键，捕获结果匹配后会标绿并保存。</div>
    </div>
    <a class="badge" href="/">返回主页面</a>
  </header>

  <section>
    <div class="toolbar">
      <label>模式筛选<select id="modeFilter"></select></label>
      <label>风量筛选<select id="fanFilter"></select></label>
      <label>温度范围<select id="tempFilter"></select></label>
      <button type="button" onclick="clearMatches()">清空匹配标记</button>
    </div>
  </section>

  <section>
    <div class="stats">
      <span class="badge" id="protocolBadge">协议 --</span>
      <span class="badge" id="countBadge">0 / 0</span>
      <span class="badge" id="lastMatchBadge">等待捕获</span>
    </div>
    <div class="capture" id="captureBox">正在读取状态...</div>
  </section>

  <section>
    <div class="grid" id="matchGrid"></div>
  </section>

  <section>
    <div class="unknown-head">
      <div>
        <h2>未知组合</h2>
        <div class="muted">这里记录矩阵没有覆盖、无法解析、或同一基础组合下编码不同的实体遥控信号。</div>
      </div>
      <button class="secondary" type="button" onclick="clearUnknowns()">清空未知</button>
    </div>
    <div class="unknown-grid" id="unknownGrid"></div>
  </section>
</main>

<script>
const $ = id => document.getElementById(id);
const modes = [
  ['cool','制冷'], ['auto','自动'], ['dry','除湿'], ['heat','制热'], ['fan','送风']
];
const fans = [
  ['auto','自动'], ['low','低风'], ['medium','中风'], ['high','高风'], ['max','最大']
];
const modeFromText = {cool:'cool', auto:'auto', dry:'dry', heat:'heat', fan:'fan'};
const features = [
  ['base','基础'], ['turbo','强劲'], ['sleep','睡眠'], ['quiet','静音'],
  ['swingV','上下摆风'], ['swingH','左右摆风'], ['filter','滤网/出风口']
];
const maxFanIndex = fans.findIndex(item => item[0] === 'max');
if (maxFanIndex >= 0) fans.splice(maxFanIndex, 1);
const fanFromText = {auto:'auto', low:'low', medium:'medium', high:'high', min:'min', quiet:'quiet'};
let state = {};
let entries = [];
let matches = {};
let unknowns = [];
let lastCaptureKey = '';
function clamp(v,min,max){ return Math.min(max, Math.max(min, v)); }
function esc(v){ return String(v ?? '').replace(/[&<>"']/g, ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch])); }
function storageKey(){
  const cfg = state.config || {};
  return `ir-ac-match-v1:${cfg.acProtocol || 'UNKNOWN'}:${cfg.acModel || 1}`;
}
function unknownStorageKey(){
  const cfg = state.config || {};
  return `ir-ac-unknown-v1:${cfg.acProtocol || 'UNKNOWN'}:${cfg.acModel || 1}`;
}
function loadMatches(){
  try { matches = JSON.parse(localStorage.getItem(storageKey()) || '{}') || {}; }
  catch(e){ matches = {}; }
}
function saveMatches(){ try { localStorage.setItem(storageKey(), JSON.stringify(matches)); } catch(e){} }
function loadUnknowns(){
  try { unknowns = JSON.parse(localStorage.getItem(unknownStorageKey()) || '[]') || []; }
  catch(e){ unknowns = []; }
}
function saveUnknowns(){ try { localStorage.setItem(unknownStorageKey(), JSON.stringify(unknowns.slice(0,80))); } catch(e){} }
function baseEntryKey(mode,temp,fan){ return `${mode}|${Number(temp).toFixed(1)}|${fan}`; }
function entryKey(mode,temp,fan,feature='base'){ return `${baseEntryKey(mode,temp,fan)}|${feature}`; }
function entryKeyText(key){
  const [mode,temp,fan,feature='base'] = String(key || '').split('|');
  const modeText = modes.find(m => m[0] === mode)?.[1] || mode;
  const fanText = fans.find(f => f[0] === fan)?.[1] || fan;
  const featureText = features.find(f => f[0] === feature)?.[1] || feature;
  return `${modeText} / ${Number(temp).toFixed(1)}℃ / ${fanText} / ${featureText}`;
}
function buildEntries(){
  const cfg = state.config || {};
  const min = Math.round(Number(cfg.minSetpoint ?? 16));
  const max = Math.round(Number(cfg.maxSetpoint ?? 32));
  entries = [];
  for (const [mode] of modes) {
    for (let temp = min; temp <= max; temp++) {
      for (const [fan] of fans) {
        for (const [feature] of features) {
          entries.push({mode,temp,fan,feature,key:entryKey(mode,temp,fan,feature)});
        }
      }
    }
  }
}
function setupFilters(){
  if (!$('featureFilter')) {
    $('tempFilter').closest('label').insertAdjacentHTML('afterend', '<label>功能筛选<select id="featureFilter"></select></label>');
  }
  $('modeFilter').innerHTML = '<option value="">全部模式</option>' + modes.map(([v,t]) => `<option value="${v}">${t}</option>`).join('');
  $('fanFilter').innerHTML = '<option value="">全部风量</option>' + fans.map(([v,t]) => `<option value="${v}">${t}</option>`).join('');
  $('tempFilter').innerHTML = '<option value="">全部温度</option><option value="16-20">16-20℃</option><option value="21-25">21-25℃</option><option value="26-32">26-32℃</option>';
  $('featureFilter').innerHTML = '<option value="">全部功能</option>' + features.map(([v,t]) => `<option value="${v}">${t}</option>`).join('');
  ['modeFilter','fanFilter','tempFilter','featureFilter'].forEach(id => $(id).addEventListener('change', renderGrid));
}
function passFilter(item){
  if ($('modeFilter').value && item.mode !== $('modeFilter').value) return false;
  if ($('fanFilter').value && item.fan !== $('fanFilter').value) return false;
  if ($('featureFilter') && $('featureFilter').value && item.feature !== $('featureFilter').value) return false;
  const tf = $('tempFilter').value;
  if (tf) {
    const [lo,hi] = tf.split('-').map(Number);
    if (item.temp < lo || item.temp > hi) return false;
  }
  return true;
}
function renderGrid(){
  const shown = entries.filter(passFilter);
  $('matchGrid').innerHTML = shown.map(item => {
    const legacyKey = baseEntryKey(item.mode, item.temp, item.fan);
    const hit = matches[item.key] || (item.feature === 'base' ? matches[legacyKey] : null);
    const modeText = modes.find(m => m[0] === item.mode)?.[1] || item.mode;
    const fanText = fans.find(f => f[0] === item.fan)?.[1] || item.fan;
    const featureText = features.find(f => f[0] === item.feature)?.[1] || item.feature;
    return `<div class="card ${hit ? 'matched' : ''}" id="k-${item.key.replaceAll('|','-')}">
      <div class="code">功能：${esc(featureText)}</div>
      <div class="title">${modeText} · ${item.temp.toFixed(1)}℃ · ${fanText}</div>
      <div class="${hit ? 'ok' : 'warn'}">${hit ? '已匹配' : '未匹配'}</div>
      <div class="code">${hit ? `编码 ${esc(hit.value)} / ${esc(hit.bits)} bits` : '等待实体遥控器捕获'}</div>
      <div class="code">${hit ? esc(hit.acDescription || '') : esc(item.key)}</div>
    </div>`;
  }).join('');
  const done = entries.filter(e => matches[e.key] || (e.feature === 'base' && matches[baseEntryKey(e.mode, e.temp, e.fan)])).length;
  $('countBadge').textContent = `${done} / ${entries.length}`;
}
function renderUnknowns(){
  const box = $('unknownGrid');
  if (!unknowns.length) {
    box.innerHTML = '<div class="muted">还没有未知信号。按实体遥控器上未覆盖的按键后会自动出现在这里。</div>';
    return;
  }
  box.innerHTML = unknowns.map(item => {
    const parsed = item.parsed || {};
    const parsedText = [parsed.mode, Number.isFinite(parsed.temp) ? `${parsed.temp.toFixed(1)}℃` : '', parsed.fan].filter(Boolean).join(' · ');
    return `<div class="unknown-card">
      <div class="unknown-reason">${esc(item.reason || '未知组合')}</div>
      <div class="code">${esc(item.protocol || '--')} / ${esc(item.value || '--')} / ${esc(item.bits || '--')} bits / Raw ${esc(item.rawLen || 0)}</div>
      <div class="code">${parsedText ? `解析：${esc(parsedText)}` : '解析：--'}</div>
      <div class="code">次数 ${esc(item.count || 1)} · ${esc(new Date(item.lastSeen || item.at || Date.now()).toLocaleString())}</div>
      <div class="code">${esc(item.acDescription || item.summary || '')}</div>
    </div>`;
  }).join('');
}
function captureId(capture){ return `${capture.protocol || '--'}|${capture.value || '--'}|${capture.bits || '--'}`; }
function recordUnknown(capture, reason, parsed){
  if (!capture || !capture.available) return;
  const id = captureId(capture);
  const existing = unknowns.find(item => item.id === id);
  if (existing) {
    existing.count = (existing.count || 1) + 1;
    existing.lastSeen = Date.now();
    existing.reason = reason || existing.reason;
    existing.parsed = parsed || existing.parsed;
    existing.acDescription = capture.acDescription || existing.acDescription;
    existing.summary = capture.summary || existing.summary;
  } else {
    unknowns.unshift({
      id,
      reason,
      parsed,
      protocol:capture.protocol,
      value:capture.value,
      bits:capture.bits,
      rawLen:capture.rawLen,
      acDescription:capture.acDescription,
      summary:capture.summary,
      count:1,
      at:Date.now(),
      lastSeen:Date.now()
    });
    unknowns = unknowns.slice(0,80);
  }
  saveUnknowns();
  renderUnknowns();
}
function normWord(text){
  text = String(text || '').toLowerCase();
  if (text.includes('cool')) return 'cool';
  if (text.includes('heat')) return 'heat';
  if (text.includes('dry')) return 'dry';
  if (text.includes('fan')) return 'fan';
  if (text.includes('auto')) return 'auto';
  if (text.includes('low')) return 'low';
  if (text.includes('medium') || text.includes('med')) return 'medium';
  if (text.includes('high')) return 'high';
  if (text.includes('max')) return 'max';
  if (text.includes('min')) return 'min';
  return '';
}
function parseAc(desc){
  const mode = normWord((desc.match(/Mode:\s*[^,(]+(?:\(([^)]+)\))?/i) || [])[1] || (desc.match(/Mode:\s*([^,]+)/i) || [])[1]);
  const fan = normWord((desc.match(/Fan:\s*[^,(]+(?:\(([^)]+)\))?/i) || [])[1] || (desc.match(/Fan:\s*([^,]+)/i) || [])[1]);
  const tempMatch = desc.match(/Temp:\s*([0-9]+(?:\.[0-9]+)?)/i);
  const temp = tempMatch ? Math.round(Number(tempMatch[1]) * 2) / 2 : NaN;
  const flag = name => new RegExp(name.replace(/[()]/g, '\\$&') + ':\\s*On', 'i').test(desc);
  return {
    mode,temp,fan,
    features:{
      turbo:flag('Turbo'),
      sleep:flag('Sleep'),
      quiet:flag('Quiet'),
      swingV:flag('Swing(V)'),
      swingH:flag('Swing(H)'),
      filter:flag('Filter')
    }
  };
}
function featureKeys(parsed){
  const result = [];
  const flags = parsed.features || {};
  for (const [key] of features) {
    if (key !== 'base' && flags[key]) result.push(key);
  }
  return result.length ? result : ['base'];
}
function handleCapture(capture){
  if (!capture || !capture.available) return;
  const capKey = `${capture.protocol}|${capture.value}|${capture.bits}|${capture.ageMs}`;
  $('captureBox').textContent = [
    `最近捕获：${capture.protocol || '--'} / ${capture.value || '--'} / Raw ${capture.rawLen || 0}`,
    capture.acDescription || capture.summary || ''
  ].join('\n');
  if (capKey === lastCaptureKey) return;
  lastCaptureKey = capKey;
  const cfg = state.config || {};
  if (capture.protocol !== cfg.acProtocol) {
    recordUnknown(capture, `协议不同：${capture.protocol || '--'}`, {});
    return;
  }
  const parsed = parseAc(capture.acDescription || '');
  if (!parsed.mode || !Number.isFinite(parsed.temp)) {
    recordUnknown(capture, '无法解析模式或温度', parsed);
    return;
  }
  const fan = parsed.fan && fanFromText[parsed.fan] ? parsed.fan : 'auto';
  const matchedKeys = [];
  for (const feature of featureKeys(parsed)) {
    const featureKey = entryKey(parsed.mode, parsed.temp, fan, feature);
    const fallbackFeatureKey = entryKey(parsed.mode, parsed.temp, 'auto', feature);
    const finalFeatureKey = entries.some(e => e.key === featureKey) ? featureKey : fallbackFeatureKey;
    if (entries.some(e => e.key === finalFeatureKey)) matchedKeys.push(finalFeatureKey);
  }
  if (matchedKeys.length) {
    for (const finalFeatureKey of matchedKeys) {
      matches[finalFeatureKey] = {
        value:capture.value,
        bits:capture.bits,
        rawLen:capture.rawLen,
        acDescription:capture.acDescription,
        at:Date.now()
      };
    }
    saveMatches();
    $('lastMatchBadge').textContent = `已匹配 ${matchedKeys.map(entryKeyText).join('，')}`;
    renderGrid();
    return;
  }
  const key = entryKey(parsed.mode, parsed.temp, fan);
  const fallbackKey = entryKey(parsed.mode, parsed.temp, 'auto');
  const finalKey = entries.some(e => e.key === key) ? key : fallbackKey;
  if (!entries.some(e => e.key === finalKey)) {
    recordUnknown(capture, `矩阵没有这个组合：${key}`, parsed);
    return;
  }
  if (matches[finalKey] && matches[finalKey].value !== capture.value) {
    recordUnknown(capture, `同一基础组合的新编码：${finalKey.replaceAll('|',' · ')}`, parsed);
  }
  matches[finalKey] = {
    value:capture.value,
    bits:capture.bits,
    rawLen:capture.rawLen,
    acDescription:capture.acDescription,
    at:Date.now()
  };
  saveMatches();
  $('lastMatchBadge').textContent = `匹配 ${finalKey.replaceAll('|',' · ')}`;
  renderGrid();
}
async function refresh(){
  try {
    const r = await fetch('/api/status');
    state = await r.json();
    $('protocolBadge').textContent = `协议 ${state.config.acProtocol} / 型号 ${state.config.acModel}`;
    if (!entries.length) {
      loadMatches();
      loadUnknowns();
      buildEntries();
      renderGrid();
      renderUnknowns();
    }
    handleCapture(state.capture);
  } catch(e) {
    $('captureBox').textContent = '状态读取失败：' + e.message;
  }
}
function clearMatches(){
  if (!confirm('清空当前协议/型号的所有匹配标记？')) return;
  matches = {};
  saveMatches();
  renderGrid();
}
function clearUnknowns(){
  if (!confirm('清空当前协议/型号记录的未知信号？')) return;
  unknowns = [];
  saveUnknowns();
  renderUnknowns();
}
setupFilters();
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
)MATCH";

const char kRemoteHtml[] PROGMEM = R"REMOTE(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <title>手机遥控器</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #e8decc;
      --remote: #eee6d8;
      --remote-edge: #e2d5bf;
      --panel: #f7efe4;
      --ink: #1f1a14;
      --muted: #776d5c;
      --button: #f7efe4;
      --button-ink: #1f1a14;
      --accent: #006a70;
      --accent-deep: #004e54;
      --accent-ink: #ffffff;
      --danger: #a93f32;
      --ok: #0a6d5e;
      --line: rgba(81,68,45,0.18);
      --shadow: 8px 8px 18px rgba(111,91,58,0.18), -8px -8px 18px rgba(255,255,255,0.58);
      --shadow-sm: 4px 4px 10px rgba(111,91,58,0.16), -4px -4px 10px rgba(255,255,255,0.56);
      --inset: inset 4px 4px 9px rgba(111,91,58,0.18), inset -4px -4px 9px rgba(255,255,255,0.58);
      --pressed: inset 3px 3px 8px rgba(31,24,12,0.28), inset -3px -3px 7px rgba(255,255,255,0.42);
    }
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
    html { width: 100%; overflow-x: hidden; }
    body {
      width: 100%;
      margin: 0;
      min-height: 100vh;
      overflow-x: hidden;
      background: var(--bg);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: var(--ink);
    }
    body[data-skin="bluehome"] {
      --bg: #cfdaec;
      --remote: #f7faff;
      --remote-edge: #edf4ff;
      --panel: #ffffff;
      --ink: #12213d;
      --muted: #7a89a3;
      --button: #ffffff;
      --button-ink: #12213d;
      --accent: #2f80ed;
      --accent-deep: #2667d8;
      --accent-ink: #ffffff;
      --danger: #ec5f67;
      --ok: #2f80ed;
      --line: rgba(83,112,158,0.13);
      --shadow: 0 24px 48px rgba(78,101,142,0.22), 0 2px 7px rgba(255,255,255,0.82);
      --shadow-sm: 0 13px 28px rgba(78,101,142,0.16), 0 1px 5px rgba(255,255,255,0.84);
      --inset: inset 0 1px 0 rgba(255,255,255,0.92), inset 0 -10px 22px rgba(73,112,178,0.05);
      --pressed: inset 0 3px 10px rgba(50,80,130,0.22);
      background: linear-gradient(180deg, #d8e3f5 0%, #cfd9eb 48%, #c4d0e5 100%);
    }
    .wrap {
      width: 100%;
      max-width: 430px;
      margin: 0 auto;
      padding: 14px 14px 104px;
      overflow: hidden;
    }
    .topbar {
      min-width: 0;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      margin-bottom: 10px;
      color: var(--muted);
      font-size: 13px;
    }
    .topbar a {
      color: var(--accent);
      text-decoration: none;
      font-weight: 700;
    }
    .topbar span { min-width: 0; overflow-wrap: anywhere; text-align: right; }
    .remote {
      width: 100%;
      max-width: 100%;
      min-width: 0;
      overflow: hidden;
      border-radius: 30px;
      border: 1px solid var(--line);
      background: linear-gradient(180deg, var(--remote), var(--remote-edge));
      box-shadow: var(--shadow);
      padding: 18px;
    }
    body[data-skin="bluehome"] .remote {
      border: 0;
      background: linear-gradient(180deg, rgba(255,255,255,.88), rgba(239,246,255,.92));
      backdrop-filter: blur(12px);
    }
    .screen {
      min-width: 0;
      min-height: 138px;
      border-radius: 22px;
      background: var(--panel);
      color: var(--ink);
      padding: 16px;
      display: grid;
      gap: 10px;
      box-shadow: var(--inset);
    }
    .screen-top, .screen-bottom {
      min-width: 0;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
    }
    .screen-top > div, .screen-bottom > div { min-width: 0; overflow-wrap: anywhere; }
    .screen-label { color: var(--muted); font-size: 12px; font-weight: 650; }
    .target { font-size: 52px; line-height: 1; font-weight: 800; letter-spacing: 0; }
    .status-pill {
      min-height: 28px;
      border-radius: 999px;
      padding: 5px 10px;
      background: rgba(0, 106, 112, 0.10);
      color: var(--accent);
      font-size: 13px;
      font-weight: 700;
      white-space: nowrap;
    }
    .remote-grid { min-width: 0; display: grid; gap: 12px; margin-top: 16px; }
    .power-row {
      min-width: 0;
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      align-items: center;
    }
    button {
      min-width: 0;
      max-width: 100%;
      min-height: 48px;
      border: 0;
      border-radius: 18px;
      background: var(--button);
      color: var(--button-ink);
      font: inherit;
      font-weight: 760;
      cursor: pointer;
      white-space: normal;
      overflow-wrap: anywhere;
      box-shadow: var(--shadow-sm);
    }
    button:active { transform: translateY(1px); box-shadow: var(--pressed); }
    .power {
      min-height: 58px;
      border-radius: 999px;
      background: linear-gradient(180deg, #bd5a4d, var(--danger));
      color: #fff;
    }
    .send {
      min-height: 58px;
      border-radius: 999px;
      background: linear-gradient(180deg, #08777d, var(--accent-deep));
      color: #fff;
    }
    body[data-skin="bluehome"] .send,
    body[data-skin="bluehome"] .button-grid button.selected,
    body[data-skin="bluehome"] .edit-toggle.active {
      background: linear-gradient(180deg, #3d8bff, var(--accent-deep));
      color: #fff;
      box-shadow: 0 14px 26px rgba(47,128,237,.28);
    }
    .temp-pad {
      min-width: 0;
      display: grid;
      grid-template-columns: 1fr 96px 1fr;
      gap: 12px;
      align-items: center;
    }
    .temp-pad button {
      min-height: 76px;
      border-radius: 22px;
      font-size: 32px;
    }
    .temp-center {
      min-height: 76px;
      border-radius: 22px;
      border: 1px solid var(--line);
      display: grid;
      place-items: center;
      color: var(--muted);
      font-weight: 760;
      background: var(--remote);
      box-shadow: var(--inset);
    }
    .button-grid {
      min-width: 0;
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }
    .button-grid button.selected {
      background: linear-gradient(180deg, #08777d, var(--accent-deep));
      color: var(--accent-ink);
      box-shadow: var(--pressed);
    }
    .learned {
      margin-top: 16px;
      border-top: 1px solid var(--line);
      padding-top: 14px;
      display: grid;
      gap: 10px;
    }
    .learned-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
    }
    .learned h2 {
      margin: 0;
      font-size: 15px;
      color: var(--ink);
    }
    .edit-toggle {
      min-height: 38px;
      width: auto;
      padding: 8px 12px;
      border-radius: 999px;
      background: var(--panel);
      color: var(--accent);
      box-shadow: var(--shadow-sm);
    }
    .edit-toggle.active {
      background: linear-gradient(180deg, #08777d, var(--accent-deep));
      color: var(--accent-ink);
    }
    .learned-list {
      min-width: 0;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(128px, 1fr));
      gap: 10px;
    }
    .learned-list button {
      min-height: 54px;
      text-align: left;
      padding: 10px 12px;
      overflow-wrap: anywhere;
    }
    .learn-capture {
      display: grid;
      gap: 10px;
      padding: 12px;
      border-radius: 16px;
      border: 1px solid var(--line);
      background: var(--panel);
      box-shadow: var(--inset);
    }
    .learn-capture pre {
      margin: 0;
      max-height: 86px;
      overflow: auto;
      white-space: pre-wrap;
      word-break: break-word;
      color: var(--muted);
      font: 12px/1.45 ui-monospace, SFMono-Regular, Consolas, monospace;
    }
    .learn-tools {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .learned-card {
      min-width: 0;
      display: grid;
      gap: 8px;
      padding: 10px;
      border-radius: 16px;
      background: var(--panel);
      border: 1px solid var(--line);
      box-shadow: var(--shadow-sm);
    }
    .learned-list.editing .learned-card {
      border-color: rgba(0, 106, 112, 0.62);
    }
    .learned-card.dragging {
      opacity: 0.72;
      transform: scale(0.99);
    }
    .learned-send {
      min-height: 64px;
      text-align: left;
      padding: 12px;
      border-radius: 14px;
    }
    .learned-title {
      min-height: 36px;
      display: grid;
      align-content: center;
      color: var(--ink);
      font-weight: 780;
      overflow-wrap: anywhere;
    }
    .learned-meta {
      min-height: 18px;
      color: var(--muted);
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    .drag-handle {
      min-height: 38px;
      display: grid;
      place-items: center;
      border: 1px dashed rgba(81,68,45,0.28);
      border-radius: 12px;
      color: var(--muted);
      font-size: 13px;
      touch-action: none;
      cursor: grab;
    }
    .learned-actions {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .learned-actions button {
      min-height: 42px;
      border-radius: 12px;
      padding: 8px;
      font-size: 13px;
    }
    .learned-actions button.danger {
      background: linear-gradient(180deg, #bd5a4d, var(--danger));
      color: #fff;
    }
    .learned-actions button.replace {
      background: #ead2a9;
      color: var(--ink);
    }
    .msg {
      min-height: 22px;
      color: var(--accent);
      font-size: 13px;
      text-align: center;
    }
    .quickbar {
      position: fixed;
      left: 50%;
      bottom: max(10px, env(safe-area-inset-bottom));
      transform: translateX(-50%);
      z-index: 10;
      width: min(410px, calc(100% - 20px));
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      padding: 8px;
      border: 1px solid var(--line);
      border-radius: 22px;
      background: rgba(238, 230, 216, 0.94);
      box-shadow: var(--shadow);
      backdrop-filter: blur(12px);
    }
    body[data-skin="bluehome"] .screen,
    body[data-skin="bluehome"] .temp-center,
    body[data-skin="bluehome"] .learn-capture,
    body[data-skin="bluehome"] .learned-card,
    body[data-skin="bluehome"] .quickbar {
      border-color: rgba(47,128,237,.08);
      background: rgba(255,255,255,.76);
      box-shadow: var(--shadow-sm);
    }
    .quickbar button {
      min-height: 48px;
      border-radius: 16px;
      font-size: 13px;
      padding: 7px 6px;
    }
    @media (max-width: 380px) {
      .wrap { padding: 10px 10px 92px; }
      .remote { padding: 14px; border-radius: 28px; }
      .target { font-size: 46px; }
      .temp-pad { grid-template-columns: 1fr 74px 1fr; gap: 8px; }
      .button-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
      .learned-list { grid-template-columns: 1fr; }
      .learn-tools { grid-template-columns: 1fr; }
    }
    @media (max-width: 330px) {
      .wrap { padding: 8px 8px 92px; }
      .remote { padding: 12px; border-radius: 24px; }
      .target { font-size: 40px; }
      .temp-pad { grid-template-columns: 1fr 64px 1fr; }
    }
  </style>
</head>
<body>
  <script>try{document.body.dataset.skin=localStorage.getItem('ir-ac-skin')==='bluehome'?'bluehome':'cream'}catch(e){}</script>
  <div class="wrap">
    <div class="topbar">
      <a href="/">返回配置页</a>
      <span id="wifi">连接中</span>
    </div>
    <div class="remote">
      <div class="screen">
        <div class="screen-top">
          <div>
            <div class="screen-label">目标温度</div>
            <div class="target"><span id="targetTemp">26.0</span><span style="font-size:24px">℃</span></div>
          </div>
          <div class="status-pill" id="powerState">开机</div>
        </div>
        <div class="screen-bottom">
          <div>
            <div class="screen-label">室内</div>
            <div id="roomState">-- ℃ / --%</div>
          </div>
          <div>
            <div class="screen-label">模式 / 风量</div>
            <div id="modeState">制冷 / 自动</div>
          </div>
        </div>
      </div>

      <div class="remote-grid">
        <div class="power-row">
          <button class="power" type="button" onclick="togglePower()" id="powerButton">关机</button>
          <button class="send" type="button" onclick="sendAc()">发送当前</button>
        </div>
        <div class="temp-pad">
          <button type="button" onclick="adjustTemp(-0.5)">-</button>
          <div class="temp-center">温度</div>
          <button type="button" onclick="adjustTemp(0.5)">+</button>
        </div>
        <div class="button-grid" id="modeButtons">
          <button type="button" data-mode="cool" onclick="setMode('cool')">制冷</button>
          <button type="button" data-mode="auto" onclick="setMode('auto')">自动</button>
          <button type="button" data-mode="dry" onclick="setMode('dry')">除湿</button>
          <button type="button" data-mode="heat" onclick="setMode('heat')">制热</button>
          <button type="button" data-mode="fan" onclick="setMode('fan')">送风</button>
        </div>
        <div class="button-grid" id="fanButtons">
          <button type="button" data-fan="auto" onclick="setFan('auto')">自动风</button>
          <button type="button" data-fan="low" onclick="setFan('low')">低风</button>
          <button type="button" data-fan="medium" onclick="setFan('medium')">中风</button>
          <button type="button" data-fan="high" onclick="setFan('high')">高风</button>
          <button type="button" data-fan="max" onclick="setFan('max')">最大</button>
        </div>
      </div>

      <div class="learned">
        <div class="learned-head">
          <h2>自定义按键</h2>
          <button class="edit-toggle" id="keyEditButton" type="button" onclick="toggleKeyEditMode()">编辑</button>
        </div>
        <div class="learned-list" id="learnedList"></div>
        <div class="learn-capture">
          <pre id="captureState">暂无红外捕获。对准接收头按实体遥控器按键后，可保存为自定义按键。</pre>
          <div class="learn-tools">
            <button type="button" onclick="createLearnedFromCapture()">保存为新按键</button>
            <button type="button" onclick="refresh()">刷新按键</button>
          </div>
        </div>
        <div class="msg" id="msg">正在读取状态...</div>
      </div>
    </div>
  </div>
  <div class="quickbar">
    <button type="button" onclick="togglePower()">电源</button>
    <button type="button" onclick="adjustTemp(-0.5)">-0.5℃</button>
    <button class="send" type="button" onclick="sendAc()">发送</button>
    <button type="button" onclick="adjustTemp(0.5)">+0.5℃</button>
  </div>

<script>
const $ = id => document.getElementById(id);
const modeText = {cool:'制冷', auto:'自动', dry:'除湿', heat:'制热', fan:'送风'};
const fanText = {auto:'自动', min:'最小', low:'低风', medium:'中风', high:'高风', max:'最大'};
const remote = {power:true, mode:'cool', fan:'auto', temp:26};
const remoteStorageKey = 'ir-ac-remote-state-v1';
let busy = false;
let liveBusy = false;
let statusBusy = false;
let learnedCommands = [];
let currentCapture = null;
let keyEditMode = false;
let dragKeyId = 0;
let dragMoved = false;

function esc(value){
  return String(value ?? '').replace(/[&<>"']/g, ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
}
function clamp(value, min, max){ return Math.min(max, Math.max(min, value)); }
function saveRemoteState(){
  try {
    localStorage.setItem(remoteStorageKey, JSON.stringify(remote));
  } catch(e) {}
}
function loadRemoteState(){
  try {
    const saved = JSON.parse(localStorage.getItem(remoteStorageKey) || '{}');
    if (typeof saved.power === 'boolean') remote.power = saved.power;
    if (modeText[saved.mode]) remote.mode = saved.mode;
    if (fanText[saved.fan]) remote.fan = saved.fan;
    if (Number.isFinite(Number(saved.temp))) remote.temp = clamp(Math.round(Number(saved.temp) * 2) / 2, 16, 32);
  } catch(e) {}
}
function applyRemoteConfig(cfg){
  if (!cfg) return;
  if (typeof cfg.remotePower === 'boolean') remote.power = cfg.remotePower;
  if (modeText[cfg.remoteMode]) remote.mode = cfg.remoteMode;
  if (fanText[cfg.remoteFan]) remote.fan = cfg.remoteFan;
  if (Number.isFinite(Number(cfg.remoteDegrees))) {
    remote.temp = clamp(Math.round(Number(cfg.remoteDegrees) * 2) / 2, 16, 32);
  }
}
function setMsg(text, warn=false){
  $('msg').textContent = text || '';
  $('msg').style.color = warn ? 'var(--danger)' : 'var(--accent)';
}
async function apiPost(url, body){
  const r = await fetch(url, {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body)
  });
  const text = await r.text();
  if (!r.ok) throw new Error(text || '操作失败');
  return text;
}
function renderRemote(){
  $('targetTemp').textContent = remote.temp.toFixed(1);
  $('powerState').textContent = remote.power ? '开机' : '关机';
  $('powerButton').textContent = remote.power ? '关机' : '开机';
  $('modeState').textContent = `${modeText[remote.mode] || remote.mode} / ${fanText[remote.fan] || remote.fan}`;
  document.querySelectorAll('[data-mode]').forEach(btn => btn.classList.toggle('selected', btn.dataset.mode === remote.mode));
  document.querySelectorAll('[data-fan]').forEach(btn => btn.classList.toggle('selected', btn.dataset.fan === remote.fan));
}
async function sendAc(){
  if (busy) return;
  busy = true;
  setMsg('正在发送...');
  try {
    saveRemoteState();
    await apiPost('/api/send-ac', {power:remote.power, mode:remote.mode, degrees:remote.temp, fan:remote.fan});
    setMsg('已发送');
  } catch(e) {
    setMsg(e.message || '发送失败', true);
  } finally {
    busy = false;
    renderRemote();
  }
}
function togglePower(){
  remote.power = !remote.power;
  saveRemoteState();
  sendAc();
}
function adjustTemp(delta){
  remote.power = true;
  remote.temp = clamp(Math.round((remote.temp + delta) * 2) / 2, 16, 32);
  saveRemoteState();
  renderRemote();
  sendAc();
}
function setMode(mode){
  remote.power = true;
  remote.mode = mode;
  saveRemoteState();
  renderRemote();
  sendAc();
}
function setFan(fan){
  remote.power = true;
  remote.fan = fan;
  saveRemoteState();
  renderRemote();
  sendAc();
}
async function sendLearned(id){
  if (busy) return;
  busy = true;
  setMsg('正在发送自定义按键...');
  try {
    await apiPost('/api/send-learned', {id});
    setMsg('自定义按键已发送');
  } catch(e) {
    setMsg(e.message || '发送失败', true);
  } finally {
    busy = false;
  }
}
function renderLearned(items){
  learnedCommands = items || [];
  const box = $('learnedList');
  box.classList.toggle('editing', keyEditMode);
  $('keyEditButton').textContent = keyEditMode ? '完成' : '编辑';
  $('keyEditButton').classList.toggle('active', keyEditMode);
  if (!items || !items.length) {
    box.innerHTML = '<div style="color:#94a3b8;font-size:13px">暂无自定义按键。先按实体遥控器，再保存为新按键。</div>';
    return;
  }
  box.innerHTML = items.map(item => `
    <div class="learned-card" data-key-id="${Number(item.id)}" onpointerdown="keyDragStart(event, ${Number(item.id)})" onpointermove="keyDragMove(event)" onpointerup="keyDragEnd(event)" onpointercancel="keyDragEnd(event)">
      ${keyEditMode ? '<div class="drag-handle">按住拖动排序</div>' : ''}
      <button class="learned-send" type="button" onclick="${keyEditMode ? `renameLearned(${Number(item.id)})` : `sendLearned(${Number(item.id)})`}">
        <span class="learned-title">${esc(item.name || ('按键 ' + item.id))}</span>
        <span class="learned-meta">${esc(item.meta || item.protocol || '')} · Raw ${esc(item.rawLen || 0)}</span>
      </button>
      ${keyEditMode ? `<div class="learned-actions">
        <button type="button" onclick="renameLearned(${Number(item.id)})">改名</button>
        <button class="replace" type="button" onclick="replaceLearnedSignal(${Number(item.id)})">替换</button>
        <button class="danger" type="button" onclick="deleteLearned(${Number(item.id)})">删除</button>
      </div>` : ''}
    </div>`).join('');
}
function renderCapture(capture){
  currentCapture = capture || null;
  if (!capture || !capture.available) {
    $('captureState').textContent = '暂无红外捕获。对准接收头按实体遥控器按键后，可保存为自定义按键。';
    return;
  }
  const age = Math.round((capture.ageMs || 0) / 1000);
  $('captureState').textContent = [
    '最近捕获：' + (capture.protocol || '未知') + ' / Raw ' + (capture.rawLen || 0),
    '码值：' + (capture.value || '--'),
    '时间：' + age + ' 秒前'
  ].join('\n');
}
function toggleKeyEditMode(){
  keyEditMode = !keyEditMode;
  renderLearned(learnedCommands);
  setMsg(keyEditMode ? '已进入编辑模式，可拖动排序或删除按键' : '已退出编辑模式');
}
function keyDragStart(evt, id){
  if (!keyEditMode || !evt.target.closest('.drag-handle')) return;
  const card = evt.currentTarget;
  dragKeyId = Number(id);
  dragMoved = false;
  card.classList.add('dragging');
  card.setPointerCapture?.(evt.pointerId);
  evt.preventDefault();
}
function keyDragMove(evt){
  if (!keyEditMode || !dragKeyId) return;
  const box = $('learnedList');
  const card = box.querySelector(`[data-key-id="${dragKeyId}"]`);
  if (!card) return;
  dragMoved = true;
  const siblings = [...box.querySelectorAll('.learned-card:not(.dragging)')];
  const before = siblings.find(el => evt.clientY < el.getBoundingClientRect().top + el.offsetHeight / 2);
  if (before) box.insertBefore(card, before);
  else box.appendChild(card);
  evt.preventDefault();
}
async function keyDragEnd(evt){
  if (!dragKeyId) return;
  const box = $('learnedList');
  const card = box.querySelector(`[data-key-id="${dragKeyId}"]`);
  card?.classList.remove('dragging');
  card?.releasePointerCapture?.(evt.pointerId);
  if (dragMoved) await persistKeyOrder();
  dragKeyId = 0;
  dragMoved = false;
}
async function persistKeyOrder(){
  const ids = [...document.querySelectorAll('#learnedList .learned-card')].map(card => Number(card.dataset.keyId)).filter(Boolean);
  if (!ids.length) return;
  try {
    await apiPost('/api/reorder-learned', {ids});
    const byId = new Map(learnedCommands.map(item => [Number(item.id), item]));
    learnedCommands = ids.map(id => byId.get(id)).filter(Boolean);
    setMsg('按键顺序已保存');
  } catch(e) {
    setMsg(e.message || '排序保存失败', true);
    await refresh();
  }
}
async function createLearnedFromCapture(){
  if (!currentCapture || !currentCapture.available) return setMsg('还没有捕获到实体遥控器信号', true);
  const name = prompt('按键名称', '自定义按键 ' + (learnedCommands.length + 1));
  if (name === null) return;
  const cleanName = name.trim();
  if (!cleanName) return setMsg('按键名称不能为空', true);
  try {
    setMsg('正在保存自定义按键...');
    await apiPost('/api/learn', {
      name: cleanName,
      freqKhz: 38,
      power: remote.power,
      mode: remote.mode,
      degrees: remote.temp,
      fan: remote.fan
    });
    setMsg('自定义按键已保存');
    await refresh();
  } catch(e) {
    setMsg(e.message || '保存失败', true);
  }
}
function learnedById(id){
  return learnedCommands.find(item => Number(item.id) === Number(id));
}
async function renameLearned(id){
  if (!keyEditMode) return setMsg('请先进入编辑模式', true);
  const item = learnedById(id);
  const name = prompt('新的按键名称', item ? item.name : '');
  if (name === null) return;
  const cleanName = name.trim();
  if (!cleanName) return setMsg('按键名称不能为空', true);
  try {
    setMsg('正在更新按键...');
    await apiPost('/api/update-learned', {id, name: cleanName});
    setMsg('按键已更新');
    await refresh();
  } catch(e) {
    setMsg(e.message || '更新失败', true);
  }
}
async function replaceLearnedSignal(id){
  if (!keyEditMode) return setMsg('请先进入编辑模式', true);
  if (!currentCapture || !currentCapture.available) return setMsg('还没有新的红外捕获可替换', true);
  if (!confirm('用最近捕获的实体遥控器信号替换这个按键？')) return;
  try {
    setMsg('正在替换按键信号...');
    await apiPost('/api/update-learned', {
      id,
      replaceFromCapture: true,
      power: remote.power,
      mode: remote.mode,
      degrees: remote.temp,
      fan: remote.fan,
      hasAcMeta: true
    });
    setMsg('按键信号已替换');
    await refresh();
  } catch(e) {
    setMsg(e.message || '替换失败', true);
  }
}
async function deleteLearned(id){
  if (!keyEditMode) return setMsg('请先进入编辑模式', true);
  if (!confirm('删除这个自定义按键？')) return;
  try {
    setMsg('正在删除按键...');
    await apiPost('/api/delete-learned', {id});
    setMsg('按键已删除');
    await refresh();
  } catch(e) {
    setMsg(e.message || '删除失败', true);
  }
}
function applyLive(state){
  $('wifi').textContent = state.wifi && state.wifi.connected ? state.wifi.ip : '热点模式';
  $('roomState').textContent = `${Number.isFinite(state.temperatureC) ? state.temperatureC.toFixed(1) : '--'} ℃ / ${Number.isFinite(state.humidity) ? state.humidity.toFixed(0) : '--'}%`;
  renderCapture(state.capture);
}
async function refreshLive(){
  if (liveBusy || statusBusy) return;
  liveBusy = true;
  try {
    const r = await fetch('/api/live');
    const state = await r.json();
    applyLive(state);
  } catch(e) {
    setMsg('实时状态读取失败', true);
  } finally {
    liveBusy = false;
  }
  renderRemote();
}
async function refresh(){
  if (statusBusy) return;
  statusBusy = true;
  try {
    const r = await fetch('/api/status');
    const state = await r.json();
    applyLive(state);
    applyRemoteConfig(state.config);
    renderLearned(state.learned);
    setMsg('状态已更新');
  } catch(e) {
    setMsg('状态读取失败', true);
  } finally {
    statusBusy = false;
  }
  renderRemote();
}
loadRemoteState();
renderRemote();
refresh();
setInterval(refreshLive, 1000);
setInterval(refresh, 15000);
</script>
</body>
</html>
)REMOTE";

String uint64ToHexString(uint64_t value) {
  char buffer[19];
  snprintf(buffer, sizeof(buffer), "0x%08llX", static_cast<unsigned long long>(value));
  return String(buffer);
}

float clampFloat(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

stdAc::opmode_t parseMode(const String &mode) {
  if (mode == "cool") return stdAc::opmode_t::kCool;
  if (mode == "heat") return stdAc::opmode_t::kHeat;
  if (mode == "dry") return stdAc::opmode_t::kDry;
  if (mode == "fan") return stdAc::opmode_t::kFan;
  return stdAc::opmode_t::kAuto;
}

stdAc::fanspeed_t parseFan(const String &fan) {
  if (fan == "min") return stdAc::fanspeed_t::kMin;
  if (fan == "low") return stdAc::fanspeed_t::kLow;
  if (fan == "medium") return stdAc::fanspeed_t::kMedium;
  if (fan == "high") return stdAc::fanspeed_t::kHigh;
  if (fan == "max") return stdAc::fanspeed_t::kMax;
  return stdAc::fanspeed_t::kAuto;
}

void normalizeExclusiveSpecials(bool &turbo, bool &quiet, bool &sleep) {
  if (turbo) {
    quiet = false;
    sleep = false;
    return;
  }
  if (sleep) {
    quiet = false;
  }
}

AcRequest normalizedAcRequest(AcRequest request) {
  if (!config.capTurbo) request.turbo = false;
  if (!config.capQuiet) request.quiet = false;
  if (!config.capSleep) request.sleep = false;
  if (!config.capSwingV) request.swingV = false;
  if (!config.capSwingH) request.swingH = false;
  if (!config.capFilter) request.filter = false;
  normalizeExclusiveSpecials(request.turbo, request.quiet, request.sleep);
  return request;
}

void normalizeConfigRemoteState() {
  if (!config.capTurbo) config.remoteTurbo = false;
  if (!config.capQuiet) config.remoteQuiet = false;
  if (!config.capSleep) config.remoteSleep = false;
  if (!config.capSwingV) config.remoteSwingV = false;
  if (!config.capSwingH) config.remoteSwingH = false;
  if (!config.capFilter) config.remoteFilter = false;
  normalizeExclusiveSpecials(config.remoteTurbo, config.remoteQuiet, config.remoteSleep);
}

void normalizePresetCommand(PresetCommand &cmd) {
  if (!config.capTurbo) cmd.turbo = false;
  if (!config.capQuiet) cmd.quiet = false;
  if (!config.capSleep) cmd.sleep = false;
  if (!config.capSwingV) cmd.swingV = false;
  if (!config.capSwingH) cmd.swingH = false;
  if (!config.capFilter) cmd.filter = false;
  normalizeExclusiveSpecials(cmd.turbo, cmd.quiet, cmd.sleep);
}

void normalizeCurveControlConfig() {
  if (config.curveControlMode != "staged" && config.curveControlMode != "fast" &&
      config.curveControlMode != "quiet") {
    config.curveControlMode = "staged";
  }
  if (config.curveEndAction != "hold" && config.curveEndAction != "poweroff") {
    config.curveEndAction = "hold";
  }
  if (config.quietSwitchMinute > 1439) config.quietSwitchMinute = 1439;
}

void normalizeTuningConfig() {
  config.sensorTempOffset = clampFloat(config.sensorTempOffset, -5.0f, 5.0f);
  config.sensorHumidityOffset = clampFloat(config.sensorHumidityOffset, -20.0f, 20.0f);
  config.targetHumidity = clampFloat(config.targetHumidity, 35.0f, 85.0f);
  config.humidityDeadband = clampFloat(config.humidityDeadband, 1.0f, 15.0f);
  config.humidityTargetTemp = clampFloat(config.humidityTargetTemp, config.minSetpoint, config.maxSetpoint);
  config.learnedFastRate = clampFloat(config.learnedFastRate, 0.0f, 2.0f);
  config.learnedQuietRate = clampFloat(config.learnedQuietRate, 0.0f, 2.0f);
  config.closedLoopFastGain = clampFloat(config.closedLoopFastGain, 0.5f, 5.0f);
  config.closedLoopQuietGain = clampFloat(config.closedLoopQuietGain, 0.2f, 3.0f);
}

bool readJsonFile(const char *path, JsonDocument &doc) {
  if (!LittleFS.exists(path)) return false;
  File file = LittleFS.open(path, "r");
  if (!file) return false;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  return !error;
}

bool writeJsonFile(const char *path, JsonDocument &doc) {
  File file = LittleFS.open(path, "w");
  if (!file) return false;
  bool ok = serializeJson(doc, file) > 0;
  file.close();
  return ok;
}

void appendTempHistorySample(uint32_t minute, int16_t temp10, int16_t humidity10, bool markDirty) {
  tempHistory[tempHistoryHead].minute = minute;
  tempHistory[tempHistoryHead].temp10 = temp10;
  tempHistory[tempHistoryHead].humidity10 = humidity10;
  tempHistoryHead = (tempHistoryHead + 1) % kTempHistoryPoints;
  if (tempHistoryCount < kTempHistoryPoints) tempHistoryCount++;
  lastHistoryMinute = minute;
  if (markDirty) {
    tempHistoryDirty = true;
    if (unsavedTempHistorySamples < UINT16_MAX) unsavedTempHistorySamples++;
  }
}

void clearTempHistory() {
  tempHistoryHead = 0;
  tempHistoryCount = 0;
  lastHistoryMinute = UINT32_MAX;
  tempHistoryDirty = false;
  unsavedTempHistorySamples = 0;
}

bool saveTempHistory() {
  File file = LittleFS.open(kTempHistoryPath, "w");
  if (!file) return false;

  file.printf("TH2,%u,%u\n", tempHistoryUsesEpoch ? 1 : 0, tempHistoryCount);
  for (uint16_t i = 0; i < tempHistoryCount; i++) {
    uint16_t idx = (tempHistoryHead + kTempHistoryPoints - tempHistoryCount + i) % kTempHistoryPoints;
    file.printf("%lu,%d,%d\n",
                static_cast<unsigned long>(tempHistory[idx].minute),
                static_cast<int>(tempHistory[idx].temp10),
                static_cast<int>(tempHistory[idx].humidity10));
  }

  bool ok = file.getWriteError() == 0;
  file.close();
  if (ok) {
    tempHistoryDirty = false;
    unsavedTempHistorySamples = 0;
    lastTempHistorySaveMs = millis();
  }
  return ok;
}

bool appendTempHistoryToFile(uint32_t minute, int16_t temp10, int16_t humidity10) {
  bool create = !LittleFS.exists(kTempHistoryPath);
  File file = LittleFS.open(kTempHistoryPath, create ? "w" : "a");
  if (!file) return false;
  if (create) file.printf("TH2,%u,0\n", tempHistoryUsesEpoch ? 1 : 0);
  file.printf("%lu,%d,%d\n",
              static_cast<unsigned long>(minute),
              static_cast<int>(temp10),
              static_cast<int>(humidity10));
  bool ok = file.getWriteError() == 0;
  file.close();
  if (ok) {
    tempHistoryDirty = true;
    if (unsavedTempHistorySamples < UINT16_MAX) unsavedTempHistorySamples++;
    lastTempHistorySaveMs = millis();
  }
  return ok;
}

void loadTempHistory() {
  clearTempHistory();
  if (!LittleFS.exists(kTempHistoryPath)) {
    lastTempHistorySaveMs = millis();
    return;
  }

  File file = LittleFS.open(kTempHistoryPath, "r");
  if (!file) {
    lastTempHistorySaveMs = millis();
    return;
  }

  String header = file.readStringUntil('\n');
  header.trim();
  unsigned int version = 0;
  unsigned int usesEpoch = 0;
  unsigned int count = 0;
  if (sscanf(header.c_str(), "TH%u,%u,%u", &version, &usesEpoch, &count) != 3 ||
      (version != 1 && version != 2)) {
    file.close();
    lastTempHistorySaveMs = millis();
    return;
  }
  tempHistoryUsesEpoch = usesEpoch != 0;
  bool needsUpgrade = version < 2;

  while (file.available() && tempHistoryCount < kTempHistoryPoints) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    unsigned long minute = 0;
    int temp10 = 0;
    int humidity10 = INT16_MIN;
    int matched = sscanf(line.c_str(), "%lu,%d,%d", &minute, &temp10, &humidity10);
    if (matched >= 2) {
      if (matched < 3) humidity10 = INT16_MIN;
      appendTempHistorySample(static_cast<uint32_t>(minute),
                              static_cast<int16_t>(temp10),
                              static_cast<int16_t>(humidity10),
                              false);
      if (matched < 3) needsUpgrade = true;
    }
  }
  file.close();
  tempHistoryDirty = false;
  unsavedTempHistorySamples = 0;
  lastTempHistorySaveMs = millis();
  if (needsUpgrade) saveTempHistory();
}

void maintainTempHistoryPersistence() {
  if (!tempHistoryDirty) return;
  uint32_t now = millis();
  bool enoughSamples = unsavedTempHistorySamples >= kTempHistoryCompactSamples;
  bool enoughTime = now - lastTempHistorySaveMs >= kTempHistoryCompactIntervalMs;
  if (enoughSamples || enoughTime) saveTempHistory();
}

void saveConfig() {
  normalizeTuningConfig();
  normalizeConfigRemoteState();
  JsonDocument doc;
  doc["staSsid"] = config.staSsid;
  doc["staPassword"] = config.staPassword;
  doc["acProtocol"] = typeToString(config.acProtocol);
  doc["acModel"] = config.acModel;
  doc["autoEnabled"] = config.autoEnabled;
  doc["autoMode"] = config.autoMode;
  doc["remotePower"] = config.remotePower;
  doc["remoteMode"] = config.remoteMode;
  doc["remoteFan"] = config.remoteFan;
  doc["remoteDegrees"] = config.remoteDegrees;
  doc["remoteTurbo"] = config.remoteTurbo;
  doc["remoteQuiet"] = config.remoteQuiet;
  doc["remoteSleep"] = config.remoteSleep;
  doc["remoteSwingV"] = config.remoteSwingV;
  doc["remoteSwingH"] = config.remoteSwingH;
  doc["remoteFilter"] = config.remoteFilter;
  doc["curveControlMode"] = config.curveControlMode;
  doc["curveEndAction"] = config.curveEndAction;
  doc["quietSwitchMinute"] = config.quietSwitchMinute;
  doc["sleepStartMinute"] = config.sleepStartMinute;
  doc["sleepDurationMinute"] = config.sleepDurationMinute;
  doc["controlIntervalSec"] = config.controlIntervalSec;
  doc["deadband"] = config.deadband;
  doc["autoSendDelta"] = config.autoSendDelta;
  doc["minSetpoint"] = config.minSetpoint;
  doc["maxSetpoint"] = config.maxSetpoint;
  doc["sensorTempOffset"] = config.sensorTempOffset;
  doc["sensorHumidityOffset"] = config.sensorHumidityOffset;
  doc["humidityControlEnabled"] = config.humidityControlEnabled;
  doc["curveHumidityEnabled"] = config.curveHumidityEnabled;
  doc["targetHumidity"] = config.targetHumidity;
  doc["humidityDeadband"] = config.humidityDeadband;
  doc["humidityTargetTemp"] = config.humidityTargetTemp;
  doc["predictiveSkipEnabled"] = config.predictiveSkipEnabled;
  doc["adaptiveControlEnabled"] = config.adaptiveControlEnabled;
  doc["learnedFastRate"] = config.learnedFastRate;
  doc["learnedQuietRate"] = config.learnedQuietRate;
  doc["closedLoopFastGain"] = config.closedLoopFastGain;
  doc["closedLoopQuietGain"] = config.closedLoopQuietGain;
  doc["capTurbo"] = config.capTurbo;
  doc["capQuiet"] = config.capQuiet;
  doc["capSleep"] = config.capSleep;
  doc["capSwingV"] = config.capSwingV;
  doc["capSwingH"] = config.capSwingH;
  doc["capFilter"] = config.capFilter;
  JsonArray curve = doc["curve"].to<JsonArray>();
  for (uint8_t i = 0; i < config.curveCount; i++) {
    JsonObject point = curve.add<JsonObject>();
    point["minute"] = config.curve[i].minute;
    point["temp"] = config.curve[i].temp;
  }
  writeJsonFile(kConfigPath, doc);
}

void loadConfig() {
  JsonDocument doc;
  if (!readJsonFile(kConfigPath, doc)) return;
  config.staSsid = doc["staSsid"] | config.staSsid;
  config.staPassword = doc["staPassword"] | config.staPassword;
  if (config.staSsid == kDefaultWifiSsid && config.staPassword.length() == 0) {
    config.staPassword = kDefaultWifiPassword;
  }
  config.acProtocol = strToDecodeType((doc["acProtocol"] | "UNKNOWN"));
  config.acModel = doc["acModel"] | 1;
  config.autoEnabled = doc["autoEnabled"] | false;
  config.autoMode = doc["autoMode"] | config.autoMode;
  config.remotePower = doc["remotePower"] | config.remotePower;
  config.remoteMode = doc["remoteMode"] | config.remoteMode;
  config.remoteFan = doc["remoteFan"] | config.remoteFan;
  config.remoteDegrees = clampFloat(doc["remoteDegrees"] | config.remoteDegrees, 16.0f, 32.0f);
  config.remoteTurbo = doc["remoteTurbo"] | config.remoteTurbo;
  config.remoteQuiet = doc["remoteQuiet"] | config.remoteQuiet;
  config.remoteSleep = doc["remoteSleep"] | config.remoteSleep;
  config.remoteSwingV = doc["remoteSwingV"] | config.remoteSwingV;
  config.remoteSwingH = doc["remoteSwingH"] | config.remoteSwingH;
  config.remoteFilter = doc["remoteFilter"] | config.remoteFilter;
  normalizeConfigRemoteState();
  config.curveControlMode = doc["curveControlMode"] | config.curveControlMode;
  config.curveEndAction = doc["curveEndAction"] | config.curveEndAction;
  config.quietSwitchMinute = doc["quietSwitchMinute"] | config.quietSwitchMinute;
  normalizeCurveControlConfig();
  config.sleepStartMinute = doc["sleepStartMinute"] | config.sleepStartMinute;
  config.sleepDurationMinute = doc["sleepDurationMinute"] | config.sleepDurationMinute;
  config.controlIntervalSec = doc["controlIntervalSec"] | config.controlIntervalSec;
  config.deadband = doc["deadband"] | config.deadband;
  config.autoSendDelta = doc["autoSendDelta"] | config.autoSendDelta;
  config.minSetpoint = 16.0f;
  config.maxSetpoint = 32.0f;
  config.sensorTempOffset = doc["sensorTempOffset"] | config.sensorTempOffset;
  config.sensorHumidityOffset = doc["sensorHumidityOffset"] | config.sensorHumidityOffset;
  config.humidityControlEnabled = doc["humidityControlEnabled"] | config.humidityControlEnabled;
  config.curveHumidityEnabled = doc["curveHumidityEnabled"] | config.curveHumidityEnabled;
  config.targetHumidity = doc["targetHumidity"] | config.targetHumidity;
  config.humidityDeadband = doc["humidityDeadband"] | config.humidityDeadband;
  config.humidityTargetTemp = doc["humidityTargetTemp"] | config.humidityTargetTemp;
  config.predictiveSkipEnabled = doc["predictiveSkipEnabled"] | config.predictiveSkipEnabled;
  config.adaptiveControlEnabled = doc["adaptiveControlEnabled"] | config.adaptiveControlEnabled;
  config.learnedFastRate = doc["learnedFastRate"] | config.learnedFastRate;
  config.learnedQuietRate = doc["learnedQuietRate"] | config.learnedQuietRate;
  config.closedLoopFastGain = doc["closedLoopFastGain"] | config.closedLoopFastGain;
  config.closedLoopQuietGain = doc["closedLoopQuietGain"] | config.closedLoopQuietGain;
  config.capTurbo = doc["capTurbo"] | config.capTurbo;
  config.capQuiet = doc["capQuiet"] | config.capQuiet;
  config.capSleep = doc["capSleep"] | config.capSleep;
  config.capSwingV = doc["capSwingV"] | config.capSwingV;
  config.capSwingH = doc["capSwingH"] | config.capSwingH;
  config.capFilter = doc["capFilter"] | config.capFilter;
  normalizeTuningConfig();
  normalizeConfigRemoteState();
  JsonArray curve = doc["curve"].as<JsonArray>();
  if (!curve.isNull()) {
    config.curveCount = 0;
    for (JsonObject point : curve) {
      if (config.curveCount >= kMaxCurvePoints) break;
      config.curve[config.curveCount].minute = point["minute"] | 0;
      config.curve[config.curveCount].temp = point["temp"] | 26.0f;
      config.curveCount++;
    }
    if (config.curveCount == 0) config.curveCount = 1;
  }
}

void saveLearnedLibrary() {
  JsonDocument doc;
  JsonArray arr = doc["commands"].to<JsonArray>();
  for (uint8_t i = 0; i < learnedCount; i++) {
    JsonObject item = arr.add<JsonObject>();
    item["id"] = learned[i].id;
    item["name"] = learned[i].name;
    item["protocol"] = typeToString(learned[i].protocol);
    item["bits"] = learned[i].bits;
    item["value"] = uint64ToHexString(learned[i].value);
    item["hasAcMeta"] = learned[i].hasAcMeta;
    item["power"] = learned[i].power;
    item["degrees"] = learned[i].degrees;
    item["mode"] = learned[i].mode;
    item["fan"] = learned[i].fan;
    item["freqKhz"] = learned[i].freqKhz;
    item["acDescription"] = learned[i].acDescription;
    JsonArray raw = item["raw"].to<JsonArray>();
    for (uint16_t j = 0; j < learned[i].rawLen; j++) raw.add(learned[i].raw[j]);
  }
  writeJsonFile(kLibraryPath, doc);
}

uint64_t parseHex64(const String &input) {
  String s = input;
  s.trim();
  if (s.startsWith("0x") || s.startsWith("0X")) s = s.substring(2);
  uint64_t out = 0;
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    uint8_t nibble = 0;
    if (c >= '0' && c <= '9') nibble = c - '0';
    else if (c >= 'a' && c <= 'f') nibble = 10 + c - 'a';
    else if (c >= 'A' && c <= 'F') nibble = 10 + c - 'A';
    else continue;
    out = (out << 4) | nibble;
  }
  return out;
}

void loadLearnedLibrary() {
  JsonDocument doc;
  if (!readJsonFile(kLibraryPath, doc)) return;
  JsonArray arr = doc["commands"].as<JsonArray>();
  if (arr.isNull()) return;
  learnedCount = 0;
  for (JsonObject item : arr) {
    if (learnedCount >= kMaxLearnedCommands) break;
    LearnedCommand &cmd = learned[learnedCount];
    cmd.id = item["id"] | static_cast<uint8_t>(learnedCount + 1);
    cmd.name = item["name"] | "";
    cmd.protocol = strToDecodeType((item["protocol"] | "UNKNOWN"));
    cmd.bits = item["bits"] | 0;
    cmd.value = parseHex64(item["value"] | "0");
    cmd.hasAcMeta = item["hasAcMeta"] | false;
    cmd.power = item["power"] | true;
    cmd.degrees = item["degrees"] | 26.0f;
    cmd.mode = item["mode"] | "cool";
    cmd.fan = item["fan"] | "auto";
    cmd.freqKhz = item["freqKhz"] | 38;
    cmd.acDescription = item["acDescription"] | "";
    cmd.rawLen = 0;
    JsonArray raw = item["raw"].as<JsonArray>();
    for (JsonVariant value : raw) {
      if (cmd.rawLen >= kMaxRawPulses) break;
      cmd.raw[cmd.rawLen++] = value.as<uint16_t>();
    }
    learnedCount++;
  }
}

void savePresetLibrary() {
  JsonDocument doc;
  JsonArray arr = doc["presets"].to<JsonArray>();
  for (uint8_t i = 0; i < presetCount; i++) {
    normalizePresetCommand(presets[i]);
    JsonObject item = arr.add<JsonObject>();
    item["id"] = presets[i].id;
    item["name"] = presets[i].name;
    item["power"] = presets[i].power;
    item["degrees"] = presets[i].degrees;
    item["mode"] = presets[i].mode;
    item["fan"] = presets[i].fan;
    item["turbo"] = presets[i].turbo;
    item["quiet"] = presets[i].quiet;
    item["sleep"] = presets[i].sleep;
    item["swingV"] = presets[i].swingV;
    item["swingH"] = presets[i].swingH;
    item["filter"] = presets[i].filter;
    item["scheduleEnabled"] = presets[i].scheduleEnabled;
    item["scheduleMinute"] = presets[i].scheduleMinute;
    item["scheduleMode"] = presets[i].scheduleMode;
    item["dayMask"] = presets[i].dayMask;
  }
  writeJsonFile(kPresetPath, doc);
}

void loadPresetLibrary() {
  JsonDocument doc;
  if (!readJsonFile(kPresetPath, doc)) return;
  JsonArray arr = doc["presets"].as<JsonArray>();
  if (arr.isNull()) return;
  presetCount = 0;
  for (JsonObject item : arr) {
    if (presetCount >= kMaxPresetCommands) break;
    PresetCommand &cmd = presets[presetCount];
    cmd.id = item["id"] | static_cast<uint8_t>(presetCount + 1);
    cmd.name = item["name"] | "";
    cmd.power = item["power"] | true;
    cmd.degrees = item["degrees"] | 26.0f;
    cmd.mode = item["mode"] | "cool";
    cmd.fan = item["fan"] | "auto";
    cmd.turbo = item["turbo"] | false;
    cmd.quiet = item["quiet"] | false;
    cmd.sleep = item["sleep"] | false;
    cmd.swingV = item["swingV"] | false;
    cmd.swingH = item["swingH"] | false;
    cmd.filter = item["filter"] | false;
    normalizePresetCommand(cmd);
    cmd.scheduleEnabled = item["scheduleEnabled"] | false;
    cmd.scheduleMinute = (item["scheduleMinute"] | static_cast<uint16_t>(7 * 60)) % 1440;
    cmd.scheduleMode = item["scheduleMode"] | "daily";
    cmd.dayMask = item["dayMask"] | static_cast<uint8_t>(0b0111110);
    presetCount++;
  }
}

uint8_t nextPresetId() {
  uint8_t next = 1;
  for (uint8_t i = 0; i < presetCount; i++) {
    if (presets[i].id >= next) next = presets[i].id + 1;
  }
  return next;
}

int findPresetIndexById(uint8_t id) {
  for (uint8_t i = 0; i < presetCount; i++) {
    if (presets[i].id == id) return i;
  }
  return -1;
}

uint8_t nextLearnedId() {
  uint8_t next = 1;
  for (uint8_t i = 0; i < learnedCount; i++) {
    if (learned[i].id >= next) next = learned[i].id + 1;
  }
  return next;
}

int findLearnedIndexById(uint8_t id) {
  for (uint8_t i = 0; i < learnedCount; i++) {
    if (learned[i].id == id) return i;
  }
  return -1;
}

void copyCaptureToLearned(LearnedCommand &cmd) {
  cmd.protocol = lastCapture.protocol;
  cmd.bits = lastCapture.bits;
  cmd.value = lastCapture.value;
  cmd.acDescription = lastCapture.acDescription;
  cmd.rawLen = lastCapture.rawLen;
  for (uint16_t i = 0; i < cmd.rawLen; i++) cmd.raw[i] = lastCapture.raw[i];
}

void swapLearnedCommands(uint8_t a, uint8_t b) {
  if (a == b || a >= learnedCount || b >= learnedCount) return;
  LearnedCommand tmp = learned[a];
  learned[a] = learned[b];
  learned[b] = tmp;
}

bool parseBody(JsonDocument &doc) {
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    server.send(400, "text/plain", "Invalid JSON");
    return false;
  }
  return true;
}

void sendOk(const String &message = "ok") {
  JsonDocument doc;
  doc["ok"] = true;
  doc["message"] = message;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void sendError(uint16_t code, const String &message) {
  JsonDocument doc;
  doc["ok"] = false;
  doc["message"] = message;
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json", out);
}

String jsonString(const String &value) {
  String out;
  out.reserve(value.length() + 8);
  out += '"';
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<uint8_t>(c) < 0x20) {
          char buffer[7];
          snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<uint8_t>(c));
          out += buffer;
        } else {
          out += c;
        }
        break;
    }
  }
  out += '"';
  return out;
}

bool useDefaultStaticIp() {
  return config.staSsid == kDefaultWifiSsid && strlen(kDefaultWifiStaticIp) > 0;
}

void configureStationIp() {
  IPAddress localIp;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns1;
  IPAddress dns2;

  if (useDefaultStaticIp() &&
      localIp.fromString(kDefaultWifiStaticIp) &&
      gateway.fromString(kDefaultWifiGateway) &&
      subnet.fromString(kDefaultWifiSubnet) &&
      dns1.fromString(kDefaultWifiDns1) &&
      dns2.fromString(kDefaultWifiDns2)) {
    WiFi.config(localIp, gateway, subnet, dns1, dns2);
    return;
  }

  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
}

void disableWifiPowerSave() {
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
}

void startAccessPoint() {
  if (apStarted) return;
  WiFi.softAP(kApSsid, kApPassword);
  dnsServer.start(53, "*", WiFi.softAPIP());
  apStarted = true;
}

void stopAccessPoint() {
  if (!apStarted) return;
  WiFi.softAPdisconnect(true);
  dnsServer.stop();
  apStarted = false;
}

void startWifi() {
  WiFi.persistent(false);
  WiFi.setHostname(kHostname);

  if (config.staSsid.length()) {
    WiFi.mode(WIFI_STA);
    disableWifiPowerSave();
    configureStationIp();
    WiFi.begin(config.staSsid.c_str(), config.staPassword.c_str());
    wifiConnectStartMs = millis();
    lastWifiAttemptMs = millis();
    return;
  }

  WiFi.mode(WIFI_AP);
  disableWifiPowerSave();
  startAccessPoint();
}

void maintainWifi() {
  if (!config.staSsid.length()) return;

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnectStartMs = 0;
    if (apStarted) {
      stopAccessPoint();
      WiFi.mode(WIFI_STA);
      disableWifiPowerSave();
    }
    return;
  }

  if (!apStarted && wifiConnectStartMs && millis() - wifiConnectStartMs > kWifiApFallbackMs) {
    WiFi.mode(WIFI_AP_STA);
    disableWifiPowerSave();
    startAccessPoint();
  }

  if (millis() - lastWifiAttemptMs < kWifiRetryMs) return;
  lastWifiAttemptMs = millis();
  WiFi.disconnect(false);
  configureStationIp();
  disableWifiPowerSave();
  WiFi.begin(config.staSsid.c_str(), config.staPassword.c_str());
  wifiConnectStartMs = millis();
}

void maintainClock() {
  if (WiFi.status() != WL_CONNECTED || ntpConfigured) return;
  configTzTime(kTimezone, kNtpServer1, kNtpServer2);
  ntpConfigured = true;
}

bool getLocalMinuteOfDay(uint16_t *minuteOut, String *timeTextOut = nullptr) {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5)) {
    *minuteOut = static_cast<uint16_t>(timeinfo.tm_hour * 60 + timeinfo.tm_min);
    if (timeTextOut != nullptr) {
      char buffer[24];
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
      *timeTextOut = buffer;
    }
    return true;
  }
  *minuteOut = static_cast<uint16_t>((millis() / 60000UL) % 1440);
  if (timeTextOut != nullptr) *timeTextOut = "unsynced";
  return false;
}

uint32_t currentHistoryMinute(bool *usesEpochOut = nullptr) {
  time_t now = time(nullptr);
  bool usesEpoch = now > 1700000000;
  if (usesEpochOut != nullptr) *usesEpochOut = usesEpoch;
  return usesEpoch ? static_cast<uint32_t>(now / 60) : millis() / 60000UL;
}

int16_t tempToTenths(float value) {
  return isnan(value) ? INT16_MIN : static_cast<int16_t>(roundf(value * 10.0f));
}

void copyText(char *dest, size_t len, const char *src) {
  if (len == 0) return;
  snprintf(dest, len, "%s", src == nullptr ? "" : src);
}

void setPendingAcContext(const char *source, const char *action, float target = NAN, float setpoint = NAN) {
  copyText(pendingAcSource, sizeof(pendingAcSource), source);
  copyText(pendingAcAction, sizeof(pendingAcAction), action);
  pendingAcTarget = target;
  pendingAcSetpoint = setpoint;
}

void resetPendingAcContext() {
  setPendingAcContext("manual", "send");
}

void clearControlEvents() {
  controlEventHead = 0;
  controlEventCount = 0;
}

void appendControlEventMemory(const ControlEvent &event) {
  controlEvents[controlEventHead] = event;
  controlEventHead = (controlEventHead + 1) % kControlEventPoints;
  if (controlEventCount < kControlEventPoints) controlEventCount++;
}

void clearDecisionLog() {
  decisionLogHead = 0;
  decisionLogCount = 0;
}

bool saveControlEvents() {
  File file = LittleFS.open(kControlEventsPath, "w");
  if (!file) return false;
  file.printf("CE1,%u,%u\n", controlEventsUseEpoch ? 1 : 0, controlEventCount);
  for (uint16_t i = 0; i < controlEventCount; i++) {
    uint16_t idx = (controlEventHead + kControlEventPoints - controlEventCount + i) % kControlEventPoints;
    const ControlEvent &event = controlEvents[idx];
    file.printf("%lu,%d,%d,%d,%s,%s,%s,%s,%u,%u,%u,%u\n",
                static_cast<unsigned long>(event.minute),
                static_cast<int>(event.room10),
                static_cast<int>(event.target10),
                static_cast<int>(event.setpoint10),
                event.source,
                event.action,
                event.mode,
                event.fan,
                event.power ? 1 : 0,
                event.turbo ? 1 : 0,
                event.quiet ? 1 : 0,
                event.sleep ? 1 : 0);
  }
  bool ok = file.getWriteError() == 0;
  file.close();
  return ok;
}

void loadControlEvents() {
  clearControlEvents();
  if (!LittleFS.exists(kControlEventsPath)) return;
  File file = LittleFS.open(kControlEventsPath, "r");
  if (!file) return;
  String header = file.readStringUntil('\n');
  header.trim();
  unsigned int usesEpoch = 0;
  unsigned int count = 0;
  if (sscanf(header.c_str(), "CE1,%u,%u", &usesEpoch, &count) != 2) {
    file.close();
    return;
  }
  controlEventsUseEpoch = usesEpoch != 0;
  while (file.available() && controlEventCount < kControlEventPoints) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    unsigned long minute = 0;
    int room10 = INT16_MIN;
    int target10 = INT16_MIN;
    int setpoint10 = INT16_MIN;
    int power = 0;
    int turbo = 0;
    int quiet = 0;
    int sleep = 0;
    char source[12] = "";
    char action[16] = "";
    char mode[8] = "";
    char fan[8] = "";
    int matched = sscanf(line.c_str(), "%lu,%d,%d,%d,%11[^,],%15[^,],%7[^,],%7[^,],%d,%d,%d,%d",
                         &minute,
                         &room10,
                         &target10,
                         &setpoint10,
                         source,
                         action,
                         mode,
                         fan,
                         &power,
                         &turbo,
                         &quiet,
                         &sleep);
    if (matched >= 12) {
      ControlEvent event;
      event.minute = static_cast<uint32_t>(minute);
      event.room10 = static_cast<int16_t>(room10);
      event.target10 = static_cast<int16_t>(target10);
      event.setpoint10 = static_cast<int16_t>(setpoint10);
      copyText(event.source, sizeof(event.source), source);
      copyText(event.action, sizeof(event.action), action);
      copyText(event.mode, sizeof(event.mode), mode);
      copyText(event.fan, sizeof(event.fan), fan);
      event.power = power != 0;
      event.turbo = turbo != 0;
      event.quiet = quiet != 0;
      event.sleep = sleep != 0;
      appendControlEventMemory(event);
    }
  }
  file.close();
}

void addDecisionLog(const char *action, const char *stage, const char *note,
                    float room, float target, float setpoint) {
  bool usesEpoch = false;
  uint32_t minute = currentHistoryMinute(&usesEpoch);
  if (decisionLogCount > 0 && usesEpoch != decisionLogUsesEpoch) {
    clearDecisionLog();
  }
  decisionLogUsesEpoch = usesEpoch;
  if (decisionLogCount > 0 && action != nullptr && strncmp(action, "skip_", 5) == 0) {
    uint16_t lastIdx = (decisionLogHead + kDecisionLogPoints - 1) % kDecisionLogPoints;
    const DecisionLogEntry &last = decisionLog[lastIdx];
    if (last.minute == minute &&
        strncmp(last.action, action == nullptr ? "" : action, sizeof(last.action)) == 0 &&
        strncmp(last.stage, stage == nullptr ? "" : stage, sizeof(last.stage)) == 0) {
      return;
    }
  }
  DecisionLogEntry &entry = decisionLog[decisionLogHead];
  entry.minute = minute;
  entry.room10 = tempToTenths(room);
  entry.target10 = tempToTenths(target);
  entry.setpoint10 = tempToTenths(setpoint);
  copyText(entry.action, sizeof(entry.action), action);
  copyText(entry.stage, sizeof(entry.stage), stage);
  copyText(entry.note, sizeof(entry.note), note);
  decisionLogHead = (decisionLogHead + 1) % kDecisionLogPoints;
  if (decisionLogCount < kDecisionLogPoints) decisionLogCount++;
}

void addControlEvent(const char *source, const char *action, const AcRequest &request,
                     float target = NAN, float setpoint = NAN) {
  bool usesEpoch = false;
  uint32_t minute = currentHistoryMinute(&usesEpoch);
  if (controlEventCount > 0 && usesEpoch != controlEventsUseEpoch) {
    clearControlEvents();
    LittleFS.remove(kControlEventsPath);
  }
  controlEventsUseEpoch = usesEpoch;
  ControlEvent event;
  event.minute = minute;
  event.room10 = tempToTenths(roomTempC);
  event.target10 = tempToTenths(target);
  event.setpoint10 = tempToTenths(isnan(setpoint) ? request.degrees : setpoint);
  copyText(event.source, sizeof(event.source), source);
  copyText(event.action, sizeof(event.action), action);
  copyText(event.mode, sizeof(event.mode), request.mode.c_str());
  copyText(event.fan, sizeof(event.fan), request.fan.c_str());
  event.power = request.power;
  event.turbo = request.turbo;
  event.quiet = request.quiet;
  event.sleep = request.sleep;
  appendControlEventMemory(event);
  saveControlEvents();
}

bool tempTrendPerMinute(uint16_t windowMinutes, float *slopeOut) {
  if (tempHistoryCount < 2 || slopeOut == nullptr) return false;
  uint16_t lastIdx = (tempHistoryHead + kTempHistoryPoints - 1) % kTempHistoryPoints;
  const TempHistorySample &last = tempHistory[lastIdx];
  int firstIdx = -1;
  for (uint16_t i = 0; i < tempHistoryCount; i++) {
    uint16_t idx = (tempHistoryHead + kTempHistoryPoints - tempHistoryCount + i) % kTempHistoryPoints;
    uint32_t age = last.minute >= tempHistory[idx].minute ? last.minute - tempHistory[idx].minute : 0;
    if (age <= windowMinutes) {
      firstIdx = idx;
      break;
    }
  }
  if (firstIdx < 0 || firstIdx == lastIdx) return false;
  int32_t deltaMin = static_cast<int32_t>(last.minute) - static_cast<int32_t>(tempHistory[firstIdx].minute);
  if (deltaMin < 3) return false;
  *slopeOut = (last.temp10 - tempHistory[firstIdx].temp10) / 10.0f / deltaMin;
  return true;
}

void updateAdaptiveRate(const String &stage, float target) {
  if (!config.adaptiveControlEnabled || isnan(roomTempC)) return;
  float slope = 0.0f;
  if (!tempTrendPerMinute(20, &slope)) return;
  bool movingTowardTarget = (roomTempC > target && slope < -0.005f) || (roomTempC < target && slope > 0.005f);
  if (!movingTowardTarget) return;
  float rate = fabs(slope);
  float *learned = stage == "fast" ? &config.learnedFastRate : &config.learnedQuietRate;
  *learned = *learned <= 0.0f ? rate : (*learned * 0.82f + rate * 0.18f);
  normalizeTuningConfig();
}

bool shouldPredictiveSkip(float target, String *reasonOut) {
  if (!config.predictiveSkipEnabled || isnan(roomTempC)) return false;
  float slope = 0.0f;
  if (!tempTrendPerMinute(15, &slope)) return false;
  float error = fabs(roomTempC - target);
  bool movingTowardTarget = (roomTempC > target && slope < -0.01f) || (roomTempC < target && slope > 0.01f);
  if (!movingTowardTarget || error <= config.deadband || error > 1.4f) return false;
  float minutesToBand = (error - config.deadband) / max(0.01f, fabs(slope));
  if (minutesToBand > 25.0f) return false;
  if (reasonOut != nullptr) {
    *reasonOut = "趋势已接近目标，预计 " + String(minutesToBand, 0) + " 分钟进入死区";
  }
  return true;
}

void recordTemperatureHistory() {
  if (isnan(roomTempC)) return;

  bool usesEpoch = false;
  uint32_t minute = currentHistoryMinute(&usesEpoch);
  if (!usesEpoch && tempHistoryUsesEpoch && tempHistoryCount > 0) return;
  if (tempHistoryCount > 0 && usesEpoch != tempHistoryUsesEpoch) {
    clearTempHistory();
    LittleFS.remove(kTempHistoryPath);
  }
  tempHistoryUsesEpoch = usesEpoch;
  if (minute == lastHistoryMinute) return;

  int16_t temp10 = static_cast<int16_t>(roundf(roomTempC * 10.0f));
  int16_t humidity10 = tempToTenths(roomHumidity);
  appendTempHistorySample(minute, temp10, humidity10, false);
  appendTempHistoryToFile(minute, temp10, humidity10);
}

void updateSensor() {
  if (!sht31Ready || millis() - lastSensorMs < 3000) return;
  lastSensorMs = millis();
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (!isnan(t)) roomTempC = t + config.sensorTempOffset;
  if (!isnan(h)) roomHumidity = clampFloat(h + config.sensorHumidityOffset, 0.0f, 100.0f);
  recordTemperatureHistory();
}

void captureIrIfAvailable() {
  if (irBusy) return;
  if (!irrecv.decode(&irResults)) return;

  lastCapture.available = true;
  lastCapture.protocol = irResults.decode_type;
  lastCapture.bits = irResults.bits;
  lastCapture.value = irResults.value;
  lastCapture.summary = resultToHumanReadableBasic(&irResults);
  lastCapture.acDescription = IRAcUtils::resultAcToString(&irResults);
  lastCapture.capturedAtMs = millis();

  uint16_t correctedLen = getCorrectedRawLength(&irResults);
  lastCapture.rawLen = correctedLen > kMaxRawPulses ? kMaxRawPulses : correctedLen;
  uint16_t *raw = resultToRawArray(&irResults);
  if (raw != nullptr) {
    for (uint16_t i = 0; i < lastCapture.rawLen; i++) lastCapture.raw[i] = raw[i];
    delete[] raw;
  }

  irrecv.resume();
}

bool sendLearnedNow(uint8_t id) {
  int idx = findLearnedIndexById(id);
  if (idx < 0) {
    lastActionResult = "learned command not found";
    return false;
  }
  if (learned[idx].rawLen == 0) {
    lastActionResult = "learned command has no raw data";
    return false;
  }
  irBusy = true;
  irrecv.disableIRIn();
  rawSender.sendRaw(learned[idx].raw, learned[idx].rawLen, learned[idx].freqKhz);
  irrecv.enableIRIn();
  irBusy = false;
  lastActionResult = "sent learned #" + String(id);
  return true;
}

int findSemanticLearnedCommand(const AcRequest &request) {
  int best = -1;
  float bestScore = 999.0f;
  for (uint8_t i = 0; i < learnedCount; i++) {
    if (!learned[i].hasAcMeta || learned[i].rawLen == 0) continue;
    if (learned[i].power != request.power) continue;
    if (learned[i].mode != request.mode) continue;
    float score = fabs(learned[i].degrees - request.degrees);
    if (score < bestScore) {
      best = i;
      bestScore = score;
    }
  }
  return best;
}

void prepareAcState(const AcRequest &request) {
  ac.next.protocol = config.acProtocol;
  ac.next.model = config.acModel;
  ac.next.power = request.power;
  ac.next.mode = parseMode(request.mode);
  ac.next.celsius = true;
  ac.next.degrees = clampFloat(request.degrees, config.minSetpoint, config.maxSetpoint);
  ac.next.fanspeed = parseFan(request.fan);
  ac.next.swingv = request.swingV ? stdAc::swingv_t::kAuto : stdAc::swingv_t::kOff;
  ac.next.swingh = request.swingH ? stdAc::swingh_t::kAuto : stdAc::swingh_t::kOff;
  ac.next.light = false;
  ac.next.beep = false;
  ac.next.econo = false;
  ac.next.filter = request.filter;
  ac.next.turbo = request.turbo;
  ac.next.quiet = request.quiet;
  ac.next.sleep = request.sleep ? 0 : -1;
  ac.next.clean = false;
  ac.next.clock = -1;
}

void rememberAcState(const AcRequest &request) {
  float degrees = clampFloat(request.degrees, config.minSetpoint, config.maxSetpoint);
  bool changed = config.remotePower != request.power ||
                 config.remoteMode != request.mode ||
                 config.remoteFan != request.fan ||
                 config.remoteTurbo != request.turbo ||
                 config.remoteQuiet != request.quiet ||
                 config.remoteSleep != request.sleep ||
                 config.remoteSwingV != request.swingV ||
                 config.remoteSwingH != request.swingH ||
                 config.remoteFilter != request.filter ||
                 fabs(config.remoteDegrees - degrees) >= 0.05f;
  if (!changed) return;
  config.remotePower = request.power;
  config.remoteMode = request.mode;
  config.remoteFan = request.fan;
  config.remoteDegrees = degrees;
  config.remoteTurbo = request.turbo;
  config.remoteQuiet = request.quiet;
  config.remoteSleep = request.sleep;
  config.remoteSwingV = request.swingV;
  config.remoteSwingH = request.swingH;
  config.remoteFilter = request.filter;
  saveConfig();
}

bool sendAcNow(const AcRequest &request, const char *source = "manual", const char *action = "send",
               float target = NAN, float setpoint = NAN) {
  AcRequest normalized = normalizedAcRequest(request);
  if (IRac::isProtocolSupported(config.acProtocol)) {
    irBusy = true;
    irrecv.disableIRIn();
    prepareAcState(normalized);
    bool ok = ac.sendAc();
    irrecv.enableIRIn();
    irBusy = false;
    lastActionResult = ok ? "sent AC " + typeToString(config.acProtocol) +
                                " power=" + String(normalized.power ? "on" : "off") +
                                " mode=" + normalized.mode +
                                " temp=" + String(normalized.degrees, 1) +
                                " fan=" + normalized.fan +
                                " turbo=" + String(normalized.turbo ? "on" : "off") +
                                " quiet=" + String(normalized.quiet ? "on" : "off") +
                                " sleep=" + String(normalized.sleep ? "on" : "off") +
                                " swingV=" + String(normalized.swingV ? "on" : "off") +
                                " swingH=" + String(normalized.swingH ? "on" : "off") +
                                " filter=" + String(normalized.filter ? "on" : "off")
                          : "AC send failed";
    if (ok) {
      rememberAcState(normalized);
      if (normalized.power) lastSentSetpoint = clampFloat(normalized.degrees, config.minSetpoint, config.maxSetpoint);
    }
    addControlEvent(source, ok ? action : "failed", normalized, target, isnan(setpoint) ? normalized.degrees : setpoint);
    addDecisionLog(ok ? "sent" : "failed", source, lastActionResult.c_str(), roomTempC, target,
                   isnan(setpoint) ? normalized.degrees : setpoint);
    return ok;
  }

  int idx = findSemanticLearnedCommand(normalized);
  if (idx >= 0) {
    bool ok = sendLearnedNow(learned[idx].id);
    if (ok) {
      rememberAcState(normalized);
      if (normalized.power) lastSentSetpoint = learned[idx].degrees;
    }
    addControlEvent(source, ok ? action : "failed", normalized, target, isnan(setpoint) ? learned[idx].degrees : setpoint);
    addDecisionLog(ok ? "sent_raw" : "failed", source, lastActionResult.c_str(), roomTempC, target,
                   isnan(setpoint) ? learned[idx].degrees : setpoint);
    return ok;
  }

  lastActionResult = "no supported AC protocol or matching learned raw command";
  addDecisionLog("failed", source, lastActionResult.c_str(), roomTempC, target, setpoint);
  return false;
}

AcRequest requestFromPreset(const PresetCommand &preset) {
  AcRequest request;
  request.power = preset.power;
  request.mode = preset.mode;
  request.fan = preset.fan;
  request.degrees = preset.degrees;
  request.turbo = preset.turbo;
  request.quiet = preset.quiet;
  request.sleep = preset.sleep;
  request.swingV = preset.swingV;
  request.swingH = preset.swingH;
  request.filter = preset.filter;
  return normalizedAcRequest(request);
}

bool presetScheduleMatchesDay(const PresetCommand &preset, uint8_t wday) {
  if (preset.scheduleMode == "daily") return true;
  if (preset.scheduleMode == "workday") return wday >= 1 && wday <= 5;
  if (preset.scheduleMode == "holiday") return wday == 0 || wday == 6;
  if (preset.scheduleMode == "custom") return (preset.dayMask & (1 << wday)) != 0;
  return true;
}

void maintainPresetSchedules() {
  if (pendingAction != PendingAction::None || irBusy) return;
  time_t now = time(nullptr);
  if (now <= 1700000000) return;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 5)) return;
  uint16_t minuteOfDay = static_cast<uint16_t>(timeinfo.tm_hour * 60 + timeinfo.tm_min);
  uint32_t minuteKey = static_cast<uint32_t>(now / 60);
  uint8_t wday = static_cast<uint8_t>(timeinfo.tm_wday);

  for (uint8_t i = 0; i < presetCount; i++) {
    PresetCommand &preset = presets[i];
    if (!preset.scheduleEnabled) continue;
    if (preset.scheduleMinute != minuteOfDay) continue;
    if (preset.lastScheduleKey == minuteKey) continue;
    if (!presetScheduleMatchesDay(preset, wday)) continue;
    pendingAc = requestFromPreset(preset);
    setPendingAcContext("schedule", "send", NAN, pendingAc.degrees);
    pendingAction = PendingAction::SendAc;
    preset.lastScheduleKey = minuteKey;
    lastActionResult = "preset schedule queued #" + String(preset.id);
    return;
  }
}

void handlePendingIr() {
  if (pendingAction == PendingAction::None) return;
  PendingAction action = pendingAction;
  pendingAction = PendingAction::None;
  if (action == PendingAction::SendAc) {
    sendAcNow(pendingAc, pendingAcSource, pendingAcAction, pendingAcTarget, pendingAcSetpoint);
    resetPendingAcContext();
  } else if (action == PendingAction::SendLearned) {
    sendLearnedNow(pendingLearnedId);
  }
}

float targetTempForSleepCurve(uint16_t elapsedMinute) {
  if (config.curveCount == 0) return 26.0f;
  if (elapsedMinute <= config.curve[0].minute) return config.curve[0].temp;
  for (uint8_t i = 1; i < config.curveCount; i++) {
    CurvePoint prev = config.curve[i - 1];
    CurvePoint next = config.curve[i];
    if (elapsedMinute <= next.minute) {
      uint16_t span = next.minute > prev.minute ? next.minute - prev.minute : 1;
      float ratio = static_cast<float>(elapsedMinute - prev.minute) / static_cast<float>(span);
      return prev.temp + (next.temp - prev.temp) * ratio;
    }
  }
  return config.curve[config.curveCount - 1].temp;
}

String curveStageForElapsed(uint16_t elapsedMinute) {
  if (config.curveControlMode != "quiet" && elapsedMinute >= config.quietSwitchMinute) return "quiet";
  if (config.curveControlMode == "quiet") return "quiet";
  return "fast";
}

float temperatureDemand(float target, const String &mode) {
  if (isnan(roomTempC)) return 0.0f;
  if (mode == "heat") return target - roomTempC;
  return roomTempC - target;
}

String fanForSleepDemand(float demand, bool quietStage, bool dehumidifying) {
  if (quietStage) return "low";
  float lowThreshold = max(config.deadband, 0.25f);
  if (demand <= lowThreshold) return "low";
  if (dehumidifying) return demand >= 1.0f ? "medium" : "low";
  return demand >= 1.0f ? "high" : "medium";
}

bool turboForSleepDemand(float demand, bool quietStage, bool dehumidifying) {
  if (quietStage || dehumidifying) return false;
  return demand >= max(1.2f, config.deadband * 3.0f);
}

bool quietForSleepDemand(bool quietStage, bool curveActive) {
  return quietStage || !curveActive;
}

float closedLoopSetpoint(float target, const String &mode, const String &stage = "quiet") {
  if (isnan(roomTempC) || mode == "fan") {
    return clampFloat(target, config.minSetpoint, config.maxSetpoint);
  }
  float error = roomTempC - target;
  float gain = stage == "fast" ? config.closedLoopFastGain : config.closedLoopQuietGain;
  float limit = stage == "fast" ? 7.0f : 2.0f;
  if (config.adaptiveControlEnabled) {
    float learned = stage == "fast" ? config.learnedFastRate : config.learnedQuietRate;
    if (learned > 0.0f && learned < 0.035f) gain *= 1.18f;
    if (learned > 0.08f) gain *= 0.92f;
  }
  float correction = clampFloat(error * gain, -limit, limit);
  return clampFloat(target - correction, config.minSetpoint, config.maxSetpoint);
}

bool sleepCurveIsActive(uint16_t nowMinute, uint16_t *elapsedOut) {
  uint16_t start = config.sleepStartMinute % 1440;
  uint16_t duration = config.sleepDurationMinute > 1439 ? 1439 : config.sleepDurationMinute;
  uint16_t elapsed = (nowMinute + 1440 - start) % 1440;
  if (elapsed <= duration) {
    *elapsedOut = elapsed;
    return true;
  }
  return false;
}

void resetAutoSendMemory() {
  lastAutoSentSetpoint = NAN;
  lastAutoSentMode = "";
  lastAutoSentFan = "";
  lastAutoSentTurbo = false;
  lastAutoSentQuiet = false;
  lastAutoSentSleep = false;
}

bool sameAutoRequestShape(const AcRequest &request) {
  return request.mode == lastAutoSentMode &&
         request.fan == lastAutoSentFan &&
         request.turbo == lastAutoSentTurbo &&
         request.quiet == lastAutoSentQuiet &&
         request.sleep == lastAutoSentSleep;
}

void rememberAutoRequestShape(const AcRequest &request) {
  lastAutoSentMode = request.mode;
  lastAutoSentFan = request.fan;
  lastAutoSentTurbo = request.turbo;
  lastAutoSentQuiet = request.quiet;
  lastAutoSentSleep = request.sleep;
}

void runAutoControl() {
  bool anyAutomaticEnabled = config.autoEnabled || config.humidityControlEnabled;
  if (!anyAutomaticEnabled) {
    sleepCurveWasActive = false;
    resetAutoSendMemory();
    return;
  }
  if (pendingAction != PendingAction::None) return;

  uint16_t nowMinute = 0;
  getLocalMinuteOfDay(&nowMinute);
  uint16_t elapsed = 0;
  bool curveActive = config.autoEnabled && sleepCurveIsActive(nowMinute, &elapsed);
  if (!curveActive) {
    if (sleepCurveWasActive && config.curveEndAction == "poweroff") {
      pendingAc.power = false;
      pendingAc.mode = config.autoMode;
      pendingAc.fan = "auto";
      pendingAc.turbo = false;
      pendingAc.quiet = false;
      pendingAc.sleep = false;
      pendingAc.swingV = false;
      pendingAc.swingH = false;
      pendingAc.filter = false;
      pendingAc.degrees = isnan(lastSentSetpoint) ? 26.0f : lastSentSetpoint;
      setPendingAcContext("auto", "end_poweroff", NAN, pendingAc.degrees);
      pendingAction = PendingAction::SendAc;
      addDecisionLog("end_poweroff", "end", "曲线结束，按设置关机", roomTempC, NAN, pendingAc.degrees);
      resetAutoSendMemory();
      lastControlMs = millis();
      sleepCurveWasActive = false;
      return;
    } else if (sleepCurveWasActive) {
      addDecisionLog("end_hold", "end", "曲线结束，保持当前空调状态，不发射红外", roomTempC, NAN, NAN);
    }
    sleepCurveWasActive = false;
    if (!config.humidityControlEnabled) return;
  } else {
    sleepCurveWasActive = true;
  }

  if (isnan(roomTempC)) {
    addDecisionLog("skip_sensor", curveActive ? "curve" : "humidity", "等待 SHT31 室温读数", roomTempC, NAN, NAN);
    lastControlMs = millis();
    return;
  }
  if (millis() - lastControlMs < static_cast<uint32_t>(config.controlIntervalSec) * 1000UL) return;

  float target = curveActive
                     ? clampFloat(targetTempForSleepCurve(elapsed), config.minSetpoint, config.maxSetpoint)
                     : clampFloat(config.humidityTargetTemp, config.minSetpoint, config.maxSetpoint);
  String stage = curveActive ? curveStageForElapsed(elapsed) : "humidity";
  bool humidityEnabledNow = curveActive ? config.curveHumidityEnabled : config.humidityControlEnabled;
  bool humidityHigh = humidityEnabledNow && !isnan(roomHumidity) &&
                      roomHumidity > config.targetHumidity + config.humidityDeadband;
  bool humidityStable = humidityEnabledNow && !isnan(roomHumidity) &&
                        roomHumidity <= config.targetHumidity;
  bool tempTooLowForDry = humidityHigh && roomTempC < target - max(0.5f, config.deadband);
  bool tempInBand = fabs(roomTempC - target) < config.deadband;

  if (humidityEnabledNow && isnan(roomHumidity)) {
    addDecisionLog("skip_humidity_sensor", stage.c_str(), "等待 SHT31 湿度读数", roomTempC, target, NAN);
    lastControlMs = millis();
    return;
  }

  if (humidityEnabledNow && humidityHigh && tempTooLowForDry) {
    String note = "湿度 " + String(roomHumidity, 0) + "% 高于目标，但室温低于目标，暂停除湿避免过冷";
    addDecisionLog("skip_dehumidify_cold", stage.c_str(), note.c_str(), roomTempC, target, NAN);
    lastControlMs = millis();
    return;
  }

  bool shouldDehumidify = humidityHigh && !tempTooLowForDry;
  bool shouldTemperatureControl = curveActive || (config.humidityControlEnabled && !curveActive && !tempInBand);
  if (!shouldDehumidify && !shouldTemperatureControl) {
    String note = humidityEnabledNow
                      ? "温湿度已稳定：湿度 " + String(roomHumidity, 0) + "% / 目标 " + String(config.targetHumidity, 0) + "%"
                      : "室温已在死区内";
    addDecisionLog(humidityStable ? "skip_humidity_ok" : "skip_deadband", stage.c_str(), note.c_str(), roomTempC, target, NAN);
    lastControlMs = millis();
    return;
  }

  bool quietStage = curveActive ? stage == "quiet" : true;
  float demand = temperatureDemand(target, shouldDehumidify ? "dry" : (curveActive ? config.autoMode : "auto"));

  pendingAc.power = true;
  pendingAc.mode = shouldDehumidify ? "dry" : (curveActive ? config.autoMode : "auto");
  pendingAc.fan = curveActive
                      ? fanForSleepDemand(demand, quietStage, shouldDehumidify)
                      : (shouldDehumidify ? "low" : "auto");
  pendingAc.turbo = curveActive ? turboForSleepDemand(demand, quietStage, shouldDehumidify) : false;
  pendingAc.quiet = curveActive ? quietForSleepDemand(quietStage, true) : true;
  pendingAc.sleep = false;
  pendingAc.swingV = false;
  pendingAc.swingH = false;
  pendingAc.filter = false;
  pendingAc.degrees = closedLoopSetpoint(target, config.autoMode, stage);

  if (!shouldDehumidify && sameAutoRequestShape(pendingAc)) {
    String predictiveReason;
    if (shouldPredictiveSkip(target, &predictiveReason)) {
      addDecisionLog("skip_predict", stage.c_str(), predictiveReason.c_str(), roomTempC, target, NAN);
      lastActionResult = "auto skipped predictive";
      lastControlMs = millis();
      return;
    }
  }

  updateAdaptiveRate(stage, target);
  if (config.adaptiveControlEnabled && millis() - lastAdaptiveSaveMs > 30UL * 60UL * 1000UL) {
    saveConfig();
    lastAdaptiveSaveMs = millis();
  }
  if (!isnan(lastAutoSentSetpoint) && sameAutoRequestShape(pendingAc) &&
      fabs(pendingAc.degrees - lastAutoSentSetpoint) <= config.autoSendDelta) {
    lastActionResult = "auto skipped delta=" + String(fabs(pendingAc.degrees - lastAutoSentSetpoint), 1) +
                       "C threshold=" + String(config.autoSendDelta, 1) + "C";
    addDecisionLog("skip_delta", stage.c_str(), "模式和设定温度变化未超过发送过滤阈值", roomTempC, target, pendingAc.degrees);
    lastControlMs = millis();
    return;
  }
  setPendingAcContext(shouldDehumidify ? "humidity" : "auto", shouldDehumidify ? "dehumidify" : "send", target, pendingAc.degrees);
  pendingAction = PendingAction::SendAc;
  String note = shouldDehumidify
                    ? "已排队除湿：湿度 " + String(roomHumidity, 0) + "% / 目标 " + String(config.targetHumidity, 0) + "%"
                    : "已排队发送温度控制指令";
  note += "，风速 " + pendingAc.fan + (pendingAc.quiet ? " / 静音" : "") +
          (pendingAc.turbo ? " / 强劲" : "") +
          "，温差 " + String(demand, 1) + "℃";
  addDecisionLog(shouldDehumidify ? "queue_dehumidify" : "queue_send", stage.c_str(), note.c_str(), roomTempC, target, pendingAc.degrees);
  lastAutoSentSetpoint = pendingAc.degrees;
  rememberAutoRequestShape(pendingAc);
  lastControlMs = millis();
}

void addConfigToJson(JsonObject obj) {
  obj["staSsid"] = config.staSsid;
  obj["acProtocol"] = typeToString(config.acProtocol);
  obj["acModel"] = config.acModel;
  obj["autoEnabled"] = config.autoEnabled;
  obj["autoMode"] = config.autoMode;
  obj["remotePower"] = config.remotePower;
  obj["remoteMode"] = config.remoteMode;
  obj["remoteFan"] = config.remoteFan;
  obj["remoteDegrees"] = config.remoteDegrees;
  obj["remoteTurbo"] = config.remoteTurbo;
  obj["remoteQuiet"] = config.remoteQuiet;
  obj["remoteSleep"] = config.remoteSleep;
  obj["remoteSwingV"] = config.remoteSwingV;
  obj["remoteSwingH"] = config.remoteSwingH;
  obj["remoteFilter"] = config.remoteFilter;
  obj["curveControlMode"] = config.curveControlMode;
  obj["curveEndAction"] = config.curveEndAction;
  obj["quietSwitchMinute"] = config.quietSwitchMinute;
  obj["sleepStartMinute"] = config.sleepStartMinute;
  obj["sleepDurationMinute"] = config.sleepDurationMinute;
  obj["controlIntervalSec"] = config.controlIntervalSec;
  obj["deadband"] = config.deadband;
  obj["autoSendDelta"] = config.autoSendDelta;
  obj["minSetpoint"] = config.minSetpoint;
  obj["maxSetpoint"] = config.maxSetpoint;
  obj["sensorTempOffset"] = config.sensorTempOffset;
  obj["sensorHumidityOffset"] = config.sensorHumidityOffset;
  obj["humidityControlEnabled"] = config.humidityControlEnabled;
  obj["curveHumidityEnabled"] = config.curveHumidityEnabled;
  obj["targetHumidity"] = config.targetHumidity;
  obj["humidityDeadband"] = config.humidityDeadband;
  obj["humidityTargetTemp"] = config.humidityTargetTemp;
  obj["predictiveSkipEnabled"] = config.predictiveSkipEnabled;
  obj["adaptiveControlEnabled"] = config.adaptiveControlEnabled;
  obj["learnedFastRate"] = config.learnedFastRate;
  obj["learnedQuietRate"] = config.learnedQuietRate;
  obj["closedLoopFastGain"] = config.closedLoopFastGain;
  obj["closedLoopQuietGain"] = config.closedLoopQuietGain;
  obj["capTurbo"] = config.capTurbo;
  obj["capQuiet"] = config.capQuiet;
  obj["capSleep"] = config.capSleep;
  obj["capSwingV"] = config.capSwingV;
  obj["capSwingH"] = config.capSwingH;
  obj["capFilter"] = config.capFilter;
  JsonArray curve = obj["curve"].to<JsonArray>();
  for (uint8_t i = 0; i < config.curveCount; i++) {
    JsonObject point = curve.add<JsonObject>();
    point["minute"] = config.curve[i].minute;
    point["temp"] = config.curve[i].temp;
  }
}

void addLiveToJson(JsonDocument &doc) {
  if (isnan(roomTempC)) {
    doc["temperatureC"] = nullptr;
  } else {
    doc["temperatureC"] = roomTempC;
  }
  if (isnan(roomHumidity)) {
    doc["humidity"] = nullptr;
  } else {
    doc["humidity"] = roomHumidity;
  }
  doc["lastAction"] = lastActionResult;

  JsonObject wifi = doc["wifi"].to<JsonObject>();
  wifi["connected"] = WiFi.status() == WL_CONNECTED;
  wifi["ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  wifi["apActive"] = apStarted;
  wifi["apIp"] = apStarted ? WiFi.softAPIP().toString() : "";

  uint16_t minuteOfDay = 0;
  String localTimeText;
  bool timeSynced = getLocalMinuteOfDay(&minuteOfDay, &localTimeText);
  JsonObject clock = doc["clock"].to<JsonObject>();
  clock["synced"] = timeSynced;
  clock["local"] = localTimeText;
  clock["minuteOfDay"] = minuteOfDay;

  uint16_t elapsed = 0;
  bool curveActive = config.autoEnabled && sleepCurveIsActive(minuteOfDay, &elapsed);
  JsonObject autoTarget = doc["autoTarget"].to<JsonObject>();
  autoTarget["enabled"] = config.autoEnabled;
  autoTarget["active"] = curveActive || config.humidityControlEnabled;
  autoTarget["curveActive"] = curveActive;
  autoTarget["mode"] = config.autoMode;
  autoTarget["endAction"] = config.curveEndAction;
  autoTarget["humidityControlEnabled"] = config.humidityControlEnabled;
  autoTarget["curveHumidityEnabled"] = config.curveHumidityEnabled;
  autoTarget["humidityActive"] = curveActive ? config.curveHumidityEnabled : config.humidityControlEnabled;
  autoTarget["targetHumidity"] = config.targetHumidity;
  autoTarget["humidityDeadband"] = config.humidityDeadband;
  if (isnan(roomHumidity)) autoTarget["humidityError"] = nullptr;
  else autoTarget["humidityError"] = roomHumidity - config.targetHumidity;
  if (curveActive) {
    float target = clampFloat(targetTempForSleepCurve(elapsed), config.minSetpoint, config.maxSetpoint);
    String stage = curveStageForElapsed(elapsed);
    bool quietStage = stage == "quiet";
    float demand = temperatureDemand(target, config.autoMode);
    autoTarget["elapsedMinute"] = elapsed;
    autoTarget["temperatureC"] = target;
    autoTarget["setpointC"] = closedLoopSetpoint(target, config.autoMode, stage);
    autoTarget["roomErrorC"] = isnan(roomTempC) ? 0 : roomTempC - target;
    autoTarget["stage"] = stage;
    autoTarget["fan"] = fanForSleepDemand(demand, quietStage, false);
    autoTarget["turbo"] = turboForSleepDemand(demand, quietStage, false);
    autoTarget["quiet"] = quietForSleepDemand(quietStage, true);
    autoTarget["sleep"] = false;
  } else if (config.humidityControlEnabled) {
    float target = clampFloat(config.humidityTargetTemp, config.minSetpoint, config.maxSetpoint);
    autoTarget["elapsedMinute"] = nullptr;
    autoTarget["temperatureC"] = target;
    autoTarget["setpointC"] = closedLoopSetpoint(target, "auto", "humidity");
    autoTarget["roomErrorC"] = isnan(roomTempC) ? 0 : roomTempC - target;
    autoTarget["stage"] = "humidity";
    autoTarget["fan"] = "low";
    autoTarget["turbo"] = false;
    autoTarget["quiet"] = true;
    autoTarget["sleep"] = false;
  } else {
    autoTarget["elapsedMinute"] = nullptr;
    autoTarget["temperatureC"] = nullptr;
    autoTarget["setpointC"] = nullptr;
    autoTarget["roomErrorC"] = nullptr;
    autoTarget["stage"] = nullptr;
    autoTarget["fan"] = nullptr;
    autoTarget["turbo"] = false;
    autoTarget["quiet"] = false;
    autoTarget["sleep"] = false;
  }

  JsonObject cap = doc["capture"].to<JsonObject>();
  cap["available"] = lastCapture.available;
  if (lastCapture.available) {
    cap["protocol"] = typeToString(lastCapture.protocol);
    cap["bits"] = lastCapture.bits;
    cap["value"] = uint64ToHexString(lastCapture.value);
    cap["rawLen"] = lastCapture.rawLen;
    cap["summary"] = lastCapture.summary;
    cap["acDescription"] = lastCapture.acDescription;
    cap["ageMs"] = millis() - lastCapture.capturedAtMs;
  }
}

void handleLive() {
  JsonDocument doc;
  addLiveToJson(doc);

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleTempHistory() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("{\"intervalSec\":60,\"hours\":72,\"usesEpoch\":");
  server.sendContent(tempHistoryUsesEpoch ? "true" : "false");
  server.sendContent(",\"persistent\":true");
  server.sendContent(",\"count\":");
  server.sendContent(String(tempHistoryCount));
  server.sendContent(",\"samples\":[");

  for (uint16_t i = 0; i < tempHistoryCount; i++) {
    uint16_t idx = (tempHistoryHead + kTempHistoryPoints - tempHistoryCount + i) % kTempHistoryPoints;
    if (i > 0) server.sendContent(",");
    char buffer[72];
    snprintf(buffer, sizeof(buffer), "{\"minute\":%lu,\"temp\":%.1f,\"humidity\":",
             static_cast<unsigned long>(tempHistory[idx].minute),
             tempHistory[idx].temp10 / 10.0f);
    server.sendContent(buffer);
    if (tempHistory[idx].humidity10 == INT16_MIN) {
      server.sendContent("null");
    } else {
      server.sendContent(String(tempHistory[idx].humidity10 / 10.0f, 1));
    }
    server.sendContent("}");
  }

  server.sendContent("]}");
  server.sendContent("");
}

void sendJsonTenths(const char *name, int16_t value, bool leadingComma = true) {
  if (leadingComma) server.sendContent(",");
  server.sendContent("\"");
  server.sendContent(name);
  server.sendContent("\":");
  if (value == INT16_MIN) {
    server.sendContent("null");
    return;
  }
  server.sendContent(String(value / 10.0f, 1));
}

void handleControlEvents() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("{\"usesEpoch\":");
  server.sendContent(controlEventsUseEpoch ? "true" : "false");
  server.sendContent(",\"count\":");
  server.sendContent(String(controlEventCount));
  server.sendContent(",\"events\":[");
  for (uint16_t i = 0; i < controlEventCount; i++) {
    uint16_t idx = (controlEventHead + kControlEventPoints - controlEventCount + i) % kControlEventPoints;
    const ControlEvent &event = controlEvents[idx];
    if (i > 0) server.sendContent(",");
    server.sendContent("{\"minute\":");
    server.sendContent(String(event.minute));
    sendJsonTenths("room", event.room10);
    sendJsonTenths("target", event.target10);
    sendJsonTenths("setpoint", event.setpoint10);
    server.sendContent(",\"source\":\"");
    server.sendContent(event.source);
    server.sendContent("\",\"action\":\"");
    server.sendContent(event.action);
    server.sendContent("\",\"mode\":\"");
    server.sendContent(event.mode);
    server.sendContent("\",\"fan\":\"");
    server.sendContent(event.fan);
    server.sendContent("\",\"power\":");
    server.sendContent(event.power ? "true" : "false");
    server.sendContent(",\"turbo\":");
    server.sendContent(event.turbo ? "true" : "false");
    server.sendContent(",\"quiet\":");
    server.sendContent(event.quiet ? "true" : "false");
    server.sendContent(",\"sleep\":");
    server.sendContent(event.sleep ? "true" : "false");
    server.sendContent("}");
  }
  server.sendContent("]}");
  server.sendContent("");
}

void handleControlLog() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("{\"usesEpoch\":");
  server.sendContent(decisionLogUsesEpoch ? "true" : "false");
  server.sendContent(",\"count\":");
  server.sendContent(String(decisionLogCount));
  server.sendContent(",\"logs\":[");
  for (uint16_t i = 0; i < decisionLogCount; i++) {
    uint16_t idx = (decisionLogHead + kDecisionLogPoints - decisionLogCount + i) % kDecisionLogPoints;
    const DecisionLogEntry &entry = decisionLog[idx];
    if (i > 0) server.sendContent(",");
    JsonDocument doc;
    doc["minute"] = entry.minute;
    doc["action"] = entry.action;
    doc["stage"] = entry.stage;
    doc["note"] = entry.note;
    if (entry.room10 == INT16_MIN) doc["room"] = nullptr;
    else doc["room"] = entry.room10 / 10.0f;
    if (entry.target10 == INT16_MIN) doc["target"] = nullptr;
    else doc["target"] = entry.target10 / 10.0f;
    if (entry.setpoint10 == INT16_MIN) doc["setpoint"] = nullptr;
    else doc["setpoint"] = entry.setpoint10 / 10.0f;
    String out;
    serializeJson(doc, out);
    server.sendContent(out);
  }
  server.sendContent("]}");
  server.sendContent("");
}

void handleStatus() {
  JsonDocument doc;
  addLiveToJson(doc);

  JsonObject cfg = doc["config"].to<JsonObject>();
  addConfigToJson(cfg);

  JsonArray supported = doc["supportedProtocols"].to<JsonArray>();
  supported.add("UNKNOWN");
  for (int i = 1; i < kLastDecodeType; i++) {
    decode_type_t protocol = static_cast<decode_type_t>(i);
    if (IRac::isProtocolSupported(protocol)) supported.add(typeToString(protocol));
  }

  JsonArray arr = doc["learned"].to<JsonArray>();
  for (uint8_t i = 0; i < learnedCount; i++) {
    JsonObject item = arr.add<JsonObject>();
    item["id"] = learned[i].id;
    item["name"] = learned[i].name;
    item["protocol"] = typeToString(learned[i].protocol);
    item["rawLen"] = learned[i].rawLen;
    item["meta"] = learned[i].hasAcMeta
                       ? String(learned[i].power ? "on " : "off ") + learned[i].mode + " " +
                             String(learned[i].degrees, 1) + "C " + learned[i].fan
                       : "";
  }

  JsonArray presetArr = doc["presets"].to<JsonArray>();
  for (uint8_t i = 0; i < presetCount; i++) {
    JsonObject item = presetArr.add<JsonObject>();
    item["id"] = presets[i].id;
    item["name"] = presets[i].name;
    item["power"] = presets[i].power;
    item["degrees"] = presets[i].degrees;
    item["mode"] = presets[i].mode;
    item["fan"] = presets[i].fan;
    item["turbo"] = presets[i].turbo;
    item["quiet"] = presets[i].quiet;
    item["sleep"] = presets[i].sleep;
    item["swingV"] = presets[i].swingV;
    item["swingH"] = presets[i].swingH;
    item["filter"] = presets[i].filter;
    item["scheduleEnabled"] = presets[i].scheduleEnabled;
    item["scheduleMinute"] = presets[i].scheduleMinute;
    item["scheduleMode"] = presets[i].scheduleMode;
    item["dayMask"] = presets[i].dayMask;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleWifiPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  String newSsid = doc["ssid"] | "";
  String newPassword = doc["password"] | "";
  newSsid.trim();

  if (newSsid == kDefaultWifiSsid && newPassword.length() == 0) {
    newPassword = kDefaultWifiPassword;
  } else if (newSsid.length() && newPassword.length() == 0) {
    if (newSsid == config.staSsid && config.staPassword.length() > 0) {
      newPassword = config.staPassword;
    } else {
      sendError(400, "WiFi password is required for a new SSID");
      return;
    }
  }

  config.staSsid = newSsid;
  config.staPassword = newSsid.length() ? newPassword : "";
  saveConfig();
  WiFi.disconnect(false);
  stopAccessPoint();
  wifiConnectStartMs = 0;
  lastWifiAttemptMs = millis();
  ntpConfigured = false;
  if (config.staSsid.length()) {
    WiFi.mode(WIFI_STA);
    disableWifiPowerSave();
    configureStationIp();
    WiFi.begin(config.staSsid.c_str(), config.staPassword.c_str());
    wifiConnectStartMs = millis();
  } else {
    WiFi.mode(WIFI_AP);
    disableWifiPowerSave();
    startAccessPoint();
  }
  sendOk("wifi saved");
}

void handleWifiScan() {
  if (apStarted && WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP_STA);
    disableWifiPowerSave();
  } else if (!apStarted && WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    disableWifiPowerSave();
  }

  int found = WiFi.scanNetworks(false, true);
  if (found < 0) {
    sendError(503, "WiFi scan failed");
    return;
  }

  constexpr uint8_t kMaxScanCandidates = 64;
  constexpr uint8_t kMaxScanResults = 40;
  uint8_t candidateCount = static_cast<uint8_t>(min(found, static_cast<int>(kMaxScanCandidates)));
  bool used[kMaxScanCandidates] = {};
  String emittedSsids[kMaxScanResults];
  uint8_t emittedCount = 0;
  String networks;
  networks.reserve(4096);

  while (emittedCount < kMaxScanResults) {
    int best = -1;
    int bestRssi = -999;
    for (uint8_t i = 0; i < candidateCount; i++) {
      if (used[i]) continue;
      String ssid = WiFi.SSID(i);
      ssid.trim();
      if (!ssid.length()) {
        used[i] = true;
        continue;
      }
      bool duplicate = false;
      for (uint8_t j = 0; j < emittedCount; j++) {
        if (emittedSsids[j] == ssid) {
          duplicate = true;
          break;
        }
      }
      if (duplicate) {
        used[i] = true;
        continue;
      }
      int rssi = WiFi.RSSI(i);
      if (best < 0 || rssi > bestRssi) {
        best = i;
        bestRssi = rssi;
      }
    }
    if (best < 0) break;

    used[best] = true;
    String ssid = WiFi.SSID(best);
    ssid.trim();
    emittedSsids[emittedCount] = ssid;
    bool open = WiFi.encryptionType(best) == WIFI_AUTH_OPEN;
    if (emittedCount > 0) networks += ",";
    networks += "{\"ssid\":";
    networks += jsonString(ssid);
    networks += ",\"rssi\":";
    networks += String(WiFi.RSSI(best));
    networks += ",\"channel\":";
    networks += String(WiFi.channel(best));
    networks += ",\"open\":";
    networks += open ? "true" : "false";
    networks += ",\"auth\":\"";
    networks += open ? "开放" : "加密";
    networks += "\"}";
    emittedCount++;
  }

  WiFi.scanDelete();
  String out = "{\"count\":" + String(emittedCount) + ",\"networks\":[" + networks + "]}";
  server.send(200, "application/json", out);
}

void handleWifiForget() {
  config.staSsid = "";
  config.staPassword = "";
  saveConfig();
  WiFi.disconnect(false);
  WiFi.mode(WIFI_AP);
  disableWifiPowerSave();
  startAccessPoint();
  wifiConnectStartMs = 0;
  ntpConfigured = false;
  sendOk("wifi forgotten");
}

void handleAcConfigPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  config.acProtocol = strToDecodeType((doc["protocol"] | "UNKNOWN"));
  config.acModel = doc["model"] | 1;
  saveConfig();
  sendOk("ac config saved");
}

void handleSendAcPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  pendingAc.power = doc["power"] | config.remotePower;
  pendingAc.mode = doc["mode"] | config.remoteMode;
  pendingAc.fan = doc["fan"] | config.remoteFan;
  pendingAc.degrees = clampFloat(doc["degrees"] | config.remoteDegrees, 16.0f, 32.0f);
  pendingAc.turbo = doc["turbo"] | config.remoteTurbo;
  pendingAc.quiet = doc["quiet"] | config.remoteQuiet;
  pendingAc.sleep = doc["sleep"] | config.remoteSleep;
  pendingAc.swingV = doc["swingV"] | config.remoteSwingV;
  pendingAc.swingH = doc["swingH"] | config.remoteSwingH;
  pendingAc.filter = doc["filter"] | config.remoteFilter;
  pendingAc = normalizedAcRequest(pendingAc);
  setPendingAcContext("manual", "send", NAN, pendingAc.degrees);
  pendingAction = PendingAction::SendAc;
  sendOk("ac send queued");
}

void handleRemoteStatePost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  config.remotePower = doc["power"] | config.remotePower;
  config.remoteMode = doc["mode"] | config.remoteMode;
  config.remoteFan = doc["fan"] | config.remoteFan;
  config.remoteDegrees = clampFloat(doc["degrees"] | config.remoteDegrees, 16.0f, 32.0f);
  config.remoteTurbo = doc["turbo"] | config.remoteTurbo;
  config.remoteQuiet = doc["quiet"] | config.remoteQuiet;
  config.remoteSleep = doc["sleep"] | config.remoteSleep;
  config.remoteSwingV = doc["swingV"] | config.remoteSwingV;
  config.remoteSwingH = doc["swingH"] | config.remoteSwingH;
  config.remoteFilter = doc["filter"] | config.remoteFilter;
  normalizeConfigRemoteState();
  saveConfig();
  sendOk("remote state saved");
}

void handlePresetCreatePost() {
  if (presetCount >= kMaxPresetCommands) {
    sendError(409, "Preset library is full");
    return;
  }
  JsonDocument doc;
  if (!parseBody(doc)) return;
  PresetCommand &cmd = presets[presetCount];
  cmd.id = nextPresetId();
  cmd.name = doc["name"] | "";
  cmd.name.trim();
  if (!cmd.name.length()) cmd.name = "preset " + String(cmd.id);
  cmd.power = doc["power"] | true;
  cmd.mode = doc["mode"] | "cool";
  cmd.fan = doc["fan"] | "auto";
  cmd.degrees = clampFloat(doc["degrees"] | 26.0f, 16.0f, 32.0f);
  cmd.turbo = doc["turbo"] | false;
  cmd.quiet = doc["quiet"] | false;
  cmd.sleep = doc["sleep"] | false;
  cmd.swingV = doc["swingV"] | false;
  cmd.swingH = doc["swingH"] | false;
  cmd.filter = doc["filter"] | false;
  normalizePresetCommand(cmd);
  cmd.scheduleEnabled = doc["scheduleEnabled"] | false;
  cmd.scheduleMinute = (doc["scheduleMinute"] | cmd.scheduleMinute) % 1440;
  cmd.scheduleMode = doc["scheduleMode"] | "daily";
  cmd.dayMask = doc["dayMask"] | cmd.dayMask;
  presetCount++;
  savePresetLibrary();
  sendOk("preset saved");
}

void handlePresetUpdatePost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  uint8_t id = doc["id"] | 0;
  int idx = findPresetIndexById(id);
  if (idx < 0) {
    sendError(404, "Preset not found");
    return;
  }
  if (doc["name"].is<const char *>()) {
    String name = doc["name"].as<String>();
    name.trim();
    if (name.length()) presets[idx].name = name;
  }
  if (doc["scheduleEnabled"].is<bool>()) presets[idx].scheduleEnabled = doc["scheduleEnabled"] | presets[idx].scheduleEnabled;
  if (doc["scheduleMinute"].is<int>()) presets[idx].scheduleMinute = (doc["scheduleMinute"] | presets[idx].scheduleMinute) % 1440;
  if (doc["scheduleMode"].is<const char *>()) presets[idx].scheduleMode = doc["scheduleMode"] | presets[idx].scheduleMode;
  if (doc["dayMask"].is<int>()) presets[idx].dayMask = doc["dayMask"] | presets[idx].dayMask;
  if (doc["power"].is<bool>()) presets[idx].power = doc["power"] | presets[idx].power;
  if (doc["mode"].is<const char *>()) presets[idx].mode = doc["mode"] | presets[idx].mode;
  if (doc["fan"].is<const char *>()) presets[idx].fan = doc["fan"] | presets[idx].fan;
  if (doc["degrees"].is<float>() || doc["degrees"].is<int>()) presets[idx].degrees = clampFloat(doc["degrees"] | presets[idx].degrees, 16.0f, 32.0f);
  if (doc["turbo"].is<bool>()) presets[idx].turbo = doc["turbo"] | presets[idx].turbo;
  if (doc["quiet"].is<bool>()) presets[idx].quiet = doc["quiet"] | presets[idx].quiet;
  if (doc["sleep"].is<bool>()) presets[idx].sleep = doc["sleep"] | presets[idx].sleep;
  if (doc["swingV"].is<bool>()) presets[idx].swingV = doc["swingV"] | presets[idx].swingV;
  if (doc["swingH"].is<bool>()) presets[idx].swingH = doc["swingH"] | presets[idx].swingH;
  if (doc["filter"].is<bool>()) presets[idx].filter = doc["filter"] | presets[idx].filter;
  normalizePresetCommand(presets[idx]);
  savePresetLibrary();
  sendOk("preset updated");
}

void handlePresetDeletePost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  uint8_t id = doc["id"] | 0;
  int idx = findPresetIndexById(id);
  if (idx < 0) {
    sendError(404, "Preset not found");
    return;
  }
  for (uint8_t i = idx; i + 1 < presetCount; i++) presets[i] = presets[i + 1];
  presetCount--;
  savePresetLibrary();
  sendOk("preset deleted");
}

void handlePresetSendPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  uint8_t id = doc["id"] | 0;
  int idx = findPresetIndexById(id);
  if (idx < 0) {
    sendError(404, "Preset not found");
    return;
  }
  pendingAc = requestFromPreset(presets[idx]);
  setPendingAcContext("preset", "send", NAN, pendingAc.degrees);
  pendingAction = PendingAction::SendAc;
  sendOk("preset queued");
}

void handleLearnPost() {
  if (!lastCapture.available) {
    sendError(409, "No IR capture available");
    return;
  }
  if (learnedCount >= kMaxLearnedCommands) {
    sendError(409, "Learned command library is full");
    return;
  }
  JsonDocument doc;
  if (!parseBody(doc)) return;
  LearnedCommand &cmd = learned[learnedCount];
  cmd.id = nextLearnedId();
  const char *postedName = doc["name"] | "";
  cmd.name = strlen(postedName) ? String(postedName) : "command " + String(cmd.id);
  cmd.freqKhz = doc["freqKhz"] | 38;
  cmd.hasAcMeta = true;
  cmd.power = doc["power"] | true;
  cmd.mode = doc["mode"] | "cool";
  cmd.fan = doc["fan"] | "auto";
  cmd.degrees = doc["degrees"] | 26.0f;
  copyCaptureToLearned(cmd);
  learnedCount++;
  saveLearnedLibrary();
  sendOk("capture saved");
}

void handleUpdateLearnedPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  uint8_t id = doc["id"] | 0;
  int idx = findLearnedIndexById(id);
  if (idx < 0) {
    sendError(404, "Learned command not found");
    return;
  }

  LearnedCommand &cmd = learned[idx];
  if (doc["name"].is<const char *>()) {
    String name = doc["name"].as<String>();
    name.trim();
    if (name.length()) cmd.name = name;
  }
  if (doc["freqKhz"].is<uint16_t>()) cmd.freqKhz = doc["freqKhz"] | cmd.freqKhz;
  if (doc["power"].is<bool>()) cmd.power = doc["power"] | cmd.power;
  if (doc["mode"].is<const char *>()) cmd.mode = doc["mode"] | cmd.mode;
  if (doc["fan"].is<const char *>()) cmd.fan = doc["fan"] | cmd.fan;
  if (doc["degrees"].is<float>() || doc["degrees"].is<int>()) cmd.degrees = doc["degrees"] | cmd.degrees;
  cmd.hasAcMeta = doc["hasAcMeta"] | cmd.hasAcMeta;

  bool replaceFromCapture = doc["replaceFromCapture"] | false;
  if (replaceFromCapture) {
    if (!lastCapture.available) {
      sendError(409, "No IR capture available");
      return;
    }
    copyCaptureToLearned(cmd);
  }

  saveLearnedLibrary();
  sendOk("learned command updated");
}

void handleSendLearnedPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  pendingLearnedId = doc["id"] | 0;
  pendingAction = PendingAction::SendLearned;
  sendOk("learned command queued");
}

void handleDeleteLearnedPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  uint8_t id = doc["id"] | 0;
  int idx = findLearnedIndexById(id);
  if (idx < 0) {
    sendError(404, "Learned command not found");
    return;
  }
  for (uint8_t i = idx; i + 1 < learnedCount; i++) learned[i] = learned[i + 1];
  learnedCount--;
  saveLearnedLibrary();
  sendOk("deleted");
}

void handleReorderLearnedPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  JsonArray ids = doc["ids"].as<JsonArray>();
  if (ids.isNull()) {
    sendError(400, "ids array is required");
    return;
  }

  uint8_t pos = 0;
  for (JsonVariant value : ids) {
    if (pos >= learnedCount) break;
    uint8_t id = value.as<uint8_t>();
    int idx = findLearnedIndexById(id);
    if (idx < 0 || idx < pos) continue;
    swapLearnedCommands(pos, static_cast<uint8_t>(idx));
    pos++;
  }

  saveLearnedLibrary();
  sendOk("learned commands reordered");
}

void handleCurvePost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  config.autoEnabled = doc["autoEnabled"] | false;
  config.autoMode = doc["autoMode"] | config.autoMode;
  config.curveControlMode = doc["curveControlMode"] | config.curveControlMode;
  config.curveEndAction = doc["curveEndAction"] | config.curveEndAction;
  if (doc["curveHumidityEnabled"].is<bool>()) {
    config.curveHumidityEnabled = doc["curveHumidityEnabled"] | config.curveHumidityEnabled;
  }
  config.quietSwitchMinute = doc["quietSwitchMinute"] | config.quietSwitchMinute;
  normalizeCurveControlConfig();
  config.sleepStartMinute = (doc["sleepStartMinute"] | config.sleepStartMinute) % 1440;
  config.sleepDurationMinute = doc["sleepDurationMinute"] | config.sleepDurationMinute;
  uint16_t requestedInterval = doc["controlIntervalSec"] | config.controlIntervalSec;
  config.controlIntervalSec = requestedInterval < 5 ? 5 : requestedInterval;
  config.deadband = doc["deadband"] | config.deadband;
  config.autoSendDelta = clampFloat(doc["autoSendDelta"] | config.autoSendDelta, 0.0f, 10.0f);
  config.minSetpoint = 16.0f;
  config.maxSetpoint = 32.0f;
  JsonArray curve = doc["curve"].as<JsonArray>();
  if (!curve.isNull()) {
    config.curveCount = 0;
    for (JsonObject point : curve) {
      if (config.curveCount >= kMaxCurvePoints) break;
      config.curve[config.curveCount].minute = point["minute"] | 0;
      config.curve[config.curveCount].temp =
          clampFloat(point["temp"] | 26.0f, config.minSetpoint, config.maxSetpoint);
      config.curveCount++;
    }
    if (config.curveCount == 0) {
      config.curve[0] = {0, 26.0f};
      config.curveCount = 1;
    }
  }
  saveConfig();
  sendOk("curve saved");
}

void applyConfigJson(JsonObject src) {
  if (src["acProtocol"].is<const char *>()) config.acProtocol = strToDecodeType((src["acProtocol"] | "UNKNOWN"));
  if (src["acModel"].is<int>()) config.acModel = src["acModel"] | config.acModel;
  if (src["autoEnabled"].is<bool>()) config.autoEnabled = src["autoEnabled"] | config.autoEnabled;
  if (src["autoMode"].is<const char *>()) config.autoMode = src["autoMode"] | config.autoMode;
  if (src["remotePower"].is<bool>()) config.remotePower = src["remotePower"] | config.remotePower;
  if (src["remoteMode"].is<const char *>()) config.remoteMode = src["remoteMode"] | config.remoteMode;
  if (src["remoteFan"].is<const char *>()) config.remoteFan = src["remoteFan"] | config.remoteFan;
  if (src["remoteDegrees"].is<float>() || src["remoteDegrees"].is<int>()) {
    config.remoteDegrees = clampFloat(src["remoteDegrees"] | config.remoteDegrees, 16.0f, 32.0f);
  }
  if (src["remoteTurbo"].is<bool>()) config.remoteTurbo = src["remoteTurbo"] | config.remoteTurbo;
  if (src["remoteQuiet"].is<bool>()) config.remoteQuiet = src["remoteQuiet"] | config.remoteQuiet;
  if (src["remoteSleep"].is<bool>()) config.remoteSleep = src["remoteSleep"] | config.remoteSleep;
  if (src["remoteSwingV"].is<bool>()) config.remoteSwingV = src["remoteSwingV"] | config.remoteSwingV;
  if (src["remoteSwingH"].is<bool>()) config.remoteSwingH = src["remoteSwingH"] | config.remoteSwingH;
  if (src["remoteFilter"].is<bool>()) config.remoteFilter = src["remoteFilter"] | config.remoteFilter;
  if (src["curveControlMode"].is<const char *>()) config.curveControlMode = src["curveControlMode"] | config.curveControlMode;
  if (src["curveEndAction"].is<const char *>()) config.curveEndAction = src["curveEndAction"] | config.curveEndAction;
  if (src["quietSwitchMinute"].is<int>()) config.quietSwitchMinute = src["quietSwitchMinute"] | config.quietSwitchMinute;
  if (src["sleepStartMinute"].is<int>()) config.sleepStartMinute = (src["sleepStartMinute"] | config.sleepStartMinute) % 1440;
  if (src["sleepDurationMinute"].is<int>()) config.sleepDurationMinute = src["sleepDurationMinute"] | config.sleepDurationMinute;
  if (src["controlIntervalSec"].is<int>()) {
    uint16_t requestedInterval = src["controlIntervalSec"] | config.controlIntervalSec;
    config.controlIntervalSec = requestedInterval < 5 ? 5 : requestedInterval;
  }
  if (src["deadband"].is<float>() || src["deadband"].is<int>()) config.deadband = src["deadband"] | config.deadband;
  if (src["autoSendDelta"].is<float>() || src["autoSendDelta"].is<int>()) {
    config.autoSendDelta = clampFloat(src["autoSendDelta"] | config.autoSendDelta, 0.0f, 10.0f);
  }
  if (src["sensorTempOffset"].is<float>() || src["sensorTempOffset"].is<int>()) config.sensorTempOffset = src["sensorTempOffset"] | config.sensorTempOffset;
  if (src["sensorHumidityOffset"].is<float>() || src["sensorHumidityOffset"].is<int>()) config.sensorHumidityOffset = src["sensorHumidityOffset"] | config.sensorHumidityOffset;
  if (src["humidityControlEnabled"].is<bool>()) config.humidityControlEnabled = src["humidityControlEnabled"] | config.humidityControlEnabled;
  if (src["curveHumidityEnabled"].is<bool>()) config.curveHumidityEnabled = src["curveHumidityEnabled"] | config.curveHumidityEnabled;
  if (src["targetHumidity"].is<float>() || src["targetHumidity"].is<int>()) config.targetHumidity = src["targetHumidity"] | config.targetHumidity;
  if (src["humidityDeadband"].is<float>() || src["humidityDeadband"].is<int>()) config.humidityDeadband = src["humidityDeadband"] | config.humidityDeadband;
  if (src["humidityTargetTemp"].is<float>() || src["humidityTargetTemp"].is<int>()) config.humidityTargetTemp = src["humidityTargetTemp"] | config.humidityTargetTemp;
  if (src["predictiveSkipEnabled"].is<bool>()) config.predictiveSkipEnabled = src["predictiveSkipEnabled"] | config.predictiveSkipEnabled;
  if (src["adaptiveControlEnabled"].is<bool>()) config.adaptiveControlEnabled = src["adaptiveControlEnabled"] | config.adaptiveControlEnabled;
  if (src["learnedFastRate"].is<float>() || src["learnedFastRate"].is<int>()) config.learnedFastRate = src["learnedFastRate"] | config.learnedFastRate;
  if (src["learnedQuietRate"].is<float>() || src["learnedQuietRate"].is<int>()) config.learnedQuietRate = src["learnedQuietRate"] | config.learnedQuietRate;
  if (src["closedLoopFastGain"].is<float>() || src["closedLoopFastGain"].is<int>()) config.closedLoopFastGain = src["closedLoopFastGain"] | config.closedLoopFastGain;
  if (src["closedLoopQuietGain"].is<float>() || src["closedLoopQuietGain"].is<int>()) config.closedLoopQuietGain = src["closedLoopQuietGain"] | config.closedLoopQuietGain;
  if (src["capTurbo"].is<bool>()) config.capTurbo = src["capTurbo"] | config.capTurbo;
  if (src["capQuiet"].is<bool>()) config.capQuiet = src["capQuiet"] | config.capQuiet;
  if (src["capSleep"].is<bool>()) config.capSleep = src["capSleep"] | config.capSleep;
  if (src["capSwingV"].is<bool>()) config.capSwingV = src["capSwingV"] | config.capSwingV;
  if (src["capSwingH"].is<bool>()) config.capSwingH = src["capSwingH"] | config.capSwingH;
  if (src["capFilter"].is<bool>()) config.capFilter = src["capFilter"] | config.capFilter;
  JsonArray curve = src["curve"].as<JsonArray>();
  if (!curve.isNull()) {
    config.curveCount = 0;
    for (JsonObject point : curve) {
      if (config.curveCount >= kMaxCurvePoints) break;
      config.curve[config.curveCount].minute = point["minute"] | 0;
      config.curve[config.curveCount].temp = clampFloat(point["temp"] | 26.0f, 16.0f, 32.0f);
      config.curveCount++;
    }
    if (config.curveCount == 0) {
      config.curve[0] = {0, 26.0f};
      config.curveCount = 1;
    }
  }
  config.minSetpoint = 16.0f;
  config.maxSetpoint = 32.0f;
  normalizeCurveControlConfig();
  normalizeTuningConfig();
  normalizeConfigRemoteState();
}

void handleSettingsPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  bool humidityFieldPresent = doc["humidityControlEnabled"].is<bool>();
  bool wasHumidityEnabled = config.humidityControlEnabled;
  JsonObject src = doc.as<JsonObject>();
  applyConfigJson(src);
  saveConfig();
  if (humidityFieldPresent && wasHumidityEnabled && !config.humidityControlEnabled) {
    pendingAc.power = false;
    pendingAc.mode = config.remoteMode.length() ? config.remoteMode : "auto";
    pendingAc.fan = "auto";
    pendingAc.turbo = false;
    pendingAc.quiet = false;
    pendingAc.sleep = false;
    pendingAc.swingV = false;
    pendingAc.swingH = false;
    pendingAc.filter = false;
    pendingAc.degrees = clampFloat(isnan(lastSentSetpoint) ? config.remoteDegrees : lastSentSetpoint,
                                   config.minSetpoint,
                                   config.maxSetpoint);
    setPendingAcContext("humidity", "off", NAN, pendingAc.degrees);
    pendingAction = PendingAction::SendAc;
    addDecisionLog("humidity_off", "humidity", "独立湿度控制关闭，已排队发送关机指令", roomTempC, NAN, pendingAc.degrees);
  }
  sendOk("settings saved");
}

void handleConfigExport() {
  JsonDocument doc;
  doc["version"] = 1;
  JsonObject cfg = doc["config"].to<JsonObject>();
  addConfigToJson(cfg);
  JsonArray presetArr = doc["presets"].to<JsonArray>();
  for (uint8_t i = 0; i < presetCount; i++) {
    JsonObject item = presetArr.add<JsonObject>();
    item["id"] = presets[i].id;
    item["name"] = presets[i].name;
    item["power"] = presets[i].power;
    item["degrees"] = presets[i].degrees;
    item["mode"] = presets[i].mode;
    item["fan"] = presets[i].fan;
    item["turbo"] = presets[i].turbo;
    item["quiet"] = presets[i].quiet;
    item["sleep"] = presets[i].sleep;
    item["swingV"] = presets[i].swingV;
    item["swingH"] = presets[i].swingH;
    item["filter"] = presets[i].filter;
    item["scheduleEnabled"] = presets[i].scheduleEnabled;
    item["scheduleMinute"] = presets[i].scheduleMinute;
    item["scheduleMode"] = presets[i].scheduleMode;
    item["dayMask"] = presets[i].dayMask;
  }
  String out;
  serializeJson(doc, out);
  server.sendHeader("Content-Disposition", "attachment; filename=ir-ac-config.json");
  server.send(200, "application/json", out);
}

void handleConfigImport() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  JsonObject cfg = doc["config"].as<JsonObject>();
  if (cfg.isNull()) cfg = doc.as<JsonObject>();
  applyConfigJson(cfg);

  JsonArray presetArr = doc["presets"].as<JsonArray>();
  if (!presetArr.isNull()) {
    presetCount = 0;
    for (JsonObject item : presetArr) {
      if (presetCount >= kMaxPresetCommands) break;
      PresetCommand &cmd = presets[presetCount];
      cmd.id = item["id"] | static_cast<uint8_t>(presetCount + 1);
      cmd.name = item["name"] | "";
      cmd.power = item["power"] | true;
      cmd.degrees = clampFloat(item["degrees"] | 26.0f, 16.0f, 32.0f);
      cmd.mode = item["mode"] | "cool";
      cmd.fan = item["fan"] | "auto";
      cmd.turbo = item["turbo"] | false;
      cmd.quiet = item["quiet"] | false;
      cmd.sleep = item["sleep"] | false;
      cmd.swingV = item["swingV"] | false;
      cmd.swingH = item["swingH"] | false;
      cmd.filter = item["filter"] | false;
      cmd.scheduleEnabled = item["scheduleEnabled"] | false;
      cmd.scheduleMinute = (item["scheduleMinute"] | static_cast<uint16_t>(7 * 60)) % 1440;
      cmd.scheduleMode = item["scheduleMode"] | "daily";
      cmd.dayMask = item["dayMask"] | static_cast<uint8_t>(0b0111110);
      normalizePresetCommand(cmd);
      presetCount++;
    }
    savePresetLibrary();
  }
  saveConfig();
  sendOk("config imported");
}

void handleOtaFinish() {
  bool ok = !Update.hasError();
  if (ok) {
    server.send(200, "application/json", "{\"ok\":true,\"message\":\"ota uploaded, restarting\"}");
    delay(300);
    ESP.restart();
  } else {
    server.send(500, "application/json", "{\"ok\":false,\"message\":\"ota failed\"}");
  }
}

void handleOtaUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    irrecv.disableIRIn();
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.abort();
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.end(true);
    irrecv.enableIRIn();
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    irrecv.enableIRIn();
  }
}

void setupRoutes() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", kIndexHtml); });
  server.on("/match", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", kMatchHtml); });
  server.on("/match/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", kMatchHtml); });
  server.on("/remote", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", kRemoteHtml); });
  server.on("/remote/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", kRemoteHtml); });
  server.on("/api/live", HTTP_GET, handleLive);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/temp-history", HTTP_GET, handleTempHistory);
  server.on("/api/control-events", HTTP_GET, handleControlEvents);
  server.on("/api/control-log", HTTP_GET, handleControlLog);
  server.on("/api/wifi-scan", HTTP_GET, handleWifiScan);
  server.on("/api/wifi", HTTP_POST, handleWifiPost);
  server.on("/api/wifi/forget", HTTP_POST, handleWifiForget);
  server.on("/api/ac-config", HTTP_POST, handleAcConfigPost);
  server.on("/api/settings", HTTP_POST, handleSettingsPost);
  server.on("/api/config-export", HTTP_GET, handleConfigExport);
  server.on("/api/config-import", HTTP_POST, handleConfigImport);
  server.on("/api/ota", HTTP_POST, handleOtaFinish, handleOtaUpload);
  server.on("/api/send-ac", HTTP_POST, handleSendAcPost);
  server.on("/api/remote-state", HTTP_POST, handleRemoteStatePost);
  server.on("/api/create-preset", HTTP_POST, handlePresetCreatePost);
  server.on("/api/update-preset", HTTP_POST, handlePresetUpdatePost);
  server.on("/api/delete-preset", HTTP_POST, handlePresetDeletePost);
  server.on("/api/send-preset", HTTP_POST, handlePresetSendPost);
  server.on("/api/learn", HTTP_POST, handleLearnPost);
  server.on("/api/update-learned", HTTP_POST, handleUpdateLearnedPost);
  server.on("/api/send-learned", HTTP_POST, handleSendLearnedPost);
  server.on("/api/delete-learned", HTTP_POST, handleDeleteLearnedPost);
  server.on("/api/reorder-learned", HTTP_POST, handleReorderLearnedPost);
  server.on("/api/curve", HTTP_POST, handleCurvePost);
  server.onNotFound([]() {
    const String uri = server.uri();
    const HTTPMethod method = server.method();
    if (method == HTTP_GET && uri == "/api/status") {
      handleStatus();
      return;
    }
    if (method == HTTP_GET && uri == "/api/live") {
      handleLive();
      return;
    }
    if (method == HTTP_GET && uri == "/api/temp-history") {
      handleTempHistory();
      return;
    }
    if (method == HTTP_GET && uri == "/api/control-events") {
      handleControlEvents();
      return;
    }
    if (method == HTTP_GET && uri == "/api/control-log") {
      handleControlLog();
      return;
    }
    if (method == HTTP_GET && uri == "/api/wifi-scan") {
      handleWifiScan();
      return;
    }
    if (method == HTTP_GET && (uri == "/remote" || uri == "/remote/")) {
      server.send_P(200, "text/html; charset=utf-8", kRemoteHtml);
      return;
    }
    if (method == HTTP_GET && (uri == "/match" || uri == "/match/")) {
      server.send_P(200, "text/html; charset=utf-8", kMatchHtml);
      return;
    }
    if (method == HTTP_POST && uri == "/api/wifi") {
      handleWifiPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/wifi/forget") {
      handleWifiForget();
      return;
    }
    if (method == HTTP_POST && uri == "/api/ac-config") {
      handleAcConfigPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/settings") {
      handleSettingsPost();
      return;
    }
    if (method == HTTP_GET && uri == "/api/config-export") {
      handleConfigExport();
      return;
    }
    if (method == HTTP_POST && uri == "/api/config-import") {
      handleConfigImport();
      return;
    }
    if (method == HTTP_POST && uri == "/api/send-ac") {
      handleSendAcPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/remote-state") {
      handleRemoteStatePost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/create-preset") {
      handlePresetCreatePost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/update-preset") {
      handlePresetUpdatePost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/delete-preset") {
      handlePresetDeletePost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/send-preset") {
      handlePresetSendPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/learn") {
      handleLearnPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/update-learned") {
      handleUpdateLearnedPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/send-learned") {
      handleSendLearnedPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/delete-learned") {
      handleDeleteLearnedPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/reorder-learned") {
      handleReorderLearnedPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/curve") {
      handleCurvePost();
      return;
    }
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(200);
  LittleFS.begin(true);
  loadConfig();
  loadTempHistory();
  loadControlEvents();
  loadLearnedLibrary();
  loadPresetLibrary();

  Wire.begin(kI2cSdaPin, kI2cSclPin);
  sht31Ready = sht31.begin(kSht31Address);
  rawSender.begin();
  irrecv.setUnknownThreshold(kMinUnknownSize);
  irrecv.enableIRIn();

  startWifi();
  setupRoutes();

  Serial.println();
  Serial.println("IR AC Controller started");
  Serial.println("STA SSID: " + config.staSsid);
  if (apStarted) {
    Serial.println("AP: " + String(kApSsid) + " / " + String(kApPassword));
    Serial.println("AP IP: " + WiFi.softAPIP().toString());
  } else {
    Serial.println("AP fallback opens after WiFi connect timeout.");
  }
  Serial.println("IR TX GPIO " + String(kIrTxPin) + ", IR RX GPIO " + String(kIrRxPin));
  Serial.println("SHT31 " + String(sht31Ready ? "ready" : "not found"));
}

void loop() {
  if (apStarted) dnsServer.processNextRequest();
  server.handleClient();
  maintainWifi();
  maintainClock();
  updateSensor();
  maintainTempHistoryPersistence();
  captureIrIfAvailable();
  runAutoControl();
  maintainPresetSchedules();
  handlePendingIr();
}
