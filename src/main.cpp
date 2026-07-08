#include <Arduino.h>
#include <Adafruit_SHT31.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <IRac.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Gree.h>
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
  uint32_t lastCurveEndPoweroffKey = 0;
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
  int16_t model = -1;
  uint16_t bits = 0;
  uint64_t value = 0;
  bool hasAcMeta = false;
  bool metaComplete = false;
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

struct AcProfile {
  uint8_t id = 0;
  String name;
  decode_type_t protocol = decode_type_t::UNKNOWN;
  int16_t model = 1;
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
  bool capTurbo = true;
  bool capQuiet = true;
  bool capSleep = true;
  bool capSwingV = true;
  bool capSwingH = true;
  bool capFilter = true;
};

struct CaptureSnapshot {
  bool available = false;
  decode_type_t protocol = decode_type_t::UNKNOWN;
  uint16_t bits = 0;
  uint64_t value = 0;
  uint16_t stateLen = 0;
  uint8_t state[kStateSizeMax] = {};
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

struct SelfTestReport {
  bool ok = false;
  bool sent = false;
  bool received = false;
  bool protocolOk = false;
  bool decodedState = false;
  bool stateOk = false;
  String message;
  String mismatches;
  AcRequest expectedRequest;
  stdAc::state_t expectedState;
  stdAc::state_t receivedState;
  decode_type_t receivedProtocol = decode_type_t::UNKNOWN;
  uint16_t receivedBits = 0;
  uint64_t receivedValue = 0;
  String receivedSummary;
  String receivedAcDescription;
};

bool snapshotToCommonState(const CaptureSnapshot &snapshot, stdAc::state_t *state, const stdAc::state_t *prev);
AcRequest requestFromCommonStateRaw(const stdAc::state_t &state);
AcRequest requestFromCommonState(const stdAc::state_t &state);

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
constexpr uint8_t kMaxAcProfiles = 6;
AcProfile acProfiles[kMaxAcProfiles];
uint8_t acProfileCount = 0;
uint8_t activeAcProfileId = 1;
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
constexpr uint16_t kControlEventPoints = 500;
constexpr char kControlEventsPath[] = "/control_events.csv";
ControlEvent controlEvents[kControlEventPoints];
uint16_t controlEventHead = 0;
uint16_t controlEventCount = 0;
bool controlEventsUseEpoch = false;
constexpr uint16_t kDecisionLogPoints = 500;
constexpr uint32_t kDecisionLogMaxFileBytes = 256UL * 1024UL;
constexpr char kDecisionLogPath[] = "/decision_log.csv";
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
uint32_t lastAutoModeChangeMs = 0;
char pendingAcSource[12] = "manual";
char pendingAcAction[16] = "send";
float pendingAcTarget = NAN;
float pendingAcSetpoint = NAN;
uint32_t lastControlMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t wifiConnectStartMs = 0;
bool wifiScanActive = false;
String wifiScanCacheNetworks;
uint8_t wifiScanCacheCount = 0;
uint32_t wifiScanCacheMs = 0;
String serialCommandBuffer;
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
      --history-temp: #d94a3a;
      --history-temp-soft: rgba(217, 74, 58, 0.16);
      --history-humidity: #2f80ed;
      --history-target: #805ad5;
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
      --history-temp: #ef5b4f;
      --history-temp-soft: rgba(239, 91, 79, 0.16);
      --history-humidity: #2f80ed;
      --history-target: #7b61ff;
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
    body[data-skin="bluehome"] main > section[data-section-key="status"],
    body[data-skin="bluehome"] main > section[data-section-key="history"],
    body[data-skin="bluehome"] main > section[data-section-key="sleep"],
    body[data-skin="bluehome"] main > section[data-section-key="control"],
    body[data-skin="bluehome"] main > section[data-section-key="log"],
    body[data-skin="bluehome"] main > section[data-section-key="wifi"],
    body[data-skin="bluehome"] main > section[data-section-key="manual"] {
      grid-column: 1 / -1;
    }
    body[data-skin="bluehome"] main > section[data-section-key="settings"] {
      grid-column: 1 / 2;
    }
    body[data-skin="bluehome"] main > section[data-section-key="learn"] {
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
    body[data-skin="bluehome"] .manual-card,
    body[data-skin="bluehome"] .manual-note,
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
    .manual-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
      gap: 12px;
    }
    .manual-card {
      min-width: 0;
      display: grid;
      gap: 9px;
      align-content: start;
      padding: 14px;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: var(--panel);
      box-shadow: var(--inset);
    }
    .manual-card ol,
    .manual-card ul {
      margin: 0;
      padding-left: 18px;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.58;
    }
    .manual-card li + li { margin-top: 5px; }
    .manual-note {
      margin-top: 12px;
      padding: 12px 14px;
      border: 1px solid var(--line);
      border-radius: 14px;
      color: var(--muted);
      line-height: 1.55;
      background: var(--panel);
      box-shadow: var(--inset);
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
    .history-line { fill: none; stroke: var(--history-temp); stroke-width: 2.6; stroke-linecap: round; stroke-linejoin: round; }
    .history-humidity-line { fill: none; stroke: var(--history-humidity); stroke-width: 2.3; stroke-linecap: round; stroke-linejoin: round; }
    .history-target-line { fill: none; stroke: var(--history-target); stroke-width: 2.1; stroke-linecap: round; stroke-linejoin: round; stroke-dasharray: 6 5; }
    .history-target-label { fill: var(--history-target); font-size: 10px; font-weight: 800; paint-order: stroke; stroke: var(--panel-strong); stroke-width: 3px; stroke-linejoin: round; }
    .history-mode-band { opacity: .18; pointer-events: none; }
    .history-mode-band.mode-cool { fill: #2f80ed; }
    .history-mode-band.mode-heat { fill: #ef5b4f; }
    .history-mode-band.mode-dry { fill: #8b5cf6; }
    .history-mode-band.mode-auto { fill: #0a8f72; }
    .history-mode-band.mode-fan { fill: #06a0b5; }
    .history-mode-band.mode-off { fill: #8d8476; }
    .history-mode-label { font-size: 10px; font-weight: 800; fill: var(--muted); paint-order: stroke; stroke: var(--panel-strong); stroke-width: 3px; stroke-linejoin: round; pointer-events: none; }
    .history-fill { fill: var(--history-temp-soft); }
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
    .history-hover-dot { fill: var(--panel-strong); stroke: var(--history-temp); stroke-width: 2; pointer-events: none; }
    .history-hover-dot.humidity { stroke: var(--history-humidity); }
    .history-hover-dot.target { stroke: var(--history-target); stroke-dasharray: 3 2; }
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
        <a class="badge" href="#manual">使用说明</a>
        <div class="skin-switch" aria-label="皮肤选择">
          <button type="button" data-skin-choice="cream" onclick="setSkin('cream')">奶油</button>
          <button type="button" data-skin-choice="bluehome" onclick="setSkin('bluehome')">蓝家居</button>
        </div>
      </div>
    </div>
    <div class="message" id="msg">正在读取状态...</div>
  </header>

  <section data-section-key="status">
    <div class="status-grid">
      <div class="metric"><div class="label">室温</div><div id="temp" class="value">--</div></div>
      <div class="metric"><div class="label">湿度</div><div id="hum" class="value">--</div></div>
      <div class="metric"><div class="label">网络</div><div id="wifi" class="value">--</div></div>
      <div class="metric"><div class="label">最近动作</div><div id="last" class="value">--</div></div>
    </div>
  </section>

  <section data-section-key="control">
    <div class="section-head">
      <div>
        <h2>空调控制</h2>
        <div class="subhead">使用已保存的遥控协议发送；如果协议不匹配，可以在下方保存原始红外命令作为自建库。</div>
      </div>
    </div>
    <div class="form-grid">
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
      <button class="secondary" type="button" onclick="selfTestAc()">自发自收校验</button>
      <button class="secondary" type="button" onclick="applyCaptureState()">套用最近捕获</button>
      <button class="secondary" type="button" onclick="compareAcCode()">对比实体遥控编码</button>
    </div>
    <div class="label" id="selfTestResult">校验会实际发射一次当前红外指令。</div>
    <div class="label" id="codeCompareResult">编码对比前，请先用实体遥控器发送同一组合；“套用最近捕获”不会发射红外，“编码对比”会实际发射一次。</div>
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

  <section data-section-key="learn">
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
          <button class="secondary" type="button" onclick="applyCaptureState()">套用到空调控制</button>
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

  <section data-section-key="sleep">
    <div class="section-head">
      <div>
        <h2>睡眠温度曲线</h2>
        <div class="subhead">曲线按入睡后的分钟数计算目标室温，用于夜间自动微调空调设定。</div>
      </div>
      <div class="header-links"><div class="badge" id="curveTargetBadge">目标 --</div><div class="badge" id="curveBadge">未启用</div></div>
    </div>
    <div class="form-grid">
      <label>自动控制<select id="autoEnabled"><option value="false">关闭</option><option value="true">开启</option></select></label>
      <label>自动模式<select id="autoMode"><option value="smart">智能判定</option><option value="cool">制冷</option><option value="auto">空调自动</option><option value="dry">除湿</option><option value="heat">制热</option><option value="fan">送风</option></select></label>
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

  <section data-section-key="settings">
    <div class="section-head">
      <div>
        <h2>维护与闭环设置</h2>
        <div class="subhead">校准自身传感器、设置库支持能力、调整闭环策略，并支持配置备份和浏览器 OTA。</div>
      </div>
      <div class="badge" id="settingsBadge">待保存</div>
    </div>
    <div class="settings-panel">
      <h3>空调方案</h3>
      <div class="form-grid">
        <label>当前方案<select id="acProfileSelect" onchange="selectAcProfile()"></select></label>
        <label>方案名称<input id="acProfileName" autocomplete="off" placeholder="卧室 MIRAGE / 客厅 GREE"></label>
      </div>
      <div class="mini-actions">
        <button type="button" onclick="saveCurrentAcProfile()">保存到当前方案</button>
        <button class="secondary" type="button" onclick="createAcProfile()">新建方案</button>
        <button class="secondary danger" type="button" onclick="deleteCurrentAcProfile()">删除方案</button>
      </div>
      <h3>遥控协议</h3>
      <div class="form-grid">
        <label>协议<select id="protocol"></select></label>
        <label>型号<input id="model" type="number" min="1" value="1"></label>
      </div>
      <div class="mini-actions">
        <button class="secondary" type="button" onclick="saveAcConfig()">保存空调协议/型号</button>
        <span class="badge" id="protocolBadge">协议读取中</span>
      </div>
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

  <section data-section-key="log">
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

  <section data-section-key="wifi">
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

  <section id="manual" data-section-key="manual">
    <div class="section-head">
      <div>
        <h2>使用说明</h2>
        <div class="subhead">给第一次使用和排查问题时看的快速说明，帮助理解各功能之间的关系。</div>
      </div>
    </div>
    <div class="manual-grid">
      <article class="manual-card">
        <h3>首次使用</h3>
        <ol>
          <li>在维护与闭环设置里确认空调协议和型号，确认后通常不用再改。</li>
          <li>在空调控制里先试一次立即发送，确认空调能响应。</li>
          <li>如果需要远程控制，先在 WiFi 配网里连接家里的路由器。</li>
          <li>如果遥控器协议不完整，再使用红外学习库补充原始按键。</li>
        </ol>
      </article>
      <article class="manual-card">
        <h3>当前状态</h3>
        <ul>
          <li>室温和湿度来自设备上的 SHT31 传感器。</li>
          <li>最近动作显示最后一次发送、跳过或失败的控制结果。</li>
          <li>网络卡片显示设备当前局域网地址，用来访问网页。</li>
        </ul>
      </article>
      <article class="manual-card">
        <h3>72小时曲线</h3>
        <ul>
          <li>红色是室温，蓝色是湿度，每分钟记录一个点。</li>
          <li>鼠标悬停可看当时时间、温度和湿度。</li>
          <li>滚轮缩放横轴，拖动左右平移；手机上可双指缩放。</li>
          <li>曲线上的事件点对应控制事件日志里的发送、跳过和结束事件。</li>
        </ul>
      </article>
      <article class="manual-card">
        <h3>睡眠温度曲线</h3>
        <ul>
          <li>开启后只在设定的开始时间和结束时间之间运行。</li>
          <li>目标温度由曲线按时间计算，系统再决定空调模式、设定温度和风速。</li>
          <li>急速阶段优先快速接近目标；安静时间到达后必须进入静音风速。</li>
          <li>结束模式选择关机时，同一个睡眠周期只会执行一次关机。</li>
        </ul>
      </article>
      <article class="manual-card">
        <h3>空调控制</h3>
        <ul>
          <li>这里用于手动发送当前电源、模式、温度、风量和特殊功能。</li>
          <li>发送成功后会记忆为模拟遥控器状态，下次打开页面会继承。</li>
          <li>快捷指令会保存当前组合，可手动点击，也可设置定时发送。</li>
          <li>特殊功能里的强劲、静音、睡眠是互斥模式，一次只选择一种。</li>
        </ul>
      </article>
      <article class="manual-card">
        <h3>湿度控制</h3>
        <ul>
          <li>独立湿度控制不受睡眠开始时间限制，打开后会持续判断湿度。</li>
          <li>湿度高于目标加死区时，优先使用除湿模式；室温偏低时会先控温。</li>
          <li>睡眠曲线运行期间，曲线除湿开关决定曲线内是否参与除湿。</li>
          <li>关闭独立湿度控制时，系统会自动发送一次关机指令。</li>
        </ul>
      </article>
      <article class="manual-card">
        <h3>事件日志</h3>
        <ul>
          <li>日志记录手动、自动曲线、湿度控制、快捷指令和系统结束事件。</li>
          <li>发送表示已经发出红外；跳过通常表示死区、过滤阈值或趋势预测生效。</li>
          <li>排查为什么没有发射信号时，优先看这里的来源和类型筛选。</li>
        </ul>
      </article>
      <article class="manual-card">
        <h3>维护设置</h3>
        <ul>
          <li>协议和型号是低频配置，只有更换空调库匹配结果时才需要修改。</li>
          <li>传感器校准用于修正设备摆放位置造成的温湿度偏差。</li>
          <li>空调库能力用于关闭当前协议不支持的强劲、静音、摆风等功能。</li>
          <li>导出配置可备份设置；导入配置和 OTA 升级用于维护设备。</li>
        </ul>
      </article>
      <article class="manual-card">
        <h3>红外学习与配网</h3>
        <ul>
          <li>红外学习库保存实体遥控器的原始红外数据，可作为协议库不完整时的补充。</li>
          <li>对码检查用于比较实体遥控器和当前协议库的按键组合是否一致。</li>
          <li>WiFi 配网支持扫描热点，保存后设备优先连接路由器，失败时保留热点模式。</li>
        </ul>
      </article>
    </div>
    <div class="manual-note">
      自动曲线、独立湿度、快捷定时和手动控制共用同一个红外发射器和模拟遥控器状态。某些情况下没有再次发射信号，并不一定是故障，可能是温度死区、发送过滤、趋势预测或同一睡眠周期结束关机保护正在生效。
    </div>
  </section>
</main>
<script>
let state = {};
let refreshInFlight = false;
let liveRefreshInFlight = false;
const $ = id => document.getElementById(id);
const dirty = new Set();
const guardedIds = ['staSsid','staPassword','acProfileName','protocol','model','power','mode','degrees','fan','specialMode','swingV','swingH','filterFlag','learnName','learnFreq','learnPower','learnMode','learnDegrees','learnFan','autoEnabled','autoMode','curveControlMode','curveEndAction','curveHumidityEnabled','quietSwitchMinute','sleepStart','sleepDuration','controlInterval','deadband','autoSendDelta','curve','sensorTempOffset','sensorHumidityOffset','humidityControlEnabled','targetHumidity','humidityDeadband','humidityTargetTemp','predictiveSkipEnabled','adaptiveControlEnabled','closedLoopFastGain','closedLoopQuietGain','capTurbo','capQuiet','capSleep','capSwingV','capSwingH','capFilter'];
const curveView = {w:720, h:280, l:44, r:14, t:10, b:30};
const historyView = {w:720, h:260, l:50, r:52, t:18, b:38};
let tempHistoryRefreshInFlight = false;
let controlLogRefreshInFlight = false;
const tempHistoryState = {samples:[], events:[], modeEvents:[], usesEpoch:false, eventsUseEpoch:false, modeEventsUseEpoch:false, start:null, end:null, minTemp:0, maxTemp:0, followLatest:true, hover:null};
const controlLogState = {logs:[], usesEpoch:false, capacity:0, persistent:false};
const tempHistoryPinch = {active:false, startDistance:0, startSpan:0, anchor:0, ratio:0.5};
const tempHistoryDrag = {active:false, pointerId:null, startClientX:0, startSvgX:0, startStart:0, startEnd:0, moved:false};
let curveRenderRange = null;
let curveAutoSaveTimer = 0;
let curveAutoSaveInFlight = false;
let curveAutoSavePending = false;
let remoteStateSaveTimer = 0;
const openPresetSchedules = new Set();
let apiQueue = Promise.resolve();
function wait(ms){ return new Promise(resolve => setTimeout(resolve, ms)); }
function cleanFetchErrorText(text, label='数据', status=0){
  const raw = String(text || '').trim();
  if (raw.startsWith('{')) {
    try {
      const parsed = JSON.parse(raw);
      if (parsed && parsed.message) return String(parsed.message);
    } catch(e) {}
  }
  if (!raw) return `${label}获取失败${status ? `（HTTP ${status}）` : ''}`;
  if (raw.length > 180 || /<html|<!doctype|stack|trace|exception|function\\s|void\\s|#include/i.test(raw)) {
    return `${label}获取失败${status ? `（HTTP ${status}）` : ''}`;
  }
  return raw;
}
async function fetchTextSafe(url, options={}, label='数据', retries=1, timeoutMs=22000, serial=true){
  const run = async () => {
    let lastError = null;
    for (let attempt = 0; attempt <= retries; attempt++) {
      const controller = window.AbortController ? new AbortController() : null;
      const timer = controller ? setTimeout(() => controller.abort(), timeoutMs) : 0;
      try {
        const r = await fetch(url, Object.assign({}, options, controller ? {signal:controller.signal} : {}));
        const text = await r.text();
        if (!r.ok) throw new Error(cleanFetchErrorText(text, label, r.status));
        if (!text.trim()) throw new Error(`${label}返回为空`);
        return text;
      } catch(e) {
        lastError = e;
        if (attempt < retries) await wait(450 + attempt * 350);
      } finally {
        if (timer) clearTimeout(timer);
      }
    }
    throw lastError || new Error(`${label}获取失败`);
  };
  if (!serial) return run();
  const job = apiQueue.then(run, run);
  apiQueue = job.catch(() => {});
  return job;
}
async function fetchJsonSafe(url, options={}, label='数据', retries=1, timeoutMs=22000, serial=true){
  const text = await fetchTextSafe(url, options, label, retries, timeoutMs, serial);
  try {
    return JSON.parse(text);
  } catch(e) {
    throw new Error(`${label}JSON不完整，已放弃本次刷新`);
  }
}
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
  const statusSection = document.querySelector('main > section');
  if (!statusSection) return;
  statusSection.insertAdjacentHTML('afterend', `
    <section data-section-key="history">
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
function orderMainSections(){
  const main = document.querySelector('main');
  if (!main) return;
  const desiredKeys = ['status','history','sleep','control','log','settings','learn','wifi','manual'];
  const titleKeys = {
    '当前状态':'status',
    '72小时温湿度曲线':'history',
    '睡眠温度曲线':'sleep',
    '空调控制':'control',
    '控制事件日志':'log',
    '维护与闭环设置':'settings',
    '红外学习库':'learn',
    'WiFi 配网':'wifi',
    '使用说明':'manual'
  };
  const sectionTitle = section => {
    const h2 = section.querySelector(':scope > .section-head h2');
    if (h2) return h2.textContent.trim();
    if (section.querySelector(':scope > .status-grid')) return '当前状态';
    return '';
  };
  const sections = Array.from(main.querySelectorAll(':scope > section'));
  const ranked = sections.map((section, index) => {
    const key = section.dataset.sectionKey || titleKeys[sectionTitle(section)] || '';
    if (key) section.dataset.sectionKey = key;
    const order = desiredKeys.indexOf(key);
    return {section, index, order: order >= 0 ? order : desiredKeys.length + index};
  });
  ranked.sort((a, b) => a.order - b.order || a.index - b.index);
  ranked.forEach(item => main.appendChild(item.section));
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
const modeText = {smart:'智能', cool:'制冷', auto:'空调自动', dry:'除湿', heat:'制热', fan:'送风'};
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
function historyDateForMinute(minute, usesEpoch, latestMinute){
  if (usesEpoch) return new Date(minute * 60000);
  return new Date(Date.now() - Math.max(0, latestMinute - minute) * 60000);
}
function historyMinuteFromDate(date, usesEpoch, latestMinute){
  if (usesEpoch) return date.getTime() / 60000;
  return latestMinute - (Date.now() - date.getTime()) / 60000;
}
function formatHistoryLabel(minute, usesEpoch, latestMinute){
  const d = historyDateForMinute(minute, usesEpoch, latestMinute);
  return `${String(d.getMonth() + 1).padStart(2,'0')}/${String(d.getDate()).padStart(2,'0')} ${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}`;
}
function normalizeCurveForHistory(raw, duration){
  const range = curveRange();
  duration = clamp(Math.round(Number(duration) || 480), 30, 1439);
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
function sleepCurveHistoryConfig(){
  const cfg = state.config || {};
  const startMinute = $('sleepStart')
    ? hhmmToMin($('sleepStart').value)
    : Number(cfg.sleepStartMinute ?? 23 * 60) % 1440;
  let duration = NaN;
  if ($('sleepStart') && $('sleepDuration')) {
    duration = minutesBetween(hhmmToMin($('sleepStart').value), hhmmToMin($('sleepDuration').value), true);
  }
  if (!Number.isFinite(duration) || duration <= 0) duration = Number(cfg.sleepDurationMinute);
  duration = clamp(Math.round(Number(duration) || 480), 30, 1439);
  let rawCurve = null;
  try {
    if ($('curve')) rawCurve = JSON.parse($('curve').value);
  } catch(e) {
    rawCurve = null;
  }
  if (!Array.isArray(rawCurve)) rawCurve = cfg.curve;
  return {startMinute, duration, points: normalizeCurveForHistory(rawCurve, duration)};
}
function targetTempFromCurvePoints(points, elapsedMinute){
  if (!points || !points.length) return null;
  const elapsed = Number(elapsedMinute);
  if (!Number.isFinite(elapsed)) return null;
  if (elapsed <= points[0].minute) return points[0].temp;
  for (let i = 1; i < points.length; i++) {
    if (elapsed <= points[i].minute) {
      const a = points[i - 1];
      const b = points[i];
      const ratio = (elapsed - a.minute) / Math.max(1, b.minute - a.minute);
      return a.temp + (b.temp - a.temp) * ratio;
    }
  }
  return points[points.length - 1].temp;
}
function sleepTargetForHistoryMinute(minute, usesEpoch, latestMinute, cfg=null){
  cfg = cfg || sleepCurveHistoryConfig();
  const d = historyDateForMinute(minute, usesEpoch, latestMinute);
  const localMinute = d.getHours() * 60 + d.getMinutes() + d.getSeconds() / 60;
  const elapsed = (localMinute + 1440 - cfg.startMinute) % 1440;
  if (elapsed > cfg.duration) return null;
  return targetTempFromCurvePoints(cfg.points, elapsed);
}
function historySleepTargetPoints(samples, latestMinute){
  const cfg = sleepCurveHistoryConfig();
  return (samples || []).map(p => ({
    minute: p.minute,
    target: sleepTargetForHistoryMinute(p.minute, tempHistoryState.usesEpoch, latestMinute, cfg)
  }));
}
function historySleepTargetLabels(start, end, minTemp, maxTemp, latestMinute){
  const cfg = sleepCurveHistoryConfig();
  const dayStart = historyDateForMinute(start, tempHistoryState.usesEpoch, latestMinute);
  const dayEnd = historyDateForMinute(end, tempHistoryState.usesEpoch, latestMinute);
  dayStart.setHours(0, 0, 0, 0);
  dayEnd.setHours(0, 0, 0, 0);
  dayStart.setDate(dayStart.getDate() - 1);
  dayEnd.setDate(dayEnd.getDate() + 1);
  const labels = [];
  let lastX = -999;
  for (const day = new Date(dayStart); day <= dayEnd; day.setDate(day.getDate() + 1)) {
    cfg.points.forEach(point => {
      const d = new Date(day);
      d.setMinutes(cfg.startMinute + point.minute, 0, 0);
      const minute = historyMinuteFromDate(d, tempHistoryState.usesEpoch, latestMinute);
      if (minute < start || minute > end) return;
      const x = historyX(minute, start, end);
      if (Math.abs(x - lastX) < 30) return;
      lastX = x;
      const y = clamp(historyY(point.temp, minTemp, maxTemp) - 8, historyView.t + 10, historyView.h - historyView.b - 8);
      labels.push(`<circle class="history-hover-dot target" cx="${x.toFixed(1)}" cy="${historyY(point.temp, minTemp, maxTemp).toFixed(1)}" r="3.6"></circle><text class="history-target-label" x="${x.toFixed(1)}" y="${y.toFixed(1)}" text-anchor="middle">${point.temp.toFixed(1)}℃</text>`);
    });
  }
  return labels.join('');
}
function normalizeControlEvents(data){
  const events = Array.isArray(data?.events) ? data.events : [];
  return events.map(e => ({
    minute:Number(e.minute),
    source:e.source,
    action:e.action,
    mode:e.mode,
    fan:e.fan,
    setpoint:Number(e.setpoint),
    target:Number(e.target),
    room:Number(e.room),
    power:!!e.power,
    turbo:!!e.turbo,
    quiet:!!e.quiet,
    sleep:!!e.sleep
  })).filter(e => Number.isFinite(e.minute));
}
function usableModeEvent(event){
  const action = String(event?.action || '').toLowerCase();
  return event && Number.isFinite(event.minute) && !action.includes('failed');
}
function modeBandClass(event){
  if (!event?.power) return 'mode-off';
  const mode = String(event.mode || '').toLowerCase();
  if (mode === 'heat') return 'mode-heat';
  if (mode === 'dry') return 'mode-dry';
  if (mode === 'auto' || mode === 'smart') return 'mode-auto';
  if (mode === 'fan') return 'mode-fan';
  return 'mode-cool';
}
function modeBandText(event){
  if (!event?.power) return '关机';
  return modeText[event.mode] || event.mode || '制冷';
}
function historyModeSummary(event){
  if (!event) return '';
  if (!event.power) return '模式底纹：关机';
  const mode = modeText[event.mode] || event.mode || '--';
  const fan = fanText[event.fan] || event.fan || '--';
  const setpoint = Number(event.setpoint);
  const setpointText = Number.isFinite(setpoint) ? ` · 设定 ${setpoint.toFixed(1)}℃` : '';
  return `模式底纹：${mode} / ${fan}${setpointText}`;
}
function historyActiveModeEvent(minute){
  if (tempHistoryState.modeEventsUseEpoch !== tempHistoryState.usesEpoch) return null;
  let active = null;
  tempHistoryState.modeEvents.forEach(e => {
    if (!usableModeEvent(e) || e.minute > minute) return;
    if (!active || e.minute >= active.minute) active = e;
  });
  return active;
}
function historyModeBands(start, end){
  if (tempHistoryState.modeEventsUseEpoch !== tempHistoryState.usesEpoch) return '';
  const events = tempHistoryState.modeEvents
    .filter(e => usableModeEvent(e) && e.minute <= end)
    .sort((a, b) => a.minute - b.minute);
  let active = null;
  let cursor = start;
  const parts = [];
  events.forEach(e => {
    if (e.minute < start) {
      active = e;
      return;
    }
    if (active && e.minute > cursor) {
      const x1 = historyX(cursor, start, end);
      const x2 = historyX(e.minute, start, end);
      const w = Math.max(0, x2 - x1);
      if (w >= 2) {
        const label = w > 46 ? `<text class="history-mode-label" x="${(x1 + w / 2).toFixed(1)}" y="${(historyView.t + 16).toFixed(1)}" text-anchor="middle">${modeBandText(active)}</text>` : '';
        parts.push(`<rect class="history-mode-band ${modeBandClass(active)}" x="${x1.toFixed(1)}" y="${historyView.t}" width="${w.toFixed(1)}" height="${(historyView.h - historyView.t - historyView.b).toFixed(1)}"></rect>${label}`);
      }
    }
    active = e;
    cursor = Math.max(start, e.minute);
  });
  if (active && cursor < end) {
    const x1 = historyX(cursor, start, end);
    const x2 = historyX(end, start, end);
    const w = Math.max(0, x2 - x1);
    if (w >= 2) {
      const label = w > 46 ? `<text class="history-mode-label" x="${(x1 + w / 2).toFixed(1)}" y="${(historyView.t + 16).toFixed(1)}" text-anchor="middle">${modeBandText(active)}</text>` : '';
      parts.push(`<rect class="history-mode-band ${modeBandClass(active)}" x="${x1.toFixed(1)}" y="${historyView.t}" width="${w.toFixed(1)}" height="${(historyView.h - historyView.t - historyView.b).toFixed(1)}"></rect>${label}`);
    }
  }
  return parts.join('');
}
function normalizeHistorySamples(data){
  if (!data) return [];
  if (Array.isArray(data.samples)) {
    return data.samples.map(p => {
      const humidity = Number(p.humidity);
      return {minute:Number(p.minute), temp:Number(p.temp), humidity:Number.isFinite(humidity) ? humidity : null};
    }).filter(p => Number.isFinite(p.minute) && Number.isFinite(p.temp));
  }
  const minutes = Array.isArray(data.m) ? data.m : (Array.isArray(data.minutes) ? data.minutes : []);
  const temps = Array.isArray(data.t) ? data.t : (Array.isArray(data.temps10) ? data.temps10 : []);
  const humidities = Array.isArray(data.h) ? data.h : (Array.isArray(data.humidity10) ? data.humidity10 : []);
  const count = Math.min(minutes.length, temps.length);
  const out = [];
  for (let i = 0; i < count; i++) {
    const minute = Number(minutes[i]);
    const temp10 = Number(temps[i]);
    const humidityRaw = humidities[i];
    const humidity10 = humidityRaw === null || humidityRaw === undefined || humidityRaw === '' ? NaN : Number(humidityRaw);
    if (!Number.isFinite(minute) || !Number.isFinite(temp10)) continue;
    out.push({
      minute,
      temp: temp10 / 10,
      humidity: Number.isFinite(humidity10) ? humidity10 / 10 : null
    });
  }
  return out;
}
function renderTempHistory(data=null){
  const svg = $('tempHistorySvg');
  if (!svg) return;
  if (data) {
    tempHistoryState.samples = normalizeHistorySamples(data);
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
  const targetPoints = historySleepTargetPoints(visible, latest);
  const targetTemps = targetPoints.map(p => Number(p.target)).filter(Number.isFinite);
  const temps = viewSamples.map(p => p.temp).concat(targetTemps);
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
  const targetPath = linePath(targetPoints, 'target', minTemp, maxTemp);
  const targetLabels = historySleepTargetLabels(start, end, minTemp, maxTemp, latest);
  const modeBands = historyModeBands(start, end);
  const fill = path ? `${path} L ${historyX(visible[visible.length - 1].minute, start, end).toFixed(1)} ${baseY} L ${historyX(visible[0].minute, start, end).toFixed(1)} ${baseY} Z` : '';
  const xTicks = [0, 0.25, 0.5, 0.75, 1].map(v => Math.round(start + (end - start) * v));
  const yTicks = [];
  for (let t = Math.ceil(minTemp); t <= Math.floor(maxTemp); t++) yTicks.push(t);
  const humidityTicks = historyTicks(minHumidity, maxHumidity, 5);
  const grid = [
    ...xTicks.map(m => `<line class="curve-grid" x1="${historyX(m,start,end).toFixed(1)}" y1="${historyView.t}" x2="${historyX(m,start,end).toFixed(1)}" y2="${baseY}"></line><text class="curve-label" x="${historyX(m,start,end).toFixed(1)}" y="${historyView.h - 12}" text-anchor="middle">${formatHistoryLabel(m, tempHistoryState.usesEpoch, latest)}</text>`),
    ...yTicks.map(t => `<line class="curve-grid" x1="${historyView.l}" y1="${historyY(t,minTemp,maxTemp).toFixed(1)}" x2="${historyView.w - historyView.r}" y2="${historyY(t,minTemp,maxTemp).toFixed(1)}"></line><text class="curve-label" x="8" y="${(historyY(t,minTemp,maxTemp) + 4).toFixed(1)}" style="fill:var(--history-temp)">${t}℃</text>`),
    ...humidityTicks.map(h => `<text class="curve-label" x="${historyView.w - 8}" y="${(historyY(h,minHumidity,maxHumidity) + 4).toFixed(1)}" text-anchor="end" style="fill:var(--history-humidity)">${Math.round(h)}%</text>`)
  ].join('');
  const last = samples[samples.length - 1];
  let hover = '';
  if (tempHistoryState.hover && tempHistoryState.hover.minute >= start && tempHistoryState.hover.minute <= end) {
    const hx = historyX(tempHistoryState.hover.minute, start, end).toFixed(1);
    const hy = historyY(tempHistoryState.hover.temp, minTemp, maxTemp).toFixed(1);
    const hh = Number(tempHistoryState.hover.humidity);
    const ht = Number(tempHistoryState.hover.target);
    const humidityDot = Number.isFinite(hh)
      ? `<circle class="history-hover-dot humidity" cx="${hx}" cy="${historyY(hh, minHumidity, maxHumidity).toFixed(1)}" r="4.5"></circle>`
      : '';
    const targetDot = Number.isFinite(ht)
      ? `<circle class="history-hover-dot target" cx="${hx}" cy="${historyY(ht, minTemp, maxTemp).toFixed(1)}" r="4.5"></circle>`
      : '';
    hover = `<line class="history-hover-line" x1="${hx}" y1="${historyView.t}" x2="${hx}" y2="${baseY}"></line><circle class="history-hover-dot" cx="${hx}" cy="${hy}" r="5"></circle>${humidityDot}${targetDot}`;
  }
  let lastNode = '';
  if (last.minute >= start && last.minute <= end) {
    const lx = historyX(last.minute,start,end).toFixed(1);
    lastNode = `<circle cx="${lx}" cy="${historyY(last.temp,minTemp,maxTemp).toFixed(1)}" r="5" fill="var(--history-temp)"></circle>`;
    if (Number.isFinite(Number(last.humidity))) lastNode += `<circle cx="${lx}" cy="${historyY(Number(last.humidity),minHumidity,maxHumidity).toFixed(1)}" r="4.5" fill="var(--history-humidity)"></circle>`;
  }
  const eventMarkers = tempHistoryState.eventsUseEpoch === tempHistoryState.usesEpoch
    ? filteredControlLogs(tempHistoryState.events).filter(e => e.minute >= start && e.minute <= end).map(e => {
      const x = historyX(e.minute, start, end).toFixed(1);
      return `<line class="history-event-line" x1="${x}" y1="${historyView.t}" x2="${x}" y2="${baseY}"></line><circle class="history-event-dot" cx="${x}" cy="${historyView.t + 10}" r="4"></circle>`;
    }).join('')
    : '';
  const legend = `<text class="curve-label" x="${historyView.l}" y="12" style="fill:var(--history-temp)">室温 ℃</text><text class="curve-label" x="${historyView.l + 76}" y="12" style="fill:var(--history-target)">睡眠目标 ℃</text><text class="curve-label" x="${historyView.w - historyView.r}" y="12" text-anchor="end" style="fill:var(--history-humidity)">湿度 %</text>`;
  svg.innerHTML = `${modeBands}${grid}${legend}<line class="curve-axis" x1="${historyView.l}" y1="${baseY}" x2="${historyView.w - historyView.r}" y2="${baseY}"></line><line class="curve-axis" x1="${historyView.l}" y1="${historyView.t}" x2="${historyView.l}" y2="${baseY}"></line><line class="curve-axis" x1="${historyView.w - historyView.r}" y1="${historyView.t}" x2="${historyView.w - historyView.r}" y2="${baseY}"></line>${fill ? `<path class="history-fill" d="${fill}"></path><path class="history-line" d="${path}"></path>` : ''}${humidityPath ? `<path class="history-humidity-line" d="${humidityPath}"></path>` : ''}${targetPath ? `<path class="history-target-line" d="${targetPath}"></path>${targetLabels}` : ''}${eventMarkers}${lastNode}${hover}`;
  if ($('tempHistoryBadge')) $('tempHistoryBadge').textContent = `${samples.length} / 4320 点`;
  if ($('tempHistoryInfo')) {
    const spanHours = Math.max(1, Math.round((end - start) / 60));
    const lastHumidity = Number(last.humidity);
    const humidityText = Number.isFinite(lastHumidity) ? ` / ${lastHumidity.toFixed(0)}%` : '';
    $('tempHistoryInfo').textContent = `最近 ${last.temp.toFixed(1)} ℃${humidityText}，当前视窗约 ${spanHours} 小时；红色室温，蓝色湿度，紫色虚线睡眠目标，底纹表示空调运行模式。`;
  }
}
async function refreshTempHistory(){
  if (tempHistoryRefreshInFlight || !$('tempHistorySvg')) return;
  tempHistoryRefreshInFlight = true;
  try {
    const history = await fetchJsonSafe('/api/temp-history', {}, '温湿度历史', 1, 30000);
    try {
      const eventData = await fetchJsonSafe('/api/control-log', {}, '事件日志', 1, 30000);
      history.events = eventData.logs || [];
      history.eventsUseEpoch = !!eventData.usesEpoch;
      controlLogState.logs = eventData.logs || [];
      controlLogState.usesEpoch = !!eventData.usesEpoch;
      controlLogState.capacity = Number(eventData.capacity || 0);
      controlLogState.persistent = !!eventData.persistent;
    } catch(e) {}
    try {
      const modeData = await fetchJsonSafe('/api/control-events', {}, '控制事件', 1, 30000);
      tempHistoryState.modeEvents = normalizeControlEvents(modeData);
      tempHistoryState.modeEventsUseEpoch = !!modeData.usesEpoch;
    } catch(e) {}
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
  if (!best) return null;
  const latest = tempHistoryState.samples[tempHistoryState.samples.length - 1]?.minute || best.minute;
  return Object.assign({}, best, {
    target: sleepTargetForHistoryMinute(best.minute, tempHistoryState.usesEpoch, latest)
  });
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
  const target = Number(point.target);
  const targetText = Number.isFinite(target) ? `<span>睡眠目标 ${target.toFixed(1)} ℃</span>` : '';
  const modeTextLine = historyModeSummary(historyActiveModeEvent(point.minute));
  const modeTextHtml = modeTextLine ? `<span>${modeTextLine}</span>` : '';
  tip.innerHTML = `<strong>${point.temp.toFixed(1)} ℃${humidityText}</strong><span>${formatHistoryLabel(point.minute, tempHistoryState.usesEpoch, tempHistoryState.samples[tempHistoryState.samples.length - 1]?.minute || point.minute)}</span>${targetText}${modeTextHtml}${historyEventText(event)}`;
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
  if ($('tempHistorySvg')) renderTempHistory();
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
  await fetchTextSafe(url, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)}, '操作', 1, 30000);
  clearDirty(clearIds);
  msg(okText || '已完成');
  await refresh(true);
}
function scheduleRemoteStateSave(){
  clearTimeout(remoteStateSaveTimer);
  remoteStateSaveTimer = setTimeout(async () => {
    try {
      await fetchTextSafe('/api/remote-state', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(remoteCommandPayload())}, '遥控状态保存', 0, 12000);
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
    '状态字节：' + (capture.stateHex || '--'),
    'Raw 长度：' + (capture.rawLen ?? '--'),
    '解析状态：' + formatAcRequestBrief(capture.request),
    '空调解析：' + (capture.acDescription || '未识别为空调协议'),
    '摘要：' + (capture.summary || '--')
  ];
  return lines.join('\n');
}
function formatAcRequestBrief(req){
  if (!req) return '--';
  const mode = modeText[req.mode] || req.mode || '--';
  const fan = fanText[req.fan] || req.fan || '--';
  const specials = enabledSpecialNames(req);
  return `${req.power ? '开机' : '关机'} / ${mode} / ${Number(req.degrees).toFixed(1)} ℃ / ${fan}${specials.length ? ' / ' + specials.join('、') : ''}`;
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
    skip_end_once:'结束已执行',
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
  const capacity = Number(controlLogState.capacity || 0);
  const capText = capacity > 0 ? ` / 上限 ${capacity}` : '';
  if (!allLogs.length) {
    box.innerHTML = '<div class="empty">暂无控制事件</div>';
    if ($('controlLogBadge')) $('controlLogBadge').textContent = `0 条${capText}`;
    return;
  }
  if (!filtered.length) {
    box.innerHTML = '<div class="empty">当前筛选下暂无事件</div>';
    if ($('controlLogBadge')) $('controlLogBadge').textContent = `0 / ${allLogs.length} 条${capText}`;
    return;
  }
  box.innerHTML = filtered.slice().reverse().map(item => {
    const hasNumber = value => value !== null && value !== undefined && value !== '' && Number.isFinite(Number(value));
    const temps = [
      hasNumber(item.room) ? `室温 ${Number(item.room).toFixed(1)}℃` : '',
      hasNumber(item.target) ? `目标 ${Number(item.target).toFixed(1)}℃` : '',
      hasNumber(item.setpoint) ? `设定 ${Number(item.setpoint).toFixed(1)}℃` : ''
    ].filter(Boolean).join(' · ');
    const source = logSource(item);
    const type = logType(item);
    const head = `${logSourceText(source)} · ${logTypeText(type)} · ${actionText(item.action)}`;
    const stage = stageText(item.stage || item.source);
    return `<div class="log-item"><strong>${formatLogMinute(item.minute)}</strong><div>${esc(head)}<br><span class="label">${esc(stage)} · ${esc(temps || '--')}</span><br>${esc(item.note || '')}</div></div>`;
  }).join('');
  if ($('controlLogBadge')) $('controlLogBadge').textContent = `${filtered.length} / ${allLogs.length} 条${capText}`;
}
async function refreshControlLog(){
  if (controlLogRefreshInFlight || !$('controlLog')) return;
  controlLogRefreshInFlight = true;
  try {
    const data = await fetchJsonSafe('/api/control-log', {}, '事件日志', 1, 30000);
    controlLogState.usesEpoch = !!data.usesEpoch;
    controlLogState.capacity = Number(data.capacity || 0);
    controlLogState.persistent = !!data.persistent;
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
    const targetText = data.autoTarget.temperatureC !== null && data.autoTarget.temperatureC !== undefined && Number.isFinite(Number(data.autoTarget.temperatureC))
      ? `${Number(data.autoTarget.temperatureC).toFixed(1)} ℃`
      : '--';
    const setpointText = data.autoTarget.setpointC !== null && data.autoTarget.setpointC !== undefined && Number.isFinite(Number(data.autoTarget.setpointC))
      ? `${Number(data.autoTarget.setpointC).toFixed(1)} ℃`
      : '--';
    const humidityPart = data.autoTarget.humidityActive && Number.isFinite(Number(data.humidity))
      ? ` · 湿度 ${Number(data.humidity).toFixed(0)}% / 目标 ${Number(data.autoTarget.targetHumidity).toFixed(0)}%`
      : '';
    const elapsedPart = data.autoTarget.curveActive ? ` / +${data.autoTarget.elapsedMinute} 分` : ' / 湿度控制';
    $('curveTargetBadge').textContent = data.autoTarget.active
      ? `室温 ${room} · 目标 ${targetText} · 设定 ${setpointText} / ${mode}${humidityPart}${elapsedPart}`
      : '目标 -- / 曲线未生效';
  }
  $('captureBadge').textContent = data.capture && data.capture.available ? '已捕获' : '等待信号';
  $('capture').textContent = captureText(data.capture);
}
async function refreshLive(){
  if (liveRefreshInFlight || refreshInFlight) return;
  liveRefreshInFlight = true;
  try {
    const live = await fetchJsonSafe('/api/live', {}, '实时状态', 1, 12000);
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
    state = await fetchJsonSafe('/api/status', {}, '完整状态', 1, 45000);
    applyLive(state);
    $('protocolBadge').textContent = state.config && state.config.acProtocol ? (`当前 ${state.config.acProtocol} / 型号 ${state.config.acModel || 1}`) : '协议未配置';
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
    renderAcProfiles(state.config, force);
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
    const data = await fetchJsonSafe('/api/wifi-scan', {}, '热点扫描', 0, 25000);
    renderWifiScan(data.networks || []);
    if (data.cached) {
      const age = Number(data.ageSec || 0);
      msg(`实时扫描失败，显示约 ${age} 秒前缓存：${data.count || 0} 个热点`, true);
    } else {
      msg(`扫描完成：${data.count || 0} 个热点`);
    }
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
    await post('/api/ac-config', {protocol:$('protocol').value, model:Number($('model').value)}, '空调协议/型号已保存', ['protocol','model']);
  } catch(e) { msg(e.message || '配置保存失败', true); }
}
function currentAcProfilePayload(){
  return Object.assign(remoteCommandPayload(), {
    id:Number($('acProfileSelect')?.value || (state.config && state.config.activeAcProfileId) || 1),
    name:($('acProfileName')?.value || '').trim(),
    protocol:$('protocol')?.value || 'UNKNOWN',
    model:Number($('model')?.value || 1),
    capTurbo:boolSelectValue('capTurbo'),
    capQuiet:boolSelectValue('capQuiet'),
    capSleep:boolSelectValue('capSleep'),
    capSwingV:boolSelectValue('capSwingV'),
    capSwingH:boolSelectValue('capSwingH'),
    capFilter:boolSelectValue('capFilter')
  });
}
function renderAcProfiles(cfg, force=false){
  cfg = cfg || {};
  const profiles = cfg.acProfiles || [];
  const select = $('acProfileSelect');
  if (!select) return;
  const activeId = Number(cfg.activeAcProfileId || 0);
  select.innerHTML = profiles.map(p => `<option value="${Number(p.id)}" ${Number(p.id) === activeId ? 'selected' : ''}>${esc(p.name || ((p.protocol || 'UNKNOWN') + ' / 型号 ' + (p.model || 1)))}</option>`).join('');
  if (activeId) select.value = String(activeId);
  const active = profiles.find(p => Number(p.id) === activeId) || profiles[0];
  if (active) {
    setValue('acProfileName', active.name || '', force);
  }
}
async function applyProfilesResponse(data, okText){
  if (!state.config) state.config = {};
  state.config.activeAcProfileId = data.activeId;
  state.config.acProfiles = data.profiles || [];
  clearDirty(['acProfileName','protocol','model','power','mode','degrees','fan','specialMode','swingV','swingH','filterFlag','capTurbo','capQuiet','capSleep','capSwingV','capSwingH','capFilter']);
  msg(okText || '空调方案已更新');
  await refresh(true);
}
async function selectAcProfile(){
  const id = Number($('acProfileSelect')?.value || 0);
  if (!id) return;
  try {
    msg('正在切换空调方案...');
    const data = await fetchJsonSafe('/api/ac-profile/select', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({id})
    }, '空调方案切换', 0, 15000);
    await applyProfilesResponse(data, '空调方案已切换');
  } catch(e) { msg(e.message || '空调方案切换失败', true); }
}
async function saveCurrentAcProfile(){
  try {
    const data = await fetchJsonSafe('/api/ac-profile/update', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(currentAcProfilePayload())
    }, '空调方案保存', 0, 15000);
    await applyProfilesResponse(data, '当前空调方案已保存');
  } catch(e) { msg(e.message || '空调方案保存失败', true); }
}
async function createAcProfile(){
  const base = currentAcProfilePayload();
  const suggested = base.name || `${base.protocol} / 型号 ${base.model}`;
  const name = prompt('新空调方案名称', suggested);
  if (name === null) return;
  try {
    const data = await fetchJsonSafe('/api/ac-profile/create', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(Object.assign(base, {name:name.trim() || suggested}))
    }, '空调方案创建', 0, 15000);
    await applyProfilesResponse(data, '新空调方案已创建');
  } catch(e) { msg(e.message || '空调方案创建失败', true); }
}
async function deleteCurrentAcProfile(){
  const id = Number($('acProfileSelect')?.value || 0);
  const name = $('acProfileName')?.value || '当前方案';
  if (!id || !confirm(`删除空调方案「${name}」？`)) return;
  try {
    const data = await fetchJsonSafe('/api/ac-profile/delete', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({id})
    }, '空调方案删除', 0, 15000);
    await applyProfilesResponse(data, '空调方案已删除');
  } catch(e) { msg(e.message || '空调方案删除失败', true); }
}
async function sendAc(){
  try {
    await post('/api/send-ac', remoteCommandPayload(), '红外命令已发送');
  } catch(e) { msg(e.message || '发送失败', true); }
}
function formatSelfTestResult(data){
  if (!data) return '未获得校验结果';
  const parts = [
    data.message || (data.ok ? '自发自收校验通过' : '自发自收校验未通过'),
    `发送 ${data.sent ? '成功' : '失败'}`,
    `接收 ${data.received ? '成功' : '失败'}`,
    `协议 ${data.protocolOk ? '一致' : '不一致'}`,
    `字段 ${data.stateOk ? '一致' : '不一致'}`
  ];
  if (data.receivedProtocol) parts.push(`收到 ${data.receivedProtocol} / ${data.receivedBits || 0} bits`);
  if (data.mismatches && data.mismatches.length) parts.push(`差异：${data.mismatches.join('，')}`);
  if (data.expected && data.expected.power === false) parts.push('关机命令主要校验协议和电源位；开机状态可校验温度/风速等字段');
  return parts.join(' · ');
}
async function selfTestAc(){
  const box = $('selfTestResult');
  try {
    if (box) box.textContent = '正在发射并监听本机接收头...';
    msg('正在执行自发自收校验...');
    const data = await fetchJsonSafe('/api/self-test-ac', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(Object.assign(remoteCommandPayload(), {timeoutMs:4500}))
    }, '自发自收校验', 0, 12000);
    const text = formatSelfTestResult(data);
    if (box) box.textContent = text;
    msg(data.ok ? '自发自收校验通过' : '自发自收校验未通过', !data.ok);
    await refresh(true);
  } catch(e) {
    if (box) box.textContent = e.message || '自发自收校验失败';
    msg(e.message || '自发自收校验失败', true);
  }
}
function formatCaptureBrief(c){
  if (!c || !c.available) return '无编码';
  const core = c.stateHex || c.value || '--';
  const decoded = c.request ? ` / ${formatAcRequestBrief(c.request)}` : '';
  return `${c.protocol || '--'} / ${c.bits || 0} bits / ${core}${decoded}`;
}
function formatCodeCompareResult(data){
  if (!data) return '未获得编码对比结果';
  const parts = [
    data.message || '编码对比完成',
    `严格 ${data.strictOk ? '一致' : '不一致'}`,
    `语义 ${data.semanticOk ? '一致' : '不一致'}`,
    `实体 ${formatCaptureBrief(data.reference)}`,
    `生成 ${formatCaptureBrief(data.generated)}`
  ];
  const mismatches = []
    .concat(data.strictMismatches || [])
    .concat(data.semanticMismatches || []);
  if (mismatches.length) parts.push(`差异：${[...new Set(mismatches)].join('，')}`);
  return parts.join(' · ');
}
async function compareAcCode(){
  const box = $('codeCompareResult');
  try {
    if (box) box.textContent = '正在生成当前设置编码并与最近实体遥控器捕获对比...';
    msg('正在对比编码...');
    const data = await fetchJsonSafe('/api/compare-ac-code', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(Object.assign(remoteCommandPayload(), {timeoutMs:4500}))
    }, '编码对比', 0, 14000);
    const text = formatCodeCompareResult(data);
    if (box) box.textContent = text;
    msg(data.ok ? '严格编码一致' : (data.semanticOk ? '语义一致，原始编码有差异' : '编码不一致'), !(data.ok || data.semanticOk));
    await refresh(true);
  } catch(e) {
    if (box) box.textContent = e.message || '编码对比失败';
    msg(e.message || '编码对比失败', true);
  }
}
async function applyCaptureState(){
  try {
    msg('正在套用最近捕获状态...');
    const data = await fetchJsonSafe('/api/apply-capture-state', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:'{}'
    }, '套用捕获状态', 0, 12000);
    clearDirty(['protocol','model','power','mode','degrees','fan','specialMode','swingV','swingH','filterFlag']);
    const text = data && data.request ? `已套用：${formatAcRequestBrief(data.request)}` : '最近捕获状态已套用';
    if ($('codeCompareResult')) $('codeCompareResult').textContent = text + '。现在可再次执行编码对比。';
    msg(text);
    await refresh(true);
  } catch(e) {
    msg(e.message || '套用最近捕获失败', true);
  }
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
    await fetchTextSafe('/api/update-preset', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({
      id,
      scheduleEnabled:enabled,
      scheduleMinute:time,
      scheduleMode:mode,
      dayMask:presetDayMask(id)
    })}, '定时保存', 1, 30000);
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
    await post('/api/learn', Object.assign(remoteCommandPayload(), {name:$('learnName').value, freqKhz:Number($('learnFreq').value), power:$('learnPower').value==='true', mode:$('learnMode').value, degrees:Number($('learnDegrees').value), fan:$('learnFan').value}), '最近捕获已保存', ['learnName','learnFreq','learnPower','learnMode','learnDegrees','learnFan']);
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
    await fetchTextSafe('/api/curve', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({
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
    })}, '睡眠曲线保存', 1, 30000);
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
orderMainSections();
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
async function startDashboardRefresh(){
  await refresh(true);
  await refreshControlLog();
  await refreshTempHistory();
}
startDashboardRefresh();
window.addEventListener('resize', () => requestAnimationFrame(fitStatusReadouts));
setInterval(refreshLive, 3000);
setInterval(() => refresh(false), 60000);
setInterval(refreshTempHistory, 120000);
setInterval(refreshControlLog, 60000);
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
    const needFull = !entries.length || !state.config;
    const data = await fetchJsonSafe(needFull ? '/api/status' : '/api/live', needFull ? '完整状态' : '实时状态', 1, needFull ? 45000 : 12000);
    state = Object.assign(state || {}, data);
    if (state.config) $('protocolBadge').textContent = `协议 ${state.config.acProtocol} / 型号 ${state.config.acModel}`;
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
function wait(ms){ return new Promise(resolve => setTimeout(resolve, ms)); }
async function fetchJsonSafe(url, label='数据', retries=1, timeoutMs=22000){
  let lastError = null;
  for (let attempt = 0; attempt <= retries; attempt++) {
    const controller = window.AbortController ? new AbortController() : null;
    const timer = controller ? setTimeout(() => controller.abort(), timeoutMs) : 0;
    try {
      const r = await fetch(url, controller ? {signal:controller.signal} : {});
      const text = await r.text();
      if (!r.ok) throw new Error(text || `${label}获取失败`);
      if (!text.trim()) throw new Error(`${label}返回为空`);
      try { return JSON.parse(text); }
      catch(parseError) { throw new Error(`${label}JSON不完整，已放弃本次刷新`); }
    } catch(e) {
      lastError = e;
      if (attempt < retries) await wait(450 + attempt * 350);
    } finally {
      if (timer) clearTimeout(timer);
    }
  }
  throw lastError || new Error(`${label}获取失败`);
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
setInterval(refresh, 3000);
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
function wait(ms){ return new Promise(resolve => setTimeout(resolve, ms)); }
function cleanFetchErrorText(text, label='数据', status=0){
  const raw = String(text || '').trim();
  if (raw.startsWith('{')) {
    try {
      const parsed = JSON.parse(raw);
      if (parsed && parsed.message) return String(parsed.message);
    } catch(e) {}
  }
  if (!raw) return `${label}获取失败${status ? `（HTTP ${status}）` : ''}`;
  if (raw.length > 180 || /<html|<!doctype|stack|trace|exception|function\\s|void\\s|#include/i.test(raw)) {
    return `${label}获取失败${status ? `（HTTP ${status}）` : ''}`;
  }
  return raw;
}
async function fetchTextSafe(url, options={}, label='数据', retries=1, timeoutMs=22000){
  let lastError = null;
  for (let attempt = 0; attempt <= retries; attempt++) {
    const controller = window.AbortController ? new AbortController() : null;
    const timer = controller ? setTimeout(() => controller.abort(), timeoutMs) : 0;
    try {
      const r = await fetch(url, Object.assign({}, options, controller ? {signal:controller.signal} : {}));
      const text = await r.text();
      if (!r.ok) throw new Error(cleanFetchErrorText(text, label, r.status));
      if (!text.trim()) throw new Error(`${label}返回为空`);
      return text;
    } catch(e) {
      lastError = e;
      if (attempt < retries) await wait(450 + attempt * 350);
    } finally {
      if (timer) clearTimeout(timer);
    }
  }
  throw lastError || new Error(`${label}获取失败`);
}
async function fetchJsonSafe(url, options={}, label='数据', retries=1, timeoutMs=22000){
  const text = await fetchTextSafe(url, options, label, retries, timeoutMs);
  try { return JSON.parse(text); }
  catch(e) { throw new Error(`${label}JSON不完整，已放弃本次刷新`); }
}
async function apiPost(url, body){
  return await fetchTextSafe(url, {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body)
  }, '操作', 1, 25000);
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
    const state = await fetchJsonSafe('/api/live', {}, '实时状态', 1, 12000);
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
    const state = await fetchJsonSafe('/api/status', {}, '完整状态', 1, 45000);
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
setInterval(refreshLive, 3000);
setInterval(refresh, 60000);
</script>
</body>
</html>
)REMOTE";

String uint64ToHexString(uint64_t value) {
  char buffer[19];
  snprintf(buffer, sizeof(buffer), "0x%08llX", static_cast<unsigned long long>(value));
  return String(buffer);
}

String bytesToHexString(const uint8_t *data, uint16_t len) {
  static const char hex[] = "0123456789ABCDEF";
  String out;
  out.reserve(2 + len * 2);
  out += "0x";
  for (uint16_t i = 0; i < len; i++) {
    out += hex[(data[i] >> 4) & 0x0F];
    out += hex[data[i] & 0x0F];
  }
  return out;
}

float clampFloat(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

String smartModeForTarget(float target);

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
  if (request.mode == "smart") request.mode = smartModeForTarget(request.degrees);
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
  if (config.autoMode != "smart" && config.autoMode != "cool" && config.autoMode != "auto" &&
      config.autoMode != "dry" && config.autoMode != "heat" && config.autoMode != "fan") {
    config.autoMode = "smart";
  }
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

int findAcProfileIndexById(uint8_t id) {
  for (uint8_t i = 0; i < acProfileCount; i++) {
    if (acProfiles[i].id == id) return i;
  }
  return -1;
}

uint8_t nextAcProfileId() {
  uint8_t next = 1;
  for (uint8_t i = 0; i < acProfileCount; i++) {
    if (acProfiles[i].id >= next) next = acProfiles[i].id + 1;
  }
  if (next == 0) next = 1;
  return next;
}

String defaultAcProfileName(const AcProfile &profile) {
  String protocol = typeToString(profile.protocol);
  if (protocol == "UNKNOWN") protocol = "未配置";
  return protocol + " / 型号 " + String(profile.model);
}

void snapshotConfigToProfile(AcProfile &profile) {
  profile.protocol = config.acProtocol;
  profile.model = config.acModel;
  profile.power = config.remotePower;
  profile.mode = config.remoteMode;
  profile.fan = config.remoteFan;
  profile.degrees = clampFloat(config.remoteDegrees, 16.0f, 32.0f);
  profile.turbo = config.remoteTurbo;
  profile.quiet = config.remoteQuiet;
  profile.sleep = config.remoteSleep;
  profile.swingV = config.remoteSwingV;
  profile.swingH = config.remoteSwingH;
  profile.filter = config.remoteFilter;
  profile.capTurbo = config.capTurbo;
  profile.capQuiet = config.capQuiet;
  profile.capSleep = config.capSleep;
  profile.capSwingV = config.capSwingV;
  profile.capSwingH = config.capSwingH;
  profile.capFilter = config.capFilter;
  if (!profile.name.length()) profile.name = defaultAcProfileName(profile);
}

void applyProfileToConfig(const AcProfile &profile) {
  config.acProtocol = profile.protocol;
  config.acModel = profile.model < 1 ? 1 : profile.model;
  config.remotePower = profile.power;
  config.remoteMode = profile.mode.length() ? profile.mode : "cool";
  config.remoteFan = profile.fan.length() ? profile.fan : "auto";
  config.remoteDegrees = clampFloat(profile.degrees, 16.0f, 32.0f);
  config.remoteTurbo = profile.turbo;
  config.remoteQuiet = profile.quiet;
  config.remoteSleep = profile.sleep;
  config.remoteSwingV = profile.swingV;
  config.remoteSwingH = profile.swingH;
  config.remoteFilter = profile.filter;
  config.capTurbo = profile.capTurbo;
  config.capQuiet = profile.capQuiet;
  config.capSleep = profile.capSleep;
  config.capSwingV = profile.capSwingV;
  config.capSwingH = profile.capSwingH;
  config.capFilter = profile.capFilter;
  normalizeConfigRemoteState();
  lastAutoSentSetpoint = NAN;
  lastAutoSentMode = "";
  lastAutoSentFan = "";
}

void ensureAcProfiles() {
  if (acProfileCount == 0) {
    acProfileCount = 1;
    acProfiles[0].id = activeAcProfileId == 0 ? 1 : activeAcProfileId;
    snapshotConfigToProfile(acProfiles[0]);
  }
  if (findAcProfileIndexById(activeAcProfileId) < 0) activeAcProfileId = acProfiles[0].id;
}

void syncConfigToActiveProfile() {
  ensureAcProfiles();
  int idx = findAcProfileIndexById(activeAcProfileId);
  if (idx >= 0) snapshotConfigToProfile(acProfiles[idx]);
}

void addAcProfileToJson(JsonObject obj, const AcProfile &profile) {
  obj["id"] = profile.id;
  obj["name"] = profile.name.length() ? profile.name : defaultAcProfileName(profile);
  obj["protocol"] = typeToString(profile.protocol);
  obj["model"] = profile.model;
  obj["power"] = profile.power;
  obj["degrees"] = profile.degrees;
  obj["mode"] = profile.mode;
  obj["fan"] = profile.fan;
  obj["turbo"] = profile.turbo;
  obj["quiet"] = profile.quiet;
  obj["sleep"] = profile.sleep;
  obj["swingV"] = profile.swingV;
  obj["swingH"] = profile.swingH;
  obj["filter"] = profile.filter;
  obj["capTurbo"] = profile.capTurbo;
  obj["capQuiet"] = profile.capQuiet;
  obj["capSleep"] = profile.capSleep;
  obj["capSwingV"] = profile.capSwingV;
  obj["capSwingH"] = profile.capSwingH;
  obj["capFilter"] = profile.capFilter;
}

void loadAcProfileFromJson(AcProfile &profile, JsonObject item) {
  profile.id = item["id"] | 0;
  profile.name = item["name"] | "";
  profile.name.trim();
  profile.protocol = strToDecodeType((item["protocol"] | "UNKNOWN"));
  profile.model = item["model"] | 1;
  profile.power = item["power"] | true;
  profile.degrees = clampFloat(item["degrees"] | 26.0f, 16.0f, 32.0f);
  profile.mode = item["mode"] | "cool";
  profile.fan = item["fan"] | "auto";
  profile.turbo = item["turbo"] | false;
  profile.quiet = item["quiet"] | false;
  profile.sleep = item["sleep"] | false;
  profile.swingV = item["swingV"] | false;
  profile.swingH = item["swingH"] | false;
  profile.filter = item["filter"] | false;
  profile.capTurbo = item["capTurbo"] | true;
  profile.capQuiet = item["capQuiet"] | true;
  profile.capSleep = item["capSleep"] | true;
  profile.capSwingV = item["capSwingV"] | true;
  profile.capSwingH = item["capSwingH"] | true;
  profile.capFilter = item["capFilter"] | true;
  if (!profile.name.length()) profile.name = defaultAcProfileName(profile);
}

void updateAcProfileFromJson(AcProfile &profile, JsonObject item) {
  if (item["name"].is<const char *>()) {
    profile.name = item["name"].as<String>();
    profile.name.trim();
  }
  if (item["protocol"].is<const char *>()) profile.protocol = strToDecodeType((item["protocol"] | "UNKNOWN"));
  if (item["model"].is<int>()) profile.model = item["model"] | profile.model;
  if (item["power"].is<bool>()) profile.power = item["power"] | profile.power;
  if (item["degrees"].is<float>() || item["degrees"].is<int>()) profile.degrees = clampFloat(item["degrees"] | profile.degrees, 16.0f, 32.0f);
  if (item["mode"].is<const char *>()) profile.mode = item["mode"] | profile.mode;
  if (item["fan"].is<const char *>()) profile.fan = item["fan"] | profile.fan;
  if (item["turbo"].is<bool>()) profile.turbo = item["turbo"] | profile.turbo;
  if (item["quiet"].is<bool>()) profile.quiet = item["quiet"] | profile.quiet;
  if (item["sleep"].is<bool>()) profile.sleep = item["sleep"] | profile.sleep;
  if (item["swingV"].is<bool>()) profile.swingV = item["swingV"] | profile.swingV;
  if (item["swingH"].is<bool>()) profile.swingH = item["swingH"] | profile.swingH;
  if (item["filter"].is<bool>()) profile.filter = item["filter"] | profile.filter;
  if (item["capTurbo"].is<bool>()) profile.capTurbo = item["capTurbo"] | profile.capTurbo;
  if (item["capQuiet"].is<bool>()) profile.capQuiet = item["capQuiet"] | profile.capQuiet;
  if (item["capSleep"].is<bool>()) profile.capSleep = item["capSleep"] | profile.capSleep;
  if (item["capSwingV"].is<bool>()) profile.capSwingV = item["capSwingV"] | profile.capSwingV;
  if (item["capSwingH"].is<bool>()) profile.capSwingH = item["capSwingH"] | profile.capSwingH;
  if (item["capFilter"].is<bool>()) profile.capFilter = item["capFilter"] | profile.capFilter;
  if (!profile.name.length()) profile.name = defaultAcProfileName(profile);
}

void saveConfig() {
  normalizeTuningConfig();
  normalizeConfigRemoteState();
  syncConfigToActiveProfile();
  JsonDocument doc;
  doc["activeAcProfileId"] = activeAcProfileId;
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
  doc["lastCurveEndPoweroffKey"] = config.lastCurveEndPoweroffKey;
  JsonArray curve = doc["curve"].to<JsonArray>();
  for (uint8_t i = 0; i < config.curveCount; i++) {
    JsonObject point = curve.add<JsonObject>();
    point["minute"] = config.curve[i].minute;
    point["temp"] = config.curve[i].temp;
  }
  JsonArray profiles = doc["acProfiles"].to<JsonArray>();
  for (uint8_t i = 0; i < acProfileCount; i++) {
    JsonObject item = profiles.add<JsonObject>();
    addAcProfileToJson(item, acProfiles[i]);
  }
  writeJsonFile(kConfigPath, doc);
}

void loadConfig() {
  JsonDocument doc;
  if (!readJsonFile(kConfigPath, doc)) {
    ensureAcProfiles();
    return;
  }
  activeAcProfileId = doc["activeAcProfileId"] | activeAcProfileId;
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
  config.lastCurveEndPoweroffKey = doc["lastCurveEndPoweroffKey"] | config.lastCurveEndPoweroffKey;
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
  JsonArray profiles = doc["acProfiles"].as<JsonArray>();
  if (!profiles.isNull()) {
    acProfileCount = 0;
    for (JsonObject item : profiles) {
      if (acProfileCount >= kMaxAcProfiles) break;
      loadAcProfileFromJson(acProfiles[acProfileCount], item);
      if (acProfiles[acProfileCount].id == 0) acProfiles[acProfileCount].id = nextAcProfileId();
      acProfileCount++;
    }
  }
  ensureAcProfiles();
  int activeIdx = findAcProfileIndexById(activeAcProfileId);
  if (activeIdx >= 0) applyProfileToConfig(acProfiles[activeIdx]);
}

void saveLearnedLibrary() {
  JsonDocument doc;
  JsonArray arr = doc["commands"].to<JsonArray>();
  for (uint8_t i = 0; i < learnedCount; i++) {
    JsonObject item = arr.add<JsonObject>();
    item["id"] = learned[i].id;
    item["name"] = learned[i].name;
    item["protocol"] = typeToString(learned[i].protocol);
    item["model"] = learned[i].model;
    item["bits"] = learned[i].bits;
    item["value"] = uint64ToHexString(learned[i].value);
    item["hasAcMeta"] = learned[i].hasAcMeta;
    item["metaComplete"] = learned[i].metaComplete;
    item["power"] = learned[i].power;
    item["degrees"] = learned[i].degrees;
    item["mode"] = learned[i].mode;
    item["fan"] = learned[i].fan;
    item["turbo"] = learned[i].turbo;
    item["quiet"] = learned[i].quiet;
    item["sleep"] = learned[i].sleep;
    item["swingV"] = learned[i].swingV;
    item["swingH"] = learned[i].swingH;
    item["filter"] = learned[i].filter;
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
    cmd.model = item["model"] | -1;
    cmd.bits = item["bits"] | 0;
    cmd.value = parseHex64(item["value"] | "0");
    cmd.hasAcMeta = item["hasAcMeta"] | false;
    cmd.metaComplete = item["metaComplete"] | false;
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

bool applyCaptureMetaToLearned(LearnedCommand &cmd) {
  stdAc::state_t decodedState;
  if (!snapshotToCommonState(lastCapture, &decodedState, nullptr)) return false;
  AcRequest request = requestFromCommonStateRaw(decodedState);
  cmd.protocol = lastCapture.protocol;
  cmd.model = decodedState.model;
  cmd.hasAcMeta = true;
  cmd.metaComplete = true;
  cmd.power = request.power;
  cmd.mode = request.mode;
  cmd.fan = request.fan;
  cmd.degrees = request.degrees;
  cmd.turbo = request.turbo;
  cmd.quiet = request.quiet;
  cmd.sleep = request.sleep;
  cmd.swingV = request.swingV;
  cmd.swingH = request.swingH;
  cmd.filter = request.filter;
  return true;
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
    server.sendHeader("Connection", "close");
    server.send(400, "text/plain", "Invalid JSON");
    return false;
  }
  return true;
}

void sendJsonResponse(uint16_t code, const String &out) {
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", out);
}

void flushJsonStreamChunk(String &chunk) {
  if (!chunk.length()) return;
  server.sendContent(chunk);
  chunk = "";
}

void appendJsonStreamChunk(String &chunk, const String &part) {
  if (chunk.length() + part.length() > 960) flushJsonStreamChunk(chunk);
  chunk += part;
}

void appendJsonStreamChar(String &chunk, char c) {
  if (chunk.length() + 1 > 960) flushJsonStreamChunk(chunk);
  chunk += c;
}

void beginJsonStreamResponse() {
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
}

void appendJsonEscapedStream(String &chunk, const char *text) {
  appendJsonStreamChunk(chunk, "\"");
  if (text != nullptr) {
    for (const char *p = text; *p != '\0'; p++) {
      char c = *p;
      if (c == '"' || c == '\\') {
        appendJsonStreamChar(chunk, '\\');
        appendJsonStreamChar(chunk, c);
      } else if (c == '\n') {
        appendJsonStreamChunk(chunk, "\\n");
      } else if (c == '\r') {
        appendJsonStreamChunk(chunk, "\\r");
      } else if (static_cast<uint8_t>(c) < 0x20) {
        appendJsonStreamChar(chunk, ' ');
      } else {
        appendJsonStreamChar(chunk, c);
      }
    }
  }
  appendJsonStreamChunk(chunk, "\"");
}

void appendJsonTenthsStream(String &chunk, const char *name, int16_t value, bool leadingComma = true) {
  if (leadingComma) appendJsonStreamChunk(chunk, ",");
  appendJsonStreamChunk(chunk, "\"");
  appendJsonStreamChunk(chunk, name);
  appendJsonStreamChunk(chunk, "\":");
  if (value == INT16_MIN) {
    appendJsonStreamChunk(chunk, "null");
  } else {
    appendJsonStreamChunk(chunk, String(value / 10.0f, 1));
  }
}

void sendOk(const String &message = "ok") {
  JsonDocument doc;
  doc["ok"] = true;
  doc["message"] = message;
  String out;
  serializeJson(doc, out);
  sendJsonResponse(200, out);
}

void sendError(uint16_t code, const String &message) {
  JsonDocument doc;
  doc["ok"] = false;
  doc["message"] = message;
  String out;
  serializeJson(doc, out);
  sendJsonResponse(code, out);
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

bool hasStationCredentials() {
  return config.staSsid.length() > 0 && config.staPassword.length() > 0;
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
  if (apStarted && WiFi.softAPIP() != IPAddress(0, 0, 0, 0)) return;

  if (hasStationCredentials()) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  disableWifiPowerSave();

  bool ok = false;
  for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
    WiFi.softAPdisconnect(false);
    delay(120);
    ok = WiFi.softAP(kApSsid, kApPassword, 6, false, 4);
    delay(250);
    ok = ok && WiFi.softAPIP() != IPAddress(0, 0, 0, 0);
  }

  if (!ok) {
    apStarted = false;
    Serial.println("AP start failed");
    return;
  }

  dnsServer.start(53, "*", WiFi.softAPIP());
  apStarted = true;
  Serial.println("AP started: " + String(kApSsid) + " IP " + WiFi.softAPIP().toString());
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

  if (hasStationCredentials()) {
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
  if (wifiScanActive) return;
  if (!hasStationCredentials()) return;

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
    Serial.println("STA connect timeout, opening AP fallback");
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

void appendDecisionLogMemory(const DecisionLogEntry &entry) {
  decisionLog[decisionLogHead] = entry;
  decisionLogHead = (decisionLogHead + 1) % kDecisionLogPoints;
  if (decisionLogCount < kDecisionLogPoints) decisionLogCount++;
}

void writeCsvText(File &file, const char *text) {
  if (text == nullptr) return;
  for (const char *p = text; *p != '\0'; p++) {
    char c = *p;
    if (c == '\r' || c == '\n') c = ' ';
    file.write(static_cast<uint8_t>(c));
  }
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

void writeDecisionLogEntry(File &file, const DecisionLogEntry &entry) {
  file.printf("%lu,%d,%d,%d,%s,%s,",
              static_cast<unsigned long>(entry.minute),
              static_cast<int>(entry.room10),
              static_cast<int>(entry.target10),
              static_cast<int>(entry.setpoint10),
              entry.action,
              entry.stage);
  writeCsvText(file, entry.note);
  file.print('\n');
}

bool saveDecisionLog() {
  File file = LittleFS.open(kDecisionLogPath, "w");
  if (!file) return false;
  file.printf("DL1,%u,%u\n", decisionLogUsesEpoch ? 1 : 0, kDecisionLogPoints);
  for (uint16_t i = 0; i < decisionLogCount; i++) {
    uint16_t idx = (decisionLogHead + kDecisionLogPoints - decisionLogCount + i) % kDecisionLogPoints;
    writeDecisionLogEntry(file, decisionLog[idx]);
  }
  bool ok = file.getWriteError() == 0;
  file.close();
  return ok;
}

bool appendDecisionLogToFile(const DecisionLogEntry &entry) {
  bool needsHeader = !LittleFS.exists(kDecisionLogPath);
  if (!needsHeader) {
    File existing = LittleFS.open(kDecisionLogPath, "r");
    if (!existing || existing.size() == 0) {
      needsHeader = true;
    } else if (existing.size() > kDecisionLogMaxFileBytes) {
      existing.close();
      return saveDecisionLog();
    }
    if (existing) existing.close();
  }

  File file = LittleFS.open(kDecisionLogPath, needsHeader ? "w" : "a");
  if (!file) return false;
  if (needsHeader) file.printf("DL1,%u,%u\n", decisionLogUsesEpoch ? 1 : 0, kDecisionLogPoints);
  writeDecisionLogEntry(file, entry);
  bool ok = file.getWriteError() == 0;
  file.close();
  if (!ok) return false;

  File check = LittleFS.open(kDecisionLogPath, "r");
  bool shouldCompact = check && check.size() > kDecisionLogMaxFileBytes;
  if (check) check.close();
  return shouldCompact ? saveDecisionLog() : true;
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

void loadDecisionLog() {
  clearDecisionLog();
  if (!LittleFS.exists(kDecisionLogPath)) return;
  File file = LittleFS.open(kDecisionLogPath, "r");
  if (!file) return;
  String header = file.readStringUntil('\n');
  header.trim();
  unsigned int usesEpoch = 0;
  unsigned int capacity = 0;
  if (sscanf(header.c_str(), "DL1,%u,%u", &usesEpoch, &capacity) != 2) {
    file.close();
    return;
  }
  decisionLogUsesEpoch = usesEpoch != 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    unsigned long minute = 0;
    int room10 = INT16_MIN;
    int target10 = INT16_MIN;
    int setpoint10 = INT16_MIN;
    char action[24] = "";
    char stage[12] = "";
    char note[160] = "";
    int matched = sscanf(line.c_str(), "%lu,%d,%d,%d,%23[^,],%11[^,],%159[^\n]",
                         &minute,
                         &room10,
                         &target10,
                         &setpoint10,
                         action,
                         stage,
                         note);
    if (matched >= 6) {
      DecisionLogEntry entry;
      entry.minute = static_cast<uint32_t>(minute);
      entry.room10 = static_cast<int16_t>(room10);
      entry.target10 = static_cast<int16_t>(target10);
      entry.setpoint10 = static_cast<int16_t>(setpoint10);
      copyText(entry.action, sizeof(entry.action), action);
      copyText(entry.stage, sizeof(entry.stage), stage);
      copyText(entry.note, sizeof(entry.note), matched >= 7 ? note : "");
      appendDecisionLogMemory(entry);
    }
  }
  bool needsCompact = file.size() > kDecisionLogMaxFileBytes;
  file.close();
  if (needsCompact) saveDecisionLog();
}

void addDecisionLog(const char *action, const char *stage, const char *note,
                    float room, float target, float setpoint) {
  bool usesEpoch = false;
  uint32_t minute = currentHistoryMinute(&usesEpoch);
  if ((decisionLogCount > 0 || LittleFS.exists(kDecisionLogPath)) && usesEpoch != decisionLogUsesEpoch) {
    clearDecisionLog();
    LittleFS.remove(kDecisionLogPath);
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
  DecisionLogEntry entry;
  entry.minute = minute;
  entry.room10 = tempToTenths(room);
  entry.target10 = tempToTenths(target);
  entry.setpoint10 = tempToTenths(setpoint);
  copyText(entry.action, sizeof(entry.action), action);
  copyText(entry.stage, sizeof(entry.stage), stage);
  copyText(entry.note, sizeof(entry.note), note);
  appendDecisionLogMemory(entry);
  appendDecisionLogToFile(entry);
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

void storeCaptureSnapshot(const decode_results &results) {
  lastCapture.available = true;
  lastCapture.protocol = results.decode_type;
  lastCapture.bits = results.bits;
  lastCapture.value = results.value;
  lastCapture.stateLen = 0;
  memset(lastCapture.state, 0, sizeof(lastCapture.state));
  if (results.decode_type == decode_type_t::GREE || results.decode_type == decode_type_t::MIRAGE || results.bits > 64) {
    uint16_t stateLen = (results.bits + 7) / 8;
    if (stateLen > kStateSizeMax) stateLen = kStateSizeMax;
    lastCapture.stateLen = stateLen;
    memcpy(lastCapture.state, results.state, stateLen);
  }
  lastCapture.summary = resultToHumanReadableBasic(&results);
  lastCapture.acDescription = IRAcUtils::resultAcToString(&results);
  lastCapture.capturedAtMs = millis();

  uint16_t correctedLen = getCorrectedRawLength(&results);
  lastCapture.rawLen = correctedLen > kMaxRawPulses ? kMaxRawPulses : correctedLen;
  uint16_t *raw = resultToRawArray(&results);
  if (raw != nullptr) {
    for (uint16_t i = 0; i < lastCapture.rawLen; i++) lastCapture.raw[i] = raw[i];
    delete[] raw;
  }
}

void captureIrIfAvailable() {
  if (irBusy) return;
  if (!irrecv.decode(&irResults)) return;

  storeCaptureSnapshot(irResults);

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

bool learnedCommandMatchesRequest(const LearnedCommand &cmd, const AcRequest &request) {
  if (!cmd.hasAcMeta || cmd.rawLen == 0) return false;
  if (cmd.protocol != decode_type_t::UNKNOWN && config.acProtocol != decode_type_t::UNKNOWN &&
      cmd.protocol != config.acProtocol) {
    return false;
  }
  if (cmd.metaComplete && cmd.model >= 0 && config.acModel >= 0 && cmd.model != config.acModel) return false;
  if (cmd.power != request.power) return false;
  if (!request.power) return true;
  if (cmd.mode != request.mode) return false;
  if (fabs(cmd.degrees - request.degrees) > 0.25f) return false;
  if (cmd.fan != request.fan) return false;
  if (!cmd.metaComplete) return true;
  return cmd.turbo == request.turbo &&
         cmd.quiet == request.quiet &&
         cmd.sleep == request.sleep &&
         cmd.swingV == request.swingV &&
         cmd.swingH == request.swingH &&
         cmd.filter == request.filter;
}

int findSemanticLearnedCommand(const AcRequest &request) {
  for (uint8_t i = 0; i < learnedCount; i++) {
    if (learnedCommandMatchesRequest(learned[i], request)) return i;
  }
  return -1;
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

AcRequest currentRemoteRequest() {
  AcRequest request;
  request.power = config.remotePower;
  request.mode = config.remoteMode;
  request.fan = config.remoteFan;
  request.degrees = config.remoteDegrees;
  request.turbo = config.remoteTurbo;
  request.quiet = config.remoteQuiet;
  request.sleep = config.remoteSleep;
  request.swingV = config.remoteSwingV;
  request.swingH = config.remoteSwingH;
  request.filter = config.remoteFilter;
  return normalizedAcRequest(request);
}

bool waitForSelfTestDecode(uint32_t timeoutMs) {
  uint32_t startMs = millis();
  while (millis() - startMs < timeoutMs) {
    if (irrecv.decode(&irResults)) return true;
    delay(12);
    yield();
  }
  return false;
}

bool stateSwingVActive(stdAc::swingv_t swing) {
  return swing != stdAc::swingv_t::kOff;
}

bool stateSwingHActive(stdAc::swingh_t swing) {
  return swing != stdAc::swingh_t::kOff;
}

String modeFromCommonState(stdAc::opmode_t mode) {
  switch (mode) {
    case stdAc::opmode_t::kCool:
      return "cool";
    case stdAc::opmode_t::kHeat:
      return "heat";
    case stdAc::opmode_t::kDry:
      return "dry";
    case stdAc::opmode_t::kFan:
      return "fan";
    case stdAc::opmode_t::kAuto:
    case stdAc::opmode_t::kOff:
    default:
      return "auto";
  }
}

String fanFromCommonState(stdAc::fanspeed_t fan) {
  switch (fan) {
    case stdAc::fanspeed_t::kMin:
      return "min";
    case stdAc::fanspeed_t::kLow:
      return "low";
    case stdAc::fanspeed_t::kMedium:
      return "medium";
    case stdAc::fanspeed_t::kHigh:
      return "high";
    case stdAc::fanspeed_t::kMax:
    case stdAc::fanspeed_t::kMediumHigh:
      return "max";
    case stdAc::fanspeed_t::kAuto:
    default:
      return "auto";
  }
}

AcRequest requestFromCommonStateRaw(const stdAc::state_t &state) {
  AcRequest request;
  request.power = state.power && state.mode != stdAc::opmode_t::kOff;
  request.mode = modeFromCommonState(state.mode);
  request.fan = fanFromCommonState(state.fanspeed);
  request.degrees = clampFloat(state.degrees, config.minSetpoint, config.maxSetpoint);
  request.turbo = state.turbo;
  request.quiet = state.quiet;
  request.sleep = state.sleep >= 0;
  request.swingV = stateSwingVActive(state.swingv);
  request.swingH = stateSwingHActive(state.swingh);
  request.filter = state.filter;
  return request;
}

AcRequest requestFromCommonState(const stdAc::state_t &state) {
  AcRequest request = requestFromCommonStateRaw(state);
  return normalizedAcRequest(request);
}

AcRequest requestFromLearnedCommand(const LearnedCommand &cmd) {
  AcRequest request;
  request.power = cmd.power;
  request.mode = cmd.mode;
  request.fan = cmd.fan;
  request.degrees = cmd.degrees;
  request.turbo = cmd.turbo;
  request.quiet = cmd.quiet;
  request.sleep = cmd.sleep;
  request.swingV = cmd.swingV;
  request.swingH = cmd.swingH;
  request.filter = cmd.filter;
  return normalizedAcRequest(request);
}

void valueToStateBytes(uint64_t value, uint8_t *out, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) out[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
}

bool greeStateToCommon(const uint8_t *stateBytes, stdAc::state_t *state) {
  if (stateBytes == nullptr || state == nullptr) return false;
  CaptureSnapshot snapshot;
  snapshot.available = true;
  snapshot.protocol = decode_type_t::GREE;
  snapshot.bits = kGreeBits;
  snapshot.stateLen = kGreeStateLength;
  memcpy(snapshot.state, stateBytes, kGreeStateLength);
  return snapshotToCommonState(snapshot, state, nullptr);
}

uint8_t greeCalcChecksum(const uint8_t *stateBytes, uint16_t len = kGreeStateLength) {
  if (stateBytes == nullptr || len == 0) return 0;
  uint8_t sum = 10;
  for (uint8_t i = 0; i < 4 && i < len - 1; i++) sum += stateBytes[i] & 0x0F;
  for (uint8_t i = 4; i < len - 1; i++) sum += stateBytes[i] >> 4;
  return sum & 0x0F;
}

void fixGreeChecksum(uint8_t *stateBytes, uint16_t len = kGreeStateLength) {
  if (stateBytes == nullptr || len == 0) return;
  stateBytes[len - 1] = (stateBytes[len - 1] & 0x0F) | (greeCalcChecksum(stateBytes, len) << 4);
}

bool shouldPatchGreeYbofb(int16_t model) {
  return model == static_cast<int16_t>(gree_ac_remote_model_t::YBOFB);
}

bool currentUsesPatchedGreeYbofb() {
  return config.acProtocol == decode_type_t::GREE && shouldPatchGreeYbofb(config.acModel);
}

void applyGreeYbofbHiddenTemplate(const AcRequest &request, uint8_t *stateBytes) {
  if (stateBytes == nullptr) return;
  stateBytes[5] = request.sleep ? 0x42 : 0x41;
  fixGreeChecksum(stateBytes);
}

bool generateGreeStateBytes(const AcRequest &request, int16_t model, uint8_t *out) {
  if (out == nullptr) return false;
  gree_ac_remote_model_t greeModel = gree_ac_remote_model_t::YAW1F;
  if (model == static_cast<int16_t>(gree_ac_remote_model_t::YBOFB)) {
    greeModel = gree_ac_remote_model_t::YBOFB;
  } else if (model == static_cast<int16_t>(gree_ac_remote_model_t::YX1FSF)) {
    greeModel = gree_ac_remote_model_t::YX1FSF;
  }
  IRGreeAC gree(kIrTxPin, greeModel);
  gree.setModel(greeModel);
  gree.setPower(request.power);
  gree.setMode(gree.convertMode(parseMode(request.mode)));
  gree.setTemp(request.degrees);
  gree.setFan(gree.convertFan(parseFan(request.fan)));
  gree.setSwingVertical(request.swingV, request.swingV ? kGreeSwingAuto : kGreeSwingLastPos);
  gree.setSwingHorizontal(request.swingH ? kGreeSwingHAuto : kGreeSwingHOff);
  gree.setIFeel(false);
  gree.setLight(false);
  gree.setTurbo(request.turbo);
  gree.setEcono(false);
  gree.setXFan(false);
  gree.setSleep(request.sleep);
  memcpy(out, gree.getRaw(), kGreeStateLength);
  if (shouldPatchGreeYbofb(model)) applyGreeYbofbHiddenTemplate(request, out);
  return true;
}

bool sendGreeStateBytesNow(const AcRequest &request, stdAc::state_t *expectedState = nullptr) {
  uint8_t stateBytes[kGreeStateLength] = {};
  if (!generateGreeStateBytes(request, config.acModel, stateBytes)) return false;
  if (expectedState != nullptr) greeStateToCommon(stateBytes, expectedState);
  rawSender.sendGree(stateBytes, kGreeStateLength, kGreeDefaultRepeat);
  return true;
}

bool greeHiddenOnlyDiff(const uint8_t *reference, const uint8_t *generated) {
  bool hasDiff = false;
  for (uint8_t i = 0; i < kGreeStateLength; i++) {
    if (reference[i] == generated[i]) continue;
    hasDiff = true;
    if (i != 5 && i != 7) return false;
  }
  return hasDiff;
}

uint8_t countByteDiffs(const uint8_t *reference, const uint8_t *generated, uint16_t len) {
  uint8_t count = 0;
  for (uint16_t i = 0; i < len; i++) {
    if (reference[i] != generated[i]) count++;
  }
  return count;
}

String byteDiffsJson(const uint8_t *reference, const uint8_t *generated, uint16_t len) {
  String out = "[";
  bool first = true;
  for (uint16_t i = 0; i < len; i++) {
    if (reference[i] == generated[i]) continue;
    if (!first) out += ",";
    out += "{\"index\":";
    out += String(i + 1);
    out += ",\"reference\":";
    out += jsonString(bytesToHexString(reference + i, 1));
    out += ",\"generated\":";
    out += jsonString(bytesToHexString(generated + i, 1));
    out += "}";
    first = false;
  }
  out += "]";
  return out;
}

void appendSelfTestMismatch(SelfTestReport &report, const String &text) {
  if (report.mismatches.length()) report.mismatches += "|";
  report.mismatches += text;
}

void compareSelfTestState(SelfTestReport &report) {
  const stdAc::state_t &expected = report.expectedState;
  const stdAc::state_t &received = report.receivedState;
  report.stateOk = true;

  if (received.power != expected.power) {
    appendSelfTestMismatch(report, "电源不一致");
    report.stateOk = false;
  }

  if (!expected.power) return;

  if (received.mode != expected.mode) {
    appendSelfTestMismatch(report, "模式不一致");
    report.stateOk = false;
  }
  if (fabs(received.degrees - expected.degrees) > 0.25f) {
    appendSelfTestMismatch(report, "温度不一致");
    report.stateOk = false;
  }
  if (received.fanspeed != expected.fanspeed) {
    appendSelfTestMismatch(report, "风速不一致");
    report.stateOk = false;
  }
  if (received.turbo != expected.turbo) {
    appendSelfTestMismatch(report, "强劲状态不一致");
    report.stateOk = false;
  }
  if (received.quiet != expected.quiet) {
    appendSelfTestMismatch(report, "静音状态不一致");
    report.stateOk = false;
  }
  if ((received.sleep >= 0) != (expected.sleep >= 0)) {
    appendSelfTestMismatch(report, "睡眠状态不一致");
    report.stateOk = false;
  }
  if (stateSwingVActive(received.swingv) != stateSwingVActive(expected.swingv)) {
    appendSelfTestMismatch(report, "上下摆风不一致");
    report.stateOk = false;
  }
  if (stateSwingHActive(received.swingh) != stateSwingHActive(expected.swingh)) {
    appendSelfTestMismatch(report, "左右摆风不一致");
    report.stateOk = false;
  }
  if (received.filter != expected.filter) {
    appendSelfTestMismatch(report, "滤网/出风口状态不一致");
    report.stateOk = false;
  }
}

String selfTestStateJson(const stdAc::state_t &state) {
  String out;
  out.reserve(256);
  out += "{\"protocol\":";
  out += jsonString(typeToString(state.protocol));
  out += ",\"model\":";
  out += String(state.model);
  out += ",\"power\":";
  out += state.power ? "true" : "false";
  out += ",\"mode\":";
  out += jsonString(IRac::opmodeToString(state.mode));
  out += ",\"degrees\":";
  out += String(state.degrees, 1);
  out += ",\"fan\":";
  out += jsonString(IRac::fanspeedToString(state.fanspeed));
  out += ",\"turbo\":";
  out += state.turbo ? "true" : "false";
  out += ",\"quiet\":";
  out += state.quiet ? "true" : "false";
  out += ",\"sleep\":";
  out += state.sleep >= 0 ? "true" : "false";
  out += ",\"swingV\":";
  out += stateSwingVActive(state.swingv) ? "true" : "false";
  out += ",\"swingH\":";
  out += stateSwingHActive(state.swingh) ? "true" : "false";
  out += ",\"filter\":";
  out += state.filter ? "true" : "false";
  out += "}";
  return out;
}

String acRequestJson(const AcRequest &request) {
  String out;
  out.reserve(220);
  out += "{\"power\":";
  out += request.power ? "true" : "false";
  out += ",\"mode\":";
  out += jsonString(request.mode);
  out += ",\"degrees\":";
  out += String(request.degrees, 1);
  out += ",\"fan\":";
  out += jsonString(request.fan);
  out += ",\"turbo\":";
  out += request.turbo ? "true" : "false";
  out += ",\"quiet\":";
  out += request.quiet ? "true" : "false";
  out += ",\"sleep\":";
  out += request.sleep ? "true" : "false";
  out += ",\"swingV\":";
  out += request.swingV ? "true" : "false";
  out += ",\"swingH\":";
  out += request.swingH ? "true" : "false";
  out += ",\"filter\":";
  out += request.filter ? "true" : "false";
  out += "}";
  return out;
}

void addAcRequestToJson(JsonObject obj, const AcRequest &request) {
  obj["power"] = request.power;
  obj["mode"] = request.mode;
  obj["degrees"] = request.degrees;
  obj["fan"] = request.fan;
  obj["turbo"] = request.turbo;
  obj["quiet"] = request.quiet;
  obj["sleep"] = request.sleep;
  obj["swingV"] = request.swingV;
  obj["swingH"] = request.swingH;
  obj["filter"] = request.filter;
}

String selfTestReportJson(const SelfTestReport &report) {
  String out;
  out.reserve(1200);
  out += "{\"ok\":";
  out += report.ok ? "true" : "false";
  out += ",\"sent\":";
  out += report.sent ? "true" : "false";
  out += ",\"received\":";
  out += report.received ? "true" : "false";
  out += ",\"protocolOk\":";
  out += report.protocolOk ? "true" : "false";
  out += ",\"decodedState\":";
  out += report.decodedState ? "true" : "false";
  out += ",\"stateOk\":";
  out += report.stateOk ? "true" : "false";
  out += ",\"message\":";
  out += jsonString(report.message);
  out += ",\"expected\":";
  out += selfTestStateJson(report.expectedState);
  out += ",\"receivedProtocol\":";
  out += jsonString(typeToString(report.receivedProtocol));
  out += ",\"receivedBits\":";
  out += String(report.receivedBits);
  out += ",\"receivedValue\":";
  out += jsonString(uint64ToHexString(report.receivedValue));
  out += ",\"receivedSummary\":";
  out += jsonString(report.receivedSummary);
  out += ",\"receivedAcDescription\":";
  out += jsonString(report.receivedAcDescription);
  out += ",\"receivedState\":";
  out += report.decodedState ? selfTestStateJson(report.receivedState) : String("null");
  out += ",\"mismatches\":[";
  int start = 0;
  bool first = true;
  while (start < report.mismatches.length()) {
    int end = report.mismatches.indexOf('|', start);
    if (end < 0) end = report.mismatches.length();
    if (!first) out += ",";
    out += jsonString(report.mismatches.substring(start, end));
    first = false;
    start = end + 1;
  }
  out += "]}";
  return out;
}

bool snapshotToCommonState(const CaptureSnapshot &snapshot, stdAc::state_t *state, const stdAc::state_t *prev = nullptr) {
  if (!snapshot.available || state == nullptr) return false;
  decode_results decoded;
  memset(&decoded, 0, sizeof(decoded));
  decoded.decode_type = snapshot.protocol;
  decoded.bits = snapshot.bits;
  if (snapshot.stateLen > 0) {
    memcpy(decoded.state, snapshot.state, min(snapshot.stateLen, static_cast<uint16_t>(kStateSizeMax)));
  } else {
    decoded.value = snapshot.value;
  }
  return IRAcUtils::decodeToState(&decoded, state, prev);
}

String captureSnapshotJson(const CaptureSnapshot &snapshot) {
  String out;
  out.reserve(900);
  stdAc::state_t decodedState;
  bool decoded = snapshotToCommonState(snapshot, &decodedState, nullptr);
  out += "{\"available\":";
  out += snapshot.available ? "true" : "false";
  out += ",\"protocol\":";
  out += jsonString(typeToString(snapshot.protocol));
  out += ",\"bits\":";
  out += String(snapshot.bits);
  out += ",\"value\":";
  out += jsonString(uint64ToHexString(snapshot.value));
  out += ",\"stateLen\":";
  out += String(snapshot.stateLen);
  out += ",\"stateHex\":";
  out += jsonString(snapshot.stateLen > 0 ? bytesToHexString(snapshot.state, snapshot.stateLen) : "");
  out += ",\"rawLen\":";
  out += String(snapshot.rawLen);
  out += ",\"acDescription\":";
  out += jsonString(snapshot.acDescription);
  out += ",\"decoded\":";
  out += decoded ? selfTestStateJson(decodedState) : String("null");
  out += ",\"request\":";
  out += decoded ? acRequestJson(requestFromCommonState(decodedState)) : String("null");
  out += "}";
  return out;
}

void appendCodeCompareMismatch(String &mismatches, const String &text) {
  if (mismatches.length()) mismatches += "|";
  mismatches += text;
}

bool commonStatesSemanticallyEqual(const stdAc::state_t &reference, const stdAc::state_t &generated, String &mismatches) {
  bool ok = true;
  if (reference.power != generated.power) {
    appendCodeCompareMismatch(mismatches, "电源不同");
    ok = false;
  }
  if (!reference.power && !generated.power) return ok;
  if (reference.mode != generated.mode) {
    appendCodeCompareMismatch(mismatches, "模式不同");
    ok = false;
  }
  if (fabs(reference.degrees - generated.degrees) > 0.25f) {
    appendCodeCompareMismatch(mismatches, "温度不同");
    ok = false;
  }
  if (reference.fanspeed != generated.fanspeed) {
    appendCodeCompareMismatch(mismatches, "风速不同");
    ok = false;
  }
  if (reference.turbo != generated.turbo) {
    appendCodeCompareMismatch(mismatches, "强劲不同");
    ok = false;
  }
  if (reference.quiet != generated.quiet) {
    appendCodeCompareMismatch(mismatches, "静音不同");
    ok = false;
  }
  if ((reference.sleep >= 0) != (generated.sleep >= 0)) {
    appendCodeCompareMismatch(mismatches, "睡眠不同");
    ok = false;
  }
  if (stateSwingVActive(reference.swingv) != stateSwingVActive(generated.swingv)) {
    appendCodeCompareMismatch(mismatches, "上下摆风不同");
    ok = false;
  }
  if (stateSwingHActive(reference.swingh) != stateSwingHActive(generated.swingh)) {
    appendCodeCompareMismatch(mismatches, "左右摆风不同");
    ok = false;
  }
  if (reference.filter != generated.filter) {
    appendCodeCompareMismatch(mismatches, "滤网/出风口不同");
    ok = false;
  }
  return ok;
}

String codeCompareMismatchesJson(const String &mismatches) {
  String out = "[";
  int start = 0;
  bool first = true;
  while (start < mismatches.length()) {
    int end = mismatches.indexOf('|', start);
    if (end < 0) end = mismatches.length();
    if (!first) out += ",";
    out += jsonString(mismatches.substring(start, end));
    first = false;
    start = end + 1;
  }
  out += "]";
  return out;
}

bool strictCaptureEqual(const CaptureSnapshot &reference, const CaptureSnapshot &generated, String &mismatches) {
  bool ok = true;
  if (reference.protocol != generated.protocol) {
    appendCodeCompareMismatch(mismatches, "协议不同");
    ok = false;
  }
  if (reference.bits != generated.bits) {
    appendCodeCompareMismatch(mismatches, "位数不同");
    ok = false;
  }
  if (reference.stateLen > 0 || generated.stateLen > 0) {
    if (reference.stateLen != generated.stateLen ||
        memcmp(reference.state, generated.state, min(reference.stateLen, generated.stateLen)) != 0) {
      appendCodeCompareMismatch(mismatches, "状态字节不同");
      ok = false;
    }
  } else if (reference.value != generated.value) {
    appendCodeCompareMismatch(mismatches, "码值不同");
    ok = false;
  }
  return ok;
}

SelfTestReport runAcSelfTest(const AcRequest &request, uint32_t timeoutMs = 3500) {
  SelfTestReport report;
  report.expectedRequest = normalizedAcRequest(request);

  if (!IRac::isProtocolSupported(config.acProtocol)) {
    report.message = "当前空调协议不支持库发送，无法自发自收校验";
    lastActionResult = "self-test failed: unsupported protocol";
    addDecisionLog("self_test_failed", "manual", report.message.c_str(), roomTempC, NAN, report.expectedRequest.degrees);
    return report;
  }

  irBusy = true;
  irrecv.resume();
  delay(80);

  if (currentUsesPatchedGreeYbofb()) {
    report.sent = sendGreeStateBytesNow(report.expectedRequest, &report.expectedState);
  } else {
    prepareAcState(report.expectedRequest);
    report.expectedState = ac.next;
    report.sent = ac.sendAc();
  }
  if (!report.sent) {
    irBusy = false;
    report.message = "红外发送失败，未执行接收校验";
    lastActionResult = "self-test failed: send failed";
    addDecisionLog("self_test_failed", "manual", report.message.c_str(), roomTempC, NAN, report.expectedRequest.degrees);
    return report;
  }

  report.received = waitForSelfTestDecode(timeoutMs);
  if (report.received) {
    storeCaptureSnapshot(irResults);
    report.receivedProtocol = irResults.decode_type;
    report.receivedBits = irResults.bits;
    report.receivedValue = irResults.value;
    report.receivedSummary = resultToHumanReadableBasic(&irResults);
    report.receivedAcDescription = IRAcUtils::resultAcToString(&irResults);
    report.protocolOk = report.receivedProtocol == config.acProtocol;
    report.decodedState = IRAcUtils::decodeToState(&irResults, &report.receivedState, &report.expectedState);
    if (!report.protocolOk) appendSelfTestMismatch(report, "协议不一致");
    if (report.decodedState) {
      compareSelfTestState(report);
    } else {
      appendSelfTestMismatch(report, "无法解析为空调通用状态");
      report.stateOk = false;
    }
    irrecv.resume();
  }

  irBusy = false;

  if (!report.received) {
    report.message = "已发送，但本机接收头未收到红外；请调整发射管和接收头位置";
    lastActionResult = "self-test failed: no receive";
  } else if (report.protocolOk && report.decodedState && report.stateOk) {
    report.ok = true;
    report.message = "自发自收校验通过";
    lastActionResult = "self-test passed " + typeToString(report.receivedProtocol);
  } else {
    report.message = "自发自收校验未通过：" + report.mismatches;
    lastActionResult = "self-test failed: " + report.mismatches;
  }

  rememberAcState(report.expectedRequest);
  if (report.expectedRequest.power) lastSentSetpoint = report.expectedRequest.degrees;
  addControlEvent("manual", report.ok ? "self_test" : "failed", report.expectedRequest, NAN, report.expectedRequest.degrees);
  addDecisionLog(report.ok ? "self_test_ok" : "self_test_failed", "manual", report.message.c_str(),
                 roomTempC, NAN, report.expectedRequest.degrees);
  return report;
}

bool sendAcNow(const AcRequest &request, const char *source = "manual", const char *action = "send",
               float target = NAN, float setpoint = NAN) {
  AcRequest normalized = normalizedAcRequest(request);
  int exactLearnedIdx = findSemanticLearnedCommand(normalized);
  if (exactLearnedIdx >= 0) {
    bool ok = sendLearnedNow(learned[exactLearnedIdx].id);
    if (ok) {
      rememberAcState(normalized);
      if (normalized.power) lastSentSetpoint = learned[exactLearnedIdx].degrees;
    }
    addControlEvent(source, ok ? "sent_raw" : "failed", normalized, target, isnan(setpoint) ? learned[exactLearnedIdx].degrees : setpoint);
    addDecisionLog(ok ? "sent_raw" : "failed", source, lastActionResult.c_str(), roomTempC, target,
                   isnan(setpoint) ? learned[exactLearnedIdx].degrees : setpoint);
    return ok;
  }

  if (IRac::isProtocolSupported(config.acProtocol)) {
    irBusy = true;
    irrecv.disableIRIn();
    bool ok = false;
    if (currentUsesPatchedGreeYbofb()) {
      ok = sendGreeStateBytesNow(normalized);
    } else {
      prepareAcState(normalized);
      ok = ac.sendAc();
    }
    irrecv.enableIRIn();
    irBusy = false;
    lastActionResult = ok ? String(currentUsesPatchedGreeYbofb() ? "sent patched AC " : "sent AC ") +
                                typeToString(config.acProtocol) +
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

bool isHeatCoolMode(const String &mode) {
  return mode == "heat" || mode == "cool";
}

String smartModeForTarget(float target) {
  if (isnan(roomTempC)) return "auto";
  float enterBand = max(config.deadband, 0.35f);
  float reverseBand = max(enterBand + 0.85f, 1.15f);
  float forceReverseBand = max(reverseBand + 0.75f, 1.9f);
  uint32_t intervalHoldMs = static_cast<uint32_t>(config.controlIntervalSec) * 4UL * 1000UL;
  uint32_t minHoldMs = intervalHoldMs > 12UL * 60UL * 1000UL ? intervalHoldMs : 12UL * 60UL * 1000UL;
  bool holdTimeOk = lastAutoModeChangeMs == 0 || millis() - lastAutoModeChangeMs >= minHoldMs;
  String heldMode = isHeatCoolMode(lastAutoSentMode)
                        ? lastAutoSentMode
                        : (config.remotePower && isHeatCoolMode(config.remoteMode) ? config.remoteMode : "");

  if (heldMode == "cool") {
    float tooCold = target - roomTempC;
    if (tooCold > forceReverseBand || (tooCold > reverseBand && holdTimeOk)) return "heat";
    return "cool";
  }
  if (heldMode == "heat") {
    float tooHot = roomTempC - target;
    if (tooHot > forceReverseBand || (tooHot > reverseBand && holdTimeOk)) return "cool";
    return "heat";
  }

  if (target - roomTempC > enterBand) return "heat";
  if (roomTempC - target > enterBand) return "cool";
  if (isHeatCoolMode(heldMode)) return heldMode;
  return "auto";
}

String effectiveControlMode(float target, const String &configuredMode) {
  if (configuredMode == "smart") return smartModeForTarget(target);
  return configuredMode;
}

float temperatureDemand(float target, const String &mode) {
  if (isnan(roomTempC)) return 0.0f;
  if (mode == "heat") return target - roomTempC;
  if (mode == "auto") return fabs(roomTempC - target);
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

uint32_t sleepCurveCycleKey(uint16_t nowMinute) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 5)) return 0;
  timeinfo.tm_hour = 12;
  timeinfo.tm_min = 0;
  timeinfo.tm_sec = 0;
  timeinfo.tm_isdst = -1;
  if (nowMinute < config.sleepStartMinute % 1440) {
    timeinfo.tm_mday -= 1;
  }
  mktime(&timeinfo);
  return static_cast<uint32_t>(timeinfo.tm_year + 1900) * 1000UL +
         static_cast<uint32_t>(timeinfo.tm_yday + 1);
}

void resetAutoSendMemory() {
  lastAutoSentSetpoint = NAN;
  lastAutoSentMode = "";
  lastAutoSentFan = "";
  lastAutoSentTurbo = false;
  lastAutoSentQuiet = false;
  lastAutoSentSleep = false;
  lastAutoModeChangeMs = 0;
}

bool sameAutoRequestShape(const AcRequest &request) {
  return request.mode == lastAutoSentMode &&
         request.fan == lastAutoSentFan &&
         request.turbo == lastAutoSentTurbo &&
         request.quiet == lastAutoSentQuiet &&
         request.sleep == lastAutoSentSleep;
}

void rememberAutoRequestShape(const AcRequest &request) {
  if (request.mode != lastAutoSentMode) lastAutoModeChangeMs = millis();
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
  bool clockSynced = getLocalMinuteOfDay(&nowMinute);
  uint16_t elapsed = 0;
  bool curveActive = clockSynced && config.autoEnabled && sleepCurveIsActive(nowMinute, &elapsed);
  if (!curveActive) {
    if (clockSynced && sleepCurveWasActive && config.humidityControlEnabled) {
      addDecisionLog("end_hold", "end", "曲线结束，独立湿度控制继续接管，不执行关机", roomTempC, NAN, NAN);
      resetAutoSendMemory();
      sleepCurveWasActive = false;
    } else if (clockSynced && sleepCurveWasActive && config.curveEndAction == "poweroff") {
      uint32_t cycleKey = sleepCurveCycleKey(nowMinute);
      if (cycleKey != 0 && config.lastCurveEndPoweroffKey == cycleKey) {
        addDecisionLog("skip_end_once", "end", "本睡眠周期结束关机已执行，跳过重复关机", roomTempC, NAN, NAN);
        resetAutoSendMemory();
        lastControlMs = millis();
        sleepCurveWasActive = false;
        return;
      }
      if (cycleKey != 0) {
        config.lastCurveEndPoweroffKey = cycleKey;
        saveConfig();
      }
      pendingAc.power = false;
      pendingAc.mode = lastAutoSentMode.length() ? lastAutoSentMode : config.remoteMode;
      if (pendingAc.mode == "smart") pendingAc.mode = "auto";
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
    } else if (clockSynced && sleepCurveWasActive) {
      addDecisionLog("end_hold", "end", "曲线结束，保持当前空调状态，不发射红外", roomTempC, NAN, NAN);
    }
    if (clockSynced) sleepCurveWasActive = false;
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

  bool dehumidifyBlockedByCold = humidityEnabledNow && humidityHigh && tempTooLowForDry;
  bool shouldDehumidify = humidityHigh && !dehumidifyBlockedByCold;
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
  String configuredMode = curveActive ? config.autoMode : "smart";
  String activeMode = shouldDehumidify ? "dry" : effectiveControlMode(target, configuredMode);
  float demand = temperatureDemand(target, activeMode);

  pendingAc.power = true;
  pendingAc.mode = activeMode;
  pendingAc.fan = curveActive
                      ? fanForSleepDemand(demand, quietStage, shouldDehumidify)
                      : (shouldDehumidify ? "low" : "auto");
  pendingAc.turbo = curveActive ? turboForSleepDemand(demand, quietStage, shouldDehumidify) : false;
  pendingAc.quiet = curveActive ? quietForSleepDemand(quietStage, true) : true;
  pendingAc.sleep = false;
  pendingAc.swingV = false;
  pendingAc.swingH = false;
  pendingAc.filter = false;
  pendingAc.degrees = closedLoopSetpoint(target, activeMode, stage);

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
  if (dehumidifyBlockedByCold) note += "，湿度偏高但室温偏低，先控温再除湿";
  if (configuredMode == "smart" && !shouldDehumidify) note += "，智能模式判定为 " + pendingAc.mode;
  note += "，风速 " + pendingAc.fan + (pendingAc.quiet ? " / 静音" : "") +
          (pendingAc.turbo ? " / 强劲" : "") +
          "，温差 " + String(demand, 1) + "℃";
  addDecisionLog(shouldDehumidify ? "queue_dehumidify" : "queue_send", stage.c_str(), note.c_str(), roomTempC, target, pendingAc.degrees);
  lastAutoSentSetpoint = pendingAc.degrees;
  rememberAutoRequestShape(pendingAc);
  lastControlMs = millis();
}

void addConfigToJson(JsonObject obj) {
  ensureAcProfiles();
  obj["activeAcProfileId"] = activeAcProfileId;
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
  obj["lastCurveEndPoweroffKey"] = config.lastCurveEndPoweroffKey;
  JsonArray curve = obj["curve"].to<JsonArray>();
  for (uint8_t i = 0; i < config.curveCount; i++) {
    JsonObject point = curve.add<JsonObject>();
    point["minute"] = config.curve[i].minute;
    point["temp"] = config.curve[i].temp;
  }
  JsonArray profiles = obj["acProfiles"].to<JsonArray>();
  for (uint8_t i = 0; i < acProfileCount; i++) {
    JsonObject item = profiles.add<JsonObject>();
    addAcProfileToJson(item, acProfiles[i]);
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
  bool curveActive = timeSynced && config.autoEnabled && sleepCurveIsActive(minuteOfDay, &elapsed);
  JsonObject autoTarget = doc["autoTarget"].to<JsonObject>();
  autoTarget["enabled"] = config.autoEnabled;
  autoTarget["active"] = curveActive || config.humidityControlEnabled;
  autoTarget["curveActive"] = curveActive;
  autoTarget["configuredMode"] = config.autoMode;
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
    bool humidityHigh = config.curveHumidityEnabled && !isnan(roomHumidity) &&
                        roomHumidity > config.targetHumidity + config.humidityDeadband;
    bool tempTooLowForDry = humidityHigh && !isnan(roomTempC) &&
                            roomTempC < target - max(0.5f, config.deadband);
    String activeMode = humidityHigh && !tempTooLowForDry ? "dry" : effectiveControlMode(target, config.autoMode);
    float demand = temperatureDemand(target, activeMode);
    autoTarget["mode"] = activeMode;
    autoTarget["elapsedMinute"] = elapsed;
    autoTarget["temperatureC"] = target;
    autoTarget["setpointC"] = closedLoopSetpoint(target, activeMode, stage);
    autoTarget["roomErrorC"] = isnan(roomTempC) ? 0 : roomTempC - target;
    autoTarget["stage"] = stage;
    autoTarget["fan"] = fanForSleepDemand(demand, quietStage, activeMode == "dry");
    autoTarget["turbo"] = turboForSleepDemand(demand, quietStage, activeMode == "dry");
    autoTarget["quiet"] = quietForSleepDemand(quietStage, true);
    autoTarget["sleep"] = false;
  } else if (config.humidityControlEnabled) {
    float target = clampFloat(config.humidityTargetTemp, config.minSetpoint, config.maxSetpoint);
    bool humidityHigh = !isnan(roomHumidity) &&
                        roomHumidity > config.targetHumidity + config.humidityDeadband;
    bool tempTooLowForDry = humidityHigh && !isnan(roomTempC) &&
                            roomTempC < target - max(0.5f, config.deadband);
    String activeMode = humidityHigh && !tempTooLowForDry ? "dry" : effectiveControlMode(target, "smart");
    autoTarget["mode"] = activeMode;
    autoTarget["elapsedMinute"] = nullptr;
    autoTarget["temperatureC"] = target;
    autoTarget["setpointC"] = closedLoopSetpoint(target, activeMode, "humidity");
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
    cap["stateLen"] = lastCapture.stateLen;
    if (lastCapture.stateLen > 0) cap["stateHex"] = bytesToHexString(lastCapture.state, lastCapture.stateLen);
    cap["rawLen"] = lastCapture.rawLen;
    cap["summary"] = lastCapture.summary;
    cap["acDescription"] = lastCapture.acDescription;
    cap["ageMs"] = millis() - lastCapture.capturedAtMs;
    stdAc::state_t decodedState;
    bool decoded = snapshotToCommonState(lastCapture, &decodedState, nullptr);
    cap["decodedAvailable"] = decoded;
    if (decoded) {
      JsonObject request = cap["request"].to<JsonObject>();
      addAcRequestToJson(request, requestFromCommonState(decodedState));
      cap["decodedModel"] = decodedState.model;
    }
  }
}

void handleLive() {
  JsonDocument doc;
  addLiveToJson(doc);

  String out;
  serializeJson(doc, out);
  sendJsonResponse(200, out);
}

void handleTempHistory() {
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  String chunk;
  chunk.reserve(1024);
  appendJsonStreamChunk(chunk, "{\"version\":3,\"intervalSec\":60,\"hours\":72,\"usesEpoch\":");
  appendJsonStreamChunk(chunk, tempHistoryUsesEpoch ? "true" : "false");
  appendJsonStreamChunk(chunk, ",\"persistent\":true,\"count\":");
  appendJsonStreamChunk(chunk, String(tempHistoryCount));

  appendJsonStreamChunk(chunk, ",\"m\":[");
  for (uint16_t i = 0; i < tempHistoryCount; i++) {
    uint16_t idx = (tempHistoryHead + kTempHistoryPoints - tempHistoryCount + i) % kTempHistoryPoints;
    if (i > 0) appendJsonStreamChunk(chunk, ",");
    appendJsonStreamChunk(chunk, String(static_cast<unsigned long>(tempHistory[idx].minute)));
  }

  appendJsonStreamChunk(chunk, "],\"t\":[");
  for (uint16_t i = 0; i < tempHistoryCount; i++) {
    uint16_t idx = (tempHistoryHead + kTempHistoryPoints - tempHistoryCount + i) % kTempHistoryPoints;
    if (i > 0) appendJsonStreamChunk(chunk, ",");
    appendJsonStreamChunk(chunk, String(static_cast<int>(tempHistory[idx].temp10)));
  }

  appendJsonStreamChunk(chunk, "],\"h\":[");
  for (uint16_t i = 0; i < tempHistoryCount; i++) {
    uint16_t idx = (tempHistoryHead + kTempHistoryPoints - tempHistoryCount + i) % kTempHistoryPoints;
    if (i > 0) appendJsonStreamChunk(chunk, ",");
    if (tempHistory[idx].humidity10 == INT16_MIN) {
      appendJsonStreamChunk(chunk, "null");
    } else {
      appendJsonStreamChunk(chunk, String(static_cast<int>(tempHistory[idx].humidity10)));
    }
  }

  appendJsonStreamChunk(chunk, "]}");
  flushJsonStreamChunk(chunk);
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

void appendJsonEscaped(String &out, const char *text) {
  out += "\"";
  if (text != nullptr) {
    for (const char *p = text; *p != '\0'; p++) {
      char c = *p;
      if (c == '"' || c == '\\') {
        out += '\\';
        out += c;
      } else if (c == '\n') {
        out += "\\n";
      } else if (c == '\r') {
        out += "\\r";
      } else if (static_cast<uint8_t>(c) < 0x20) {
        out += ' ';
      } else {
        out += c;
      }
    }
  }
  out += "\"";
}

void appendJsonTenths(String &out, const char *name, int16_t value) {
  out += ",\"";
  out += name;
  out += "\":";
  if (value == INT16_MIN) out += "null";
  else out += String(value / 10.0f, 1);
}

void handleControlEvents() {
  beginJsonStreamResponse();
  String chunk;
  chunk.reserve(1024);
  appendJsonStreamChunk(chunk, "{\"usesEpoch\":");
  appendJsonStreamChunk(chunk, controlEventsUseEpoch ? "true" : "false");
  appendJsonStreamChunk(chunk, ",\"persistent\":true,\"capacity\":");
  appendJsonStreamChunk(chunk, String(kControlEventPoints));
  appendJsonStreamChunk(chunk, ",\"count\":");
  appendJsonStreamChunk(chunk, String(controlEventCount));
  appendJsonStreamChunk(chunk, ",\"events\":[");
  for (uint16_t i = 0; i < controlEventCount; i++) {
    uint16_t idx = (controlEventHead + kControlEventPoints - controlEventCount + i) % kControlEventPoints;
    const ControlEvent &event = controlEvents[idx];
    if (i > 0) appendJsonStreamChunk(chunk, ",");
    appendJsonStreamChunk(chunk, "{\"minute\":");
    appendJsonStreamChunk(chunk, String(event.minute));
    appendJsonTenthsStream(chunk, "room", event.room10);
    appendJsonTenthsStream(chunk, "target", event.target10);
    appendJsonTenthsStream(chunk, "setpoint", event.setpoint10);
    appendJsonStreamChunk(chunk, ",\"source\":");
    appendJsonEscapedStream(chunk, event.source);
    appendJsonStreamChunk(chunk, ",\"action\":");
    appendJsonEscapedStream(chunk, event.action);
    appendJsonStreamChunk(chunk, ",\"mode\":");
    appendJsonEscapedStream(chunk, event.mode);
    appendJsonStreamChunk(chunk, ",\"fan\":");
    appendJsonEscapedStream(chunk, event.fan);
    appendJsonStreamChunk(chunk, ",\"power\":");
    appendJsonStreamChunk(chunk, event.power ? "true" : "false");
    appendJsonStreamChunk(chunk, ",\"turbo\":");
    appendJsonStreamChunk(chunk, event.turbo ? "true" : "false");
    appendJsonStreamChunk(chunk, ",\"quiet\":");
    appendJsonStreamChunk(chunk, event.quiet ? "true" : "false");
    appendJsonStreamChunk(chunk, ",\"sleep\":");
    appendJsonStreamChunk(chunk, event.sleep ? "true" : "false");
    appendJsonStreamChunk(chunk, "}");
  }
  appendJsonStreamChunk(chunk, "]}");
  flushJsonStreamChunk(chunk);
  server.sendContent("");
}

void handleControlLog() {
  beginJsonStreamResponse();
  String chunk;
  chunk.reserve(1024);
  appendJsonStreamChunk(chunk, "{\"usesEpoch\":");
  appendJsonStreamChunk(chunk, decisionLogUsesEpoch ? "true" : "false");
  appendJsonStreamChunk(chunk, ",\"persistent\":true,\"capacity\":");
  appendJsonStreamChunk(chunk, String(kDecisionLogPoints));
  appendJsonStreamChunk(chunk, ",\"count\":");
  appendJsonStreamChunk(chunk, String(decisionLogCount));
  appendJsonStreamChunk(chunk, ",\"logs\":[");
  for (uint16_t i = 0; i < decisionLogCount; i++) {
    uint16_t idx = (decisionLogHead + kDecisionLogPoints - decisionLogCount + i) % kDecisionLogPoints;
    const DecisionLogEntry &entry = decisionLog[idx];
    if (i > 0) appendJsonStreamChunk(chunk, ",");
    appendJsonStreamChunk(chunk, "{\"minute\":");
    appendJsonStreamChunk(chunk, String(entry.minute));
    appendJsonStreamChunk(chunk, ",\"action\":");
    appendJsonEscapedStream(chunk, entry.action);
    appendJsonStreamChunk(chunk, ",\"stage\":");
    appendJsonEscapedStream(chunk, entry.stage);
    appendJsonStreamChunk(chunk, ",\"note\":");
    appendJsonEscapedStream(chunk, entry.note);
    appendJsonTenthsStream(chunk, "room", entry.room10);
    appendJsonTenthsStream(chunk, "target", entry.target10);
    appendJsonTenthsStream(chunk, "setpoint", entry.setpoint10);
    appendJsonStreamChunk(chunk, "}");
  }
  appendJsonStreamChunk(chunk, "]}");
  flushJsonStreamChunk(chunk);
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
    if (learned[i].hasAcMeta) {
      String meta = String(learned[i].power ? "on " : "off ") + learned[i].mode + " " +
                    String(learned[i].degrees, 1) + "C " + learned[i].fan;
      if (learned[i].turbo) meta += " turbo";
      if (learned[i].quiet) meta += " quiet";
      if (learned[i].sleep) meta += " sleep";
      if (learned[i].swingV) meta += " swingV";
      if (learned[i].swingH) meta += " swingH";
      if (learned[i].filter) meta += " filter";
      if (learned[i].model >= 0) meta += " model=" + String(learned[i].model);
      item["meta"] = meta;
    } else {
      item["meta"] = "";
    }
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
  sendJsonResponse(200, out);
}

void handleLearnedMatchReport() {
  const int16_t models[] = {
      static_cast<int16_t>(gree_ac_remote_model_t::YAW1F),
      static_cast<int16_t>(gree_ac_remote_model_t::YBOFB),
      static_cast<int16_t>(gree_ac_remote_model_t::YX1FSF),
  };
  uint16_t strictCount[3] = {};
  uint16_t semanticCount[3] = {};
  uint16_t hiddenOnlyCount[3] = {};
  uint16_t diffSum[3] = {};
  uint16_t analyzable = 0;

  String items;
  items.reserve(9000);
  items += "[";
  bool firstItem = true;
  for (uint8_t i = 0; i < learnedCount; i++) {
    if (learned[i].protocol != decode_type_t::GREE || learned[i].bits != kGreeBits || learned[i].value == 0) continue;
    uint8_t reference[kGreeStateLength] = {};
    valueToStateBytes(learned[i].value, reference, kGreeStateLength);
    stdAc::state_t referenceState;
    bool referenceDecoded = greeStateToCommon(reference, &referenceState);
    AcRequest request = referenceDecoded ? requestFromCommonStateRaw(referenceState) : requestFromLearnedCommand(learned[i]);
    analyzable++;

    if (!firstItem) items += ",";
    items += "{\"id\":";
    items += String(learned[i].id);
    items += ",\"name\":";
    items += jsonString(learned[i].name);
    items += ",\"reference\":";
    items += jsonString(bytesToHexString(reference, kGreeStateLength));
    items += ",\"decoded\":";
    items += referenceDecoded ? selfTestStateJson(referenceState) : String("null");
    items += ",\"request\":";
    items += acRequestJson(request);
    items += ",\"models\":[";

    bool firstModel = true;
    for (uint8_t m = 0; m < 3; m++) {
      uint8_t generated[kGreeStateLength] = {};
      bool generatedOk = generateGreeStateBytes(request, models[m], generated);
      stdAc::state_t generatedState;
      bool generatedDecoded = generatedOk && greeStateToCommon(generated, &generatedState);
      String semanticMismatches;
      bool semanticOk = referenceDecoded && generatedDecoded &&
                        commonStatesSemanticallyEqual(referenceState, generatedState, semanticMismatches);
      bool strictOk = generatedOk && memcmp(reference, generated, kGreeStateLength) == 0;
      bool hiddenOnly = semanticOk && !strictOk && greeHiddenOnlyDiff(reference, generated);
      uint8_t diffs = generatedOk ? countByteDiffs(reference, generated, kGreeStateLength) : kGreeStateLength;

      if (strictOk) strictCount[m]++;
      if (semanticOk) semanticCount[m]++;
      if (hiddenOnly) hiddenOnlyCount[m]++;
      diffSum[m] += diffs;

      if (!firstModel) items += ",";
      items += "{\"model\":";
      items += String(models[m]);
      items += ",\"generated\":";
      items += jsonString(generatedOk ? bytesToHexString(generated, kGreeStateLength) : "");
      items += ",\"strictOk\":";
      items += strictOk ? "true" : "false";
      items += ",\"semanticOk\":";
      items += semanticOk ? "true" : "false";
      items += ",\"hiddenOnly\":";
      items += hiddenOnly ? "true" : "false";
      items += ",\"diffCount\":";
      items += String(diffs);
      items += ",\"diffs\":";
      items += generatedOk ? byteDiffsJson(reference, generated, kGreeStateLength) : String("[]");
      items += ",\"semanticMismatches\":";
      items += codeCompareMismatchesJson(semanticMismatches);
      items += "}";
      firstModel = false;
    }
    items += "]}";
    firstItem = false;
  }
  items += "]";

  int best = 0;
  for (uint8_t m = 1; m < 3; m++) {
    if (semanticCount[m] > semanticCount[best] ||
        (semanticCount[m] == semanticCount[best] && strictCount[m] > strictCount[best]) ||
        (semanticCount[m] == semanticCount[best] && strictCount[m] == strictCount[best] && diffSum[m] < diffSum[best])) {
      best = m;
    }
  }

  String out;
  out.reserve(items.length() + 1200);
  out += "{\"total\":";
  out += String(learnedCount);
  out += ",\"analyzable\":";
  out += String(analyzable);
  out += ",\"bestModel\":";
  out += String(models[best]);
  out += ",\"models\":[";
  for (uint8_t m = 0; m < 3; m++) {
    if (m > 0) out += ",";
    out += "{\"model\":";
    out += String(models[m]);
    out += ",\"strict\":";
    out += String(strictCount[m]);
    out += ",\"semantic\":";
    out += String(semanticCount[m]);
    out += ",\"hiddenOnly\":";
    out += String(hiddenOnlyCount[m]);
    out += ",\"diffSum\":";
    out += String(diffSum[m]);
    out += "}";
  }
  out += "],\"items\":";
  out += items;
  out += "}";
  sendJsonResponse(200, out);
}

void handleWifiPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  String newSsid = doc["ssid"] | "";
  String newPassword = doc["password"] | "";
  newSsid.trim();

  if (newSsid == kDefaultWifiSsid && newPassword.length() == 0 && strlen(kDefaultWifiPassword) > 0) {
    newPassword = kDefaultWifiPassword;
  } else if (newSsid.length() && newPassword.length() == 0) {
    if (newSsid == config.staSsid && config.staPassword.length() > 0) {
      newPassword = config.staPassword;
    } else {
      sendError(400, "请输入 WiFi 密码；如果是开放热点，请先清除 WiFi 后再配置");
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
  if (hasStationCredentials()) {
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

int waitForWifiScanIdle(uint32_t timeoutMs) {
  int status = WiFi.scanComplete();
  if (status >= 0) {
    WiFi.scanDelete();
    return status;
  }
  if (status != WIFI_SCAN_RUNNING) return status;

  uint32_t startMs = millis();
  while (millis() - startMs < timeoutMs) {
    delay(100);
    status = WiFi.scanComplete();
    if (status != WIFI_SCAN_RUNNING) {
      WiFi.scanDelete();
      return status;
    }
  }

  esp_wifi_scan_stop();
  delay(80);
  WiFi.scanDelete();
  return WIFI_SCAN_FAILED;
}

void prepareWifiScanMode() {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (apStarted && !connected) {
    WiFi.mode(WIFI_AP_STA);
    disableWifiPowerSave();
    esp_wifi_disconnect();
    wifiConnectStartMs = 0;
    lastWifiAttemptMs = millis();
    delay(220);
  } else if (!apStarted && !connected) {
    WiFi.mode(WIFI_STA);
    disableWifiPowerSave();
    delay(120);
  } else {
    disableWifiPowerSave();
  }
}

int performWifiScan() {
  wifiScanActive = true;
  prepareWifiScanMode();
  waitForWifiScanIdle(2500);

  int found = WIFI_SCAN_FAILED;
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    WiFi.scanDelete();
    bool passive = attempt == 1;
    uint32_t maxMs = passive ? 120 : 180;
    found = WiFi.scanNetworks(false, true, passive, maxMs, 0);
    Serial.println("WiFi scan attempt " + String(attempt + 1) +
                   (passive ? " passive" : " active") +
                   " result " + String(found));
    if (found >= 0) {
      wifiScanActive = false;
      lastWifiAttemptMs = millis();
      return found;
    }

    if (found == WIFI_SCAN_RUNNING) {
      int completed = waitForWifiScanIdle(4000);
      Serial.println("WiFi scan completed after wait result " + String(completed));
      if (completed >= 0) {
        wifiScanActive = false;
        lastWifiAttemptMs = millis();
        return completed;
      }
    } else {
      esp_wifi_scan_stop();
      WiFi.scanDelete();
    }
    delay(180 + attempt * 220);
  }
  wifiScanActive = false;
  lastWifiAttemptMs = millis();
  return found;
}

void printWifiScanResultsToSerial(int found) {
  if (found < 0) {
    Serial.println("Serial WiFi scan failed result " + String(found));
    return;
  }

  Serial.println("Serial WiFi scan found " + String(found) + " raw networks");
  uint8_t limit = static_cast<uint8_t>(min(found, 64));
  for (uint8_t i = 0; i < limit; i++) {
    String ssid = WiFi.SSID(i);
    ssid.trim();
    if (!ssid.length()) ssid = "<hidden>";
    bool open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
    Serial.println(String(i + 1) + ". " + ssid +
                   " rssi=" + String(WiFi.RSSI(i)) +
                   " ch=" + String(WiFi.channel(i)) +
                   " auth=" + (open ? "open" : "secured"));
  }
  WiFi.scanDelete();
}

void runSerialWifiScan() {
  Serial.println("Serial WiFi scan command received");
  Serial.println("mode=" + String(static_cast<int>(WiFi.getMode())) +
                 " status=" + String(static_cast<int>(WiFi.status())) +
                 " ap=" + String(apStarted ? 1 : 0));
  int found = performWifiScan();
  printWifiScanResultsToSerial(found);
}

void runSerialSelfTest(bool forcePowerOn = false) {
  AcRequest request = currentRemoteRequest();
  if (forcePowerOn) request.power = true;
  Serial.println("Serial IR self-test command received");
  Serial.println("protocol=" + typeToString(config.acProtocol) +
                 " model=" + String(config.acModel) +
                 " power=" + String(request.power ? "on" : "off") +
                 " mode=" + request.mode +
                 " temp=" + String(request.degrees, 1) +
                 " fan=" + request.fan);
  SelfTestReport report = runAcSelfTest(request, 4500);
  Serial.println(report.message);
  Serial.println("sent=" + String(report.sent ? 1 : 0) +
                 " received=" + String(report.received ? 1 : 0) +
                 " protocolOk=" + String(report.protocolOk ? 1 : 0) +
                 " decodedState=" + String(report.decodedState ? 1 : 0) +
                 " stateOk=" + String(report.stateOk ? 1 : 0));
  if (report.received) {
    Serial.println("received protocol=" + typeToString(report.receivedProtocol) +
                   " bits=" + String(report.receivedBits) +
                   " value=" + uint64ToHexString(report.receivedValue));
    Serial.println("received AC: " + report.receivedAcDescription);
  }
  if (report.mismatches.length()) Serial.println("mismatches=" + report.mismatches);
}

void handleSerialConsole() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      serialCommandBuffer.trim();
      if (serialCommandBuffer.length()) {
        String cmd = serialCommandBuffer;
        cmd.toLowerCase();
        if (cmd == "wifi-scan" || cmd == "scan") {
          runSerialWifiScan();
        } else if (cmd == "self-test" || cmd == "ir-test") {
          runSerialSelfTest(false);
        } else if (cmd == "self-test-on" || cmd == "ir-test-on") {
          runSerialSelfTest(true);
        } else if (cmd == "help" || cmd == "?") {
          Serial.println("Commands: wifi-scan, scan, self-test, self-test-on, ir-test, ir-test-on, help");
        } else {
          Serial.println("Unknown command: " + serialCommandBuffer);
          Serial.println("Commands: wifi-scan, scan, self-test, self-test-on, ir-test, ir-test-on, help");
        }
      }
      serialCommandBuffer = "";
    } else if (c >= 32 && c <= 126 && serialCommandBuffer.length() < 64) {
      serialCommandBuffer += c;
    }
  }
}

void handleWifiScan() {
  Serial.println("WiFi scan request received mode=" + String(static_cast<int>(WiFi.getMode())) +
                 " status=" + String(static_cast<int>(WiFi.status())) +
                 " ap=" + String(apStarted ? 1 : 0));
  int found = performWifiScan();
  if (found < 0) {
    Serial.println("WiFi scan failed result " + String(found));
    if (wifiScanCacheMs > 0) {
      uint32_t ageSec = (millis() - wifiScanCacheMs) / 1000;
      String cached = "{\"count\":" + String(wifiScanCacheCount) +
                      ",\"cached\":true,\"ageSec\":" + String(ageSec) +
                      ",\"networks\":[" + wifiScanCacheNetworks + "]}";
      sendJsonResponse(200, cached);
      return;
    }
    sendError(503, "热点扫描失败，请稍后重试");
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
  wifiScanCacheNetworks = networks;
  wifiScanCacheCount = emittedCount;
  wifiScanCacheMs = millis();
  String out = "{\"count\":" + String(emittedCount) + ",\"cached\":false,\"ageSec\":0,\"networks\":[" + networks + "]}";
  sendJsonResponse(200, out);
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

void handleSelfTestAcPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  AcRequest request;
  request.power = doc["power"] | config.remotePower;
  request.mode = doc["mode"] | config.remoteMode;
  request.fan = doc["fan"] | config.remoteFan;
  request.degrees = clampFloat(doc["degrees"] | config.remoteDegrees, 16.0f, 32.0f);
  request.turbo = doc["turbo"] | config.remoteTurbo;
  request.quiet = doc["quiet"] | config.remoteQuiet;
  request.sleep = doc["sleep"] | config.remoteSleep;
  request.swingV = doc["swingV"] | config.remoteSwingV;
  request.swingH = doc["swingH"] | config.remoteSwingH;
  request.filter = doc["filter"] | config.remoteFilter;
  uint32_t timeoutMs = constrain(static_cast<uint32_t>(doc["timeoutMs"] | 3500), 1200UL, 8000UL);
  SelfTestReport report = runAcSelfTest(request, timeoutMs);
  sendJsonResponse(200, selfTestReportJson(report));
}

void handleApplyCaptureStatePost() {
  if (!lastCapture.available) {
    sendError(409, "请先用实体遥控器发送一次，让控制器捕获到空调状态");
    return;
  }
  stdAc::state_t decodedState;
  if (!snapshotToCommonState(lastCapture, &decodedState, nullptr)) {
    sendError(422, "最近捕获无法解析为空调状态，不能自动套用");
    return;
  }
  AcRequest request = requestFromCommonState(decodedState);
  if (lastCapture.protocol != decode_type_t::UNKNOWN) config.acProtocol = lastCapture.protocol;
  if (decodedState.model >= 0) config.acModel = decodedState.model;
  rememberAcState(request);
  syncConfigToActiveProfile();
  saveConfig();

  String out;
  out.reserve(520);
  out += "{\"ok\":true,\"protocol\":";
  out += jsonString(typeToString(config.acProtocol));
  out += ",\"model\":";
  out += String(config.acModel);
  out += ",\"request\":";
  out += acRequestJson(request);
  out += ",\"capture\":";
  out += captureSnapshotJson(lastCapture);
  out += "}";
  addDecisionLog("capture_applied", "manual", "已把最近实体遥控捕获套用到当前空调方案", roomTempC, NAN, request.degrees);
  sendJsonResponse(200, out);
}

String acProfilesJson() {
  ensureAcProfiles();
  String out;
  out.reserve(1600);
  out += "{\"activeId\":";
  out += String(activeAcProfileId);
  out += ",\"count\":";
  out += String(acProfileCount);
  out += ",\"max\":";
  out += String(kMaxAcProfiles);
  out += ",\"profiles\":[";
  for (uint8_t i = 0; i < acProfileCount; i++) {
    if (i > 0) out += ",";
    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    addAcProfileToJson(obj, acProfiles[i]);
    String item;
    serializeJson(doc, item);
    out += item;
  }
  out += "]}";
  return out;
}

void handleAcProfilesGet() {
  sendJsonResponse(200, acProfilesJson());
}

void handleAcProfileCreatePost() {
  if (acProfileCount >= kMaxAcProfiles) {
    sendError(409, "空调方案数量已达上限");
    return;
  }
  JsonDocument doc;
  if (!parseBody(doc)) return;
  AcProfile &profile = acProfiles[acProfileCount];
  profile = AcProfile();
  profile.id = nextAcProfileId();
  snapshotConfigToProfile(profile);
  updateAcProfileFromJson(profile, doc.as<JsonObject>());
  activeAcProfileId = profile.id;
  acProfileCount++;
  applyProfileToConfig(profile);
  saveConfig();
  sendJsonResponse(200, acProfilesJson());
}

void handleAcProfileUpdatePost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  uint8_t id = doc["id"] | activeAcProfileId;
  int idx = findAcProfileIndexById(id);
  if (idx < 0) {
    sendError(404, "空调方案不存在");
    return;
  }
  if (id == activeAcProfileId) snapshotConfigToProfile(acProfiles[idx]);
  updateAcProfileFromJson(acProfiles[idx], doc.as<JsonObject>());
  if (id == activeAcProfileId) applyProfileToConfig(acProfiles[idx]);
  saveConfig();
  sendJsonResponse(200, acProfilesJson());
}

void handleAcProfileSelectPost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  uint8_t id = doc["id"] | activeAcProfileId;
  int nextIdx = findAcProfileIndexById(id);
  if (nextIdx < 0) {
    sendError(404, "空调方案不存在");
    return;
  }
  int currentIdx = findAcProfileIndexById(activeAcProfileId);
  if (currentIdx >= 0) snapshotConfigToProfile(acProfiles[currentIdx]);
  activeAcProfileId = id;
  applyProfileToConfig(acProfiles[nextIdx]);
  saveConfig();
  sendJsonResponse(200, acProfilesJson());
}

void handleAcProfileDeletePost() {
  JsonDocument doc;
  if (!parseBody(doc)) return;
  uint8_t id = doc["id"] | 0;
  int idx = findAcProfileIndexById(id);
  if (idx < 0) {
    sendError(404, "空调方案不存在");
    return;
  }
  if (acProfileCount <= 1) {
    sendError(409, "至少保留一个空调方案");
    return;
  }
  for (uint8_t i = idx; i + 1 < acProfileCount; i++) acProfiles[i] = acProfiles[i + 1];
  acProfileCount--;
  if (activeAcProfileId == id) {
    activeAcProfileId = acProfiles[0].id;
    applyProfileToConfig(acProfiles[0]);
  }
  saveConfig();
  sendJsonResponse(200, acProfilesJson());
}

void handleCompareAcCodePost() {
  if (!lastCapture.available) {
    sendError(409, "请先用实体遥控器发送同一组合，让控制器捕获到基准编码");
    return;
  }
  CaptureSnapshot reference = lastCapture;
  JsonDocument doc;
  if (!parseBody(doc)) return;
  AcRequest request;
  request.power = doc["power"] | config.remotePower;
  request.mode = doc["mode"] | config.remoteMode;
  request.fan = doc["fan"] | config.remoteFan;
  request.degrees = clampFloat(doc["degrees"] | config.remoteDegrees, 16.0f, 32.0f);
  request.turbo = doc["turbo"] | config.remoteTurbo;
  request.quiet = doc["quiet"] | config.remoteQuiet;
  request.sleep = doc["sleep"] | config.remoteSleep;
  request.swingV = doc["swingV"] | config.remoteSwingV;
  request.swingH = doc["swingH"] | config.remoteSwingH;
  request.filter = doc["filter"] | config.remoteFilter;
  uint32_t timeoutMs = constrain(static_cast<uint32_t>(doc["timeoutMs"] | 4500), 1200UL, 8000UL);

  SelfTestReport generatedReport = runAcSelfTest(request, timeoutMs);
  CaptureSnapshot generated = lastCapture;
  lastCapture = reference;

  String strictMismatches;
  bool strictOk = generatedReport.received && strictCaptureEqual(reference, generated, strictMismatches);

  String semanticMismatches;
  stdAc::state_t referenceState;
  stdAc::state_t generatedState;
  bool referenceDecoded = snapshotToCommonState(reference, &referenceState, &generatedReport.expectedState);
  bool generatedDecoded = snapshotToCommonState(generated, &generatedState, &generatedReport.expectedState);
  bool semanticOk = false;
  if (referenceDecoded && generatedDecoded) {
    semanticOk = commonStatesSemanticallyEqual(referenceState, generatedState, semanticMismatches);
  } else {
    if (!referenceDecoded) appendCodeCompareMismatch(semanticMismatches, "实体遥控器编码无法解析为空调状态");
    if (!generatedDecoded) appendCodeCompareMismatch(semanticMismatches, "控制器生成编码无法解析为空调状态");
  }

  String message;
  bool ok = false;
  if (!generatedReport.received) {
    message = "控制器发码后本机未收到，无法对比生成编码";
  } else if (strictOk && semanticOk) {
    message = "严格编码一致，离线适配可信度最高";
    ok = true;
  } else if (semanticOk) {
    message = "语义一致但原始编码不同，可能存在时钟/显示/定时位差异";
  } else {
    message = "编码不一致，当前库设置不建议用于控制这台空调";
  }

  String out;
  out.reserve(2200);
  out += "{\"ok\":";
  out += ok ? "true" : "false";
  out += ",\"strictOk\":";
  out += strictOk ? "true" : "false";
  out += ",\"semanticOk\":";
  out += semanticOk ? "true" : "false";
  out += ",\"referenceDecoded\":";
  out += referenceDecoded ? "true" : "false";
  out += ",\"generatedDecoded\":";
  out += generatedDecoded ? "true" : "false";
  out += ",\"message\":";
  out += jsonString(message);
  out += ",\"strictMismatches\":";
  out += codeCompareMismatchesJson(strictMismatches);
  out += ",\"semanticMismatches\":";
  out += codeCompareMismatchesJson(semanticMismatches);
  out += ",\"reference\":";
  out += captureSnapshotJson(reference);
  out += ",\"generated\":";
  out += captureSnapshotJson(generated);
  out += ",\"selfTest\":";
  out += selfTestReportJson(generatedReport);
  out += "}";
  addDecisionLog(ok ? "code_match_ok" : "code_match_bad", "manual", message.c_str(), roomTempC, NAN, request.degrees);
  sendJsonResponse(200, out);
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
  copyCaptureToLearned(cmd);
  if (!applyCaptureMetaToLearned(cmd)) {
    cmd.hasAcMeta = true;
    cmd.metaComplete = false;
    cmd.power = doc["power"] | true;
    cmd.mode = doc["mode"] | "cool";
    cmd.fan = doc["fan"] | "auto";
    cmd.degrees = doc["degrees"] | 26.0f;
    cmd.turbo = doc["turbo"] | false;
    cmd.quiet = doc["quiet"] | false;
    cmd.sleep = doc["sleep"] | false;
    cmd.swingV = doc["swingV"] | false;
    cmd.swingH = doc["swingH"] | false;
    cmd.filter = doc["filter"] | false;
  }
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
  if (doc["turbo"].is<bool>()) cmd.turbo = doc["turbo"] | cmd.turbo;
  if (doc["quiet"].is<bool>()) cmd.quiet = doc["quiet"] | cmd.quiet;
  if (doc["sleep"].is<bool>()) cmd.sleep = doc["sleep"] | cmd.sleep;
  if (doc["swingV"].is<bool>()) cmd.swingV = doc["swingV"] | cmd.swingV;
  if (doc["swingH"].is<bool>()) cmd.swingH = doc["swingH"] | cmd.swingH;
  if (doc["filter"].is<bool>()) cmd.filter = doc["filter"] | cmd.filter;
  cmd.hasAcMeta = doc["hasAcMeta"] | cmd.hasAcMeta;

  bool replaceFromCapture = doc["replaceFromCapture"] | false;
  if (replaceFromCapture) {
    if (!lastCapture.available) {
      sendError(409, "No IR capture available");
      return;
    }
    copyCaptureToLearned(cmd);
    applyCaptureMetaToLearned(cmd);
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
  if (src["activeAcProfileId"].is<int>()) activeAcProfileId = src["activeAcProfileId"] | activeAcProfileId;
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
  if (src["lastCurveEndPoweroffKey"].is<uint32_t>() || src["lastCurveEndPoweroffKey"].is<int>()) {
    config.lastCurveEndPoweroffKey = src["lastCurveEndPoweroffKey"] | config.lastCurveEndPoweroffKey;
  }
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
  JsonArray profiles = src["acProfiles"].as<JsonArray>();
  if (!profiles.isNull()) {
    acProfileCount = 0;
    for (JsonObject item : profiles) {
      if (acProfileCount >= kMaxAcProfiles) break;
      loadAcProfileFromJson(acProfiles[acProfileCount], item);
      if (acProfiles[acProfileCount].id == 0) acProfiles[acProfileCount].id = nextAcProfileId();
      acProfileCount++;
    }
  }
  config.minSetpoint = 16.0f;
  config.maxSetpoint = 32.0f;
  normalizeCurveControlConfig();
  normalizeTuningConfig();
  normalizeConfigRemoteState();
  ensureAcProfiles();
  int activeIdx = findAcProfileIndexById(activeAcProfileId);
  if (!profiles.isNull() && activeIdx >= 0) applyProfileToConfig(acProfiles[activeIdx]);
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
  sendJsonResponse(200, out);
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
    sendJsonResponse(200, "{\"ok\":true,\"message\":\"ota uploaded, restarting\"}");
    delay(300);
    ESP.restart();
  } else {
    sendJsonResponse(500, "{\"ok\":false,\"message\":\"ota failed\"}");
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
  server.on("/api/learned-match-report", HTTP_GET, handleLearnedMatchReport);
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
  server.on("/api/self-test-ac", HTTP_POST, handleSelfTestAcPost);
  server.on("/api/apply-capture-state", HTTP_POST, handleApplyCaptureStatePost);
  server.on("/api/compare-ac-code", HTTP_POST, handleCompareAcCodePost);
  server.on("/api/ac-profiles", HTTP_GET, handleAcProfilesGet);
  server.on("/api/ac-profile/create", HTTP_POST, handleAcProfileCreatePost);
  server.on("/api/ac-profile/update", HTTP_POST, handleAcProfileUpdatePost);
  server.on("/api/ac-profile/select", HTTP_POST, handleAcProfileSelectPost);
  server.on("/api/ac-profile/delete", HTTP_POST, handleAcProfileDeletePost);
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
    if (method == HTTP_GET && uri == "/api/learned-match-report") {
      handleLearnedMatchReport();
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
    if (method == HTTP_POST && uri == "/api/self-test-ac") {
      handleSelfTestAcPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/apply-capture-state") {
      handleApplyCaptureStatePost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/compare-ac-code") {
      handleCompareAcCodePost();
      return;
    }
    if (method == HTTP_GET && uri == "/api/ac-profiles") {
      handleAcProfilesGet();
      return;
    }
    if (method == HTTP_POST && uri == "/api/ac-profile/create") {
      handleAcProfileCreatePost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/ac-profile/update") {
      handleAcProfileUpdatePost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/ac-profile/select") {
      handleAcProfileSelectPost();
      return;
    }
    if (method == HTTP_POST && uri == "/api/ac-profile/delete") {
      handleAcProfileDeletePost();
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
  loadDecisionLog();
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
  handleSerialConsole();
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
