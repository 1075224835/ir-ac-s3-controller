#pragma once

#include <Arduino.h>

#if __has_include("AppSecrets.local.h")
#include "AppSecrets.local.h"
#endif

#ifndef APP_DEFAULT_WIFI_SSID
#define APP_DEFAULT_WIFI_SSID ""
#endif
#ifndef APP_DEFAULT_WIFI_PASSWORD
#define APP_DEFAULT_WIFI_PASSWORD ""
#endif
#ifndef APP_DEFAULT_WIFI_STATIC_IP
#define APP_DEFAULT_WIFI_STATIC_IP ""
#endif
#ifndef APP_DEFAULT_WIFI_GATEWAY
#define APP_DEFAULT_WIFI_GATEWAY "192.168.31.1"
#endif
#ifndef APP_DEFAULT_WIFI_SUBNET
#define APP_DEFAULT_WIFI_SUBNET "255.255.255.0"
#endif
#ifndef APP_DEFAULT_WIFI_DNS1
#define APP_DEFAULT_WIFI_DNS1 "192.168.31.1"
#endif
#ifndef APP_DEFAULT_WIFI_DNS2
#define APP_DEFAULT_WIFI_DNS2 "223.5.5.5"
#endif

namespace appcfg {

// Change these three pins to match your GOOUUU ESP32-S3 wiring.
// Avoid ESP32-S3 strapping pins and pins already used by USB/flash/PSRAM.
constexpr uint8_t kIrTxPin = 4;
constexpr uint8_t kIrRxPin = 14;
constexpr uint8_t kI2cSdaPin = 8;
constexpr uint8_t kI2cSclPin = 9;

constexpr uint32_t kSerialBaud = 115200;
constexpr uint8_t kSht31Address = 0x44;

constexpr char kApSsid[] = "IR-AC-S3";
constexpr char kApPassword[] = "12345678";

constexpr char kDefaultWifiSsid[] = APP_DEFAULT_WIFI_SSID;
constexpr char kDefaultWifiPassword[] = APP_DEFAULT_WIFI_PASSWORD;
constexpr char kDefaultWifiStaticIp[] = APP_DEFAULT_WIFI_STATIC_IP;
constexpr char kDefaultWifiGateway[] = APP_DEFAULT_WIFI_GATEWAY;
constexpr char kDefaultWifiSubnet[] = APP_DEFAULT_WIFI_SUBNET;
constexpr char kDefaultWifiDns1[] = APP_DEFAULT_WIFI_DNS1;
constexpr char kDefaultWifiDns2[] = APP_DEFAULT_WIFI_DNS2;
constexpr char kHostname[] = "ir-ac-s3";
constexpr char kTimezone[] = "CST-8";
constexpr char kNtpServer1[] = "ntp.aliyun.com";
constexpr char kNtpServer2[] = "pool.ntp.org";
constexpr uint32_t kWifiApFallbackMs = 15000;
constexpr uint32_t kWifiRetryMs = 30000;

constexpr uint16_t kIrCaptureBufferSize = 1024;
constexpr uint8_t kIrTimeoutMs = 50;
constexpr uint16_t kMinUnknownSize = 12;
constexpr uint16_t kMaxRawPulses = 900;
constexpr uint8_t kMaxLearnedCommands = 14;
constexpr uint8_t kMaxCurvePoints = 8;

constexpr char kConfigPath[] = "/config.json";
constexpr char kLibraryPath[] = "/library.json";

}  // namespace appcfg
