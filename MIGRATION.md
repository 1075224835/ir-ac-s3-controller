# 迁移使用说明

这个工程是 GOOUUU ESP32-S3 红外空调控制器固件，基于 PlatformIO + Arduino ESP32。

## 迁移包内容

- `src/`: 固件主程序和网页界面。
- `include/`: 硬件引脚、WiFi、静态 IP、NTP、红外参数配置。
- `data/`: LittleFS 数据目录，当前只保留说明文件。
- `platformio.ini`: PlatformIO 编译和烧写配置。
- `esptool.cfg`: GOOUUU ESP32-S3 烧写后的硬复位配置，避免烧写后手动复位。
- `README.md`: 英文项目说明。
- `MIGRATION.md`: 本中文迁移说明。

迁移包不包含 `.pio/`、`.vscode/`、`*.bin`、`*.elf`、`*.map` 等本机缓存和构建产物。

## 新电脑准备

1. 安装 VS Code 和 PlatformIO 插件，或者安装可用的 PlatformIO CLI。
2. 解压迁移包，打开工程根目录。
3. 如果设备串口不是 `COM8`，修改 `platformio.ini` 里的 `upload_port`。
4. 如果目标网络不同，修改 `include/AppConfig.h` 里的默认 WiFi、静态 IP、网关、子网和 DNS。

## 编译和烧写

在工程根目录执行：

```powershell
python -m platformio run
python -m platformio run -t upload
python -m platformio device monitor -b 115200
```

如果你的环境里 `pio` 命令可用，也可以使用：

```powershell
pio run
pio run -t upload
pio device monitor -b 115200
```

当前工程已经配置 `esptool.cfg` 和 `board_upload.after_reset = hard_reset`，正常烧写结束后会通过 RTS 自动硬复位。

## 设备网页入口

- 主机名：`ir-ac-s3`
- STA 静态 IP：`http://192.168.0.57/`
- 手机遥控器页：`http://192.168.0.57/remote`
- AP 兜底热点：`IR-AC-S3`
- AP 默认密码：`12345678`
- AP 兜底地址：`http://192.168.4.1/`

如果修改了 `include/AppConfig.h` 里的静态 IP，请使用新的地址访问。

## 当前硬件默认接线

| 模块 | ESP32-S3 GPIO |
| --- | --- |
| 红外发送模块 IN | GPIO 4 |
| 红外接收模块 OUT | GPIO 14 |
| SHT31 SDA | GPIO 8 |
| SHT31 SCL | GPIO 9 |
| 供电 | 3V3 / GND |

如果接线不同，修改 `include/AppConfig.h` 中的 `kIrTxPin`、`kIrRxPin`、`kI2cSdaPin`、`kI2cSclPin`。

## 迁移注意事项

- 源码里的默认 WiFi 和静态 IP 会随迁移包一起带走，适合你自己换电脑继续使用。
- 如果要把工程发给别人，先修改或清空 `include/AppConfig.h` 里的 WiFi 密码和内网地址。
- 设备运行后在网页里保存的 WiFi、学习到的红外按键、睡眠曲线存放在 ESP32 的 LittleFS 中，不会自动出现在源码迁移包里。
- 如果更换 ESP32 设备，需要重新烧写；如果还要迁移已学习按键，需要在网页中重新录入，或者后续增加导入/导出功能。
