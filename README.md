# Intelligent-home STM32 Client

智能家庭控制系統的 STM32 BLE 周邊節點韌體。對應主文件：
- [系統開發文件](../docs/智能家庭控制系統開發文件%20(System%20Development%20Document).md)
- [STM32 Client 開發文件](../docs/STM32%20Client%20開發文件.md)（GATT 規格、任務模型、進度狀態）

---

## 1. 硬體

| 項目 | 規格 |
| --- | --- |
| 開發板 | **STM32 B-L475E-IOT01A**（STM32L475VG, Cortex-M4F, 80 MHz） |
| BLE 模組 | **X-NUCLEO-IDB05A2**（BlueNRG-MS, SPI3） |
| 板載感測器 | HTS221（溫濕度）、LSM6DSL（加速度＋陀螺儀）、MP34DT01（PDM 麥克風） |
| Debug log | ST-LINK VCP → USART1 → **115200 8N1** |

---

## 2. BLE 廣告

- **Local Name**：`HOME-XXXX`（`XXXX` 為 BD_ADDR 末 2 byte 的 hex，於開機時生成、每個板子不同）
- **Address Type**：Static Random
- **Advertising Interval**：100 – 200 ms
- **Advertising Type**：`ADV_IND`（可連線、可掃描）
- **Pairing / Encryption**：無（v1 open BLE，僅限本地測試環境使用）

掃描範例：用 nRF Connect / `bluetoothctl` / `bluepy` 找開頭為 `HOME-` 的裝置即可。

---

## 3. GATT 表

> 所有 UUID 為 128-bit，base = `xxxxxxxx-8E22-4541-9D4C-21EDAE82ED19`，前 4 byte 由各 Service / Characteristic 自帶。
> 此表為**對外契約**，燒錄後保持穩定；任何欄位變更請同步更新 [STM32 Client 開發文件 §5.4](../docs/STM32%20Client%20開發文件.md) 與 RPi Server 對應頻道設定。

### 3.1 Home Sensor Service

> Service UUID：**`1A220001-8E22-4541-9D4C-21EDAE82ED19`**

| Characteristic | UUID | 屬性 | 資料格式 | 單位 / 編碼 |
| --- | --- | --- | --- | --- |
| **Temperature** | `1A220002-…` | Read + Notify | `float32` (LE) | °C |
| **Humidity** | `1A220003-…` | Read + Notify | `float32` (LE) | % RH |
| **AccelMagnitude** | `1A220004-…` | Read + Notify | `float32` (LE) | g（含重力） |
| **GyroMagnitude** | `1A220005-…` | Read + Notify | `float32` (LE) | dps |
| **MotionAlert** | `1A220006-…` | Read + Notify | `uint8` | `0`=normal / `1`=abnormal |
| **MicLevel** | `1A220007-…` | Read + Notify | `uint16` (LE) | 0 – 1023 normalized energy |
| **LoudAlert** | `1A220008-…` | Read + Notify | `uint8` | `0`=quiet / `1`=loud |

### 3.2 Home Control Service

> Service UUID：**`1A22F001-8E22-4541-9D4C-21EDAE82ED19`**

| Characteristic | UUID | 屬性 | 資料格式 | 編碼 |
| --- | --- | --- | --- | --- |
| **Led1State** | `1A22F002-…` | Read + Write + Write w/o Resp | `uint8` | `0`=off / `1`=on |
| **ControlFlag** | `1A22F003-…` | Read + Write + Write w/o Resp | `uint8` | 低 4 bit 為通用旗標 |

> ⚠️ Cortex-M 為 little-endian，因此 `float32` / `uint16` 不需要 swap，直接 memcpy 即為 wire 格式。

---

## 4. 串接流程（給未來整合者）

中央端（RPi server / nRF Connect / 自製 client）的標準流程：

```
1. Scan：尋找 local name 開頭為 "HOME-" 的廣告封包
2. Connect：靜態隨機位址，無需配對
3. 對 Display Characteristic 訂閱 Notify（CCC descriptor 寫 0x0001）
4. 對 Controller Characteristic 直接 Write Value（uint8）
```

### 4.1 用 nRF Connect 手動驗證

| 動作 | 預期 |
| --- | --- |
| Scan | 看到 `HOME-XXXX`，RSSI 約 -40 ~ -70 dBm |
| Connect | 連上後展開 GATT services |
| Enable Notify on `1A220002` | 收到溫度推播（v1 為週期值） |
| Write `01` to `1A22F002` | LED1（PA5）亮 |
| Write `00` to `1A22F002` | LED1 熄滅 |

### 4.2 Python (bluepy) 範例

```python
from bluepy.btle import Peripheral, UUID, DefaultDelegate

BASE = "8E22-4541-9D4C-21EDAE82ED19"
TEMP_UUID = UUID(f"1A220002-{BASE}")
LED1_UUID = UUID(f"1A22F002-{BASE}")

class Listener(DefaultDelegate):
    def handleNotification(self, cHandle, data):
        # data 是 bytes；float32_le → struct.unpack('<f', data)[0]
        print("notify", cHandle, data.hex())

p = Peripheral("XX:XX:XX:XX:XX:XX", addrType="random")
p.setDelegate(Listener())

# 訂閱溫度 Notify
temp = p.getCharacteristics(uuid=TEMP_UUID)[0]
p.writeCharacteristic(temp.getHandle() + 1, b"\x01\x00", withResponse=True)

# 寫 LED1
led1 = p.getCharacteristics(uuid=LED1_UUID)[0]
led1.write(b"\x01")

while True:
    p.waitForNotifications(1.0)
```

---

## 5. 專案檔案架構（常用）

```
Intelligent-home-STM32-client/
├── Intelligent-home-STM32-client.ioc      ← STM32CubeMX 設定（peripheral / pinout / clock / RTOS）
├── README.md                              ← 本文件
├── CMakeLists.txt                         ← 頂層 CMake；user 自建檔案加在這（CubeMX 不會沖）
├── CMakePresets.json                      ← CMake preset（Debug / Release）
├── STM32L475XX_FLASH.ld                   ← Linker script（CubeMX 生成，勿動）
├── startup_stm32l475xx.s                  ← Startup（CubeMX 生成，勿動）
├── cmake/                                 ← CubeMX 生成的 CMake（每次 regen 重寫，勿手改）
│
├── Core/                                  ← 應用層（可改，注意 USER CODE 區塊）
│   ├── Inc/
│   │   ├── main.h                         ← Pin label 巨集（例如 LED1_PIN_*）
│   │   ├── b_l475e_iot01a1_conf.h         ← BSP 設定（USE_COM_LOG / BUS_UART1_BAUDRATE）
│   │   ├── FreeRTOSConfig.h               ← FreeRTOS 設定（CubeMX 生成）
│   │   ├── dfsdm.h / dma.h / i2c…         ← peripheral handle 宣告（CubeMX 生成）
│   │   └── …
│   └── Src/
│       ├── main.c                         ← 進入點 + clock + 週邊 init
│       ├── freertos.c                     ← Task / Queue 建立（USER 區塊：BleTask / SensorTask / AudioTask / NotifyQueue）
│       ├── dfsdm.c / dma.c                ← DFSDM + DMA init / MSP（CubeMX 生成）
│       ├── stm32l4xx_it.c                 ← ISR（DMA1_Ch4 / DFSDM IRQ → HAL handler）
│       ├── stm32l4xx_hal_msp.c            ← HAL MSP init（CubeMX 生成）
│       └── …
│
├── BlueNRG_MS/                            ← BLE 應用層（可改）
│   ├── App/
│   │   ├── app_bluenrg_ms.{c,h}           ← BLE Stack init、main-loop callback（BleTask 呼叫）
│   │   ├── sensor.{c,h}                   ← BLE GAP / 廣告（HOME-XXXX）/ HCI event router（名稱沿用 ST template）
│   │   ├── gatt_db.{c,h}                  ← **GATT 表單一真實來源**（§3 表的對應實作 + Write callback）
│   │   ├── notify_queue.{c,h}             ← sensor/audio task → BleTask 的推播佇列（序列化 ACI 呼叫）
│   │   ├── sensor_task.{c,h}              ← HTS221 + LSM6DSL 採集（I²C2 via BSP）→ NotifyQueue
│   │   └── audio_task.{c,h}               ← MP34DT01 麥克風（DFSDM + DMA interrupt）→ NotifyQueue
│   └── Target/
│       ├── bluenrg_conf.h                 ← `BLE1_DEBUG` 開關（=1 才會印 log）
│       └── hci_tl_interface.{c,h}         ← BlueNRG HCI 傳輸層（勿動）
│
├── X-CUBE-MEMS1/Target/                   ← MEMS 設定；custom_mems_conf.h 把 HTS221/LSM6DSL I/O hook 到 BSP_I2C2_*
│
├── Drivers/                               ← HAL + BSP（勿動）
│   ├── BSP/B-L475E-IOT01A1/               ← 板載 LED / Button / COM / I²C bus driver
│   ├── BSP/Components/{hts221,lsm6dsl}/   ← MEMS component driver（X-CUBE-MEMS1 匯入）
│   ├── STM32L4xx_HAL_Driver/              ← HAL driver
│   └── CMSIS/                             ← CMSIS
│
├── Middlewares/                           ← FreeRTOS / BLE stack / DSP（勿動）
│   ├── Third_Party/FreeRTOS/              ← FreeRTOS + CMSIS-RTOS2 wrapper
│   ├── Third_Party/ARM/ , ST/ARM/DSP/     ← CMSIS-DSP（X-CUBE-ALGOBUILD 匯入）
│   └── ST/BlueNRG-MS/                     ← BlueNRG-MS BLE stack
│
├── build/                                ← 編譯產物（不入 git）
└── .claude/
    └── rules/                             ← 給 Claude Code 的協作規範
        └── file-modification-scope.md     ← 哪些檔案可改、哪些要走 .ioc
```

> 資料流：`SensorTask` / `AudioTask` 採集後把資料丟進 `NotifyQueue`，只有 `BleTask`
> 會呼叫 BlueNRG ACI（`gatt_db.c` 的 `Home_*_Update`），確保 SPI3 上的 HCI 呼叫序列化。
> 詳細任務模型見 [STM32 Client 開發文件 §6](../docs/STM32%20Client%20開發文件.md)。

### 5.1 可修改範圍速查

| 路徑 | 可改？ | 注意 |
| --- | --- | --- |
| `BlueNRG_MS/App/**` | ✅ 可自由修改 | `app_bluenrg_ms.c` 內保留 `USER CODE` 標記；regenerate 風險見下 |
| `BlueNRG_MS/Target/bluenrg_conf.h` | ✅ 可改 | `BLE1_DEBUG`、HCI buffer size 等 |
| `Core/Inc/`, `Core/Src/` | ✅ 可改 | 只能在 `/* USER CODE BEGIN … */ … /* USER CODE END … */` 區塊內 |
| `Core/Inc/b_l475e_iot01a1_conf.h` | ✅ 可改 | BSP 設定（baud rate、USE_COM_LOG…） |
| `*.ioc` | ⚠️ 經由 CubeMX | 改 pinout / peripheral / FreeRTOS 設定走這條 |
| `CMakeLists.txt`（頂層）| ✅ 可改 | 新增 user source 加在 `target_sources` 的 user 區塊 |
| `Drivers/**`, `Middlewares/**` | ❌ 勿動 | HAL / BSP / MEMS component / 中介層；regenerate 或升級會覆蓋 |
| `X-CUBE-MEMS1/**`, `cmake/**` | ❌ 勿動 | CubeMX / 軟體包生成；regenerate 會重寫 |
| Linker / Startup | ❌ 勿動 | 由 CubeMX 維護 |

詳見 [`.claude/rules/file-modification-scope.md`](./.claude/rules/file-modification-scope.md)。

---

## 6. 編譯與燒錄

### 6.1 STM32CubeIDE（建議）

1. Import 專案：`File → Open Projects from File System → 選資料夾`
2. Build：`Project → Build All`（或 `Ctrl+B`）
3. Flash：`Run → Run`（透過 ST-LINK）

### 6.2 CMake (CLI)

專案已內建 CMake preset：

```bash
cmake --preset Debug
cmake --build --preset Debug
# 燒錄：用 STM32_Programmer_CLI 或 OpenOCD
```

### 6.3 重新生成程式碼（CubeMX）

> ⚠️ 規則：請先告知協作者再 regenerate。

需要動到 pinout、peripheral 啟用、FreeRTOS 設定時：

1. STM32CubeIDE 開 `.ioc`
2. 改設定
3. **建議先 `git commit`** — `BlueNRG_MS/App/` 內檔案可能會被 X-CUBE-BLE1 template 沖回 SensorDemo
4. `GENERATE CODE`
5. 若 `app_bluenrg_ms.c` / `sensor.c` / `gatt_db.c` 被覆寫，用 `git checkout BlueNRG_MS/App/` 復原
6. 或者更治本：在 CubeMX 內把 BLE Application 從 `SensorDemoBLESensor` 換成 `Custom Template`

---

## 7. Debug Log

板上的所有 log 都走 USART1 → ST-LINK VCP：

- Baud rate：**115200**, 8N1（若 CubeMX regen 後變回 9600，改 `Core/Inc/b_l475e_iot01a1_conf.h` 的 `BUS_UART1_BAUDRATE`）
- 來源：`PRINTF(...)` macro（需 `BLE1_DEBUG=1`，已在 `BlueNRG_MS/Target/bluenrg_conf.h` 設好）
- 開機預期（穩定後三個 task 並行）：

  ```
  HWver ?, FWver ?
  BLE stack initialised.
  Home Sensor Service added.
  Home Control Service added.
  Advertising as HOME-XXXX
  HTS221  WHO_AM_I = 0xBC (expected 0xBC)
  LSM6DSL WHO_AM_I = 0x6A (expected 0x6A)
  SensorTask started (HTS221=OK, LSM6DSL=OK).
  AudioTask started (DFSDM + DMA interrupt, 1600-sample window @ ~8 kHz).
  [imu] a=(0.01,0.00,1.00)g |a|=1.00  g=(0,0,0)dps |g|=0.0     ← 每秒
  [env] T=25.3C H=42.1%                                        ← 每秒
  [mic] rms=2  lvl=16                                          ← 每秒
  Write LED1State = 1                                          ← 寫 char 才出
  [motion] ALERT (|a|=2.13 |g|=87)                             ← 搖晃時
  [loud] ALERT (mic=520)                                       ← 大聲時
  Connected to XX:XX:XX:XX:XX:XX                               ← nRF Connect 連上
  ```

---

## 8. 進度

開發階段請見 [STM32 Client 開發文件 §11](../docs/STM32%20Client%20開發文件.md)。
本檔案描述的是**對外契約**（GATT 表）與**檔案位置**，實作細節請參閱開發文件。

---

## License

Original code based on STMicroelectronics SensorDemo_BLESensor-App example.
Modifications copyright 2026 Intelligent-home team.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component. If no LICENSE file comes
with this software, it is provided AS-IS.
