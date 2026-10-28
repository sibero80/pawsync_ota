# PawSync Feeder & VeSync OTA Container Architecture

This document provides a technical deep dive into the firmware architecture of the **Pawsync PPF-R361S Smart Pet Feeder** and the reverse-engineered **129-byte VeSync OTA Container** that enables wireless installation of ESPHome without hardware disassembly.

---

## 1. Hardware Architecture & Dual-MCU Bus

The Pawsync PPF-R361S employs an asymmetric dual-microcontroller design:

```
┌─────────────────────────────────┐           ┌─────────────────────────────────┐
│     Espressif ESP32-C3FH4       │           │   Sinowealth SC95F8766P (8051)  │
│                                 │   UART1   │                                 │
│  • Wi-Fi 802.11 b/g/n & BLE 5.0 │──────────>│  • Stepper Motor Driver (Feeder)│
│  • ESPHome Local Framework      │  GPIO19   │  • HX711/ADC Load Cell Scale    │
│  • Native Home Assistant API    │<──────────│  • Front Panel Key & LEDs       │
│  • Time Sync & 7-Plan Scheduler │  GPIO18   │  • Low-level Fault Protection   │
└─────────────────────────────────┘  (115200) └─────────────────────────────────┘
                │
                │ UART0 (Console / Flashing)
                ▼
      [ TX (GPIO21) / RX (GPIO20) / BOOT (GPIO9) / GND ]
```

### Serial Interfaces
- **UART0 (`GPIO21` TX / `GPIO20` RX)**: Connected to the labeled test pads on the underside of the main logic board. Operates at 115200 baud for ROM bootloader flashing, esptool disaster recovery, and serial console logs.
- **UART1 (`GPIO19` TX / `GPIO18` RX)**: Hardwired internal serial link to the Sinowealth 8051 coprocessor. Operates at **115200 Baud 8N1** using a framed Key-Length-Value (KLV) protocol with 8-bit checksums (`sum % 256`).
- **USB Pin Multiplexing**: On the ESP32-C3, `GPIO18` (D-) and `GPIO19` (D+) are the native USB-Serial-JTAG pins. Because they are wired to the coprocessor, native USB CDC is disabled on external pins.

---

## 2. SPI Flash Layout & Partition Boundaries

The feeder uses a **4MB (32Mbit) SPI NOR Flash**. To prevent bricking the feeder or corrupting factory calibration during OTA transitions, the partition table in `partitions.csv` strictly mirrors the factory layout:

```
Offset     Size      Name         Type  SubType  Purpose
────────────────────────────────────────────────────────────────────────────
0x000000   0x008000  bootloader   boot  -        ESP-IDF 2nd Stage Bootloader
0x008000   0x001000  partitions   data  ota      ESP-IDF Partition Table
0x009000   0x004000  nvs          data  nvs      Non-Volatile Storage
0x00d000   0x002000  otadata      data  ota      Active Boot Slot Selector
0x00f000   0x001000  phy_init     data  phy      RF Calibration
0x010000   0x140000  ota_0        app   ota_0    App Slot 0 (1,310,720 bytes)
0x150000   0x140000  ota_1        app   ota_1    App Slot 1 (1,310,720 bytes)
0x290000   0x004000  usercfg      data  nvs      Factory User Config
0x294000   0x002000  fac_data     data  nvs      Factory Production Data
0x296000   0x010000  storage      data  nvs      Stock Event Storage
0x2a6000   0x058000  historydata  data  nvs      Consumption Records
0x2ff000   0x100000  custom_data  data  nvs      Reserved Data Area
```

> [!CRITICAL]
> **Hard Partition Boundary (`1,310,720` bytes / `0x140000`):**  
> If an OTA binary exceeds 1,310,720 bytes, writing to `ota_0` will overwrite the `usercfg` and `fac_data` partitions at `0x290000`, permanently destroying factory calibration and device MAC registrations. The build tool `tools/build_vesync_ota.py` enforces this limit as a non-bypassable compile-time assertion.

---

## 3. The 129-Byte VeSync OTA Container

When the stock manufacturer firmware receives an over-the-air update trigger (`upFirmware`), its internal download handler (`main_dev_fw_dl_cb`) does not accept raw binary images. Instead, it expects an exact **129-byte encrypted container header** followed by the raw image.

### Container Wire Structure

```
Byte 0       Bytes 1 .. 128                       Bytes 129 .. End
┌──────────┬─────────────────────────────────────┬─────────────────────────────────┐
│  0x01    │   AES-128-CBC Encrypted Header      │     Raw ESP32-C3 Firmware       │
│ (Crypto) │   (128 bytes, PKCS#7 block-aligned) │     (Starts with 0xE9 Magic)    │
└──────────┴─────────────────────────────────────┴─────────────────────────────────┘
```

### Unencrypted 128-Byte Header Structure

| Byte Offset | Size (Bytes) | Field Name | Description |
| :---: | :---: | :--- | :--- |
| `0x00` | 1 | `crypto_type` | Fixed to `0x01` (AES-128-CBC) |
| `0x01 - 0x20` | 32 | `payload_md5` | 32-byte uppercase ASCII MD5 hash of raw payload |
| `0x21` | 1 | `null_terminator` | `0x00` |
| `0x22 - 0x31` | 16 | `model_name` | ASCII string (`PPF-R361S\0`), checked via `strncmp` |
| `0x32 - 0x41` | 16 | `version_str` | Firmware version string (e.g., `1.1.2\0`) |
| `0x42 - 0x45` | 4 | `hw_version` | Hardware version (`1.0\0`) |
| `0x4A - 0x4B` | 2 | `region_tag` | Whitelist code (`UN` = Universal) |
| `0x4C - 0x7F` | 52 | `reserved` | Zero-padded padding to complete 128 bytes |

### Cryptographic Key Derivation

Disassembly of the vendor stock firmware OTA task reveals dynamic AES-128 Key and IV derivation:

$$\text{Key} = \text{MD5}(\text{"Levoit"}) = \text{0x} \dots \text{ (16 bytes)}$$
$$\text{IV} = \text{MD5}(\text{"Levoit"}) = \text{0x} \dots \text{ (16 bytes)}$$

Legacy VeSync devices used static ASCII strings `llwantaeskey1.01` and `llwantaesivv1.01`, both of which are supported in `tools/build_vesync_ota.py` and `tools/verify_ota.py`.

---

## 4. OTA Streaming & Pacing Dynamics

When staging `pawsync_ota_update.bin` to a stock feeder over HTTP, two critical timing constraints must be observed:

1. **Header Handshake & Partition Erase Pause (8.0 seconds):**  
   The stock downloader reads exactly 129 bytes, validates the crypto header, and begins an asynchronous non-blocking SPI flash sector erase across the entire 1.25MB partition. Streaming data chunks during this erase period overflows the feeder's network buffers and causes a socket timeout. The HTTP server must send Byte 0..128 and pause for **8.0 seconds** before sending Byte 129+.

2. **Chunk Pacing (2048 Bytes @ 20ms):**  
   The stock lwIP TCP client has limited socket buffer memory (64KB TCP window). Streaming raw binary at full LAN line speed starves the main control loop. Pacing chunks at **2048 bytes with ~20ms inter-chunk pauses** (~100 KB/s) guarantees zero packet drops and stable write cycles.
