# PawSync OTA Generator & Inspection Toolkit

[![Build](https://github.com/sibero80/pawsync_ota/actions/workflows/build.yml/badge.svg)](https://github.com/sibero80/pawsync_ota/actions/workflows/build.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Hardware: ESP32-C3](https://img.shields.io/badge/Hardware-ESP32--C3FH4-orange.svg)](ARCHITECTURE.md)

A standalone, fully transparent build and inspection toolchain to compile, verify, and reverse-engineer **`pawsync_ota_update.bin`** for the **Pawsync PPF-R361S Smart Pet Feeder**.

This repository is completely self-contained and decoupled from Home Assistant, allowing security researchers, developers, and users to:
1. **Audit the complete source code** and verify that no hidden backdoors or analytics exist.
2. **Compile the binary independently** on any Linux or macOS machine with minimal dependencies.
3. **Inspect and dissect existing container binaries** using cryptographic forensic tools.
4. **Understand the proprietary 129-byte VeSync OTA container** required by the stock bootloader.

---

## ⚡ Quickstart: One-Command Build

The included `build.sh` script automates dependency probing, isolated virtual environment creation, secret generation, ESPHome compilation, and container packaging:

```bash
git clone https://github.com/sibero80/pawsync_ota.git
cd pawsync_ota
./build.sh --auto
```

That's it! When the script finishes, your verified container is ready at:
`./pawsync_ota_update.bin`

### CLI Options

| Command | Description |
| :--- | :--- |
| **`./build.sh`** | Guided interactive builder; confirms settings before starting. |
| **`./build.sh --auto`** | Non-interactive zero-prompt mode; perfect for CI and fast builds. |
| **`./build.sh --clean`** | Wipes `.esphome/` build cache before recompiling from scratch. |
| **`./build.sh --help`** | Shows available options and syntax. |

---

## 🔍 Forensic Verification & Binary Inspection

You don't need to trust pre-compiled binaries from the internet. Use the included `verify_ota.py` tool to dissect, decrypt, and validate any `pawsync_ota_update.bin`:

```bash
# Verify integrity, inspect metadata, and validate payload MD5
./.venv/bin/python tools/verify_ota.py pawsync_ota_update.bin
```

### Example Verification Output:
```text
======================================================================
Inspecting VeSync OTA Package: pawsync_ota_update.bin
======================================================================
Total File Size   : 1,222,993 bytes
Package SHA-256   : c55ebf5a7666af57bfab82f690a2a8a240b739b12ca13225f44dd51d1c57dcaa
Container Byte 0  : 0x01 (Expected: 0x01) ✓
Header Decryption : SUCCESS (Key mode: MD5('Levoit')) ✓
Declared Model    : 'PPF-R361S'
Declared Version  : '1.1.2'
Hardware Version  : '1.0'
Region Tag        : 'UN'
Header MD5 Hash   : 8E4724F1665873E22832A085B83D27C6
----------------------------------------------------------------------
Firmware Payload Verification:
Payload Size      : 1,222,864 bytes
Computed MD5      : 8E4724F1665873E22832A085B83D27C6
MD5 Integrity     : MATCH (Cryptographically Valid) ✓
Image Magic Byte  : 0xE9 (Expected: 0xE9 for ESP32 image) ✓
Partition Limit   : 1,310,720 bytes (ota_0 @ 0x010000)
Partition Headroom: 87,856 bytes (85.8 KB free) ✓
Payload SHA-256   : 776c9666f90c2ef96bf3a2e8c48bc5d897d252249db1133c4899c9706939469c
======================================================================
RESULT: VALID VeSync Container & Verified ESP32-C3 Firmware Image
======================================================================
```

### Extracting the Raw ESP32 Binary:
If you want to extract the inner ESP32-C3 firmware binary for disassembly (e.g. in Ghidra) or direct flashing via `esptool`:

```bash
./.venv/bin/python tools/verify_ota.py pawsync_ota_update.bin --extract raw_firmware.bin
```

---

## 🧠 Why Does `pawsync_ota_update.bin` Exist?

When the pet feeder leaves the factory running stock VeSync/Levoit firmware, its second-stage bootloader **rejects standard ESP-IDF binaries**.

To accept an over-the-air firmware update, the stock bootloader requires:
1. **129-Byte Header**:
   - **Byte 0**: Crypto indicator flag (`0x01`).
   - **Bytes 1–128**: An AES-128-CBC encrypted block derived from `MD5("Levoit")`. This block encodes the uppercase 32-byte MD5 checksum of the inner binary, the target hardware model (`PPF-R361S`), and the firmware version.
2. **Bytes 129+**: The unencrypted ESP32-C3 binary (beginning with ESP image magic byte `0xE9`).

Without this exact 129-byte cryptographic wrapper, the stock firmware will immediately abort the download and roll back. See [ARCHITECTURE.md](ARCHITECTURE.md) for full protocol disassembly and wire specifications.

---

## 🛡️ Partition Safety & Boundaries

The Pawsync PPF-R361S uses a 4MB SPI NOR flash layout:

```
0x010000 ┌───────────────────────────────────────┐
         │ ota_0 (1.25 MB / 1,310,720 bytes)     │ ◄ Target OTA Slot
0x150000 ├───────────────────────────────────────┤
         │ ota_1 (1.25 MB / 1,310,720 bytes)     │ ◄ Alternate Slot
0x290000 ├───────────────────────────────────────┤
         │ usercfg & fac_data (Factory Calib)    │ ◄ MUST NEVER BE OVERWRITTEN
0x400000 └───────────────────────────────────────┘
```

The packaging tool enforces a **hard non-bypassable check** preventing any binary larger than `1,310,720` bytes from being packaged, protecting factory calibration and Wi-Fi calibration sectors from corruption.

---

## 📁 Repository Layout

```text
├── build.sh                  # One-command CLI build & dependency wizard
├── pawsync.yaml              # Core ESPHome device configuration
├── partitions.csv            # 4MB flash partition table
├── secrets.yaml.example      # Template secrets with auto-generation support
├── ARCHITECTURE.md           # Deep dive into VeSync OTA container & dual-MCU bus
├── SECURITY.md               # Cryptographic disclosure & reporting policy
├── LICENSE                   # MIT License
├── components/
│   └── ppf_r361s/            # C++ dual-MCU UART bridge component
└── tools/
    ├── build_vesync_ota.py   # 129-byte AES-128 container generator
    └── verify_ota.py         # Forensic dissector & integrity validator
```

---

## 🤝 Contributing

Contributions, improvements to build portability, and testing on different hardware revisions are welcome! Please open an issue or pull request.

## 📄 License

This project is licensed under the [MIT License](LICENSE).
