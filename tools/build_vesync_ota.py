#!/usr/bin/env python3
"""
tools/build_vesync_ota.py - Hardened VeSync OTA Container Generator for Pawsync PPF-R361S

Wraps an ESP32-C3 firmware binary into the proprietary 129-byte VeSync OTA container:
  - Byte 0: crypto_type (0x01)
  - Bytes 1..128: AES-128-CBC encrypted header (Key: MD5('Levoit'), IV: MD5('Levoit'))
  - Bytes 129+: Raw ESP32-C3 firmware binary (.bin)

Safety Guardrails:
  - Strict assertion: len(bin) <= 1,310,720 bytes (0x140000, max size for ota_0).
  - ESP32 image magic byte (0xE9) validation.
  - Automated post-packaging self-test and MD5 roundtrip verification.
"""

import sys
import os
import hashlib
import argparse
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

# Key derivation: Stock firmware ota task dynamically derives AES-128 Key & IV
# at address 0x42017704 in vendor ROM: MD5("Levoit") -> 16 bytes raw binary
LEVOIT_MD5_KEY = hashlib.md5(b"Levoit").digest()
LEVOIT_MD5_IV  = hashlib.md5(b"Levoit").digest()
LEGACY_KEY     = b"llwantaeskey1.01"
LEGACY_IV      = b"llwantaesivv1.01"

DEFAULT_KEY = LEVOIT_MD5_KEY
DEFAULT_IV  = LEVOIT_MD5_IV
DEFAULT_MODEL = 'PPF-R361S'
DEFAULT_VERSION = '9.9.9'
MAX_OTA_PARTITION_SIZE = 1310720  # 1.25 MB (0x140000)
ESP_IMAGE_HEADER_MAGIC = 0xE9

DEFAULT_INPUT_BIN = ".esphome/build/pawsync-feeder/build/firmware.ota.bin"


def create_vesync_ota_package(
    bin_path: str = DEFAULT_INPUT_BIN,
    output_path: str = "pawsync_ota_update.bin",
    model: str = DEFAULT_MODEL,
    version: str = DEFAULT_VERSION,
    force: bool = False,
    key: bytes = DEFAULT_KEY,
    iv: bytes = DEFAULT_IV
) -> str:
    if not os.path.exists(bin_path):
        if bin_path == DEFAULT_INPUT_BIN:
            alt_bin = ".esphome/build/pawsync-feeder/build/pawsync-feeder.bin"
            if os.path.exists(alt_bin):
                bin_path = alt_bin
            else:
                print(f"Error: input binary '{bin_path}' not found", file=sys.stderr)
                print("Did you run 'esphome compile pawsync.yaml' (or './build.sh') first?", file=sys.stderr)
                sys.exit(1)
        else:
            print(f"Error: user-specified input binary '{bin_path}' not found", file=sys.stderr)
            sys.exit(1)

    with open(bin_path, "rb") as f:
        bin_data = f.read()

    bin_len = len(bin_data)

    # 1. Validate ESP32 Image Magic Byte
    if bin_len == 0:
        print(f"Error: input binary '{bin_path}' is empty", file=sys.stderr)
        sys.exit(1)

    if bin_data[0] != ESP_IMAGE_HEADER_MAGIC and not force:
        print(f"Error: input binary '{bin_path}' does not start with ESP32 magic byte 0xE9 (found 0x{bin_data[0]:02X})", file=sys.stderr)
        print("Use --force if you intentionally wish to bypass ESP image header validation.", file=sys.stderr)
        sys.exit(1)

    # 2. Hard Partition Boundary Enforcement
    if bin_len > MAX_OTA_PARTITION_SIZE:
        print(f"FATAL ERROR: Binary size {bin_len:,} bytes exceeds maximum ota_0 partition capacity ({MAX_OTA_PARTITION_SIZE:,} bytes)!", file=sys.stderr)
        print(f"Overflow: {bin_len - MAX_OTA_PARTITION_SIZE:,} bytes over limit. Flashing would corrupt usercfg at 0x290000.", file=sys.stderr)
        sys.exit(1)

    # 3. Calculate MD5 hex digest (32 bytes ASCII, uppercase required by device strcmp)
    md5_hex = hashlib.md5(bin_data).hexdigest().upper().encode('ascii')
    sha256_hex = hashlib.sha256(bin_data).hexdigest()

    # 4. Construct 128-byte unencrypted header
    header = bytearray(128)
    header[0] = 0x01  # crypto_type = 1
    header[1:33] = md5_hex  # 32-byte MD5 ASCII string
    header[33] = 0x00  # null terminator

    # Model string (16 bytes checked via strncmp in stock firmware)
    model_bytes = model.encode('ascii')[:15]
    header[34:34 + len(model_bytes)] = model_bytes

    # Version string
    ver_bytes = version.encode('ascii')[:15]
    header[50:50 + len(ver_bytes)] = ver_bytes

    # Hardware version
    header[66:70] = b'1.0\x00'

    # Region / country whitelist
    header[74:76] = b'UN'

    # 5. Encrypt 128-byte header with AES-128-CBC
    cipher = Cipher(algorithms.AES(key), modes.CBC(iv))
    encryptor = cipher.encryptor()
    enc_header = encryptor.update(bytes(header)) + encryptor.finalize()
    if len(enc_header) != 128:
        raise RuntimeError(f"Encrypted header size mismatch: {len(enc_header)}")

    # 6. Assemble container: [crypto_type (1 byte)] + [enc_header (128 bytes)] + [raw firmware]
    package_data = bytes([0x01]) + enc_header + bin_data
    total_len = len(package_data)

    # 7. Self-Verification Roundtrip
    decryptor = cipher.decryptor()
    dec_header = decryptor.update(enc_header) + decryptor.finalize()
    if dec_header[0] != 0x01:
        raise RuntimeError("Self-check failed: header[0] != 0x01")
    if dec_header[1:33] != md5_hex:
        raise RuntimeError("Self-check failed: MD5 mismatch")
    if dec_header[34:34 + len(model_bytes)] != model_bytes:
        raise RuntimeError("Self-check failed: model mismatch")

    with open(output_path, "wb") as f:
        f.write(package_data)

    headroom = MAX_OTA_PARTITION_SIZE - bin_len
    pkg_sha256 = hashlib.sha256(package_data).hexdigest()

    print("=" * 70)
    print("VeSync OTA Package Generated Successfully")
    print("=" * 70)
    print(f"  Input Binary    : {bin_path} ({bin_len:,} bytes)")
    print(f"  Container Size  : {total_len:,} bytes (Header: 129 bytes)")
    print(f"  Partition Limit : {MAX_OTA_PARTITION_SIZE:,} bytes (ota_0 @ 0x010000)")
    print(f"  Headroom Margin : {headroom:,} bytes ({headroom / 1024:.1f} KB free)")
    print(f"  Payload MD5     : {md5_hex.decode('ascii')}")
    print(f"  Payload SHA-256 : {sha256_hex}")
    print(f"  Package SHA-256 : {pkg_sha256}")
    print(f"  Target Model    : {model}")
    print(f"  Target Version  : {version}")
    print(f"  Output Package  : {output_path}")
    print("=" * 70)

    return output_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Package ESP32-C3 firmware into hardened VeSync OTA container")
    parser.add_argument("input_bin", nargs="?", default=DEFAULT_INPUT_BIN, help=f"Path to input raw firmware binary (.bin) (default: {DEFAULT_INPUT_BIN})")
    parser.add_argument("-o", "--output", default="pawsync_ota_update.bin", help="Output package path (default: pawsync_ota_update.bin)")
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"Target device model (default: {DEFAULT_MODEL})")
    parser.add_argument("--version", default=DEFAULT_VERSION, help=f"Target firmware version (default: {DEFAULT_VERSION})")
    parser.add_argument("--key-mode", choices=["levoit", "legacy"], default="levoit", help="AES key derivation mode: 'levoit' (MD5('Levoit'), actual stock ota engine key) or 'legacy' (llwantaeskey1.01)")
    parser.add_argument("--force", action="store_true", help="Bypass ESP32 image magic byte validation")
    args = parser.parse_args()

    if args.key_mode == "legacy":
        k, iv = LEGACY_KEY, LEGACY_IV
    else:
        k, iv = LEVOIT_MD5_KEY, LEVOIT_MD5_IV

    create_vesync_ota_package(args.input_bin, args.output, args.model, args.version, args.force, key=k, iv=iv)
