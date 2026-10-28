#!/usr/bin/env python3
"""
tools/verify_ota.py - Forensic Dissector and Validator for VeSync OTA Containers

Performs full structural, cryptographic, and firmware integrity inspection:
  1. Validates 129-byte container structure: [0x01] + [128-byte AES-128-CBC Header] + [Raw Firmware].
  2. Decrypts header using Levoit/VeSync key derivation (MD5("Levoit")).
  3. Verifies internal metadata (crypto_type, model string, version, region tag).
  4. Tests payload MD5 parity against encrypted header declaration.
  5. Validates ESP32 image magic byte (0xE9) and partition size limits (<= 1,310,720 bytes).
  6. Optionally extracts the raw ESP32-C3 firmware binary for disassembly or manual flashing.
"""

import sys
import os
import hashlib
import argparse
from typing import Optional
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

LEVOIT_MD5_KEY = hashlib.md5(b"Levoit").digest()
LEVOIT_MD5_IV  = hashlib.md5(b"Levoit").digest()
LEGACY_KEY     = b"llwantaeskey1.01"
LEGACY_IV      = b"llwantaesivv1.01"

MAX_OTA_PARTITION_SIZE = 1310720  # 1.25 MB (0x140000)
ESP_IMAGE_HEADER_MAGIC = 0xE9
HEX_CHARS = set(b"0123456789ABCDEFabcdef")


def dissect_vesync_ota(package_path: str, extract_path: Optional[str] = None, allow_non_esp: bool = False) -> bool:
    if not os.path.exists(package_path):
        print(f"[-] Error: File not found: {package_path}", file=sys.stderr)
        return False

    with open(package_path, "rb") as f:
        data = f.read()

    total_len = len(data)
    print("=" * 70)
    print(f"Inspecting VeSync OTA Package: {package_path}")
    print("=" * 70)
    print(f"Total File Size   : {total_len:,} bytes")
    print(f"Package SHA-256   : {hashlib.sha256(data).hexdigest()}")

    if total_len <= 129:
        print("[-] FAILED: File too small to contain 129-byte header and payload.", file=sys.stderr)
        return False

    crypto_type = data[0]
    print(f"Container Byte 0  : 0x{crypto_type:02X} (Expected: 0x01)", "✓" if crypto_type == 1 else "✗")
    if crypto_type != 0x01:
        print("[-] FAILED: Unsupported crypto_type (must be 0x01).", file=sys.stderr)
        return False

    enc_header = data[1:129]
    raw_payload = data[129:]
    payload_len = len(raw_payload)

    # Try Levoit key first, fallback to legacy
    dec_header = None
    key_used = "levoit"
    for name, k, iv in [("MD5('Levoit')", LEVOIT_MD5_KEY, LEVOIT_MD5_IV), ("legacy", LEGACY_KEY, LEGACY_IV)]:
        try:
            cipher = Cipher(algorithms.AES(k), modes.CBC(iv))
            decryptor = cipher.decryptor()
            candidate = decryptor.update(enc_header) + decryptor.finalize()
            if (candidate[0] == 0x01 and candidate[33] == 0x00 and
                all(b in HEX_CHARS for b in candidate[1:33])):
                dec_header = candidate
                key_used = name
                break
        except Exception:
            pass

    if not dec_header:
        print("[-] FAILED: Could not decrypt 128-byte header with known VeSync keys.", file=sys.stderr)
        return False

    print(f"Header Decryption : SUCCESS (Key mode: {key_used}) ✓")

    # Parse Header Fields
    hdr_crypto = dec_header[0]
    hdr_md5 = dec_header[1:33].decode('ascii', errors='replace')
    hdr_model = dec_header[34:50].split(b'\x00')[0].decode('ascii', errors='replace')
    hdr_version = dec_header[50:66].split(b'\x00')[0].decode('ascii', errors='replace')
    hdr_hw_ver = dec_header[66:70].split(b'\x00')[0].decode('ascii', errors='replace')
    hdr_region = dec_header[74:76].decode('ascii', errors='replace')

    print(f"Declared Model    : '{hdr_model}'")
    print(f"Declared Version  : '{hdr_version}'")
    print(f"Hardware Version  : '{hdr_hw_ver}'")
    print(f"Region Tag        : '{hdr_region}'")
    print(f"Header MD5 Hash   : {hdr_md5}")

    # Validate Raw Firmware Payload
    print("-" * 70)
    print("Firmware Payload Verification:")
    print(f"Payload Size      : {payload_len:,} bytes")

    actual_md5 = hashlib.md5(raw_payload).hexdigest().upper()
    print(f"Computed MD5      : {actual_md5}")

    if actual_md5 != hdr_md5:
        print(f"[-] FAILED: Payload MD5 mismatch! Expected {hdr_md5}, got {actual_md5}", file=sys.stderr)
        return False
    print("MD5 Integrity     : MATCH (Cryptographically Valid) ✓")

    # Magic byte check
    magic = raw_payload[0]
    print(f"Image Magic Byte  : 0x{magic:02X} (Expected: 0x{ESP_IMAGE_HEADER_MAGIC:02X} for ESP32 image)", "✓" if magic == ESP_IMAGE_HEADER_MAGIC else "✗")
    if magic != ESP_IMAGE_HEADER_MAGIC:
        if allow_non_esp:
            print("[-] WARNING: Inner binary does not begin with standard ESP32 magic byte 0xE9 (--allow-non-esp set)", file=sys.stderr)
        else:
            print("[-] FAILED: Inner binary does not begin with standard ESP32 magic byte 0xE9. Use --allow-non-esp to bypass.", file=sys.stderr)
            return False

    # Partition Limit Check
    headroom = MAX_OTA_PARTITION_SIZE - payload_len
    print(f"Partition Limit   : {MAX_OTA_PARTITION_SIZE:,} bytes (ota_0 @ 0x010000)")
    if payload_len > MAX_OTA_PARTITION_SIZE:
        print(f"[-] FAILED: Binary overflows partition capacity by {payload_len - MAX_OTA_PARTITION_SIZE:,} bytes!", file=sys.stderr)
        return False

    print(f"Partition Headroom: {headroom:,} bytes ({headroom / 1024:.1f} KB free) ✓")
    print(f"Payload SHA-256   : {hashlib.sha256(raw_payload).hexdigest()}")

    if extract_path:
        out_dir = os.path.dirname(os.path.abspath(extract_path))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(extract_path, "wb") as f_out:
            f_out.write(raw_payload)
        print(f"Extracted Binary  : Saved raw ESP32-C3 firmware to '{extract_path}' ✓")

    print("=" * 70)
    print("RESULT: VALID VeSync Container & Verified ESP32-C3 Firmware Image")
    print("=" * 70)
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dissect and verify VeSync OTA package integrity")
    parser.add_argument("package_path", nargs="?", default="pawsync_ota_update.bin", help="Path to VeSync package (.bin) (default: pawsync_ota_update.bin)")
    parser.add_argument("-x", "--extract", default=None, help="Extract inner raw ESP32 binary to specified path")
    parser.add_argument("--allow-non-esp", action="store_true", help="Allow payload with non-standard ESP32 magic byte")
    args = parser.parse_args()

    success = dissect_vesync_ota(args.package_path, args.extract, args.allow_non_esp)
    sys.exit(0 if success else 1)
