# Security Policy

## Vulnerability Reporting

If you identify a security issue, vulnerability, or cryptographic weakness in this container generator, please report it responsibly:

1. **GitHub Security Advisory**: Open a private draft security advisory under the **Security** tab of this repository.
2. **Direct Contact**: If private advisories are unavailable, contact the maintainer directly via GitHub profile details.

Please **do not** open public issues for zero-day vulnerabilities or security disclosures.

## Cryptographic Guarantees & Intent

1. **Isolated Bootstrap Function**:
   `pawsync_ota_update.bin` is strictly a transitional, one-time bootstrap vehicle enabling zero-hardware conversion from factory VeSync firmware to local ESPHome. It contains no analytics, phone-home code, or proprietary blobs.
2. **Reproducible Byte Verification**:
   The `tools/verify_ota.py` utility is provided to allow any user or independent researcher to inspect, decrypt, and hash the inner binary without trusting pre-compiled binaries blindly.
3. **Partition Bounds Assertion**:
   The build tooling enforces a strict compile-time check ensuring the binary does not exceed the `0x140000` (1,310,720 bytes) boundary of the `ota_0` partition, preventing SPI flash corruption.
