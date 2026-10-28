#!/usr/bin/env bash
# ==============================================================================
# PawSync OTA Build Wizard
# Fully automated dependency setup, ESPHome compilation, and VeSync OTA packaging.
# Repository: https://github.com/sibero80/pawsync_ota
# ==============================================================================

set -euo pipefail

# Error handling trap
trap 'echo -e "\n${RED}[!] Build script aborted or encountered an error at line ${BASH_LINENO[0]:-unknown}.${RESET}"' ERR INT TERM

# Text styling
BOLD="\033[1m"
GREEN="\033[0;32m"
CYAN="\033[0;36m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
RESET="\033[0m"

AUTO_MODE=false
CLEAN_BUILD=false
VENV_DIR=".venv"

print_banner() {
  echo -e "${CYAN}${BOLD}"
  echo "╔════════════════════════════════════════════════════════════════════╗"
  echo "║                   PawSync OTA Firmware Builder                     ║"
  echo "║     Compiles & Packages pawsync_ota_update.bin (PPF-R361S)         ║"
  echo "╚════════════════════════════════════════════════════════════════════╝"
  echo -e "${RESET}"
}

print_help() {
  echo "Usage: ./build.sh [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  -a, --auto     Run in non-interactive mode with zero prompts"
  echo "  -c, --clean    Wipe existing build cache before compiling"
  echo "  -h, --help     Display this help message"
  echo ""
}

# Parse command line flags
while [[ $# -gt 0 ]]; do
  case "$1" in
    -a|--auto|-y)
      AUTO_MODE=true
      shift
      ;;
    -c|--clean)
      CLEAN_BUILD=true
      shift
      ;;
    -h|--help)
      print_banner
      print_help
      exit 0
      ;;
    *)
      echo -e "${RED}Unknown option: $1${RESET}"
      print_help
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

print_banner

if [[ "$AUTO_MODE" == true ]]; then
  echo -e "${CYAN}Mode: Non-interactive / Automated (--auto)${RESET}\n"
fi

# ------------------------------------------------------------------------------
# 1. System Dependency Checks
# ------------------------------------------------------------------------------
echo -e "${BOLD}[1/5] Checking system prerequisites...${RESET}"

detect_missing_packages() {
  local missing=()
  if ! command -v python3 &>/dev/null; then
    missing+=("python3")
  elif ! python3 -c "import venv, ensurepip" &>/dev/null; then
    missing+=("python3-venv")
  fi
  if ! command -v git &>/dev/null; then
    missing+=("git")
  fi
  if ! command -v gcc &>/dev/null && ! command -v clang &>/dev/null; then
    if [[ "$(uname -s)" == "Darwin" ]]; then
      missing+=("xcode-cli-tools")
    else
      missing+=("build-essential")
    fi
  fi
  echo "${missing[@]}"
}

MISSING_DEPS=$(detect_missing_packages)
if [[ -n "$MISSING_DEPS" ]]; then
  echo -e "${RED}[!] Missing required system tools: ${MISSING_DEPS}${RESET}"
  echo "Please install them via your package manager:"
  if command -v apt-get &>/dev/null; then
    echo "  sudo apt-get update && sudo apt-get install -y python3 python3-venv python3-pip git build-essential"
  elif command -v dnf &>/dev/null; then
    echo "  sudo dnf install -y python3 git gcc make"
  elif command -v pacman &>/dev/null; then
    echo "  sudo pacman -S python git base-devel"
  elif command -v brew &>/dev/null; then
    echo "  brew install python git"
  elif [[ "$(uname -s)" == "Darwin" ]]; then
    echo "  xcode-select --install"
    echo "  brew install python git"
  else
    echo "  Ensure Python 3.10+ (with venv), Git, and a C compiler (gcc/clang) are installed."
  fi
  exit 1
fi

# Verify Python version >= 3.10
PY_VER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
PY_MAJOR=$(echo "$PY_VER" | cut -d. -f1)
PY_MINOR=$(echo "$PY_VER" | cut -d. -f2)

if (( PY_MAJOR < 3 || (PY_MAJOR == 3 && PY_MINOR < 10) )); then
  echo -e "${RED}[!] Python 3.10 or newer is required (found: Python ${PY_VER})${RESET}"
  exit 1
fi
echo -e "${GREEN}  ✓ Python ${PY_VER} detected${RESET}"
echo -e "${GREEN}  ✓ Git & C toolchain available${RESET}"

# ------------------------------------------------------------------------------
# 2. Virtual Environment Setup
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}[2/5] Setting up isolated Python virtual environment...${RESET}"

if [[ ! -d "$VENV_DIR" ]]; then
  echo "  Creating virtual environment in ${VENV_DIR}..."
  python3 -m venv "$VENV_DIR"
fi

VENV_PY="${VENV_DIR}/bin/python3"
VENV_PIP="${VENV_DIR}/bin/pip"

# Check if esphome & cryptography are installed inside venv
NEED_INSTALL=false
if ! "$VENV_PY" -c "import esphome" &>/dev/null || ! "$VENV_PY" -c "import cryptography" &>/dev/null; then
  NEED_INSTALL=true
fi

if [[ "$NEED_INSTALL" == true ]]; then
  echo "  Installing/updating build tools in virtual environment..."
  "$VENV_PIP" install --quiet --upgrade pip
  "$VENV_PIP" install --quiet "esphome>=2024.6.0" "cryptography>=41.0.0"
fi
echo -e "${GREEN}  ✓ Virtual environment ready (ESPHome & Cryptography installed)${RESET}"

# ------------------------------------------------------------------------------
# 3. Secrets Configuration
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}[3/5] Verifying secrets.yaml...${RESET}"

if [[ ! -f "secrets.yaml" ]]; then
  echo "  secrets.yaml not found. Generating default mock secrets..."
  cp secrets.yaml.example secrets.yaml

  # Generate random cryptographic keys
  "$VENV_PY" -c "
import secrets, base64

with open('secrets.yaml', 'r') as f:
    content = f.read()

noise_psk = base64.b64encode(secrets.token_bytes(32)).decode('ascii')
ota_pass = secrets.token_urlsafe(16)
ap_pass = secrets.token_urlsafe(16)

content = content.replace('\"Generated_Base64_Key\"', f'\"{noise_psk}\"')
content = content.replace('\"ChangeThisOtaPassword123\"', f'\"{ota_pass}\"')
content = content.replace('\"ChangeThisFallbackPassword123\"', f'\"{ap_pass}\"')

with open('secrets.yaml', 'w') as f:
    f.write(content)
"
  echo -e "${GREEN}  ✓ Generated clean secrets.yaml with cryptographically random keys${RESET}"
  if [[ "$AUTO_MODE" != true ]]; then
    echo -e "${CYAN}  (Note: Default Wi-Fi credentials in secrets.yaml are mocks for compilation. Edit before flashing to connect to your home Wi-Fi).${RESET}"
  fi
else
  echo -e "${GREEN}  ✓ secrets.yaml exists${RESET}"
fi

# ------------------------------------------------------------------------------
# 4. Clean Build (Optional) & ESPHome Compilation
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}[4/5] Compiling ESPHome firmware for ESP32-C3...${RESET}"

if [[ "$CLEAN_BUILD" == true && -d ".esphome" ]]; then
  echo "  Cleaning previous build cache..."
  rm -rf .esphome/
fi

VENV_ESPHOME="${VENV_DIR}/bin/esphome"
echo "  Running 'esphome compile pawsync.yaml'..."
"$VENV_ESPHOME" compile pawsync.yaml

RAW_BIN=".esphome/build/pawsync-feeder/build/firmware.ota.bin"
if [[ ! -f "$RAW_BIN" ]]; then
  RAW_BIN=".esphome/build/pawsync-feeder/build/pawsync-feeder.bin"
fi

if [[ ! -f "$RAW_BIN" ]]; then
  echo -e "${RED}[!] Error: Compiled binary not found at ${RAW_BIN}${RESET}"
  exit 1
fi
echo -e "${GREEN}  ✓ ESPHome compilation successful!${RESET}"

# ------------------------------------------------------------------------------
# 5. VeSync 129-Byte Container Packaging & Inspection
# ------------------------------------------------------------------------------
echo -e "\n${BOLD}[5/5] Packaging & verifying VeSync OTA container...${RESET}"

"$VENV_PY" tools/build_vesync_ota.py "$RAW_BIN" -o pawsync_ota_update.bin

echo ""
echo "Running automated forensic verification on generated container:"
"$VENV_PY" tools/verify_ota.py pawsync_ota_update.bin

echo -e "\n${GREEN}${BOLD}======================================================================${RESET}"
echo -e "${GREEN}${BOLD}✓ SUCCESS: pawsync_ota_update.bin generated and verified successfully!${RESET}"
echo -e "${GREEN}${BOLD}======================================================================${RESET}"
echo "You can find your finished container at: $(pwd)/pawsync_ota_update.bin"
echo ""
echo "To inspect or extract the raw inner binary at any time, run:"
echo "  ./.venv/bin/python tools/verify_ota.py pawsync_ota_update.bin --extract raw_esp32.bin"
echo ""
