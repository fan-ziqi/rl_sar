#!/bin/bash

# ========================
# Common Shell Script Utilities
# Shared functions for build and setup scripts
# ========================

# Color definitions
COLOR_ERROR='\033[0;31m'     # Red
COLOR_SUCCESS='\033[0;32m'   # Green
COLOR_WARNING='\033[1;33m'   # Yellow
COLOR_INFO='\033[0;34m'      # Blue
COLOR_DEBUG='\033[0;36m'     # Cyan
COLOR_RESET='\033[0m'        # Reset

# ========================
# Output Functions
# ========================

print_success() {
    echo -e "${COLOR_SUCCESS}[SUCCESS]${COLOR_RESET} $1"
}

print_warning() {
    echo -e "${COLOR_WARNING}[WARNING]${COLOR_RESET} $1"
}

print_error() {
    echo -e "${COLOR_ERROR}[ERROR]${COLOR_RESET} $1"
}

print_info() {
    echo -e "${COLOR_INFO}[INFO]${COLOR_RESET} $1"
}

print_debug() {
    echo -e "${COLOR_DEBUG}[DEBUG]${COLOR_RESET} $1"
}

print_separator() {
    echo -e "${COLOR_INFO}-------------------------------------------------------------------${COLOR_RESET}"
}

print_header() {
    print_separator
    echo -e "${COLOR_INFO}$1${COLOR_RESET}"
}

# ========================
# Utility Functions
# ========================

ask_confirmation() {
    local message="$1"
    echo -e -n "${COLOR_WARNING}$message (y/n): ${COLOR_RESET}"
    read -r response
    case "$response" in
        [yY]) return 0 ;;
        [nN]) return 1 ;;
        *)
            print_error "Please enter 'y' or 'n'"
            ask_confirmation "$message"
            ;;
    esac
}

# Get absolute directory of the calling script
get_script_dir() {
    echo "$( cd "$( dirname "${BASH_SOURCE[1]}" )" &> /dev/null && pwd )"
}

# Detect platform information
detect_platform() {
    export OS_TYPE="$(uname -s)"
    export ARCH_TYPE="$(uname -m)"
    print_debug "Platform: ${OS_TYPE} (${ARCH_TYPE})"
}

