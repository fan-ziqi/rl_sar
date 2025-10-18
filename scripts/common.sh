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

# Check if network is available
# Returns 0 if online, 1 if offline
is_online() {
    local host="${1:-github.com}"
    local timeout="${2:-3}"

    # Try multiple methods for better compatibility
    if command -v curl &> /dev/null; then
        # Use curl with timeout
        curl --connect-timeout "$timeout" --silent --head "$host" &> /dev/null
        return $?
    elif command -v wget &> /dev/null; then
        # Use wget with timeout
        wget --timeout="$timeout" --tries=1 --spider --quiet "$host" &> /dev/null
        return $?
    elif command -v ping &> /dev/null; then
        # Fallback to ping (may not work if ICMP is blocked)
        ping -c 1 -W "$timeout" "$host" &> /dev/null
        return $?
    else
        # No tool available, assume offline to be safe
        return 1
    fi
}

# Check network with user feedback
check_network_status() {
    local host="${1:-github.com}"

    if is_online "$host"; then
        print_info "Network available, checking for updates..."
        return 0
    else
        print_warning "Network unavailable or cannot access $host"
        print_info "Using local files (offline mode)"
        return 1
    fi
}

