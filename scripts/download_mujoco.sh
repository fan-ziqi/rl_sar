#!/bin/bash

# Automatic download script for MuJoCo
# Usage: ./download_mujoco.sh [target_dir]
#   target_dir: Target directory (default: library)
#   Example: ./download_mujoco.sh library

set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Get project root (parent directory of scripts)
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Load common utilities
source "${SCRIPT_DIR}/common.sh"

# Detect platform and architecture
OS_TYPE="$(uname -s)"
ARCH_TYPE="$(uname -m)"

# Parse arguments
if [ $# -eq 0 ]; then
    TARGET_DIR="library"
else
    TARGET_DIR="$1"
fi

# MuJoCo storage path
MUJOCO_BASE_DIR="${PROJECT_ROOT}/${TARGET_DIR}"
mkdir -p "${MUJOCO_BASE_DIR}"

# MuJoCo version and path
MUJOCO_VERSION="3.2.7"
MUJOCO_DIR="${MUJOCO_BASE_DIR}/mujoco"

# Function: Validate MuJoCo installation
is_mujoco_valid() {
    if [ ! -d "$MUJOCO_DIR" ]; then
        return 1
    fi

    # Check based on platform
    case "${OS_TYPE}" in
        Darwin)
            # macOS uses framework structure
            if [ -f "$MUJOCO_DIR/Headers/mujoco.h" ] && [ -f "$MUJOCO_DIR/Versions/Current/libmujoco.${MUJOCO_VERSION}.dylib" ]; then
                return 0
            fi
            ;;
        *)
            # Linux/Windows use standard structure
            if [ -d "$MUJOCO_DIR/lib" ] && [ -d "$MUJOCO_DIR/include" ] && [ -f "$MUJOCO_DIR/include/mujoco/mujoco.h" ]; then
                return 0
            fi
            ;;
    esac

    return 1
}

# Function: Download MuJoCo
download_mujoco() {
    local url=""
    local archive_name=""

    # Select download link based on platform
    case "${OS_TYPE}" in
        Darwin)
            if [ "${ARCH_TYPE}" = "arm64" ]; then
                url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-macos-universal2.dmg"
                archive_name="mujoco-${MUJOCO_VERSION}-macos-universal2.dmg"
                print_info "Platform: macOS Apple Silicon (arm64)"
            elif [ "${ARCH_TYPE}" = "x86_64" ]; then
                url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-macos-universal2.dmg"
                archive_name="mujoco-${MUJOCO_VERSION}-macos-universal2.dmg"
                print_info "Platform: macOS Intel (x86_64)"
            else
                print_error "Unsupported macOS architecture: ${ARCH_TYPE}"
                exit 1
            fi
            ;;
        Linux)
            if [ "${ARCH_TYPE}" = "x86_64" ]; then
                url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
                archive_name="mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
                print_info "Platform: Linux x86_64"
            elif [ "${ARCH_TYPE}" = "aarch64" ]; then
                url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-aarch64.tar.gz"
                archive_name="mujoco-${MUJOCO_VERSION}-linux-aarch64.tar.gz"
                print_info "Platform: Linux aarch64 (ARM64)"
            else
                print_error "Unsupported Linux architecture: ${ARCH_TYPE}"
                exit 1
            fi
            ;;
        MINGW*|MSYS*|CYGWIN*)
            if [ "${ARCH_TYPE}" = "x86_64" ]; then
                url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-windows-x86_64.zip"
                archive_name="mujoco-${MUJOCO_VERSION}-windows-x86_64.zip"
                print_info "Platform: Windows x86_64"
            else
                print_error "Unsupported Windows architecture: ${ARCH_TYPE}"
                exit 1
            fi
            ;;
        *)
            print_error "Unsupported operating system: ${OS_TYPE}"
            exit 1
            ;;
    esac

    print_header "[Downloading MuJoCo ${MUJOCO_VERSION}]"
    print_info "Download URL: $url"
    print_separator

    # Create temporary directory
    local temp_dir="${MUJOCO_BASE_DIR}/.mujoco_download_tmp"
    mkdir -p "$temp_dir"
    cd "$temp_dir"

    # Download archive
    print_info "Downloading MuJoCo..."
    if command -v wget &> /dev/null; then
        wget -q --show-progress "$url" -O "$archive_name" || {
            print_error "Download failed"
            rm -rf "$temp_dir"
            exit 1
        }
    elif command -v curl &> /dev/null; then
        curl -L --progress-bar "$url" -o "$archive_name" || {
            print_error "Download failed"
            rm -rf "$temp_dir"
            exit 1
        }
    else
        print_error "Neither wget nor curl is available"
        rm -rf "$temp_dir"
        exit 1
    fi

    print_success "Download completed!"

    # Extract archive
    print_info "Extracting MuJoCo..."
    local extract_dir="${temp_dir}/extracted"
    mkdir -p "${extract_dir}"

    if [[ "$archive_name" == *.tar.gz ]]; then
        tar -xzf "$archive_name" -C "${extract_dir}" || {
            print_error "Extraction failed"
            rm -rf "$temp_dir"
            exit 1
        }

        # Find extracted directory
        local extracted_dir=$(find "${extract_dir}" -maxdepth 1 -type d -name "mujoco-*" | head -1)
        if [ -d "$extracted_dir" ]; then
            mv "$extracted_dir" "${MUJOCO_DIR}"
        else
            print_error "Extracted directory not found"
            rm -rf "$temp_dir"
            exit 1
        fi
    elif [[ "$archive_name" == *.zip ]]; then
        if command -v unzip &> /dev/null; then
            unzip -o -q "$archive_name" -d "${extract_dir}" || {
                print_error "Extraction failed"
                rm -rf "$temp_dir"
                exit 1
            }
        else
            print_error "unzip command not found. Please install unzip."
            rm -rf "$temp_dir"
            exit 1
        fi

        # Find extracted directory
        local extracted_dir=$(find "${extract_dir}" -maxdepth 1 -type d -name "mujoco-*" | head -1)
        if [ -d "$extracted_dir" ]; then
            mv "$extracted_dir" "${MUJOCO_DIR}"
        else
            print_error "Extracted directory not found"
            rm -rf "$temp_dir"
            exit 1
        fi
    elif [[ "$archive_name" == *.dmg ]]; then
        # Automatically extract from DMG on macOS
        print_info "Extracting MuJoCo from DMG..."
        local mount_point="${temp_dir}/mujoco_dmg_mount"
        mkdir -p "${mount_point}"
        hdiutil attach "$archive_name" -mountpoint "${mount_point}" || {
            print_error "Failed to mount DMG"
            rm -rf "$temp_dir"
            exit 1
        }

        if [ -d "${mount_point}/MuJoCo.app/Contents/Frameworks/MuJoCo.framework" ]; then
            mkdir -p "${MUJOCO_DIR}"
            cp -r "${mount_point}/MuJoCo.app/Contents/Frameworks/MuJoCo.framework/"* "${MUJOCO_DIR}/" || {
                print_error "Failed to copy MuJoCo framework"
                hdiutil detach "${mount_point}"
                rm -rf "$temp_dir"
                exit 1
            }
            hdiutil detach "${mount_point}"

            # Fix dylib install_name to use simple @rpath instead of framework path
            DYLIB_PATH="${MUJOCO_DIR}/Versions/Current/libmujoco.${MUJOCO_VERSION}.dylib"
            if [ -f "${DYLIB_PATH}" ]; then
                print_info "Fixing MuJoCo dylib install_name for cross-platform compatibility..."
                install_name_tool -id "@rpath/libmujoco.${MUJOCO_VERSION}.dylib" "${DYLIB_PATH}" || {
                    print_warning "Failed to modify dylib install_name, but continuing..."
                }

                # Re-sign the dylib after modification (required on macOS)
                print_info "Re-signing MuJoCo dylib..."
                codesign --force --sign - "${DYLIB_PATH}" || {
                    print_warning "Failed to re-sign dylib, but continuing..."
                }
            fi
        else
            print_error "Could not find MuJoCo.framework in DMG"
            hdiutil detach "${mount_point}"
            rm -rf "$temp_dir"
            exit 1
        fi
    fi

    # Cleanup temporary directory
    cd "$PROJECT_ROOT"
    rm -rf "$temp_dir"

    print_success "MuJoCo extracted successfully!"
}

# Function: Save version info
save_version_info() {
    echo "${MUJOCO_VERSION}" > "${MUJOCO_DIR}/VERSION_NUMBER"
    print_info "MuJoCo version ${MUJOCO_VERSION} installed to: ${MUJOCO_DIR}"
}

# Main execution
print_header "[MuJoCo Setup]"

# Check if MuJoCo is already installed and valid
if is_mujoco_valid; then
    if [ -f "${MUJOCO_DIR}/VERSION_NUMBER" ]; then
        installed_version=$(cat "${MUJOCO_DIR}/VERSION_NUMBER")
        if [ "$installed_version" = "$MUJOCO_VERSION" ]; then
            print_success "MuJoCo ${MUJOCO_VERSION} is already installed and valid"
            exit 0
        else
            print_warning "MuJoCo version mismatch: installed=$installed_version, required=$MUJOCO_VERSION"
            print_info "Reinstalling MuJoCo..."
            rm -rf "${MUJOCO_DIR}"
        fi
    else
        print_warning "MuJoCo version info not found"
        print_info "Reinstalling MuJoCo..."
        rm -rf "${MUJOCO_DIR}"
    fi
else
    print_info "MuJoCo not found or invalid"
    # Remove incomplete installation
    if [ -d "${MUJOCO_DIR}" ]; then
        print_warning "MuJoCo directory incomplete, re-downloading..."
        rm -rf "${MUJOCO_DIR}"
    fi
fi

# Download and install MuJoCo
download_mujoco

# Verify installation
if ! is_mujoco_valid; then
    print_error "MuJoCo installation failed"
    exit 1
fi

save_version_info

print_separator
print_success "MuJoCo ${MUJOCO_VERSION} setup completed successfully!"
print_info "Installation path: ${MUJOCO_DIR}"
