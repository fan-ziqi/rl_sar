#!/bin/bash

# Automatic download script for LibTorch and ONNX Runtime
# Usage: ./download_inference_runtime.sh [target_dir] [libtorch|onnx|all]
#   target_dir: Target directory (default: inference_runtime)
#   Example: ./download_inference_runtime.sh inference_runtime all
#            ./download_inference_runtime.sh all

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

# Parse arguments: support both new and old usage
if [ $# -eq 0 ]; then
    TARGET_DIR="library/inference_runtime"
    DOWNLOAD_TARGET="all"
elif [ $# -eq 1 ]; then
    if [[ "$1" == "libtorch" || "$1" == "onnx" || "$1" == "all" ]]; then
        TARGET_DIR="library/inference_runtime"
        DOWNLOAD_TARGET="$1"
    else
        TARGET_DIR="$1"
        DOWNLOAD_TARGET="all"
    fi
else
    TARGET_DIR="$1"
    DOWNLOAD_TARGET="$2"
fi

# Inference runtime storage path
MODEL_INTERFACE_DIR="${PROJECT_ROOT}/${TARGET_DIR}"
mkdir -p "${MODEL_INTERFACE_DIR}"

# LibTorch version and path
LIBTORCH_VERSION="2.3.0"
LIBTORCH_DIR="${MODEL_INTERFACE_DIR}/libtorch"

# ONNX Runtime version and path
ONNXRUNTIME_VERSION="1.22.0"
ONNXRUNTIME_DIR="${MODEL_INTERFACE_DIR}/onnxruntime"

# Function: Validate LibTorch installation
is_libtorch_valid() {
    if [ ! -d "$LIBTORCH_DIR" ]; then
        return 1
    fi

    if [ ! -d "$LIBTORCH_DIR/lib" ] || [ ! -d "$LIBTORCH_DIR/include" ]; then
        return 1
    fi

    if [ -f "$LIBTORCH_DIR/include/torch/torch.h" ] || [ -f "$LIBTORCH_DIR/include/torch/csrc/api/include/torch/torch.h" ]; then
        return 0
    fi

    return 1
}

# Function: Download LibTorch
download_libtorch() {
    # Check if Jetson platform
    if [ "${IS_JETSON}" = true ]; then
        print_info "Jetson platform detected - using install_pytorch_jetson.sh"
        if [ -x "${SCRIPT_DIR}/install_pytorch_jetson.sh" ]; then
            "${SCRIPT_DIR}/install_pytorch_jetson.sh" "${LIBTORCH_DIR}"
            return $?
        else
            print_error "install_pytorch_jetson.sh not found or not executable"
            print_info "Please ensure ${SCRIPT_DIR}/install_pytorch_jetson.sh exists"
            return 1
        fi
    fi

    local url=""
    local archive_name=""

    # Select download link based on platform
    case "${OS_TYPE}" in
        Darwin)
            if [ "${ARCH_TYPE}" = "arm64" ]; then
                url="https://download.pytorch.org/libtorch/cpu/libtorch-macos-arm64-${LIBTORCH_VERSION}.zip"
                archive_name="libtorch-macos-arm64-${LIBTORCH_VERSION}.zip"
            else
                url="https://download.pytorch.org/libtorch/cpu/libtorch-macos-x86_64-${LIBTORCH_VERSION}.zip"
                archive_name="libtorch-macos-x86_64-${LIBTORCH_VERSION}.zip"
            fi
            ;;
        Linux)
            if [ "${ARCH_TYPE}" = "aarch64" ]; then
                print_error "ARM64 Linux detected but not identified as Jetson"
                print_error "LibTorch prebuilt binaries for generic ARM64 Linux are not available"
                print_info "If this is a Jetson device, please check /etc/nv_tegra_release"
                exit 1
            fi
            url="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcpu.zip"
            archive_name="libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}+cpu.zip"
            ;;
        MINGW*|MSYS*)
            url="https://download.pytorch.org/libtorch/cpu/libtorch-win-shared-with-deps-${LIBTORCH_VERSION}%2Bcpu.zip"
            archive_name="libtorch-win-shared-with-deps-${LIBTORCH_VERSION}+cpu.zip"
            ;;
        *)
            print_error "Unsupported OS: ${OS_TYPE}"
            exit 1
            ;;
    esac

    local archive_path="${MODEL_INTERFACE_DIR}/${archive_name}"

    print_info "Downloading LibTorch ${LIBTORCH_VERSION}..."
    print_info "Platform: ${OS_TYPE} (${ARCH_TYPE})"
    print_info "URL: ${url}"

    # Download
    if command -v curl &> /dev/null; then
        curl -L --progress-bar -o "${archive_path}" "${url}" || {
            print_error "Download failed"
            rm -f "${archive_path}"
            exit 1
        }
    elif command -v wget &> /dev/null; then
        wget --show-progress -O "${archive_path}" "${url}" || {
            print_error "Download failed"
            rm -f "${archive_path}"
            exit 1
        }
    else
        print_error "curl or wget is required to download files"
        exit 1
    fi

    print_info "Extracting LibTorch..."
    local temp_dir="${MODEL_INTERFACE_DIR}/temp_extract"
    rm -rf "${temp_dir}"
    mkdir -p "${temp_dir}"

    unzip -o -q "${archive_path}" -d "${temp_dir}" || {
        print_error "Extraction failed"
        rm -rf "${temp_dir}" "${archive_path}"
        exit 1
    }

    # Move extracted files
    if [ -d "${temp_dir}/libtorch" ]; then
        mv "${temp_dir}/libtorch" "${LIBTORCH_DIR}"
    else
        print_error "Incorrect directory structure after extraction"
        rm -rf "${temp_dir}" "${archive_path}"
        exit 1
    fi

    # Cleanup
    rm -rf "${temp_dir}" "${archive_path}"

    print_success "LibTorch ${LIBTORCH_VERSION} installed successfully"
}

# Function: Validate ONNX Runtime installation
is_onnxruntime_valid() {
    if [ ! -d "$ONNXRUNTIME_DIR" ]; then
        return 1
    fi

    if [ ! -d "$ONNXRUNTIME_DIR/lib" ] || [ ! -d "$ONNXRUNTIME_DIR/include" ]; then
        return 1
    fi

    return 0
}

# Function: Download ONNX Runtime
download_onnxruntime() {
    local url=""
    local archive_name=""

    # Select download link based on platform
    case "${OS_TYPE}" in
        Darwin)
            if [ "${ARCH_TYPE}" = "arm64" ]; then
                url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-osx-arm64-${ONNXRUNTIME_VERSION}.tgz"
                archive_name="onnxruntime-osx-arm64-${ONNXRUNTIME_VERSION}.tgz"
            else
                url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-osx-x86_64-${ONNXRUNTIME_VERSION}.tgz"
                archive_name="onnxruntime-osx-x86_64-${ONNXRUNTIME_VERSION}.tgz"
            fi
            ;;
        Linux)
            if [ "${ARCH_TYPE}" = "aarch64" ]; then
                url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz"
                archive_name="onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz"
            else
                url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz"
                archive_name="onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz"
            fi
            ;;
        MINGW*|MSYS*)
            url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-win-x64-${ONNXRUNTIME_VERSION}.zip"
            archive_name="onnxruntime-win-x64-${ONNXRUNTIME_VERSION}.zip"
            ;;
        *)
            print_error "Unsupported OS: ${OS_TYPE}"
            exit 1
            ;;
    esac

    local archive_path="${MODEL_INTERFACE_DIR}/${archive_name}"

    print_info "Downloading ONNX Runtime ${ONNXRUNTIME_VERSION}..."
    print_info "Platform: ${OS_TYPE} (${ARCH_TYPE})"
    print_info "URL: ${url}"

    # Download
    if command -v curl &> /dev/null; then
        curl -L --progress-bar -o "${archive_path}" "${url}" || {
            print_error "Download failed"
            rm -f "${archive_path}"
            exit 1
        }
    elif command -v wget &> /dev/null; then
        wget --show-progress -O "${archive_path}" "${url}" || {
            print_error "Download failed"
            rm -f "${archive_path}"
            exit 1
        }
    else
        print_error "curl or wget is required to download files"
        exit 1
    fi

    print_info "Extracting ONNX Runtime..."
    local temp_dir="${MODEL_INTERFACE_DIR}/temp_extract_onnx"
    rm -rf "${temp_dir}"
    mkdir -p "${temp_dir}"

    # Extract based on file type
    if [[ "${archive_name}" == *.zip ]]; then
        unzip -o -q "${archive_path}" -d "${temp_dir}" || {
            print_error "Extraction failed"
            rm -rf "${temp_dir}" "${archive_path}"
            exit 1
        }
    else
        tar -xzf "${archive_path}" -C "${temp_dir}" || {
            print_error "Extraction failed"
            rm -rf "${temp_dir}" "${archive_path}"
            exit 1
        }
    fi

    # Move extracted files (ONNX Runtime directory includes version)
    local extracted_dir=$(find "${temp_dir}" -maxdepth 1 -type d -name "onnxruntime-*" | head -1)
    if [ -d "${extracted_dir}" ]; then
        mv "${extracted_dir}" "${ONNXRUNTIME_DIR}"
    else
        print_error "Incorrect directory structure after extraction"
        rm -rf "${temp_dir}" "${archive_path}"
        exit 1
    fi

    # Cleanup
    rm -rf "${temp_dir}" "${archive_path}"

    print_success "ONNX Runtime ${ONNXRUNTIME_VERSION} installed successfully"
}

# Main workflow
case "$DOWNLOAD_TARGET" in
    libtorch)
        print_info "Checking LibTorch..."
        if is_libtorch_valid; then
            print_success "LibTorch already exists and is valid"
        else
            if [ -d "$LIBTORCH_DIR" ]; then
                print_warning "LibTorch directory incomplete, re-downloading..."
                rm -rf "$LIBTORCH_DIR"
            fi
            download_libtorch
            if ! is_libtorch_valid; then
                print_error "LibTorch installation failed"
                exit 1
            fi
        fi
        ;;

    onnx)
        print_info "Checking ONNX Runtime..."
        if is_onnxruntime_valid; then
            print_success "ONNX Runtime already exists and is valid"
        else
            if [ -d "$ONNXRUNTIME_DIR" ]; then
                print_warning "ONNX Runtime directory incomplete, re-downloading..."
                rm -rf "$ONNXRUNTIME_DIR"
            fi
            download_onnxruntime
            if ! is_onnxruntime_valid; then
                print_error "ONNX Runtime installation failed"
                exit 1
            fi
        fi
        ;;

    all)
        print_info "Checking inference runtimes..."

        # Check LibTorch
        if is_libtorch_valid; then
            print_success "LibTorch already exists and is valid"
        else
            if [ -d "$LIBTORCH_DIR" ]; then
                print_warning "LibTorch directory incomplete, re-downloading..."
                rm -rf "$LIBTORCH_DIR"
            fi
            download_libtorch
            if ! is_libtorch_valid; then
                print_error "LibTorch installation failed"
                exit 1
            fi
        fi

        # Check ONNX Runtime
        if is_onnxruntime_valid; then
            print_success "ONNX Runtime already exists and is valid"
        else
            if [ -d "$ONNXRUNTIME_DIR" ]; then
                print_warning "ONNX Runtime directory incomplete, re-downloading..."
                rm -rf "$ONNXRUNTIME_DIR"
            fi
            download_onnxruntime
            if ! is_onnxruntime_valid; then
                print_error "ONNX Runtime installation failed"
                exit 1
            fi
        fi
        ;;

    *)
        echo "Usage: $0 [libtorch|onnx|all]"
        exit 1
        ;;
esac

print_success "All inference runtimes are ready"
