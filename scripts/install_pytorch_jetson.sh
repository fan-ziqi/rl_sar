#!/bin/bash

# PyTorch installation and LibTorch creation script for NVIDIA Jetson platform
# This script automatically detects JetPack version, installs PyTorch, and creates LibTorch
# Usage: ./install_pytorch_jetson.sh [target_dir]
#   target_dir: Target directory for LibTorch (default: ../library/inference_runtime/libtorch)

set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Get project root (parent directory of scripts)
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Load common utilities
source "${SCRIPT_DIR}/common.sh"

# Parse arguments
if [ $# -eq 0 ]; then
    LIBTORCH_DIR="${PROJECT_ROOT}/library/inference_runtime/libtorch"
else
    LIBTORCH_DIR="$1"
fi

TEMP_DIR="$(dirname "$LIBTORCH_DIR")"
mkdir -p "${TEMP_DIR}"

# Function: Detect if running on Jetson
is_jetson_platform() {
    if [ "$(uname -s)" != "Linux" ] || [ "$(uname -m)" != "aarch64" ]; then
        return 1
    fi

    # Check for Jetson-specific indicators
    if [ -f /etc/nv_tegra_release ] || [ -d /usr/local/cuda-*/targets/aarch64-linux ]; then
        return 0
    fi

    return 1
}

# Function: Detect JetPack version
detect_jetpack_version() {
    local jetpack_version=""

    # Try to get version from nv_tegra_release
    if [ -f /etc/nv_tegra_release ]; then
        # Example: "# R35 (release), REVISION: 4.1"
        jetpack_version=$(grep -oP 'R\K[0-9]+' /etc/nv_tegra_release | head -1)
    fi

    # Try alternative method using dpkg
    if [ -z "$jetpack_version" ]; then
        if command -v dpkg &> /dev/null; then
            local nvidia_l4t=$(dpkg -l | grep nvidia-l4t-core | awk '{print $3}' | grep -oP '^[0-9]+' | head -1)
            if [ -n "$nvidia_l4t" ]; then
                jetpack_version=$nvidia_l4t
            fi
        fi
    fi

    echo "$jetpack_version"
}

# Function: Get PyTorch wheel URL for JetPack version
get_pytorch_wheel_url() {
    local jetpack_ver="$1"
    local wheel_url=""
    local python_version=""

    # Detect Python version
    for py_cmd in python3 python; do
        if command -v $py_cmd &> /dev/null; then
            python_version=$($py_cmd -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')" 2>/dev/null || echo "")
            if [ -n "$python_version" ]; then
                break
            fi
        fi
    done

    # Default to Python 3.8 if detection fails
    if [ -z "$python_version" ]; then
        python_version="38"
    fi

    # Map JetPack version to PyTorch wheel
    case "$jetpack_ver" in
        35)
            # JetPack 5.1.x (L4T R35.x) - PyTorch 2.1.0
            wheel_url="https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp${python_version}-cp${python_version}-linux_aarch64.whl"
            ;;
        34)
            # JetPack 5.0.x (L4T R34.x) - PyTorch 2.0.0
            wheel_url="https://developer.download.nvidia.cn/compute/redist/jp/v50/pytorch/torch-2.0.0+nv23.05-cp${python_version}-cp${python_version}-linux_aarch64.whl"
            ;;
        32)
            # JetPack 4.6.x (L4T R32.x) - PyTorch 1.13.0
            wheel_url="https://developer.download.nvidia.cn/compute/redist/jp/v461/pytorch/torch-1.13.0a0+340c4120.nv22.06-cp${python_version}-cp${python_version}-linux_aarch64.whl"
            ;;
        *)
            # Default to JetPack 5.1.2 for unknown versions
            print_warning "Unknown JetPack version: R${jetpack_ver}, using default PyTorch 2.1.0"
            wheel_url="https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp${python_version}-cp${python_version}-linux_aarch64.whl"
            ;;
    esac

    echo "$wheel_url"
}

# Function: Install PyTorch on Jetson
install_pytorch_jetson() {
    print_info "Installing PyTorch for Jetson platform..."

    # Detect JetPack version
    local jetpack_ver=$(detect_jetpack_version)

    if [ -z "$jetpack_ver" ]; then
        print_error "Cannot detect JetPack version"
        print_info "Please check /etc/nv_tegra_release or install jetson-stats:"
        print_info "  sudo pip install jetson-stats"
        print_info "  sudo jtop"
        return 1
    fi

    print_info "Detected JetPack version: R${jetpack_ver}"

    # Get PyTorch wheel URL
    local wheel_url=$(get_pytorch_wheel_url "$jetpack_ver")
    local wheel_name=$(basename "$wheel_url")
    local wheel_path="${TEMP_DIR}/${wheel_name}"

    print_info "PyTorch wheel: ${wheel_name}"
    print_info "Downloading from: ${wheel_url}"

    # Install dependencies first
    print_info "Installing dependencies..."
    sudo apt-get update -qq 2>&1 | grep -v "^Ign:" | grep -v "^Get:" || true
    sudo apt-get install -y -qq python3-pip python3-dev 2>&1 | grep -v "is already the newest version" || {
        print_warning "Some dependencies may have failed to install, continuing..."
    }

    # Download PyTorch wheel
    print_info "Downloading PyTorch wheel (this may take several minutes)..."
    if command -v curl &> /dev/null; then
        curl -L --progress-bar -o "${wheel_path}" "${wheel_url}" || {
            print_error "Failed to download PyTorch wheel"
            rm -f "${wheel_path}"
            return 1
        }
    elif command -v wget &> /dev/null; then
        wget --show-progress -O "${wheel_path}" "${wheel_url}" || {
            print_error "Failed to download PyTorch wheel"
            rm -f "${wheel_path}"
            return 1
        }
    else
        print_error "curl or wget is required to download files"
        return 1
    fi

    # Install PyTorch wheel
    print_info "Installing PyTorch wheel..."

    # Try with different Python commands
    local install_success=false
    for py_cmd in python3 python; do
        if command -v $py_cmd &> /dev/null; then
            print_info "Installing with ${py_cmd}..."
            if $py_cmd -m pip install --user "${wheel_path}" 2>&1 | tee /dev/tty | grep -q "Successfully installed"; then
                install_success=true
                break
            fi
        fi
    done

    # Clean up wheel file
    rm -f "${wheel_path}"

    if [ "$install_success" = false ]; then
        print_error "Failed to install PyTorch wheel"
        print_info "You may need to install it manually:"
        print_info "  pip install ${wheel_url}"
        return 1
    fi

    print_success "PyTorch installed successfully"
    return 0
}

# Function: Create LibTorch from installed PyTorch
create_libtorch_from_pytorch() {
    print_info "Creating LibTorch from PyTorch installation..."

    local torch_path=""
    local python_cmd=""

    # Try to detect PyTorch installation
    for py_cmd in python3 python; do
        if command -v $py_cmd &> /dev/null; then
            torch_path=$($py_cmd -c "import torch, os; print(os.path.dirname(torch.__file__))" 2>/dev/null || echo "")
            if [ -n "$torch_path" ] && [ -d "$torch_path" ]; then
                python_cmd=$py_cmd
                break
            fi
        fi
    done

    if [ -z "$torch_path" ] || [ ! -d "$torch_path" ]; then
        print_error "PyTorch not found"
        return 1
    fi

    print_success "Found PyTorch at: ${torch_path}"

    # Get PyTorch version
    local torch_version=$($python_cmd -c "import torch; print(torch.__version__)" 2>/dev/null || echo "unknown")
    print_info "PyTorch version: ${torch_version}"

    # Check if required directories exist
    local missing_dirs=()
    for dir in lib include; do
        if [ ! -d "${torch_path}/${dir}" ]; then
            missing_dirs+=("$dir")
        fi
    done

    if [ ${#missing_dirs[@]} -gt 0 ]; then
        print_error "PyTorch installation incomplete. Missing directories: ${missing_dirs[*]}"
        return 1
    fi

    print_info "Creating LibTorch directory at: ${LIBTORCH_DIR}"

    # Remove existing LibTorch if any
    if [ -d "${LIBTORCH_DIR}" ]; then
        print_warning "Removing existing LibTorch directory..."
        rm -rf "${LIBTORCH_DIR}"
    fi

    # Create LibTorch directory
    mkdir -p "${LIBTORCH_DIR}"

    # Copy required directories
    print_info "Copying PyTorch files to LibTorch directory..."
    local copied_dirs=()
    local optional_dirs=()

    for dir in bin include lib share; do
        if [ -d "${torch_path}/${dir}" ]; then
            print_info "Copying ${dir}..."
            cp -r "${torch_path}/${dir}" "${LIBTORCH_DIR}/" || {
                print_error "Failed to copy ${dir}"
                rm -rf "${LIBTORCH_DIR}"
                return 1
            }
            copied_dirs+=("$dir")
        else
            optional_dirs+=("$dir")
        fi
    done

    if [ ${#copied_dirs[@]} -eq 0 ]; then
        print_error "No directories were copied"
        rm -rf "${LIBTORCH_DIR}"
        return 1
    fi

    print_success "Copied directories: ${copied_dirs[*]}"
    if [ ${#optional_dirs[@]} -gt 0 ]; then
        print_info "Optional directories not found (skipped): ${optional_dirs[*]}"
    fi

    # Verify the installation
    if [ ! -d "${LIBTORCH_DIR}/lib" ] || [ ! -d "${LIBTORCH_DIR}/include" ]; then
        print_error "LibTorch creation failed: missing required directories"
        rm -rf "${LIBTORCH_DIR}"
        return 1
    fi

    print_success "LibTorch created successfully from PyTorch ${torch_version}"
    print_info "LibTorch location: ${LIBTORCH_DIR}"

    return 0
}

# Main execution
main() {
    print_header "Jetson PyTorch Installation and LibTorch Setup"

    # Check if running on Jetson
    if ! is_jetson_platform; then
        print_error "This script is designed for NVIDIA Jetson platforms only"
        print_info "Detected platform: $(uname -s) $(uname -m)"
        exit 1
    fi

    print_success "Detected NVIDIA Jetson platform"

    # Check if PyTorch is already installed
    print_info "Checking for existing PyTorch installation..."
    local torch_exists=false
    for py_cmd in python3 python; do
        if command -v $py_cmd &> /dev/null; then
            if $py_cmd -c "import torch" 2>/dev/null; then
                torch_exists=true
                local torch_version=$($py_cmd -c "import torch; print(torch.__version__)" 2>/dev/null)
                print_success "PyTorch ${torch_version} is already installed"
                break
            fi
        fi
    done

    # Install PyTorch if not found
    if [ "$torch_exists" = false ]; then
        print_warning "PyTorch not found, installing..."
        if ! install_pytorch_jetson; then
            print_error "Failed to install PyTorch"
            exit 1
        fi
    fi

    # Create LibTorch
    if ! create_libtorch_from_pytorch; then
        print_error "Failed to create LibTorch"
        exit 1
    fi

    # Success message
    print_separator
    print_success "Setup completed successfully!"
    print_info "LibTorch has been created at: ${LIBTORCH_DIR}"
    print_separator
}

# Run main function
main "$@"

