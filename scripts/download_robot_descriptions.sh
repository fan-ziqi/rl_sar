#!/bin/bash

# Automatic download script for robot descriptions
# Usage: ./download_robot_descriptions.sh [target_dir]
#   target_dir: Target directory (default: src/rl_sar_zoo)
#   Example: ./download_robot_descriptions.sh src/rl_sar_zoo

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
    TARGET_DIR="src/rl_sar_zoo"
else
    TARGET_DIR="$1"
fi

# Robot descriptions storage path
ROBOT_DESC_DIR="${PROJECT_ROOT}/${TARGET_DIR}"

# Repository configuration
REPO_URL="https://github.com/fan-ziqi/rl_sar_zoo.git"
REPO_BRANCH="main"

# Expected version - update this when URDF files need to be updated
EXPECTED_VERSION="1.0.1"

# Robot descriptions VERSION file (installed version)
ROBOT_DESC_VERSION_FILE="${ROBOT_DESC_DIR}/VERSION"

# Function: Get current installed version
get_current_version() {
    if [ -f "$ROBOT_DESC_VERSION_FILE" ]; then
        cat "$ROBOT_DESC_VERSION_FILE" | tr -d '[:space:]'
    else
        echo "0.0.0"
    fi
}

# Function: Check if version is up-to-date
is_version_uptodate() {
    local current_version=$(get_current_version)
    if [ "$current_version" = "$EXPECTED_VERSION" ]; then
        return 0
    fi
    return 1
}

# Function: Validate robot descriptions installation
is_robot_descriptions_valid() {
    if [ ! -d "$ROBOT_DESC_DIR" ]; then
        return 1
    fi

    # Check if directory contains expected robot description files
    # Look for at least one robot description directory
    local robot_count=$(find "$ROBOT_DESC_DIR" -maxdepth 1 -type d -name "*_description" | wc -l)
    if [ "$robot_count" -gt 0 ]; then
        return 0
    fi

    return 1
}

# Main execution
print_header "[Robot Descriptions Setup]"

# Check if robot descriptions already exist and are valid
if is_robot_descriptions_valid; then
    # Check if version is up-to-date
    if is_version_uptodate; then
        current_version=$(get_current_version)
        print_success "Robot descriptions are up-to-date (version: ${current_version})"
        print_info "Installation path: ${ROBOT_DESC_DIR}"
        exit 0
    fi

    # Version mismatch - need to update
    current_version=$(get_current_version)
    print_warning "Version mismatch: installed=${current_version}, expected=${EXPECTED_VERSION}"

    # Check network status
    if check_network_status "github.com"; then
        # Online: try to update if it's a git repository
        if [ -d "$ROBOT_DESC_DIR/.git" ]; then
            print_info "Attempting to update robot descriptions..."
            cd "$ROBOT_DESC_DIR"

            git pull origin "$REPO_BRANCH" || {
                print_warning "Update failed, continuing with existing version"
                cd "$PROJECT_ROOT"
                exit 0
            }

            cd "$PROJECT_ROOT"
            print_success "Robot descriptions updated successfully"
        else
            print_warning "Not a git repository, cannot update"
        fi
    else
        print_warning "Network unavailable, skipping update"
    fi

    print_success "Robot descriptions are available"
    print_info "Installation path: ${ROBOT_DESC_DIR}"
    exit 0
fi

# Robot descriptions not found or invalid, need to download
print_warning "Robot descriptions not found or invalid"

# Check if directory exists but is not valid
if [ -d "$ROBOT_DESC_DIR" ]; then
    if [ -d "$ROBOT_DESC_DIR/.git" ]; then
        # Check network before attempting update
        if ! check_network_status "github.com"; then
            print_error "Cannot fix incomplete installation in offline mode"
            print_info "Please run with network access or manually remove directory: $ROBOT_DESC_DIR"
            exit 1
        fi

        print_warning "Robot descriptions directory exists but is incomplete"
        print_info "Attempting to update repository..."
        cd "$ROBOT_DESC_DIR"

        git pull origin "$REPO_BRANCH" || {
            print_error "Update failed"
            cd "$PROJECT_ROOT"
            exit 1
        }

        cd "$PROJECT_ROOT"

        # Recheck validity after update attempt
        if is_robot_descriptions_valid; then
            print_success "Robot descriptions are now valid"
            print_info "Installation path: ${ROBOT_DESC_DIR}"
            exit 0
        else
            print_error "Robot descriptions still invalid after update"
            print_info "Consider removing directory and re-cloning: $ROBOT_DESC_DIR"
            exit 1
        fi
    else
        print_error "Target directory exists but is not a git repository: $ROBOT_DESC_DIR"
        print_info "Please remove or rename it manually"
        exit 1
    fi
fi

# Need to clone new repository
# Check network before cloning
if ! check_network_status "github.com"; then
    print_error "Cannot download robot descriptions in offline mode"
    print_info "Please run this script with network access or manually download to: $ROBOT_DESC_DIR"
    exit 1
fi

print_info "Cloning robot descriptions repository..."

git clone --branch "$REPO_BRANCH" "$REPO_URL" "$ROBOT_DESC_DIR" || {
    print_error "Clone failed"
    print_info "Please check your network connection and repository URL"
    exit 1
}

# Verify installation
if ! is_robot_descriptions_valid; then
    print_error "Robot descriptions installation failed"
    exit 1
fi

print_separator
print_success "Robot descriptions setup completed!"
print_info "Installation path: ${ROBOT_DESC_DIR}"
print_info "Version: ${EXPECTED_VERSION}"
