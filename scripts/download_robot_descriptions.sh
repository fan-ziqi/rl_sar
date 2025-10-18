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

# Main execution
print_header "[Robot Descriptions Setup]"

# Check if directory exists and is a git repository
if [ -d "$ROBOT_DESC_DIR/.git" ]; then
    print_info "Robot descriptions repository found, updating..."
    cd "$ROBOT_DESC_DIR"

    git pull origin "$REPO_BRANCH" || {
        print_error "Failed to update repository"
        exit 1
    }

    cd "$PROJECT_ROOT"
    print_success "Robot descriptions updated successfully!"
else
    # Clone new repository
    print_info "Cloning robot descriptions repository..."

    if [ -d "$ROBOT_DESC_DIR" ] && [ -n "$(ls -A $ROBOT_DESC_DIR 2>/dev/null)" ]; then
        print_error "Target directory exists but is not a git repository: $ROBOT_DESC_DIR"
        print_info "Please remove or rename it manually"
        exit 1
    fi

    git clone --branch "$REPO_BRANCH" "$REPO_URL" "$ROBOT_DESC_DIR" || {
        print_error "Failed to clone repository"
        print_info "Please check your network connection and repository URL"
        exit 1
    }

    print_success "Robot descriptions cloned successfully!"
fi

print_separator
print_success "Robot descriptions setup completed!"
print_info "Installation path: ${ROBOT_DESC_DIR}"
