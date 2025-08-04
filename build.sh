#!/bin/bash
set -e

# ========================
# Configuration
# ========================

# Color definitions
COLOR_ERROR='\033[0;31m'     # Red
COLOR_SUCCESS='\033[0;32m'   # Green
COLOR_WARNING='\033[1;33m'   # Yellow
COLOR_INFO='\033[0;34m'      # Blue
COLOR_DEBUG='\033[0;36m'     # Cyan
COLOR_RESET='\033[0m'        # Reset

# ========================
# Helper Functions
# ========================

print_separator() {
    echo -e "${COLOR_INFO}-------------------------------------------------------------------${COLOR_RESET}"
}

print_header() {
    print_separator
    echo -e "${COLOR_INFO}$1${COLOR_RESET}"
}

print_success() {
    echo -e "${COLOR_SUCCESS}$1${COLOR_RESET}"
}

print_warning() {
    echo -e "${COLOR_WARNING}$1${COLOR_RESET}"
}

print_error() {
    echo -e "${COLOR_ERROR}$1${COLOR_RESET}"
}

print_info() {
    echo -e "${COLOR_INFO}$1${COLOR_RESET}"
}

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

# ========================
# Build Functions
# ========================

run_cmake_build() {
    local simulator="$1"
    local real_robot="$2"

    print_header "[Running CMake Build]"

    if [[ "$simulator" != "MUJOCO" ]]; then
        print_error "CMake build only supports MUJOCO simulator"
        exit 1
    fi

    print_warning "NOTE: CMake build is for hardware deployment and mujoco simulation without ros."
    print_separator

    # Set default real robot if not specified
    if [[ -z "$real_robot" ]]; then
        real_robot="ALL"
        print_info "Building ALL real robots by default"
    fi

    cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON -DSIMULATOR=MUJOCO -DREAL_ROBOT="$real_robot"
    cmake --build cmake_build -j4

    print_success "CMake build completed!"
}

run_ros_build() {
    local packages=()
    local build_args=()
    local cmake_args=()

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --*)
                if [[ "$1" == "--cmake-args" ]]; then
                    shift
                    while [[ $# -gt 0 && "$1" != --* ]]; do
                        cmake_args+=("$1")
                        shift
                    done
                else
                    build_args+=("$1")
                    shift
                fi
                ;;
            *)
                packages+=("$1")
                shift
                ;;
        esac
    done

    local package_list=$(IFS=' '; echo "${packages[*]}")
    local build_args_list=$(IFS=' '; echo "${build_args[*]}")
    local cmake_args_list=$(IFS=' '; echo "${cmake_args[*]}")

    print_header "[Running ROS Build]"

    # Clean existing symlinks
    clean_existing_symlinks "${packages[@]}"

    # Detect incompatible artifacts
    detect_incompatible_build_artifacts

    # Create appropriate symlinks
    if [ ${#packages[@]} -eq 0 ]; then
        create_symlinks_for_all_packages
    else
        create_symlinks_for_specific_packages "${packages[@]}"
    fi

    # Execute build
    if [ ${#packages[@]} -eq 0 ]; then
        if [[ "$ROS_DISTRO" == "noetic" ]]; then
            print_header "[Using catkin build]"
            print_info "Building all packages with args: ${build_args_list} ${cmake_args_list} --no-warn-unused-cli"
            catkin build ${build_args_list} --cmake-args ${cmake_args_list} --no-warn-unused-cli
        else
            print_header "[Using colcon build]"
            print_info "Building all packages with args: ${build_args_list} ${cmake_args_list} --no-warn-unused-cli"
            colcon build --merge-install --symlink-install ${build_args_list} --cmake-args ${cmake_args_list} --no-warn-unused-cli
        fi
    else
        if [[ "$ROS_DISTRO" == "noetic" ]]; then
            print_header "[Using catkin build]"
            print_info "Building specific packages: $package_list with args: ${build_args_list} ${cmake_args_list} --no-warn-unused-cli"
            catkin build ${package_list} ${build_args_list} --cmake-args ${cmake_args_list}
        else
            print_header "[Using colcon build]"
            print_info "Building specific packages: $package_list with args: ${build_args_list} ${cmake_args_list} --no-warn-unused-cli"
            colcon build --merge-install --symlink-install --packages-select ${package_list} ${build_args_list} --cmake-args ${cmake_args_list} --no-warn-unused-cli
        fi
    fi

    print_success "ROS build completed!"
}

# ========================
# Clean Functions
# ========================

clean_workspace() {
    local packages=("$@")

    print_header "[Cleaning Workspace]"

    # Show what will be cleaned
    print_info "The following will be cleaned:"
    if [ ${#packages[@]} -eq 0 ]; then
        echo "  - All package.xml symlinks in directory src/"
    else
        echo "  - Package.xml symlinks for: ${packages[*]}"
    fi
    echo "  - directory build/"
    echo "  - directory devel/"
    echo "  - directory install/"
    echo "  - directory log/"
    echo "  - directory logs/"
    echo "  - directory .catkin_tools/"
    echo "  - directory cmake_build/"

    # Ask for confirmation
    if [ ${#packages[@]} -eq 0 ]; then
        if ! ask_confirmation "Are you sure you want to clean ALL symlinks and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    else
        if ! ask_confirmation "Are you sure you want to clean symlinks for specified packages and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    fi

    # Remove package.xml symlinks
    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        print_success "Removed all symlinks"
    else
        print_info "Removing symlinks for specific packages..."
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ]; then
                if [ -L "$package_dir/package.xml" ]; then
                    rm -f "$package_dir/package.xml"
                    print_success "Removed symlink from $package_name"
                else
                    print_warning "No symlink found for $package_name"
                fi
            else
                print_error "Package '$package_name' not found in src directory"
            fi
        done
    fi

    # Clean build artifacts
    print_info "Cleaning build artifacts..."
    rm -rf build/ devel/ install/ log/ logs/ .catkin_tools/ cmake_build/

    print_success "Clean completed!"
}

clean_existing_symlinks() {
    local packages=("$@")

    print_header "[Cleaning Existing Symlinks]"

    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all existing package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        print_success "Removed all existing symlinks"
    else
        print_info "Removing existing symlinks for specified packages..."
        removed_packages=()
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ] && [ -L "$package_dir/package.xml" ]; then
                rm -f "$package_dir/package.xml"
                removed_packages+=("$package_name")
            fi
        done

        if [ ${#removed_packages[@]} -gt 0 ]; then
            print_success "Removed existing symlinks from: ${removed_packages[*]}"
        else
            print_warning "No existing symlinks found"
        fi
    fi
}

# ========================
# ROS Specific Functions
# ========================

detect_incompatible_build_artifacts() {
    print_header "[Checking for Incompatible Build Artifacts]"

    local needs_cleanup=false

    # Check for ROS1 artifacts when using ROS2
    if [[ "$ROS_DISTRO" != "noetic" ]]; then
        if [ -d "devel" ] || [ -d ".catkin_tools" ]; then
            print_warning "Found ROS1 build artifacts (devel/ or .catkin_tools/) while using ROS2. Cleaning workspace..."
            needs_cleanup=true
        fi
    fi

    # Check for ROS2 artifacts when using ROS1
    if [[ "$ROS_DISTRO" == "noetic" ]]; then
        if [ -d "install" ] || [ -d "log" ]; then
            print_warning "Found ROS2 build artifacts (install/ or log/) while using ROS1. Cleaning workspace..."
            needs_cleanup=true
        fi
    fi

    if [ "$needs_cleanup" = true ]; then
        clean_workspace
    else
        print_success "No incompatible build artifacts found"
    fi
}

create_symlinks_for_package() {
    local package_dir="$1"
    local package_name=$(basename "$package_dir")

    if [ -d "$package_dir" ]; then
        if [ -f "$package_dir/package.ros1.xml" ] && [ -f "$package_dir/package.ros2.xml" ]; then
            [ -e "$package_dir/package.xml" ] && rm -f "$package_dir/package.xml"

            if [[ "$ROS_DISTRO" == "noetic" ]]; then
                ln -s package.ros1.xml "$package_dir/package.xml"
                return 0
            elif [[ "$ROS_DISTRO" == "foxy" || "$ROS_DISTRO" == "humble" ]]; then
                ln -s package.ros2.xml "$package_dir/package.xml"
                return 0
            else
                print_error "Unknown ROS version: $ROS_DISTRO"
                return 1
            fi
        fi
    fi
    return 1
}

create_symlinks_for_all_packages() {
    print_header "[Creating Symlinks for All Packages]"

    created_packages=()
    while IFS= read -r -d '' package_dir; do
        package_dir=$(dirname "$package_dir")
        package_name=$(basename "$package_dir")
        if create_symlinks_for_package "$package_dir"; then
            created_packages+=("$package_name")
        fi
    done < <(find src -name "package.ros1.xml" -print0)

    if [ ${#created_packages[@]} -gt 0 ]; then
        print_success "Created symlinks for: ${created_packages[*]}"
    else
        print_warning "No packages with dual ROS support found"
    fi
}

create_symlinks_for_specific_packages() {
    local packages=("$@")

    print_header "[Creating Symlinks for Specific Packages]"
    print_info "Packages to process: ${packages[*]}"

    created_packages=()
    for package_name in "${packages[@]}"; do
        package_dir=$(find src -name "$package_name" -type d | head -n 1)
        if [ -n "$package_dir" ] && create_symlinks_for_package "$package_dir"; then
            created_packages+=("$package_name")
        fi
    done

    if [ ${#created_packages[@]} -gt 0 ]; then
        print_success "Created symlinks for: ${created_packages[*]}"
    fi
}

# ========================
# Main Script
# ========================

show_usage() {
    print_header "[Build System Usage]"
    echo -e "Usage: $0 [OPTIONS] [PACKAGE_NAMES...]"
    echo ""
    echo -e "${COLOR_INFO}Options:${COLOR_RESET}"
    echo -e "  -c, --clean      Clean workspace (remove symlinks and build artifacts)"
    echo -e "  -m, --cmake      Build using CMake (for hardware deployment and mujoco simulation without ros)"
    echo -e "  -s, --sim SIM    Build with simulator (SIM: gazebo or mujoco)"
    echo -e "  -r, --real NAME  Build real robot package (NAME: all/a1/go2/g1/lite3/l4w4)"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build all ROS packages (default to gazebo simulator)"
    echo -e "  $0 package1 package2  # Build specific ROS packages"
    echo -e "  $0 -c                 # Clean all symlinks and build artifacts"
    echo -e "  $0 --clean package1   # Clean specific package and build artifacts"
    echo -e "  $0 -m                 # Build with CMake (ALL real robots and mujoco simulator)"
    echo -e "  $0 -m -s mujoco       # Build with CMake and mujoco simulator (ALL real robots)"
    echo -e "  $0 -m -s mujoco -r a1 # Build with CMake, mujoco simulator and a1 real robot"
    echo -e "  $0 -s gazebo          # Build with gazebo simulator (with ROS)"
    echo -e "  $0 -s mujoco          # Build with mujoco simulator (with ROS)"
    echo -e "  $0 -r a1              # Build a1 real robot (with ROS)"
    echo -e "  $0 -r go2             # Build go2/go2w real robot (with ROS)"
    echo -e "  $0 -r g1              # Build g1 real robot (with ROS)"
    echo -e "  $0 -r lite3           # Build lite3 real robot (with ROS)"
    echo -e "  $0 -r l4w4            # Build l4w4 real robot (with ROS)"
    echo -e "  $0 -r all             # Build all real robots (with ROS)"
    echo -e "  $0 -s gazebo -r a1    # Build gazebo simulator and a1 real robot (with ROS)"
    echo -e "  $0 -s mujoco -r go2   # Build mujoco simulator and go2 real robot (with ROS)"
    echo -e "  $0 -m -s gazebo       # Error: CMake only supports mujoco"
}

main() {
    local packages=()
    local clean_mode=false
    local cmake_mode=false
    local sim_mode=false
    local sim_name=""
    local real_mode=false
    local real_name=""

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -m|--cmake) cmake_mode=true; shift ;;
            -s|--sim)
                sim_mode=true
                if [[ -n $2 && $2 != -* ]]; then
                    sim_name=$(echo "$2" | tr '[:lower:]' '[:upper:]')
                    shift 2
                else
                    print_error "Missing SIMULATOR for --sim/-s option"
                    show_usage; exit 1
                fi
                ;;
            -r|--real)
                real_mode=true
                if [[ -n $2 && $2 != -* ]]; then
                    real_name=$(echo "$2" | tr '[:lower:]' '[:upper:]')
                    shift 2
                else
                    print_error "Missing NAME for --real/-r option"
                    show_usage; exit 1
                fi
                ;;
            -h|--help) show_usage; exit 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    # Set default simulator if not specified
    if [[ "$cmake_mode" == true && "$sim_mode" == false ]]; then
        sim_name="GAZEBO"
        sim_mode=true
    fi

    # Validate simulator name
    if [[ "$sim_mode" == true ]]; then
        if [[ "$sim_name" != "GAZEBO" && "$sim_name" != "MUJOCO" ]]; then
            print_error "Invalid simulator: $sim_name. Must be 'gazebo' or 'mujoco'"
            show_usage; exit 1
        fi
    fi

    # Check for invalid combinations
    if [[ "$cmake_mode" == true && "$sim_name" == "GAZEBO" ]]; then
        print_error "Cannot combine CMake build (-m) with Gazebo (-s gazebo). CMake only supports MUJOCO."
        show_usage; exit 1
    fi

    # Handle CMake build mode
    if [ "$cmake_mode" = true ]; then
        run_cmake_build "$sim_name" "$real_name"
        exit 0
    fi

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace "${packages[@]}"
        exit 0
    fi

    # Handle ROS build
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS environment not detected. Please source your ROS setup.bash first."
        print_info "For hardware deployment, use the --cmake option instead."
        exit 1
    fi

    # Prepare CMake args based on build mode
    local cmake_args=()

    if [ "$real_mode" = true ]; then
        case "$real_name" in
            ALL) cmake_args+=("-DREAL_ROBOT=ALL") ;;
            A1) cmake_args+=("-DREAL_ROBOT=A1") ;;
            GO2) cmake_args+=("-DREAL_ROBOT=GO2") ;;
            G1) cmake_args+=("-DREAL_ROBOT=G1") ;;
            LITE3) cmake_args+=("-DREAL_ROBOT=LITE3") ;;
            L4W4) cmake_args+=("-DREAL_ROBOT=L4W4") ;;
            *)
                print_error "Unknown real robot name: $real_name"
                show_usage; exit 1
                ;;
        esac
    fi

    if [ "$sim_mode" = true ]; then
        cmake_args+=("-DSIMULATOR=$sim_name")
        if [[ "$sim_name" == "MUJOCO" ]]; then
            packages=("rl_sar" "robot_msgs" "robot_joint_controller")
        fi
    fi

    run_ros_build "${packages[@]}" --cmake-args "${cmake_args[@]}"
}

main "$@"