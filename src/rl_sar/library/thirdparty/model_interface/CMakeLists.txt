# Model Interface CMake Configuration
# This file handles automatic detection and setup of ML inference backends (ONNX Runtime and LibTorch)

cmake_minimum_required(VERSION 3.5)

# ========================
# Platform and Architecture Detection
# ========================

# Detect platform and architecture for both ONNX and LibTorch
set(IS_JETSON FALSE)  # Initialize IS_JETSON for all platforms

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
        set(PLATFORM_NAME "linux")
        set(ARCH_NAME "x64")
        set(TORCH_ARCH "x86_64")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(PLATFORM_NAME "linux")
        set(ARCH_NAME "aarch64")
        set(TORCH_ARCH "aarch64")
        set(LIB_EXT "so")

        # Check if this is a Jetson device
        set(IS_JETSON FALSE)
        if(EXISTS "/etc/nv_tegra_release" OR EXISTS "/sys/firmware/devicetree/base/model")
            if(EXISTS "/sys/firmware/devicetree/base/model")
                file(READ "/sys/firmware/devicetree/base/model" DEVICE_MODEL)
                if(DEVICE_MODEL MATCHES "NVIDIA Jetson")
                    set(IS_JETSON TRUE)
                endif()
            endif()
            if(EXISTS "/etc/nv_tegra_release")
                set(IS_JETSON TRUE)
            endif()
        endif()
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
        set(PLATFORM_NAME "win")
        set(ARCH_NAME "x64")
        set(TORCH_ARCH "x86_64")
        set(LIB_EXT "dll")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "ARM64")
        set(PLATFORM_NAME "win")
        set(ARCH_NAME "arm64")
        set(TORCH_ARCH "arm64")
        set(LIB_EXT "dll")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
        set(PLATFORM_NAME "osx")
        set(ARCH_NAME "x86_64")
        set(TORCH_ARCH "x86_64")
        set(LIB_EXT "dylib")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|ARM64")
        set(PLATFORM_NAME "osx")
        set(ARCH_NAME "arm64")
        set(TORCH_ARCH "arm64")
        set(LIB_EXT "dylib")
    endif()
endif()

# ========================
# ONNX Runtime Setup
# ========================

set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")

# Function to download and setup ONNX Runtime prebuilt package
function(setup_onnx_runtime_prebuilt version platform arch)
    set(ONNX_PREBUILT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime")

    if(platform STREQUAL "linux")
        set(ONNX_URL "https://github.com/microsoft/onnxruntime/releases/download/v${version}/onnxruntime-${platform}-${arch}-${version}.tgz")
        set(ONNX_ARCHIVE_NAME "onnxruntime-${platform}-${arch}-${version}.tgz")
        set(ONNX_EXTRACT_CMD ${CMAKE_COMMAND} -E tar xzf)
    elseif(platform STREQUAL "win")
        set(ONNX_URL "https://github.com/microsoft/onnxruntime/releases/download/v${version}/onnxruntime-${platform}-${arch}-${version}.zip")
        set(ONNX_ARCHIVE_NAME "onnxruntime-${platform}-${arch}-${version}.zip")
        set(ONNX_EXTRACT_CMD ${CMAKE_COMMAND} -E tar xf)
    elseif(platform STREQUAL "osx")
        set(ONNX_URL "https://github.com/microsoft/onnxruntime/releases/download/v${version}/onnxruntime-${platform}-${arch}-${version}.tgz")
        set(ONNX_ARCHIVE_NAME "onnxruntime-${platform}-${arch}-${version}.tgz")
        set(ONNX_EXTRACT_CMD ${CMAKE_COMMAND} -E tar xzf)
    endif()

    set(ONNX_ARCHIVE_PATH "${ONNX_PREBUILT_DIR}/${ONNX_ARCHIVE_NAME}")
    set(ONNX_TEMP_EXTRACT_DIR "${ONNX_PREBUILT_DIR}/temp_extract")

    # Check if prebuilt package is already set up (has include and lib directories)
    if(NOT EXISTS "${ONNX_PREBUILT_DIR}/include" OR NOT EXISTS "${ONNX_PREBUILT_DIR}/lib")
        message(STATUS "Setting up ONNX Runtime prebuilt package v${version} for ${platform}-${arch}...")

        # Clean up any existing content
        if(EXISTS "${ONNX_PREBUILT_DIR}")
            file(REMOVE_RECURSE "${ONNX_PREBUILT_DIR}")
        endif()
        file(MAKE_DIRECTORY "${ONNX_PREBUILT_DIR}")
        file(MAKE_DIRECTORY "${ONNX_TEMP_EXTRACT_DIR}")

        # Download the prebuilt package
        message(STATUS "Downloading from: ${ONNX_URL}")
        file(DOWNLOAD "${ONNX_URL}" "${ONNX_ARCHIVE_PATH}"
             SHOW_PROGRESS
             STATUS download_status
             TIMEOUT 300)

        list(GET download_status 0 status_code)
        if(NOT status_code EQUAL 0)
            list(GET download_status 1 error_message)
            message(WARNING "Failed to download ONNX Runtime prebuilt package: ${error_message}")
            return()
        endif()

        # Extract the package to temp directory
        execute_process(
            COMMAND ${ONNX_EXTRACT_CMD} "${ONNX_ARCHIVE_PATH}"
            WORKING_DIRECTORY "${ONNX_TEMP_EXTRACT_DIR}"
            RESULT_VARIABLE extract_result
        )

        if(NOT extract_result EQUAL 0)
            message(WARNING "Failed to extract ONNX Runtime prebuilt package")
            return()
        endif()

        # Move contents from extracted directory to target directory
        file(GLOB EXTRACTED_DIRS "${ONNX_TEMP_EXTRACT_DIR}/*")
        foreach(extracted_dir ${EXTRACTED_DIRS})
            if(IS_DIRECTORY "${extracted_dir}")
                # Copy contents of the extracted directory to the target directory
                file(GLOB_RECURSE ALL_FILES "${extracted_dir}/*")
                foreach(file_path ${ALL_FILES})
                    file(RELATIVE_PATH rel_path "${extracted_dir}" "${file_path}")
                    get_filename_component(dest_dir "${ONNX_PREBUILT_DIR}/${rel_path}" DIRECTORY)
                    file(MAKE_DIRECTORY "${dest_dir}")
                    file(COPY "${file_path}" DESTINATION "${dest_dir}")
                endforeach()
                break()
            endif()
        endforeach()

        # Clean up temp files
        file(REMOVE "${ONNX_ARCHIVE_PATH}")
        file(REMOVE_RECURSE "${ONNX_TEMP_EXTRACT_DIR}")
        message(STATUS "ONNX Runtime prebuilt package setup completed")
    else()
        message(STATUS "Using existing ONNX Runtime prebuilt package")
    endif()

    # Set global variables for use outside function
    set(ONNX_PREBUILT_ROOT "${ONNX_PREBUILT_DIR}" PARENT_SCOPE)
endfunction()

# Try to use prebuilt package first (if platform is supported)
if(PLATFORM_NAME AND ARCH_NAME)
    message(STATUS "Detected platform: ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR} -> ${PLATFORM_NAME}-${ARCH_NAME}")

    # Try different versions in order of preference (newer versions first)
    set(ONNX_VERSIONS "1.16.3" "1.15.1" "1.14.1" "1.13.1")

    foreach(version ${ONNX_VERSIONS})
        setup_onnx_runtime_prebuilt(${version} ${PLATFORM_NAME} ${ARCH_NAME})
        if(ONNX_PREBUILT_ROOT AND EXISTS "${ONNX_PREBUILT_ROOT}")
            set(USE_ONNX "ON")
            add_compile_definitions(USE_ONNX)

            # Set include directories
            include_directories("${ONNX_PREBUILT_ROOT}/include")

            # Find the library file
            if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
                set(LIB_NAMES onnxruntime.lib)
                set(LIB_SEARCH_PATH "${ONNX_PREBUILT_ROOT}/lib")
            else()
                set(LIB_NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT})
                set(LIB_SEARCH_PATH "${ONNX_PREBUILT_ROOT}/lib")
            endif()

            find_library(ONNX_RUNTIME_LIB_PATH
                NAMES ${LIB_NAMES}
                PATHS "${LIB_SEARCH_PATH}"
                NO_DEFAULT_PATH
            )

            if(ONNX_RUNTIME_LIB_PATH)
                # Create imported target
                if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
                    add_library(onnxruntime_prebuilt STATIC IMPORTED GLOBAL)
                else()
                    add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
                endif()

                set_target_properties(onnxruntime_prebuilt PROPERTIES
                    IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
                    INTERFACE_INCLUDE_DIRECTORIES "${ONNX_PREBUILT_ROOT}/include"
                )

                set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
                message(STATUS "ONNX Runtime prebuilt v${version} configured successfully")
                message(STATUS "Library: ${ONNX_RUNTIME_LIB_PATH}")

                # Store version for later display in parent scope
                set(ONNX_RUNTIME_VERSION ${version} CACHE INTERNAL "ONNX Runtime version")
                break()
            else()
                message(WARNING "ONNX Runtime library not found in prebuilt package v${version}")
            endif()
        endif()
    endforeach()
else()
    message(STATUS "Unsupported platform for ONNX Runtime prebuilt packages: ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT USE_ONNX)
    message(WARNING "ONNX Runtime not available - prebuilt package not found")
    message(STATUS "To enable ONNX Runtime support:")
    message(STATUS "  1. Ensure internet connection for automatic prebuilt download")
    message(STATUS "  2. Check if your platform (${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}) is supported")
endif()

# ========================
# LibTorch Setup
# ========================

set(USE_TORCH "OFF")
set(TORCH_LIBRARIES "")

# Function to detect Jetson JetPack version
function(detect_jetpack_version)
    set(JETPACK_VERSION "" PARENT_SCOPE)

    if(IS_JETSON)
        # Try to detect JetPack version from various sources
        if(EXISTS "/etc/nv_tegra_release")
            file(READ "/etc/nv_tegra_release" TEGRA_RELEASE)
            if(TEGRA_RELEASE MATCHES "R35")
                set(JETPACK_VERSION "5.1.2" PARENT_SCOPE)
            elseif(TEGRA_RELEASE MATCHES "R32")
                set(JETPACK_VERSION "4.6.1" PARENT_SCOPE)
            endif()
        endif()

        # Alternative: try to get version from dpkg if available
        find_program(DPKG_PROGRAM dpkg)
        if(DPKG_PROGRAM)
            execute_process(
                COMMAND ${DPKG_PROGRAM} -l nvidia-jetpack
                OUTPUT_VARIABLE JETPACK_INFO
                ERROR_QUIET
            )
            if(JETPACK_INFO MATCHES "5\\.")
                set(JETPACK_VERSION "5.1.2" PARENT_SCOPE)
            elseif(JETPACK_INFO MATCHES "4\\.")
                set(JETPACK_VERSION "4.6.1" PARENT_SCOPE)
            endif()
        endif()
    endif()
endfunction()

# Function to download and setup LibTorch prebuilt package
function(setup_libtorch_prebuilt platform arch is_jetson)
    set(TORCH_PREBUILT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libtorch")

    if(is_jetson)
        # For Jetson devices, try to automatically create LibTorch from existing PyTorch installation
        message(STATUS "========================================")
        message(STATUS "        JETSON DEVICE DETECTED         ")
        message(STATUS "========================================")

        # Try to find existing PyTorch installation
        find_program(PYTHON_EXECUTABLE python3)
        set(TORCH_FROM_PYTHON FALSE)

        if(PYTHON_EXECUTABLE)
            # Check if PyTorch is installed
            execute_process(
                COMMAND ${PYTHON_EXECUTABLE} -c "import torch; print(torch.__file__)"
                OUTPUT_VARIABLE TORCH_PATH
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE
                RESULT_VARIABLE TORCH_CHECK_RESULT
            )

            if(TORCH_CHECK_RESULT EQUAL 0 AND TORCH_PATH)
                get_filename_component(TORCH_INSTALL_DIR ${TORCH_PATH} DIRECTORY)
                message(STATUS "Found existing PyTorch installation at: ${TORCH_INSTALL_DIR}")

                # Check if required directories exist
                if(EXISTS "${TORCH_INSTALL_DIR}/include" AND EXISTS "${TORCH_INSTALL_DIR}/lib" AND EXISTS "${TORCH_INSTALL_DIR}/share")
                    message(STATUS "Creating LibTorch from PyTorch installation...")

                    # Create LibTorch directory structure
                    file(MAKE_DIRECTORY "${TORCH_PREBUILT_DIR}")
                    file(MAKE_DIRECTORY "${TORCH_PREBUILT_DIR}/include")
                    file(MAKE_DIRECTORY "${TORCH_PREBUILT_DIR}/lib")
                    file(MAKE_DIRECTORY "${TORCH_PREBUILT_DIR}/share")

                    # Copy necessary files
                    execute_process(
                        COMMAND ${CMAKE_COMMAND} -E copy_directory "${TORCH_INSTALL_DIR}/include" "${TORCH_PREBUILT_DIR}/include"
                        RESULT_VARIABLE COPY_INCLUDE_RESULT
                    )
                    execute_process(
                        COMMAND ${CMAKE_COMMAND} -E copy_directory "${TORCH_INSTALL_DIR}/lib" "${TORCH_PREBUILT_DIR}/lib"
                        RESULT_VARIABLE COPY_LIB_RESULT
                    )
                    execute_process(
                        COMMAND ${CMAKE_COMMAND} -E copy_directory "${TORCH_INSTALL_DIR}/share" "${TORCH_PREBUILT_DIR}/share"
                        RESULT_VARIABLE COPY_SHARE_RESULT
                    )

                    # Copy bin directory if it exists
                    if(EXISTS "${TORCH_INSTALL_DIR}/bin")
                        file(MAKE_DIRECTORY "${TORCH_PREBUILT_DIR}/bin")
                        execute_process(
                            COMMAND ${CMAKE_COMMAND} -E copy_directory "${TORCH_INSTALL_DIR}/bin" "${TORCH_PREBUILT_DIR}/bin"
                            RESULT_VARIABLE COPY_BIN_RESULT
                        )
                    endif()

                    if(COPY_INCLUDE_RESULT EQUAL 0 AND COPY_LIB_RESULT EQUAL 0 AND COPY_SHARE_RESULT EQUAL 0)
                        message(STATUS "Successfully created LibTorch from PyTorch installation")
                        set(TORCH_FROM_PYTHON TRUE)

                        # Get PyTorch version
                        execute_process(
                            COMMAND ${PYTHON_EXECUTABLE} -c "import torch; print(torch.__version__)"
                            OUTPUT_VARIABLE PYTORCH_VERSION
                            ERROR_QUIET
                            OUTPUT_STRIP_TRAILING_WHITESPACE
                        )
                        if(PYTORCH_VERSION)
                            message(STATUS "PyTorch version: ${PYTORCH_VERSION}")
                        endif()
                    else()
                        message(WARNING "Failed to copy PyTorch files to LibTorch directory")
                    endif()
                else()
                    message(WARNING "PyTorch installation incomplete - missing required directories")
                endif()
            endif()
        endif()

        if(NOT TORCH_FROM_PYTHON)
            # Provide installation instructions if automatic setup failed
            message(STATUS "LibTorch automatic setup failed. Manual installation required:")
            message(STATUS "")
            message(STATUS "1. Check JetPack version:")
            message(STATUS "   sudo pip install jetson-stats")
            message(STATUS "   sudo jtop")
            message(STATUS "")
            message(STATUS "2. Download PyTorch for your JetPack version:")
            message(STATUS "   # For JetPack 5.1.2:")
            message(STATUS "   wget https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl")
            message(STATUS "   sudo apt install python-is-python3 python3.9-dev")
            message(STATUS "   pip install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl")
            message(STATUS "")
            message(STATUS "3. Re-run cmake after installing PyTorch")
            message(STATUS "")
        endif()

        message(STATUS "========================================")

        # Set global variables for use outside function
        if(TORCH_FROM_PYTHON)
            set(TORCH_PREBUILT_ROOT "${TORCH_PREBUILT_DIR}" PARENT_SCOPE)
        endif()

        return()
    endif()

    # For non-Jetson devices, proceed with automatic download
    if(platform STREQUAL "linux")
        if(arch STREQUAL "x86_64")
            set(TORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcpu.zip")
            set(TORCH_ARCHIVE_NAME "libtorch-cxx11-abi-shared-with-deps-2.1.0+cpu.zip")
        else()
            # For aarch64 non-Jetson (like Raspberry Pi), use CPU version
            set(TORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcpu.zip")
            set(TORCH_ARCHIVE_NAME "libtorch-cxx11-abi-shared-with-deps-2.1.0+cpu.zip")
        endif()
        set(TORCH_EXTRACT_CMD ${CMAKE_COMMAND} -E tar xf)
    elseif(platform STREQUAL "win")
        set(TORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-win-shared-with-deps-2.1.0%2Bcpu.zip")
        set(TORCH_ARCHIVE_NAME "libtorch-win-shared-with-deps-2.1.0+cpu.zip")
        set(TORCH_EXTRACT_CMD ${CMAKE_COMMAND} -E tar xf)
    elseif(platform STREQUAL "osx")
        if(arch STREQUAL "x86_64")
            set(TORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-macos-x86_64-2.8.0.zip")
            set(TORCH_ARCHIVE_NAME "libtorch-macos-x86_64-2.8.0.zip")
        else()
            # For Apple Silicon (ARM64)
            set(TORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-macos-arm64-2.8.0.zip")
            set(TORCH_ARCHIVE_NAME "libtorch-macos-arm64-2.8.0.zip")
        endif()
        set(TORCH_EXTRACT_CMD ${CMAKE_COMMAND} -E tar xf)
    endif()

    set(TORCH_ARCHIVE_PATH "${TORCH_PREBUILT_DIR}/${TORCH_ARCHIVE_NAME}")
    set(TORCH_TEMP_EXTRACT_DIR "${TORCH_PREBUILT_DIR}/temp_extract")

    # Check if prebuilt package is already set up
    if(NOT EXISTS "${TORCH_PREBUILT_DIR}/include" OR NOT EXISTS "${TORCH_PREBUILT_DIR}/lib")
        message(STATUS "Setting up LibTorch prebuilt package for ${platform}-${arch}...")

        # Clean up any existing content
        if(EXISTS "${TORCH_PREBUILT_DIR}")
            file(REMOVE_RECURSE "${TORCH_PREBUILT_DIR}")
        endif()
        file(MAKE_DIRECTORY "${TORCH_PREBUILT_DIR}")
        file(MAKE_DIRECTORY "${TORCH_TEMP_EXTRACT_DIR}")

        # Download the prebuilt package
        message(STATUS "Downloading from: ${TORCH_URL}")
        file(DOWNLOAD "${TORCH_URL}" "${TORCH_ARCHIVE_PATH}"
             SHOW_PROGRESS
             STATUS download_status
             TIMEOUT 600)

        list(GET download_status 0 status_code)
        if(NOT status_code EQUAL 0)
            list(GET download_status 1 error_message)
            message(WARNING "Failed to download LibTorch prebuilt package: ${error_message}")
            return()
        endif()

        # Extract the package to temp directory
        execute_process(
            COMMAND ${TORCH_EXTRACT_CMD} "${TORCH_ARCHIVE_PATH}"
            WORKING_DIRECTORY "${TORCH_TEMP_EXTRACT_DIR}"
            RESULT_VARIABLE extract_result
        )

        if(NOT extract_result EQUAL 0)
            message(WARNING "Failed to extract LibTorch prebuilt package")
            return()
        endif()

        # Move contents from extracted directory to target directory
        file(GLOB EXTRACTED_DIRS "${TORCH_TEMP_EXTRACT_DIR}/*")
        foreach(extracted_dir ${EXTRACTED_DIRS})
            if(IS_DIRECTORY "${extracted_dir}")
                # Look for libtorch directory structure
                if(EXISTS "${extracted_dir}/include" AND EXISTS "${extracted_dir}/lib")
                    # Copy contents of the libtorch directory to the target directory
                    file(GLOB_RECURSE ALL_FILES "${extracted_dir}/*")
                    foreach(file_path ${ALL_FILES})
                        file(RELATIVE_PATH rel_path "${extracted_dir}" "${file_path}")
                        get_filename_component(dest_dir "${TORCH_PREBUILT_DIR}/${rel_path}" DIRECTORY)
                        file(MAKE_DIRECTORY "${dest_dir}")
                        file(COPY "${file_path}" DESTINATION "${dest_dir}")
                    endforeach()
                    break()
                endif()
            endif()
        endforeach()

        # Clean up temp files
        file(REMOVE "${TORCH_ARCHIVE_PATH}")
        file(REMOVE_RECURSE "${TORCH_TEMP_EXTRACT_DIR}")
        message(STATUS "LibTorch prebuilt package setup completed")
    else()
        message(STATUS "Using existing LibTorch prebuilt package")
    endif()

    # Set global variables for use outside function
    set(TORCH_PREBUILT_ROOT "${TORCH_PREBUILT_DIR}" PARENT_SCOPE)
endfunction()

# LibTorch - Automatic detection and setup for all platforms
if(PLATFORM_NAME AND TORCH_ARCH)
    if(IS_JETSON)
        detect_jetpack_version()
        message(STATUS "Jetson device detected")
        if(JETPACK_VERSION)
            message(STATUS "Detected JetPack version: ${JETPACK_VERSION}")
        endif()
    endif()

    setup_libtorch_prebuilt(${PLATFORM_NAME} ${TORCH_ARCH} ${IS_JETSON})

    if(TORCH_PREBUILT_ROOT AND EXISTS "${TORCH_PREBUILT_ROOT}")
        set(USE_TORCH "ON")
        add_compile_definitions(USE_TORCH)

        # Set up LibTorch for CMake
        set(CMAKE_PREFIX_PATH "${TORCH_PREBUILT_ROOT};${CMAKE_PREFIX_PATH}")

        # Try to find Torch with the prebuilt package
        find_package(Torch QUIET)
        if(Torch_FOUND)
            if(IS_JETSON)
                message(STATUS "LibTorch created from PyTorch installation successfully")
            else()
                message(STATUS "LibTorch prebuilt package configured successfully")
            endif()
            message(STATUS "Torch version: ${Torch_VERSION}")
            message(STATUS "Torch libraries: ${TORCH_LIBRARIES}")
        else()
            set(USE_TORCH "OFF")
            message(WARNING "LibTorch package found but find_package(Torch) failed")
        endif()
    endif()
else()
    message(STATUS "Unsupported platform for LibTorch prebuilt packages: ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT USE_TORCH AND NOT IS_JETSON)
    message(WARNING "LibTorch not available - prebuilt package not found")
    message(STATUS "To enable LibTorch support:")
    message(STATUS "  1. Ensure internet connection for automatic prebuilt download")
    message(STATUS "  2. Check if your platform (${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}) is supported")
    message(STATUS "  3. Or manually install LibTorch and set Torch_DIR environment variable")
endif()

# ========================
# Export Variables to Parent Scope
# ========================

# Export ONNX variables
set(USE_ONNX ${USE_ONNX} PARENT_SCOPE)
set(ONNX_RUNTIME_LIB ${ONNX_RUNTIME_LIB} PARENT_SCOPE)
set(ONNX_RUNTIME_VERSION ${ONNX_RUNTIME_VERSION} PARENT_SCOPE)

# Export LibTorch variables
set(USE_TORCH ${USE_TORCH} PARENT_SCOPE)
set(TORCH_LIBRARIES ${TORCH_LIBRARIES} PARENT_SCOPE)
set(IS_JETSON ${IS_JETSON} PARENT_SCOPE)
set(JETPACK_VERSION ${JETPACK_VERSION} PARENT_SCOPE)

# Export CMAKE_PREFIX_PATH to parent scope so LibTorch can be found
if(TORCH_PREBUILT_ROOT)
    set(CMAKE_PREFIX_PATH "${TORCH_PREBUILT_ROOT};${CMAKE_PREFIX_PATH}" PARENT_SCOPE)
endif()

# ========================
# Global RPATH Configuration
# ========================

# Set global RPATH settings for prebuilt libraries in parent scope
if(USE_TORCH OR USE_ONNX)
    # Use RPATH instead of RUNPATH and include our prebuilt lib directories
    set(CMAKE_SKIP_BUILD_RPATH FALSE PARENT_SCOPE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE PARENT_SCOPE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE PARENT_SCOPE)

    # Add our prebuilt library paths to RPATH
    set(PREBUILT_LIB_PATHS "")
    if(USE_TORCH)
        list(APPEND PREBUILT_LIB_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/libtorch/lib")
    endif()
    if(USE_ONNX)
        list(APPEND PREBUILT_LIB_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/lib")
    endif()
    set(CMAKE_INSTALL_RPATH "${PREBUILT_LIB_PATHS}" PARENT_SCOPE)

    # Force use of RPATH instead of RUNPATH (Linux only)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags" PARENT_SCOPE)
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags" PARENT_SCOPE)
    endif()

    message(STATUS "Configured RPATH for prebuilt libraries:")
    foreach(path ${PREBUILT_LIB_PATHS})
        message(STATUS "  - ${path}")
    endforeach()
endif()
