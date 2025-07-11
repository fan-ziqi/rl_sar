cmake_minimum_required(VERSION 3.10)
project(gamepad)
# SET(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
# SET(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLE "Example for gamepad sdk in C++" ON)

# Add your source files
file(GLOB SOURCES  "src/*.cpp")
include_directories(
  ./include
)

add_library(gamepad_receiver STATIC ${SOURCES})
target_link_libraries(gamepad_receiver PRIVATE -lpthread -lm -lrt -ldl -lstdc++)

if(BUILD_EXAMPLE)
  add_subdirectory(example)
endif()
