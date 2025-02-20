cmake_minimum_required(VERSION 3.29)
project(UnsentencedKalman)

set(CMAKE_CXX_STANDARD 20)

set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)


find_package(Eigen3 CONFIG REQUIRED)

find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")


include(FetchContent)
set(SERIAL_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        serial
        GIT_REPOSITORY https://github.com/markaren/serial.git
        GIT_TAG fd4ef255f693679f0d589df2145176b7c6eccb1c
)
FetchContent_MakeAvailable(serial)


add_executable(UnsentencedKalman
        main.cpp
        src/maglevSystemDynamics.cpp
        include/maglevObject.hpp)


target_link_libraries(UnsentencedKalman PRIVATE serial Eigen3::Eigen)
target_include_directories(UnsentencedKalman PUBLIC ${INCLUDE_DIR} ${RAPIDCSV_INCLUDE_DIRS})