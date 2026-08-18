cmake_minimum_required(VERSION 3.20)
project(sdv_diag_gateway LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(vsomeip3)
find_package(Threads REQUIRED)
enable_testing()

# GoogleTest (system gtest-devel 1.15.2)
find_package(GTest REQUIRED)

# Ported C UDS stack (from obd-simulator, adapted). Platform shim included.
add_library(diag_stack STATIC
    src/diag/uds_service.c
    src/diag/diag_session.c
    src/diag/plat_linux.c
    src/diag/ota_stub.c)
target_include_directories(diag_stack PUBLIC src/diag)
set_target_properties(diag_stack PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)

if(vsomeip3_FOUND)
add_executable(diag_service src/diag_service.cpp src/diag_bridge.cpp)
target_link_libraries(diag_service PRIVATE diag_stack vsomeip3 Threads::Threads)

add_executable(diag_client src/diag_client.cpp)
target_link_libraries(diag_client PRIVATE vsomeip3 Threads::Threads)
else()
message(STATUS "vsomeip3 not found — building tests only")
endif()

# --- Test suite (links diag_stack + diag_bridge, NO vsomeip) ---
add_executable(diag_tests
    tests/test_uds_dispatcher.cpp
    src/diag_bridge.cpp)
target_link_libraries(diag_tests PRIVATE
    diag_stack
    GTest::gtest_main
    Threads::Threads)
target_include_directories(diag_tests PRIVATE src)
include(GoogleTest)
gtest_discover_tests(diag_tests)
