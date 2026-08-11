cmake_minimum_required(VERSION 3.20)
project(sdv_diag_gateway LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(vsomeip3 REQUIRED)
find_package(Threads REQUIRED)

add_executable(diag_service src/diag_service.cpp)
target_link_libraries(diag_service PRIVATE vsomeip3 Threads::Threads)

add_executable(diag_client src/diag_client.cpp)
target_link_libraries(diag_client PRIVATE vsomeip3 Threads::Threads)
