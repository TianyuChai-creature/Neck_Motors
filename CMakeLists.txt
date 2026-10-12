cmake_minimum_required(VERSION 3.16)
project(f32c_sdk VERSION 0.2.2 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(F32C_BUILD_ZED "Build the legacy camera example outside the installed motor SDK" OFF)

add_library(f32c SHARED
    src/serial_port.cpp
    src/motor.cpp
    src/gimbal.cpp
    src/c_api.cpp
)
target_include_directories(f32c
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(test_protocol tests/test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE f32c)

add_executable(test_serial_log tests/test_serial_log.cpp)
target_include_directories(test_serial_log PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_serial_log PRIVATE f32c util)

add_executable(test_sdk_api tests/test_sdk_api.cpp)
target_link_libraries(test_sdk_api PRIVATE f32c)

enable_testing()
add_test(NAME protocol COMMAND test_protocol)
add_test(NAME serial_log COMMAND test_serial_log)
add_test(NAME sdk_api COMMAND test_sdk_api)

add_executable(f32c_ping examples/ping.cpp)
target_link_libraries(f32c_ping PRIVATE f32c)

add_executable(f32c_hold examples/hold.cpp)
target_link_libraries(f32c_hold PRIVATE f32c)

add_executable(f32c_zero examples/zero.cpp)
target_link_libraries(f32c_zero PRIVATE f32c)

if(F32C_BUILD_ZED)
    set(LINK_SHARED_ZED ON)
    find_package(ZED REQUIRED)

    add_library(f32c_zed SHARED src/zed_mini.cpp src/zed_c_api.cpp)
    target_include_directories(f32c_zed
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
               $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS}
    )
    target_link_directories(f32c_zed PRIVATE ${ZED_LIBRARY_DIR} ${CUDA_LIBRARY_DIRS})
    target_link_libraries(f32c_zed PRIVATE
        ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY}
    )

    add_executable(f32c_zed_capture examples/zed_capture.cpp)
    target_link_libraries(f32c_zed_capture PRIVATE f32c_zed)
endif()

install(TARGETS f32c EXPORT f32cTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/f32c/gimbal.hpp include/f32c/c_api.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/f32c)
install(FILES docs/SDK使用说明.md DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/f32c)
install(EXPORT f32cTargets NAMESPACE f32c:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/f32c)
configure_package_config_file(cmake/f32cConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/f32cConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/f32c)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/f32cConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMinorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/f32cConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/f32cConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/f32c)
