cmake_minimum_required(VERSION 3.16)

project(issac_modbus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ISSAC_MODBUS_ENABLE_CLANG_TIDY "启用 clang-tidy" OFF)
option(ISSAC_MODBUS_BUILD_EXAMPLES "构建 Linux 主站/从站示例" ON)

find_package(Threads REQUIRED)
include(CTest)

set(ISSAC_MODBUS_ASIO_INCLUDE_DIR "" CACHE PATH
    "Standalone Asio include directory")
if(NOT ISSAC_MODBUS_ASIO_INCLUDE_DIR)
    find_path(ISSAC_MODBUS_ASIO_INCLUDE_DIR asio.hpp
        PATH_SUFFIXES asio/include)
endif()
if(NOT ISSAC_MODBUS_ASIO_INCLUDE_DIR)
    message(FATAL_ERROR
        "Standalone Asio was not found. Set ISSAC_MODBUS_ASIO_INCLUDE_DIR.")
endif()

add_library(issac_modbus
    src/util/modbusError.cpp
    src/util/modbusMapping.cpp
    src/util/modbusException.cpp
    src/util/modbusRequest.cpp
    src/util/modbusResponse.cpp
    src/modbusRtu/modbusSerial.cpp
    src/modbusRtu/detail/rtuCodec.cpp
    src/modbusRtu/detail/serialTransport.cpp
    src/modbusRtu/rtuMaster.cpp
    src/modbusRtu/rtuSlave.cpp
    src/modbusTcp/modbusTcpConnection.cpp
    src/modbusTcp/modbusTcpServer.cpp)

add_library(issac_modbus::issac_modbus ALIAS issac_modbus)

target_compile_features(issac_modbus PUBLIC cxx_std_17)
target_compile_definitions(issac_modbus PUBLIC ASIO_STANDALONE)
target_include_directories(issac_modbus
    PUBLIC
        $<BUILD_INTERFACE:${ISSAC_MODBUS_ASIO_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(issac_modbus PUBLIC Threads::Threads)

if(ISSAC_MODBUS_BUILD_EXAMPLES)
    foreach(example_name IN ITEMS tcp_slave tcp_master rtu_slave rtu_master)
        add_executable(issac_modbus_${example_name} examples/${example_name}.cpp)
        target_link_libraries(issac_modbus_${example_name} PRIVATE issac_modbus)
        target_include_directories(issac_modbus_${example_name}
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
    endforeach()
endif()

if(BUILD_TESTING)
    find_library(ISSAC_MODBUS_UTIL_LIBRARY util REQUIRED)
    add_executable(issac_modbus_rtu_tests tests/rtu_roles.cpp)
    target_link_libraries(issac_modbus_rtu_tests
        PRIVATE issac_modbus ${ISSAC_MODBUS_UTIL_LIBRARY})
    add_test(NAME issac_modbus_rtu_roles COMMAND issac_modbus_rtu_tests)
    set_tests_properties(issac_modbus_rtu_roles PROPERTIES TIMEOUT 30)
endif()

if(ISSAC_MODBUS_ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set_target_properties(issac_modbus PROPERTIES
        CXX_CLANG_TIDY "${CLANG_TIDY_EXECUTABLE}")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS issac_modbus
    EXPORT issac_modbusTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT issac_modbusTargets
    FILE issac_modbusTargets.cmake
    NAMESPACE issac_modbus::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/issac_modbus)

configure_package_config_file(
    cmake/issac_modbusConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/issac_modbusConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/issac_modbus)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/issac_modbusConfigVersion.cmake
    COMPATIBILITY SameMajorVersion)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/issac_modbusConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/issac_modbusConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/issac_modbus)

install(FILES
    LICENSE
    README.md
    DESTINATION ${CMAKE_INSTALL_DOCDIR})
