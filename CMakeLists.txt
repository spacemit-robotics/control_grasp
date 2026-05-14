cmake_minimum_required(VERSION 3.10)
project(grasp LANGUAGES C CXX)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find pthreads
find_package(Threads REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Core source files
set(GRASP_SOURCES
    src/grasp.c
    src/drivers/drv_dummy.c
)

# Driver source files
set(GRASP_DRIVER_SOURCES "")

set(GRASP_MOTOR_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../peripherals/motor)

# Try to locate motor headers for SO101 gripper driver
find_path(MOTOR_INCLUDE_DIR motor.h
    HINTS
        ${GRASP_MOTOR_SOURCE_DIR}/include
        ${MOTOR_INCLUDE_PATH}
)

if(MOTOR_INCLUDE_DIR)
    message(STATUS "[grasp] Found motor include at ${MOTOR_INCLUDE_DIR}")
    list(APPEND GRASP_DRIVER_SOURCES src/drivers/drv_uart_so101_gripper.c)
    # Build motor as subdirectory early so TARGET motor is available for grasp linking
    if(NOT TARGET motor)
        set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech"
            CACHE STRING "Enabled motor drivers (semicolon-separated). Empty means auto-detect." FORCE)
        set(BUILD_TESTS OFF CACHE BOOL "Build test programs" FORCE)
        add_subdirectory(${GRASP_MOTOR_SOURCE_DIR}
                         ${CMAKE_CURRENT_BINARY_DIR}/motor)
    endif()
else()
    message(STATUS "[grasp] motor headers not found; SO101 gripper driver will be disabled")
endif()

# All source files
set(GRASP_ALL_SOURCES
    ${GRASP_SOURCES}
    ${GRASP_DRIVER_SOURCES}
)

# Create shared library for Python ctypes binding
add_library(grasp SHARED ${GRASP_ALL_SOURCES})
set_target_properties(grasp PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(grasp PRIVATE Threads::Threads m)

target_include_directories(grasp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(MOTOR_INCLUDE_DIR)
    target_compile_definitions(grasp PRIVATE HAVE_MOTOR)
    target_include_directories(grasp PRIVATE ${MOTOR_INCLUDE_DIR})

    # STS3215 register header is in manipulator/include
    find_path(MANIP_INCLUDE_DIR sts3215_regs.h
        HINTS
            ${CMAKE_CURRENT_SOURCE_DIR}/../manipulator/include
    )
    if(MANIP_INCLUDE_DIR)
        target_include_directories(grasp PRIVATE ${MANIP_INCLUDE_DIR})
    endif()

    # Link motor library
    # Try to find motor target first (if built as part of larger project)
    if(TARGET motor)
        # --whole-archive preserves motor driver auto-registration constructors
        target_link_libraries(grasp PUBLIC -Wl,--whole-archive motor -Wl,--no-whole-archive)
        set_target_properties(grasp PROPERTIES LINKER_LANGUAGE CXX)
    else()
        # Otherwise, try to find prebuilt motor library
        find_library(MOTOR_LIB motor
            HINTS
                ${CMAKE_INSTALL_PREFIX}/lib
                ${GRASP_MOTOR_SOURCE_DIR}/build
                ${MOTOR_LIB_PATH}
        )
        if(MOTOR_LIB)
            message(STATUS "[grasp] Found motor library at ${MOTOR_LIB}")
            # --whole-archive preserves motor driver auto-registration constructors
            target_link_libraries(grasp PUBLIC -Wl,--whole-archive ${MOTOR_LIB} -Wl,--no-whole-archive)
            set_target_properties(grasp PROPERTIES LINKER_LANGUAGE CXX)
        else()
            message(WARNING "[grasp] motor library not found; SO101 gripper driver may not work")
        endif()
    endif()
endif()

# Compiler flags
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(grasp PRIVATE -Wall -Wextra)
endif()

# Install rules
install(TARGETS grasp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(FILES include/grasp.h DESTINATION include)

# Tests
option(GRASP_BUILD_TESTS "Build grasp test programs" ON)
option(GRASP_BUILD_HW_TEST "Build grasp hardware test programs" OFF)

if(GRASP_BUILD_TESTS)
    enable_testing()

    # grasp.so already contains motor symbols via --whole-archive
    add_executable(test_grasp test/test_grasp.c)
    target_link_libraries(test_grasp PRIVATE grasp Threads::Threads)
    set_target_properties(test_grasp PROPERTIES LINKER_LANGUAGE CXX)
    add_test(NAME grasp_test COMMAND test_grasp)
endif()

if(GRASP_BUILD_HW_TEST)
    if(MOTOR_INCLUDE_DIR)
        # Build motor library as subdirectory (contains C++ feetech adapter)
        if(NOT TARGET motor)
            set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech"
                CACHE STRING "Enabled motor drivers (semicolon-separated). Empty means auto-detect." FORCE)
            set(BUILD_TESTS OFF CACHE BOOL "Build test programs" FORCE)
            add_subdirectory(${GRASP_MOTOR_SOURCE_DIR}
                             ${CMAKE_CURRENT_BINARY_DIR}/motor)
        endif()

        add_executable(test_hw_so101_gripper test/test_hw_so101_gripper.c)
        # grasp already links motor with --whole-archive, so linking both grasp and motor
        # again under --whole-archive here duplicates motor objects and causes multiple
        # definition errors at link time.
        target_link_libraries(test_hw_so101_gripper PRIVATE 
            grasp motor
            Threads::Threads stdc++)
        # Motor library contains C++ (feetech adapter), so link with C++ linker
        set_target_properties(test_hw_so101_gripper PROPERTIES LINKER_LANGUAGE CXX)
        
        message(STATUS "[grasp] Hardware test program enabled: test_hw_so101_gripper")
    else()
        message(WARNING "[grasp] GRASP_BUILD_HW_TEST=ON but motor library not found; hardware test disabled")
    endif()
endif()
