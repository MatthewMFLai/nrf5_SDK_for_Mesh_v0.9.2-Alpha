# Setup unit test build
set(CMOCK_ROOT "${CMAKE_SOURCE_DIR}/../CMock")
find_dependency(CMOCK_ROOT
    "Path to CMock repository"
    "${CMOCK_ROOT}"
    "src/cmock.c")

set(UNITY_ROOT "${CMOCK_ROOT}/vendor/unity")
find_dependency(UNITY_ROOT
    "Path to Unity repository"
    "${UNITY_ROOT}"
    "src/unity.c")

if (NOT CMOCK_ROOT OR NOT UNITY_ROOT)
    message(FATAL_ERROR "Path(s) to CMock and/or Unity not found. "
        "Looked in:\n"
        "CMock: ${CMOCK_ROOT}\n"
        "Unity: ${UNITY_ROOT}\n"
        "Please call \n\tgit clone https://github.com/ThrowTheSwitch/CMock.git --recursive\n"
        "in the parent directory of the nRF5 SDK for Bluetooth Mesh.")
endif ()

find_program(RUBY_EXECUTABLE "ruby"
    DOC "Path to ruby executable")

if (NOT RUBY_EXECUTABLE)
    message(FATAL_ERROR "Ruby not found")
endif()

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads)

# Adds a unit test, with automatic test runner generation.
function(add_unit_test testname sources)
    add_executable(ut_${testname}
        ${sources}
        ${UNITY_ROOT}/src/unity.c
        ${CMOCK_ROOT}/src/cmock.c
        ${CMAKE_CURRENT_BINARY_DIR}/ut_${testname}_runner.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/test_assert.c)

    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ut_${testname}_runner.c
        COMMAND ${RUBY_EXECUTABLE} ${UNITY_ROOT}/auto/generate_test_runner.rb ${CMAKE_CURRENT_SOURCE_DIR}/src/ut_${testname}.c
        ${CMAKE_CURRENT_BINARY_DIR}/ut_${testname}_runner.c
        DEPENDS ${sources})

    target_compile_options(ut_${testname} PUBLIC
        ${${PLATFORM}_DEFINES})

    target_compile_options(ut_${testname} PUBLIC
        "-DNRF_MESH_LOG_ENABLE=1"
        "-DLOG_CALLBACK_DEFAULT=log_callback_stdout"
        "-DUNIT_TEST=1"
        "-DINTERNAL_EVT_ENABLE=0")

    target_include_directories(ut_${testname} PUBLIC
        "${CMAKE_SOURCE_DIR}/mesh/core/api"
        "${CMAKE_SOURCE_DIR}/mesh/core/include"
        "${CMAKE_SOURCE_DIR}/mesh/access/api"
        "${CMAKE_SOURCE_DIR}/mesh/access/include"
        "${CMAKE_SOURCE_DIR}/mesh/bearer/api"
        "${CMAKE_SOURCE_DIR}/mesh/bearer/include"
        "${CMAKE_SOURCE_DIR}/mesh/prov/api"
        "${CMAKE_SOURCE_DIR}/mesh/prov/include"
        "${CMAKE_SOURCE_DIR}/mesh/dfu/api"
        "${CMAKE_SOURCE_DIR}/mesh/dfu/include"
        "${CMAKE_SOURCE_DIR}/mesh/serial/api"
        "${CMAKE_SOURCE_DIR}/mesh/serial/include"
        "${CMAKE_SOURCE_DIR}/mesh/test/include"
        "${CMAKE_SOURCE_DIR}/mesh/test/include/device_wrappers"
        "${CMAKE_SOURCE_DIR}/models/config/include"
        "${CMAKE_SOURCE_DIR}/models/pb_remote/include"
        "${CMAKE_SOURCE_DIR}/models/health/include"
        "${CMAKE_SOURCE_DIR}/external/micro-ecc"
        "${CMOCK_BIN}"
        "${CMOCK_ROOT}/src"
        "${UNITY_ROOT}/src"
        ${${PLATFORM}_INCLUDE_DIRS}
        ${${SOFTDEVICE}_INCLUDE_DIRS})

    add_test(${testname} ut_${testname})

    # Add optional definitions
    if (ARGV2)
        target_compile_options(ut_${testname} PUBLIC "${ARGV2}")
    endif (ARGV2)

    # TODO: Add linting
endfunction(add_unit_test)

# Adds a multi-threaded unit test, with automatic test runner generation.
function(add_mtt_test testname sources)
    if(CMAKE_USE_PTHREADS_INIT)
        add_executable(ut_${testname} ${sources})
        target_include_directories(ut_${testname} PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}/mttest")

        target_compile_options(ut_${testname} PUBLIC
            "-DNRF_MESH_LOG_ENABLE=1"
            "-DLOG_CALLBACK_DEFAULT=log_callback_stdout"
            "-DMTT_TEST=1")

        target_compile_options(ut_${testname} PUBLIC
            ${${PLATFORM}_DEFINES})

        target_include_directories(ut_${testname} PUBLIC
            "${CMAKE_SOURCE_DIR}/mesh/core/api"
            "${CMAKE_SOURCE_DIR}/mesh/core/include"
            "${CMAKE_SOURCE_DIR}/mesh/access/api"
            "${CMAKE_SOURCE_DIR}/mesh/access/include"
            "${CMAKE_SOURCE_DIR}/mesh/prov/api"
            "${CMAKE_SOURCE_DIR}/mesh/prov/include"
            "${CMAKE_SOURCE_DIR}/mesh/dfu/api"
            "${CMAKE_SOURCE_DIR}/mesh/dfu/include"
            "${CMAKE_SOURCE_DIR}/mesh/serial/api"
            "${CMAKE_SOURCE_DIR}/mesh/serial/include"
            "${CMAKE_SOURCE_DIR}/mesh/test/include"
            "${CMAKE_SOURCE_DIR}/models/config/include"
            "${CMAKE_SOURCE_DIR}/models/pb_remote/include"
            "${CMOCK_BIN}"
            "${CMOCK_ROOT}/src"
            "${UNITY_ROOT}/src"
            ${${PLATFORM}_INCLUDE_DIRS}
            ${${SOFTDEVICE}_INCLUDE_DIRS})

        # Explicitly add -lpthread, which is required in some cases which are not always
        # detected by CMake:
        target_link_libraries(ut_${testname} mttest Threads::Threads pthread)
        add_test(${testname} ut_${testname})

        if(ARGV2)
            target_compile_definitions(ut_${testname} PUBLIC ${ARGV2})
        endif(ARGV2)
    endif()
endfunction(add_mtt_test)

enable_testing()
