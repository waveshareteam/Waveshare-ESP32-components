cmake_minimum_required(VERSION 3.16)

project(pcf85063a_host_tests LANGUAGES C)

include(CTest)

add_executable(pcf85063a_calibration_test
    test_pcf85063a_calibration.c
    ../pcf85063a.c
)

target_compile_features(pcf85063a_calibration_test PRIVATE c_std_99)
target_include_directories(pcf85063a_calibration_test PRIVATE
    stubs
    ../include
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(pcf85063a_calibration_test PRIVATE -Wall -Wextra)
endif()

add_test(NAME pcf85063a_calibration COMMAND pcf85063a_calibration_test)
