cmake_minimum_required(VERSION 3.22)
project(control_tests LANGUAGES CXX)

enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(smc_test smc_test.cpp)
target_include_directories(smc_test PRIVATE
    ../include
    ../../math/include
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(smc_test PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME smc_test COMMAND smc_test)
