cmake_minimum_required(VERSION 3.16)

project(monte_carlo_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mcl STATIC mcl.cpp)
target_include_directories(mcl PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

if(MSVC)
    target_compile_options(mcl PRIVATE /W4 /permissive-)
else()
    target_compile_options(mcl PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(mcl_demo demo.cpp)
target_link_libraries(mcl_demo PRIVATE mcl)

add_executable(mcl_tests tests/mcl_tests.cpp)
target_link_libraries(mcl_tests PRIVATE mcl)

if(MSVC)
    target_compile_options(mcl_demo PRIVATE /W4 /permissive-)
    target_compile_options(mcl_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(mcl_demo PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(mcl_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_test(NAME mcl_tests COMMAND mcl_tests)
