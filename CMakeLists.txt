cmake_minimum_required(VERSION 3.16)
project(yoomi_cell C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_compile_options(-Wall -Wextra -Werror -Wshadow -Wpedantic)

add_library(yoomi
    src/motion.c
    src/gripper.c
    src/safety.c
    src/modbus.c
    src/mqtt_pkt.c
    src/json_msg.c
    src/cell.c
    ports/esp32/port_main.c
)
target_include_directories(yoomi PUBLIC include)

add_executable(yoomi-cell src/main.c)
target_link_libraries(yoomi-cell PRIVATE yoomi)

add_executable(yoomi-tests
    tests/test_main.c
    tests/test_motion.c
    tests/test_protocol.c
    tests/test_cell.c
)
target_include_directories(yoomi-tests PRIVATE tests)
target_link_libraries(yoomi-tests PRIVATE yoomi)

enable_testing()
add_test(NAME yoomi-tests COMMAND yoomi-tests)
