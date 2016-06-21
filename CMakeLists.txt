cmake_minimum_required(VERSION 2.8)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        )
project( blob )
find_package( OpenCV REQUIRED )

File(GLOB SRC_FILES src/*.cpp)
message(src: ${SRC_FILES})
add_executable(blob ${SRC_FILES})

target_compile_features( blob PRIVATE cxx_range_for)

target_link_libraries( blob ${OpenCV_LIBS} )
target_link_libraries(blob ftdi1)

include_directories(BEFORE ${CMAKE_SOURCE_DIR}/src)
