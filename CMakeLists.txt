cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        )
project( HotSpotter )
find_package( OpenCV REQUIRED )

File(GLOB SRC_FILES src/*.cpp)
message(src: ${SRC_FILES})
add_executable(HotSpotter ${SRC_FILES})

target_compile_features( HotSpotter PRIVATE cxx_range_for)

target_link_libraries( HotSpotter ${OpenCV_LIBS} )
target_link_libraries(HotSpotter ftdi1)

include_directories(BEFORE ${CMAKE_SOURCE_DIR}/src)

add_subdirectory(libextendxmpp)
include_directories(libextendxmpp)
## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)

    set(CMAKE_CXX_FLAGS "-fno-stack-protector")        ## Optimize
    ##set(CMAKE_CXX_FLAGS "-fno-stack-protector-all")
endif()
target_link_libraries(HotSpotter extendxmpp)