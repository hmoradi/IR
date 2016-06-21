cmake_minimum_required(VERSION 2.8)
project( blob )
find_package( OpenCV REQUIRED )

File(GLOB SRC_FILES src/*.cpp)
message(src: ${SRC_FILES})
add_executable(blob ${SRC_FILES})

#add_executable( blob blob.cpp )


target_compile_features( blob PRIVATE cxx_range_for)
target_link_libraries( blob ${OpenCV_LIBS} )

