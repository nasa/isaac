cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(libtorch_mrcnn_test)

#set(CMAKE_OSX_ARCHITECTURES "armv7l")
list(APPEND CMAKE_PREFIX_PATH "libtorch")
list(APPEND CMAKE_PREFIX_PATH "vision/cmake")

add_executable(libtorch_mrcnn_test main.cc)

find_package(Python3 COMPONENTS Development)
find_package(TorchVision REQUIRED)

target_compile_features(libtorch_mrcnn_test PUBLIC cxx_range_for)
target_link_libraries(libtorch_mrcnn_test TorchVision::TorchVision)
set_property(TARGET libtorch_mrcnn_test PROPERTY CXX_STANDARD 17)
