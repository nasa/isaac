cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(libtorch_mrcnn_test)

find_package(Torch REQUIRED)

add_executable(libtorch_mrcnn_test libtorch_mrcnn_test.cc)
target_link_libraries(libtorch_mrcnn_test "${TORCH_LIBRARIES}")
set_property(TARGET libtorch_mrcnn_test PROPERTY CXX_STANDARD 17)
