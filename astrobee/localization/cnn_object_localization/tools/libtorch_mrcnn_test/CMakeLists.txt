cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(libtorch_mrcnn_test)

#set(CMAKE_OSX_ARCHITECTURES "armv7l")
find_package(Torch REQUIRED)
find_package(TorchVision REQUIRED)
find_package(Python3 COMPONENTS Development)

add_executable(libtorch_mrcnn_test main.cc)

target_compile_features(libtorch_mrcnn_test PUBLIC cxx_range_for)
target_link_libraries(libtorch_mrcnn_test 
	${TORCH_LIBRARIES}
	TorchVision::TorchVision
	Python3::Python)
set_property(TARGET libtorch_mrcnn_test PROPERTY CXX_STANDARD 17)
