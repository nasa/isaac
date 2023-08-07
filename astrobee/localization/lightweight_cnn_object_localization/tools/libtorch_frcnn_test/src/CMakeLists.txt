cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(libtorch_frcnn_test)

find_package(Torch REQUIRED)
find_package(TorchVision REQUIRED)
find_package(Python3 COMPONENTS Development)

add_executable(libtorch_frcnn_test main.cc)

target_compile_features(libtorch_frcnn_test PUBLIC cxx_range_for)
target_link_libraries(libtorch_frcnn_test 
	${TORCH_LIBRARIES}
	TorchVision::TorchVision
	Python3::Python)
set_property(TARGET libtorch_frcnn_test PROPERTY CXX_STANDARD 14)

install(TARGETS libtorch_frcnn_test
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS libtorch_frcnn_test DESTINATION bin)
