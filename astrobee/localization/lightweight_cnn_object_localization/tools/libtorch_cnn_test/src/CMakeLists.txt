cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(libtorch_cnn_test)

find_package(Torch REQUIRED)
find_package(TorchVision REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Development)

add_executable(libtorch_cnn_test main.cc)

target_compile_features(libtorch_cnn_test PUBLIC cxx_range_for)
target_link_libraries(libtorch_cnn_test 
	${TORCH_LIBRARIES}
	${OpenCV_LIBS}
	TorchVision::TorchVision
	Python3::Python)
set_property(TARGET libtorch_cnn_test PROPERTY CXX_STANDARD 14)

install(TARGETS libtorch_cnn_test
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS libtorch_cnn_test DESTINATION bin)
