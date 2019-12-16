cmake_minimum_required(VERSION 2.8)
project(smooth)

add_compile_options(-std=c++11 -O3)

set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH          ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/lib)
find_package( Eigen3  REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${INC_DIR} )

set(ipopt_dir "/usr/local")

include_directories(${ipopt_dir}/include/coin ${ipopt_dir}/include/casadi)
include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/common ${PROJECT_SOURCE_DIR}/smoother_model ${PROJECT_SOURCE_DIR}/bezier)

link_directories(${ipopt_dir}/lib)

add_executable(ipopt_smooth_node main.cc plan_path_smoother.cc common/line_segment2d.cc smoother_model/casadi_smoother.cc smoother_model/cg_smoother.cc common/linear_interpolation.cc bezier/NumericalMaths.cpp)
target_link_libraries(ipopt_smooth_node ipopt casadi boost_system boost_filesystem boost_iostreams ${Eigen3_LIBRARIES})


