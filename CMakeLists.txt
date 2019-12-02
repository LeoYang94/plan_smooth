cmake_minimum_required(VERSION 2.8)
project(smooth)

add_compile_options(-std=c++11 -O3)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH          ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/lib)

set(ipopt_dir "/usr/local")

include_directories(${ipopt_dir}/include/coin ${ipopt_dir}/include/adolc)

link_directories(${ipopt_dir}/lib)
link_directories(${ipopt_dir}/lib64)

add_executable(ipopt_smooth_node main.cc)
target_link_libraries(ipopt_smooth_node pthread ipopt adolc)


