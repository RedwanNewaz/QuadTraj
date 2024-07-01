cmake_minimum_required(VERSION 3.20)
project(quad_traj)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp 0.7.0 REQUIRED)
find_package(Boost COMPONENTS filesystem program_options REQUIRED)
add_subdirectory(ThirdParties/limbo)
add_subdirectory(ThirdParties/pybind11)

include_directories(include
        /usr/include/python3.10
        /usr/local/include
        ${EIGNE3_INCLUDE_DIRS}
)
link_directories(lib)
add_executable(quad_traj main.cpp)

target_link_libraries(quad_traj PRIVATE
        ${Boost_LIBRARIES}
        Eigen3::Eigen
        -lnlopt -lm
        yaml-cpp::yaml-cpp
        libcmaes.so libccd.so libfcl.so)
