cmake_minimum_required(VERSION 2.8)

project(pangolin_c_api)

add_definitions(
        -std=c++14
        -fPIC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -fPIC -O3")


find_package(Pangolin 0.6 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("-- Found Eigen version ${EIGEN_VERSION}: ${EIGEN3_INCLUDE_DIRS}")

message(${Pangolin_LIBRARIES})

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_library(${PROJECT_NAME} SHARED
        src/pangolin_c_api.cc)
target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

