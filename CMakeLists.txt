cmake_minimum_required(VERSION 3.0)

project(cuda_icp_custom)

#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS "-fopenmp -Wall -fPIC -Wno-sign-compare $ENV{CXXFLAGS}")

find_package(GLEW)

find_package(OpenGL)
include_directories( ${OPENGL_INCLUDE_DIR} )

find_package(glfw3)

find_package(OpenMP)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

include_directories( ${GLFW3_INCLUDE_DIR} )

include_directories( include )


FIND_PATH( GLEW_INCLUDE_PATH GL/glew.h
           /usr/include
           /usr/local/include
           /sw/include
           /opt/local/include
           DOC "The directory where GL/glew.h resides")
FIND_LIBRARY( GLEW_LIBRARY
              NAMES GLEW glew
              PATHS
              /usr/lib64
              /usr/lib
              /usr/local/lib64
              /usr/local/lib
              /sw/lib
              /opt/local/lib
              DOC "The GLEW library")


IF (GLEW_INCLUDE_PATH)
SET( GLEW_FOUND 1 CACHE STRING "Set to 1 if GLEW is found, 0 otherwise")
ELSE (GLEW_INCLUDE_PATH)
SET( GLEW_FOUND 0 CACHE STRING "Set to 1 if GLEW is found, 0 otherwise")
ENDIF (GLEW_INCLUDE_PATH)

MARK_AS_ADVANCED( GLEW_FOUND )

FIND_PACKAGE(GLUT REQUIRED)
## Find required dependencies
FIND_PACKAGE(OpenGL REQUIRED QUIET)
FIND_PACKAGE(GLEW REQUIRED)
## Apply required dependencies settings
LIST(APPEND LINK_LIBS "${OPENGL_LIBRARIES};${GLEW_LIBRARY};${GLUT_LIBRARY};${OPENGL_LIBRARIES}")
LIST(APPEND LIB_DIRS  "${OPENGL_LIBRARY_DIR};${GLEW_LIBRARY_DIR}" )
LINK_DIRECTORIES(${LIB_DIRS})
LINK_LIBRARIES(${LINK_LIBS})
MESSAGE(STATUS "link dirs: ${LIB_DIRS}")
MESSAGE(STATUS "link libs: ${LINK_LIBS}")



# Crucial magic for CUDA linking
find_package(Threads REQUIRED)


find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


# Enable CUDA debug info in debug mode builds
list(APPEND CUDA_NVCC_FLAGS_DEBUG -G -g)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-8.0)
add_definitions(-DCUDA_ON)
find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O3 --default-stream per-thread -Xcompiler -fopenmp")
#include(${CUDA_INCLUDE_DIRS})

#set(SOURCE_FILES
#    "include/cuda_icp_custom/glslUtility.hpp"
#    "include/cuda_icp_custom/utilityCore.hpp"
#    "include/cuda_icp_custom/kernel.h"
#    "include/cuda_icp_custom/pointcloud.h"
#    "include/cuda_icp_custom/kdtree.hpp"
#    "include/cuda_icp_custom/svd3.h"
#    "include/cuda_icp_custom/cudaMat4.hpp"
#    "src/glslUtility.cpp"
#    "src/utilityCore.cpp"
#    "src/kernel.cu"
#    "src/pointcloud.cpp"
#    "src/kdtree.cpp"
#    )
#
#cuda_add_library(src
#    ${SOURCE_FILES}
#    OPTIONS -arch=sm_50
#    )
#
#cuda_add_executable(${PROJECT_NAME}
#    "include/cuda_icp_custom/main.hpp"
#    "src/main.cpp"
#    )


SET(icp_cuda_srcs  src/kernel.cu)
SET(icp_srcs  src/kdtree.cpp src/pointcloud.cpp src/utilityCore.cpp src/glslUtility.cpp)

CUDA_COMPILE(icp_cuda_objs ${icp_cuda_srcs})

add_library(${PROJECT_NAME}
            ${icp_srcs}
            ${icp_cuda_srcs}
            ${icp_cuda_objs} 
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
    ${GLEW_LIBRARIES} 
    ${GLUT_LIBRARIES} 
    ${OPENGL_LIBRARIES}
    ${OpenMP_CXX_LIBRARIES}
    glfw
    ${catkin_LIBRARIES}
    ${CUDA_LIBRARIES} 
    ${CUDA_CUBLAS_LIBRARIES}
    )


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})