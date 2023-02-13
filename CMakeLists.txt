cmake_minimum_required(VERSION 3.11)

set(PROJECT_NAME KinectFusion)
project(${PROJECT_NAME})



set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libs CACHE PATH "Path to lib folder")
set(OpenCV_DIR ${LIBRARY_DIR}/build)

# Use pre-compiled libraries on Windows
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to FreeImage header file")
        set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to FreeImage .lib/.dll folder")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x32/ CACHE PATH "Path to FreeImage header file")
        set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x32/ CACHE PATH "Path to FreeImage .lib/.dll folder")
    endif()
endif(WIN32)

# Set C++ flags
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lcudart")
set(CMAKE_BUILD_TYPE Debug)

#find required packages 
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "==> Included OpenCV dir: " ${OpenCV_INCLUDE_DIRS})

find_package(CUDA REQUIRED)
IF (CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})
    message(STATUS "==> Found CUDA： " ${CUDA_INCLUDE_DIRS})
    # Optional: Specify the arch of your CUDA hardware here
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++17 -gencode arch=compute_86,code=sm_86 --expt-relaxed-constexpr )
ENDIF ()



set(SOURCES
    main.cpp
    src/FreeImageHelper.cpp
    src/Frame.cpp
    src/Pose_estimation.cpp
    src/Voxels.cpp
    src/RayCasting.cpp
    src/SurfaceReconstruction.cpp
)

set(CUDA_SOURCES
    src/cuda/ray_casting_cuda.cu
    src/cuda/Pose_estimation_cuda.cu
)


cuda_add_library(cuda_sources ${CUDA_SOURCES})


link_directories(${FreeImage_LIBRARY_DIR})
link_directories(${OpenCV_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_executable(KinectFusion ${SOURCES})
target_include_directories(KinectFusion PUBLIC ${EIGEN3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR} include)
target_link_libraries(KinectFusion general Eigen3::Eigen freeimage ${OpenCV_LIBS} cuda_sources ${CUDA_LIBRARIES})



if(WIN32)
    # Visual Studio properties
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT KinectFusion)
    set_property(TARGET KinectFusion PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)

    # On Windows copy dll to executable folder
    add_custom_command(TARGET KinectFusion POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${FreeImage_LIBRARY_DIR}/FreeImage.dll"
            ${CMAKE_BINARY_DIR})
endif(WIN32)

