cmake_minimum_required(VERSION 3.6)

set(PROJECT_NAME KinectFusion)
project(${PROJECT_NAME})




set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libs CACHE PATH "Path to lib folder")

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
set(CMAKE_CXX_STANDARD 14)

#find required packages 
find_package(Eigen3 REQUIRED)

set(HEADERS 
    Eigen.h
    FreeImageHelper.h
    VirtualSensor.h
    Frame.h
    Pose_estimation.h
)

set(SOURCES
    main.cpp
    FreeImageHelper.cpp
    Frame.cpp
    Pose_estimation.cpp
)


link_directories(${FreeImage_LIBRARY_DIR})
add_executable(KinectFusion ${HEADERS} ${SOURCES})
target_include_directories(KinectFusion PUBLIC ${EIGEN3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR})
target_link_libraries(KinectFusion general Eigen3::Eigen freeimage)



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

