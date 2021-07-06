cmake_minimum_required (VERSION 3.1)
add_definitions(-w)
project (PlaneDetection)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

# Set OS-specific things here
if(WIN32)
elseif(CYGWIN)
elseif(APPLE)
elseif(UNIX)
    add_definitions(-DUNIX)
    add_compile_options(-Wno-deprecated-declarations)
    add_compile_options(-Wno-unused-result)
    add_definitions(-O3)
endif(WIN32)

# Open3D
if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")

    # link_directories must be before add_executable
    link_directories(${Open3D_LIBRARY_DIRS})


    # put FindGLFW.cmake in the source directory of CMakeLists.txt
    find_package(OpenCV REQUIRED)
    find_package(PCL 1.9 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS} /usr/include/eigen3)


    add_library(${PROJECT_NAME} SHARED

            plane_detection.cpp
            )
    file(GLOB sources "*.cpp")

    add_executable(MeshPlaneDetection ${sources})

    target_link_libraries(MeshPlaneDetection ${OpenCV_LIBS} ${Open3D_LIBRARIES} ${PCL_LIBRARIES})
    target_include_directories(MeshPlaneDetection PUBLIC ${Open3D_INCLUDE_DIRS})

    # Hot fix windows dll not found issue, assumming we're using the Release build
    option(BUILD_SHARED_LIBS "Whether Open3D was build as shared library" OFF)
    if(WIN32 AND BUILD_SHARED_LIBS)
        message("Will copy Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
        add_custom_command(TARGET MeshPlaneDetection POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                ${CMAKE_CURRENT_BINARY_DIR}/Release)
    endif()

else ()
    message(SEND_ERROR "Open3D not found")
endif ()