cmake_minimum_required(VERSION 3.16.0)

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(
    -Og
    -g3
  )
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(
    -O3
    -flto
    -march=native
    -mtune=native
    -g0
    -ftree-vectorize
    -fomit-frame-pointer
    -fvisibility-inlines-hidden
    -DNDEBUG
  )
  add_link_options(-flto=full)
endif()
add_compile_options(
  -fno-rtti
  -fstack-protector
    -Wall
    -Wextra
    -Wpedantic
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(PhysicCore VERSION 2.0.0)

set(PROJECT_SOURCE_DIR "${CMAKE_SOURCE_DIR}")
set(MATRIX_SOURCES
  ${PROJECT_SOURCE_DIR}/src/Matrix.cpp
)
set(NASAVAL_SOURCES
    ${PROJECT_SOURCE_DIR}/src/NasaSimulationVal.cpp
    ${PROJECT_SOURCE_DIR}/src/Vec3.cpp
    ${PROJECT_SOURCE_DIR}/src/Matrix.cpp
    ${PROJECT_SOURCE_DIR}/src/WorldPhysics.cpp
    ${PROJECT_SOURCE_DIR}/src/Integrator.cpp
    ${PROJECT_SOURCE_DIR}/src/Gravity.cpp
    ${PROJECT_SOURCE_DIR}/src/Body.cpp
    ${PROJECT_SOURCE_DIR}/src/Mat3.cpp
    ${PROJECT_SOURCE_DIR}/src/PreInitializer.cpp
)

set(CSPICE_ROOT "${CMAKE_SOURCE_DIR}/deps/cspice")

add_executable(MATRIX  ${MATRIX_SOURCES})
add_executable(NASAVAL ${NASAVAL_SOURCES})

set(PROJECT_EXECUTABLES
    MATRIX
    NASAVAL
)

foreach(target ${PROJECT_EXECUTABLES})
    target_include_directories(${target} PRIVATE
        include
        ${CSPICE_ROOT}/include
    )

    target_link_libraries(${target} PRIVATE
        ${CSPICE_ROOT}/lib/libcspice.a
    )

    if(WIN32)
#        set_target_properties(${target} PROPERTIES
#            WIN32_EXECUTABLE TRUE
#        )

        target_compile_definitions(${target} PRIVATE
            _WIN32_WINNT=0x0A00
        )

        target_link_libraries(${target} PRIVATE
            ws2_32
            mswsock
            user32
            gdi32
            shell32
            crypt32
            bcrypt
            ncrypt
            secur32
            ole32
            uuid
        )
    endif()
endforeach()