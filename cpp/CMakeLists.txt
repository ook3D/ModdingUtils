cmake_minimum_required(VERSION 3.20)
project(moddingtool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(MSVC)
    add_compile_options(/W4 /permissive- /EHsc)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(moddingtool SHARED
    src/api.cpp
    src/mesh.cpp
    src/collision.cpp
    src/remesh.cpp
    src/decal.cpp
    src/blend_decal.cpp
)

target_include_directories(moddingtool
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_definitions(moddingtool PRIVATE MT_BUILDING_DLL)

set_target_properties(moddingtool PROPERTIES
    OUTPUT_NAME "moddingtool"
    PREFIX ""
)

set(ADDON_BIN_DIR "${CMAKE_SOURCE_DIR}/../moddingtool/bin")
add_custom_command(TARGET moddingtool POST_BUILD
    COMMAND ${CMAKE_COMMAND}
            -DSRC=$<TARGET_FILE:moddingtool>
            -DDST=${ADDON_BIN_DIR}/$<TARGET_FILE_NAME:moddingtool>
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/StageDll.cmake"
    COMMENT "Staging moddingtool DLL into Blender add-on bin/"
    VERBATIM
)
