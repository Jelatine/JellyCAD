#
# Copyright (c) 2024. Li Jianbin. All rights reserved.
# MIT License
#
cmake_minimum_required(VERSION 3.14)
project(JellyCAD)
set(CMAKE_CXX_STANDARD 17)
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    enable_language("RC")
    set(WIN32_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resource/icon.rc)
endif ()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(OpenCASCADE REQUIRED QUIET)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Lua REQUIRED)
find_package(sol2 CONFIG REQUIRED)

set(SRC_FILES main.cpp
        resource/res.qrc
        include/jy_shape.h
        src/jy_shape.cpp
        include/jy_lua_virtual_machine.h
        src/jy_lua_virtual_machine.cpp
        include/jy_main_window.h
        src/jy_main_window.cpp
        include/jy_code_editor.h
        src/jy_code_editor.cpp
        include/jy_activity_bar.h
        src/jy_activity_bar.cpp
        src/jy_3d_widget.cpp
        include/jy_3d_widget.h)

add_executable(JellyCAD ${SRC_FILES} ${WIN32_RESOURCES})
target_include_directories(JellyCAD PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_include_directories(JellyCAD PRIVATE include)
target_link_libraries(JellyCAD PRIVATE
        Qt5::Widgets
        ${OpenCASCADE_LIBRARIES}
        sol2
        ${LUA_LIBRARIES})

if ((${CMAKE_SYSTEM_NAME} MATCHES "Windows") AND (${CMAKE_BUILD_TYPE} STREQUAL "Release"))
    set_target_properties(JellyCAD PROPERTIES WIN32_EXECUTABLE ON)
endif ()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/scripts)
file(GLOB EXAMPLE_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.lua")
file(COPY ${EXAMPLE_SCRIPTS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/scripts/)