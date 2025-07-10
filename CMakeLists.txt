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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/bin>)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(OpenCASCADE CONFIG REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Lua CONFIG REQUIRED)
find_package(sol2 CONFIG REQUIRED)

message(STATUS "occ version:${OpenCASCADE_VERSION}")
message(STATUS "qt5 version:${Qt5_VERSION}")
message(STATUS "sol2 version:${sol2_VERSION}")

add_definitions(-DVERSION_DISPLAY_OCC=\"${OpenCASCADE_VERSION}\")
add_definitions(-DVERSION_DISPLAY_QT5=\"${Qt5_VERSION}\")
add_definitions(-DVERSION_DISPLAY_SOL2=\"${sol2_VERSION}\")

if (TAG_VERSION)
    add_definitions(-DJELLY_CAD_VERSION=\"${TAG_VERSION}\")
    message(STATUS "software(input):${TAG_VERSION}")
else ()
    execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} describe --tags --abbrev=0
            TIMEOUT 3 OUTPUT_VARIABLE TAG_VERSION RESULT_VARIABLE RES OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (RES EQUAL 0)
        add_definitions(-DJELLY_CAD_VERSION=\"${TAG_VERSION}\")
        message(STATUS "software:${TAG_VERSION}")
    endif ()
endif ()

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
#        sol2
        ${LUA_LIBRARIES})

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

if ((${CMAKE_SYSTEM_NAME} MATCHES "Windows") AND (${CMAKE_BUILD_TYPE} STREQUAL "Release"))
    set_target_properties(JellyCAD PROPERTIES WIN32_EXECUTABLE ON)
endif ()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/scripts)
file(GLOB EXAMPLE_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.lua")
file(COPY ${EXAMPLE_SCRIPTS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin/scripts)

install(TARGETS JellyCAD DESTINATION ./)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts DESTINATION ./)
set(CPACK_GENERATOR "STGZ")
set(CPACK_PACKAGE_NAME "JellyCAD")
set(CPACK_PACKAGE_VERSION ${TAG_VERSION})
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
include(CPack)
