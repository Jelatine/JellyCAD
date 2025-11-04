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
if (MSVC)
    add_compile_options(/bigobj) # 对象文件（.obj）包含的节（sections）数量超过了标准对象文件格式的限制
endif ()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/bin>)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(OpenCASCADE CONFIG REQUIRED)


# 添加编译选项：强制使用Qt5
option(FORCE_QT5 "Force use Qt5 instead of Qt6" OFF)
# Qt版本检测：优先Qt6，可通过FORCE_QT5选项强制使用Qt5
if(NOT FORCE_QT5)
    find_package(Qt6 QUIET COMPONENTS Core Widgets)
endif()
if (Qt6_FOUND)
    message(STATUS "Found Qt6, using Qt6")
    set(QT_VERSION_MAJOR 6)
    set(QT_VERSION ${Qt6_VERSION})
else()
    find_package(Qt5 REQUIRED COMPONENTS Widgets)
    message(STATUS "using Qt5")
    set(QT_VERSION_MAJOR 5)
    set(QT_VERSION ${Qt5_VERSION})
    if (MSVC)
        # 解决UTF-8存在中文时，MSVC编译失败
        add_compile_options("/source-charset:utf-8")
        add_compile_options("/execution-charset:utf-8")
    endif ()
endif()

find_package(Lua CONFIG REQUIRED)
find_package(sol2 CONFIG REQUIRED)

message(STATUS "occ version:${OpenCASCADE_VERSION}")
message(STATUS "qt version:${QT_VERSION}")
message(STATUS "sol2 version:${sol2_VERSION}")

add_definitions(-DVERSION_DISPLAY_OCC=\"${OpenCASCADE_VERSION}\")
add_definitions(-DVERSION_DISPLAY_QT=\"${QT_VERSION}\")
add_definitions(-DVERSION_DISPLAY_SOL2=\"${sol2_VERSION}\")

if (TAG_VERSION)
    message(STATUS "software(input):${TAG_VERSION}")
else ()
    execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} describe
            TIMEOUT 3 OUTPUT_VARIABLE TAG_VERSION RESULT_VARIABLE RES OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "software:${TAG_VERSION}")
endif ()
add_definitions(-DJELLY_CAD_VERSION=\"${TAG_VERSION}\")

file(GLOB_RECURSE SRC_FILES src/*.cpp src/*.h resource/*.qrc)

add_executable(JellyCAD ${SRC_FILES} ${WIN32_RESOURCES})
target_include_directories(JellyCAD PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_include_directories(JellyCAD PRIVATE src src/widgets src/shapes)

# Link Qt libraries based on version
if (QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(JellyCAD PRIVATE
            Qt6::Core
            Qt6::Widgets
            ${OpenCASCADE_LIBRARIES}
            ${LUA_LIBRARIES})
    if(WIN32)
        # 在构建时根据配置类型（Debug/Release）动态复制Qt插件
        # 使用 $<CONFIG> 生成器表达式确保多配置生成器（如Visual Studio）能正确工作
        add_custom_command(TARGET JellyCAD POST_BUILD
            COMMAND ${CMAKE_COMMAND}
                -DCONFIG_TYPE=$<CONFIG>
                -DQt6_DIR=${Qt6_DIR}
                -DTARGET_DIR=$<TARGET_FILE_DIR:JellyCAD>
                -P ${CMAKE_CURRENT_SOURCE_DIR}/copy_qt_plugins.cmake
            COMMENT "Copying Qt6 plugins for $<CONFIG> configuration..."
        )

        # 生成 qt.conf 配置文件
        add_custom_command(TARGET JellyCAD POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E echo "[Paths]" > "$<TARGET_FILE_DIR:JellyCAD>/qt.conf"
            COMMAND ${CMAKE_COMMAND} -E echo "Plugins=plugins" >> "$<TARGET_FILE_DIR:JellyCAD>/qt.conf"
            COMMENT "Generating qt.conf for $<CONFIG>"
        )
    endif()
else()
    target_link_libraries(JellyCAD PRIVATE
            Qt5::Widgets
            ${OpenCASCADE_LIBRARIES}
            ${LUA_LIBRARIES})
endif()

if ((${CMAKE_SYSTEM_NAME} MATCHES "Windows") AND (${CMAKE_BUILD_TYPE} STREQUAL "Release"))
    set_target_properties(JellyCAD PROPERTIES WIN32_EXECUTABLE ON)
endif ()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/scripts)
file(GLOB EXAMPLE_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.lua")
file(COPY ${EXAMPLE_SCRIPTS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin/scripts)

set(CPACK_PACKAGE_NAME "JellyCAD")
set(CPACK_PACKAGE_VERSION ${TAG_VERSION})
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_Setup_${CPACK_PACKAGE_VERSION}")
if (APPLE)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/scripts DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin/JellyCAD.app/Contents/MacOS)
    set(CPACK_GENERATOR "DragNDrop")
    set_target_properties(JellyCAD PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/resource/Info.plist.in
            MACOSX_BUNDLE_BUNDLE_NAME "JellyCAD"
            MACOSX_BUNDLE_BUNDLE_VERSION ${TAG_VERSION}
            MACOSX_BUNDLE_SHORT_VERSION_STRING ${TAG_VERSION}
            MACOSX_BUNDLE_LONG_VERSION_STRING ${TAG_VERSION}
            MACOSX_BUNDLE_EXECUTABLE_NAME "JellyCAD"
            MACOSX_BUNDLE_GUI_IDENTIFIER "jelatine.github.io"
            MACOSX_BUNDLE_COPYRIGHT "Copyright (c) 2024 Li Jianbin"
            MACOSX_BUNDLE_ICON_FILE "mac_icon.icns"
    )
    # 如果有图标文件，复制到Resources目录
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource/mac_icon.icns")
        set_source_files_properties(
                "${CMAKE_CURRENT_SOURCE_DIR}/resource/mac_icon.icns"
                PROPERTIES
                MACOSX_PACKAGE_LOCATION "Resources"
        )
        target_sources(JellyCAD PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/resource/mac_icon.icns")
    endif()
    install(TARGETS JellyCAD BUNDLE DESTINATION .)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts
            DESTINATION JellyCAD.app/Contents/MacOS/
            USE_SOURCE_PERMISSIONS
    )
    set(CPACK_DMG_VOLUME_NAME "JellyCAD")
    set(CPACK_DMG_FORMAT "UDBZ")  # 压缩格式
else ()
    set(CPACK_GENERATOR "STGZ")
    install(TARGETS JellyCAD RUNTIME DESTINATION .)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts DESTINATION .)
endif ()

# Unit tests configuration
option(BUILD_TESTS "Build unit tests" OFF)
find_package(GTest CONFIG QUIET)
if(BUILD_TESTS AND GTest_FOUND)
    include(CTest)
    enable_testing()
    file(GLOB_RECURSE TEST_FILES tests/*.cpp)
    file(GLOB_RECURSE TEST_SRC_FILES
        src/shapes/*.cpp
        src/shapes/*.h
    )
    add_executable(JellyCAD_tests ${TEST_FILES} ${TEST_SRC_FILES})
    target_include_directories(JellyCAD_tests PRIVATE
        ${OpenCASCADE_INCLUDE_DIR}
        src
        src/widgets
        src/shapes
    )
    target_link_libraries(JellyCAD_tests PRIVATE
        GTest::gtest_main
        ${OpenCASCADE_LIBRARIES}
        ${LUA_LIBRARIES}
    )
    include(GoogleTest)
    gtest_discover_tests(JellyCAD_tests)
    message(STATUS "Unit tests enabled")
endif()

include(CPack)