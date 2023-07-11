cmake_minimum_required(VERSION 3.0.0)
project(jelly_cad VERSION 0.1.0)

# QT Lib
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED Widgets)

# OpenCASCADE LIB
find_package(freetype CONFIG REQUIRED)
find_package(OpenCASCADE CONFIG REQUIRED)
include_directories(${OpenCASCADE_INCLUDE_DIR})

# ASSIMP LIB
find_package(assimp CONFIG REQUIRED)

# USER
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(USER_SOURCE
    main.cpp
    cmainwindow.cpp
    c3dwidget.cpp
    cmodel.cpp
    cmainwindow.h
    c3dwidget.h
    makebottle.h
    cmodel.h
    cmainwindow.ui
    res.qrc
)

add_executable(${CMAKE_PROJECT_NAME} ${USER_SOURCE})
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
        Qt5::Widgets
        ${OpenCASCADE_LIBRARIES}
        assimp::assimp)
