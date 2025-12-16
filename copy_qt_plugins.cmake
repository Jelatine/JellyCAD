# copy_qt_plugins.cmake
# 根据构建配置类型动态选择并复制Qt插件目录
#
# 使用方法：
#   cmake -DCONFIG_TYPE=<Debug|Release> -DQt6_DIR=<path> -DTARGET_DIR=<path> -P copy_qt_plugins.cmake

if(NOT DEFINED CONFIG_TYPE)
    message(FATAL_ERROR "CONFIG_TYPE is not defined")
endif()

if(NOT DEFINED Qt6_DIR)
    message(FATAL_ERROR "Qt6_DIR is not defined")
endif()

if(NOT DEFINED TARGET_DIR)
    message(FATAL_ERROR "TARGET_DIR is not defined")
endif()

# 根据配置类型选择正确的Qt插件目录
if(CONFIG_TYPE STREQUAL "Debug")
    set(QT_PLUGINS_DIR "${Qt6_DIR}/../../debug/Qt6/plugins")
    set(QT_BIN_DIR "${Qt6_DIR}/../../debug/bin")
    set(CONFIG_NAME "Debug")
else()
    set(QT_PLUGINS_DIR "${Qt6_DIR}/../../Qt6/plugins")
    set(QT_BIN_DIR "${Qt6_DIR}/../../bin")
    set(CONFIG_NAME "Release")
endif()

# 规范化路径
get_filename_component(QT_PLUGINS_DIR "${QT_PLUGINS_DIR}" ABSOLUTE)
get_filename_component(QT_BIN_DIR "${QT_BIN_DIR}" ABSOLUTE)

# 检查源目录是否存在
if(EXISTS "${QT_PLUGINS_DIR}")
    message(STATUS "Copying Qt6 ${CONFIG_NAME} plugins...")
    message(STATUS "  From: ${QT_PLUGINS_DIR}")
    message(STATUS "  To:   ${TARGET_DIR}/plugins")

    # 复制插件目录
    file(COPY "${QT_PLUGINS_DIR}/"
         DESTINATION "${TARGET_DIR}/plugins"
         PATTERN "*.pdb" EXCLUDE)  # 排除PDB调试符号文件（可选）

    message(STATUS "Qt6 plugins copied successfully")
else()
    message(WARNING "Qt6 plugins directory not found: ${QT_PLUGINS_DIR}")
    message(WARNING "Plugins may not be copied. Application might not start correctly.")
endif()

# 复制libssl-3-x64.dll到bin目录
set(SSL_DLL "${QT_BIN_DIR}/libssl-3-x64.dll")
if(EXISTS "${SSL_DLL}")
    message(STATUS "Copying libssl-3-x64.dll...")
    message(STATUS "  From: ${SSL_DLL}")
    message(STATUS "  To:   ${TARGET_DIR}/")

    file(COPY "${SSL_DLL}"
         DESTINATION "${TARGET_DIR}/")

    message(STATUS "libssl-3-x64.dll copied successfully")
else()
    message(WARNING "libssl-3-x64.dll not found: ${SSL_DLL}")
endif()
