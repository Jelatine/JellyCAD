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
    set(CONFIG_NAME "Debug")
else()
    set(QT_PLUGINS_DIR "${Qt6_DIR}/../../Qt6/plugins")
    set(CONFIG_NAME "Release")
endif()

# 规范化路径
get_filename_component(QT_PLUGINS_DIR "${QT_PLUGINS_DIR}" ABSOLUTE)

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
