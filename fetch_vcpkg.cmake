# fetch_vcpkg.cmake
# 从 Jelatine/JellyCAD-vcpkg 的 Release 下载预编译 vcpkg 依赖，并设置 CMAKE_TOOLCHAIN_FILE
#
# 需在 project() 之前 include，开启方式：
#   cmake --preset release                       （推荐，见 CMakePresets.json）
#   cmake -B build -DJELLYCAD_PREBUILT_VCPKG=ON
#
# 产物解压到 <源码目录>/.vcpkg/<版本>/vcpkg，多个构建目录共享同一份依赖。
# 更换版本后需使用 --fresh 重新配置（工具链文件只在首次配置时生效）。
# 产物仅适用于与 JellyCAD-vcpkg 构建环境相同的系统和编译器。

option(JELLYCAD_PREBUILT_VCPKG "Download prebuilt vcpkg dependencies from Jelatine/JellyCAD-vcpkg" OFF)
set(JELLYCAD_VCPKG_VERSION "2026.06.24" CACHE STRING "Release version of Jelatine/JellyCAD-vcpkg (tag: vcpkg-<version>)")

if (NOT JELLYCAD_PREBUILT_VCPKG)
    return()
endif ()

if (CMAKE_VERSION VERSION_LESS 3.19)
    message(FATAL_ERROR "JELLYCAD_PREBUILT_VCPKG requires CMake 3.19 or newer")
endif ()

set(_vcpkg_repo "Jelatine/JellyCAD-vcpkg")
set(_vcpkg_tag "vcpkg-${JELLYCAD_VCPKG_VERSION}")
set(_vcpkg_root "${CMAKE_CURRENT_SOURCE_DIR}/.vcpkg/${JELLYCAD_VCPKG_VERSION}")
set(_vcpkg_toolchain "${_vcpkg_root}/vcpkg/scripts/buildsystems/vcpkg.cmake")

# 已指定其他工具链时不覆盖，避免与用户自己的 vcpkg 混用
if (DEFINED CMAKE_TOOLCHAIN_FILE AND NOT CMAKE_TOOLCHAIN_FILE STREQUAL _vcpkg_toolchain)
    message(FATAL_ERROR "JELLYCAD_PREBUILT_VCPKG is ON but CMAKE_TOOLCHAIN_FILE is already set to:\n"
            "  ${CMAKE_TOOLCHAIN_FILE}\n"
            "Turn off JELLYCAD_PREBUILT_VCPKG, or reconfigure with --fresh after changing JELLYCAD_VCPKG_VERSION.")
endif ()

# 与 Release 中的分卷命名一致：vcpkg-<Windows|Linux|macOS>.tar.gz.part-*
if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
    set(_vcpkg_platform "Windows")
elseif (CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
    set(_vcpkg_platform "macOS")
elseif (CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    set(_vcpkg_platform "Linux")
else ()
    message(FATAL_ERROR "No prebuilt vcpkg for host system: ${CMAKE_HOST_SYSTEM_NAME}")
endif ()

if (NOT EXISTS "${_vcpkg_root}/.complete")
    set(_download_dir "${_vcpkg_root}/download")
    file(REMOVE_RECURSE "${_vcpkg_root}")
    file(MAKE_DIRECTORY "${_download_dir}")

    # 有 token 时携带认证，避免 GitHub API 匿名访问频率限制
    set(_auth_header)
    if (DEFINED ENV{GH_TOKEN})
        set(_auth_header HTTPHEADER "Authorization: Bearer $ENV{GH_TOKEN}")
    elseif (DEFINED ENV{GITHUB_TOKEN})
        set(_auth_header HTTPHEADER "Authorization: Bearer $ENV{GITHUB_TOKEN}")
    endif ()

    # 通过 Release API 获取分卷列表及大小
    set(_release_json "${_download_dir}/release.json")
    file(DOWNLOAD "https://api.github.com/repos/${_vcpkg_repo}/releases/tags/${_vcpkg_tag}" "${_release_json}"
            STATUS _status ${_auth_header})
    list(GET _status 0 _status_code)
    if (_status_code)
        message(FATAL_ERROR "Failed to query release ${_vcpkg_repo}@${_vcpkg_tag}: ${_status}")
    endif ()
    file(READ "${_release_json}" _release)

    set(_parts)
    string(JSON _asset_count LENGTH "${_release}" assets)
    math(EXPR _asset_last "${_asset_count} - 1")
    foreach (_i RANGE ${_asset_last})
        string(JSON _name GET "${_release}" assets ${_i} name)
        if (NOT _name MATCHES "^vcpkg-${_vcpkg_platform}\\.tar\\.gz\\.part-")
            continue()
        endif ()
        string(JSON _url GET "${_release}" assets ${_i} browser_download_url)
        string(JSON _size GET "${_release}" assets ${_i} size)
        set(_part "${_download_dir}/${_name}")
        message(STATUS "Downloading ${_name} (${_size} bytes)...")
        file(DOWNLOAD "${_url}" "${_part}" STATUS _status SHOW_PROGRESS ${_auth_header})
        list(GET _status 0 _status_code)
        file(SIZE "${_part}" _actual_size)
        if (_status_code OR NOT _actual_size EQUAL _size)
            message(FATAL_ERROR "Failed to download ${_url}: ${_status} (got ${_actual_size} of ${_size} bytes)")
        endif ()
        list(APPEND _parts "${_part}")
    endforeach ()
    if (NOT _parts)
        message(FATAL_ERROR "No vcpkg-${_vcpkg_platform}.tar.gz.part-* found in ${_vcpkg_repo}@${_vcpkg_tag}")
    endif ()

    # 合并分卷（按名称排序：part-aa, part-ab, ...）
    list(SORT _parts)
    list(LENGTH _parts _part_count)
    if (_part_count EQUAL 1)
        set(_archive "${_parts}")
    else ()
        set(_archive "${_download_dir}/vcpkg-${_vcpkg_platform}.tar.gz")
        message(STATUS "Merging ${_part_count} parts...")
        # cmake -E cat 以二进制方式输出，Windows 下同样适用
        execute_process(COMMAND "${CMAKE_COMMAND}" -E cat ${_parts} OUTPUT_FILE "${_archive}" RESULT_VARIABLE _result)
        if (_result)
            message(FATAL_ERROR "Failed to merge vcpkg archive parts")
        endif ()
    endif ()

    message(STATUS "Extracting vcpkg to ${_vcpkg_root} ...")
    file(ARCHIVE_EXTRACT INPUT "${_archive}" DESTINATION "${_vcpkg_root}")
    if (NOT EXISTS "${_vcpkg_toolchain}")
        message(FATAL_ERROR "Extracted archive does not contain vcpkg/scripts/buildsystems/vcpkg.cmake")
    endif ()

    file(REMOVE_RECURSE "${_download_dir}")
    file(WRITE "${_vcpkg_root}/.complete" "${_vcpkg_tag}\n")
endif ()

message(STATUS "Using prebuilt vcpkg: ${_vcpkg_root}/vcpkg")
set(CMAKE_TOOLCHAIN_FILE "${_vcpkg_toolchain}" CACHE FILEPATH "vcpkg toolchain file" FORCE)
