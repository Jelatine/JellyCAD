# JellyCAD

A simple CAD software.

## Develepment

### cmake
- Windows 11
- CMake 3.24.0
- Visual Studio 17 2022
- qt5:x64-windows 5.15.7
- assimp:x64-windows 5.2.5
- opencascade:x64-windows 7.6.2

### qmake
- Developed on Windows 7 platform
- Based on:
- Qt 5.9.0
- MSVC 14
- OpenCASCADE-7.3.0

## Building

### cmake
replace the first line of `CMakeLists.txt`(`CMAKE_TOOLCHAIN_FILE`) with your `vcpkg` directory
```powershell
vcpkg install qt5:x64-windows
vcpkg install assimp:x64-windows
vcpkg install opencascade:x64-windows
cd build
cmake ..
cmake --build .
```

### qmake
1. Download latest-release OpenCASCADE from:https://www.opencascade.com/content/latest-release
2. Installed OpenCASCADE with binaries,or building OCC from sources.
3. Open JellyCAD.pro with Qt.
4. Configure OpenCASCADE environment.

## Effect
<img src="https://img-blog.csdnimg.cn/20190806225153284.gif">

## Feedback

Jelatine(lijianbinmail@163.com)
