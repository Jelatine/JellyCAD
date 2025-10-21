# 安装指南

JellyCAD 是一个基于 Lua 脚本的参数化三维建模工具，支持 Windows 和 Linux 平台。本指南将帮助你快速安装和配置 JellyCAD。

## 系统要求

### Windows
- **操作系统**：Windows 10 或更高版本（64位）
- **内存**：建议 4GB 以上
- **显卡**：支持 OpenGL 3.3 或更高版本
- **磁盘空间**：至少 500MB 可用空间

### Linux
- **操作系统**：Ubuntu 22.04 / 24.04 或其他兼容的 Linux 发行版
- **内存**：建议 4GB 以上
- **显卡**：支持 OpenGL 3.3 或更高版本
- **磁盘空间**：至少 500MB 可用空间

---

## 下载安装包

访问 [JellyCAD Releases 页面](https://github.com/Jelatine/JellyCAD/releases) 选择适合你系统的最新版本下载。

**当前最新版本：** v0.3.1

| 平台 | 文件名 | 大小 |
|------|--------|------|
| Windows | `JellyCAD_Setup_v0.3.1.exe` | ~20MB |
| Ubuntu | `JellyCAD_Setup_v0.3.1.sh` | ~200MB |

---

## Windows 安装

### 安装步骤

1. **下载安装包**
   - 下载 `JellyCAD_Setup_v0.3.1.exe` 到本地

2. **运行安装程序**
   - 双击 `JellyCAD_Setup_v0.3.1.exe` 启动安装向导
   - 如果出现安全提示，点击"更多信息" → "仍要运行"

3. **选择安装位置**
   - 默认安装路径：`C:\Users\<username>\AppData\Roaming\JellyCAD`
   - 也可以自定义安装路径

4. **完成安装**
   - 等待安装完成
   - 勾选"启动 JellyCAD"可以立即运行程序
   - 安装程序会自动在桌面和开始菜单创建快捷方式

### 启动程序

安装完成后，可以通过以下方式启动 JellyCAD：
- 双击桌面上的 JellyCAD 图标
- 在开始菜单中搜索"JellyCAD"
- 在安装目录运行 `JellyCAD.exe`

### 卸载

如需卸载 JellyCAD：
1. 打开"设置" → "应用" → "应用和功能"
2. 找到"JellyCAD"并点击"卸载"
3. 或运行安装目录下的 `uninstall.exe`

---

## Ubuntu 安装

### 方法一：使用安装脚本（推荐）

**步骤：**

1. **下载安装脚本**
   ```bash
   wget https://github.com/Jelatine/JellyCAD/releases/download/v0.3.1/JellyCAD_Setup_v0.3.1.sh
   ```

2. **添加执行权限**
   ```bash
   chmod +x JellyCAD_Setup_v0.3.1.sh
   ```

3. **运行安装脚本**
   ```bash
   ./JellyCAD_Setup_v0.3.1.sh
   ```

4. **按照提示完成安装**
   - 安装路径：`~/JellyCAD_Setup_v0.3.1`
   - 等待安装完成

### 安装依赖库

JellyCAD 依赖一些系统库，如果启动时遇到库文件缺失错误，请按以下步骤安装：

**Ubuntu 22.04 / 24.04:**

```bash
# 更新软件包列表
sudo apt update

# 安装 Qt 相关依赖
sudo apt install -y libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
                    libxcb-render-util0 libxcb-xinerama0 libxcb-xkb1

# 安装会话管理库
sudo apt install -y libsm6 libice6

# 安装 X11 键盘支持
sudo apt install -y libxkbcommon-x11-0

# 安装 OpenGL 支持
sudo apt install -y libglu1-mesa libgl1-mesa-glx
```

**常见依赖问题解决：**

| 错误信息 | 解决方法 |
|----------|----------|
| `libxcb-icccm.so.4: cannot open shared object file` | `sudo apt install libxcb-icccm4` |
| `libSM.so.6: cannot open shared object file` | `sudo apt install libsm6` |
| `libxkbcommon-x11.so.0: cannot open shared object file` | `sudo apt install libxkbcommon-x11-0` |
| `libGLU.so.1: cannot open shared object file` | `sudo apt install libglu1-mesa` |

::: tip 提示
如果遇到其他库文件缺失错误，可以尝试安装完整的 Qt 依赖：
```bash
sudo apt install qt5-default  # Ubuntu 20.04
sudo apt install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools  # Ubuntu 22.04+
```
:::

### 启动程序

安装完成后，可以通过以下方式启动：

```bash
# 直接运行可执行文件
~/JellyCAD_Setup_v0.3.1/JellyCAD

# 后台运行
~/JellyCAD_Setup_v0.3.1/JellyCAD &
```

**创建桌面快捷方式：**

```bash
# 创建 .desktop 文件
cat > ~/.local/share/applications/jellycad.desktop <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=JellyCAD
Comment=Lua-based parametric 3D modeling tool
Exec=$HOME/JellyCAD_Setup_v0.3.1/JellyCAD
Icon=$HOME/JellyCAD_Setup_v0.3.1/jellycad.png
Terminal=false
Categories=Graphics;3DGraphics;Engineering;
EOF

# 更新应用程序数据库
update-desktop-database ~/.local/share/applications/
```

### 卸载

如需卸载 JellyCAD：

```bash
# 删除安装目录
rm -rf ~/JellyCAD_Setup_v0.3.1

# 删除桌面快捷方式
rm ~/.local/share/applications/jellycad.desktop

# 删除配置文件（可选）
rm -rf ~/.config/JellyCAD
```

---

## 首次运行

### 验证安装

首次启动 JellyCAD 后，你应该看到：
1. 主窗口界面（包含脚本编辑器和3D视图区）
2. 欢迎页面或默认示例
3. 无错误提示信息

### 测试脚本

在脚本编辑器中输入以下测试代码：

```lua
-- 创建一个简单的圆角立方体
b = box.new(10, 10, 10)
b:fillet(1, {})
b:color("blue")
b:show()
```

按下保存键（会自动执行），如果在3D视图区看到蓝色圆角立方体，说明安装成功！

---

## 故障排除

### Windows 常见问题

**问题1：程序无法启动，提示缺少 DLL 文件**

解决方法：
- 安装 [Microsoft Visual C++ Redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe)
- 重启计算机后再次尝试

**问题2：3D视图显示黑屏**

解决方法：
- 更新显卡驱动程序
- 检查显卡是否支持 OpenGL 3.3+
- 在设置中尝试切换渲染模式

**问题3：防病毒软件阻止安装**

解决方法：
- 临时禁用防病毒软件
- 将 JellyCAD 添加到信任列表
- 从官方 GitHub 下载确保文件安全

### Linux 常见问题

**问题1：无法执行安装脚本**

解决方法：
```bash
# 检查文件权限
ls -l JellyCAD_Setup_v0.3.1.sh

# 确保有执行权限
chmod +x JellyCAD_Setup_v0.3.1.sh

# 使用 bash 显式运行
bash JellyCAD_Setup_v0.3.1.sh
```

**问题2：库依赖错误**

解决方法：
```bash
# 查看缺失的库
ldd ~/JellyCAD_Setup_v0.3.1/JellyCAD | grep "not found"

# 根据输出安装对应的库
# 例如：sudo apt install lib<package-name>
```

**问题3：启动后立即崩溃**

解决方法：
```bash
# 在终端运行查看错误信息
~/JellyCAD_Setup_v0.3.1/JellyCAD

# 检查 OpenGL 支持
glxinfo | grep "OpenGL version"

# 如果没有 glxinfo，先安装
sudo apt install mesa-utils
```

**问题4：Wayland 兼容性问题**

如果在 Wayland 会话中遇到问题，可以尝试使用 XWayland：
```bash
# 设置环境变量强制使用 X11
export QT_QPA_PLATFORM=xcb
~/JellyCAD_Setup_v0.3.1/JellyCAD
```

---

## 更新

### 检查更新

访问 [GitHub Releases](https://github.com/Jelatine/JellyCAD/releases) 查看是否有新版本发布。

### 升级到新版本

**Windows:**
1. 下载新版本的安装程序
2. 运行安装程序（会自动覆盖旧版本）
3. 重启应用程序

**Linux:**
1. 备份重要脚本文件
2. 卸载旧版本
3. 安装新版本

::: tip 提示
在升级前，建议备份以下内容：
- `~/.config/JellyCAD/` - 配置文件
- 你的脚本项目文件
:::

---

## 下一步

安装完成后，建议阅读以下文档：
- [界面交互指南](./interaction.md) - 了解如何使用界面
- [脚本函数参考](./functions.md) - 学习建模函数
- [圆角和倒角操作](./fillet_chamfer.md) - 掌握边缘处理技巧
- [机器人开发指南](./robot_develop.md) - 创建机器人模型

## 获取帮助

如果遇到问题，可以通过以下途径获取帮助：
- 查看 [GitHub Issues](https://github.com/Jelatine/JellyCAD/issues)
- 提交新的问题报告
- 参与社区讨论

---

**祝你使用愉快！** 🎉

