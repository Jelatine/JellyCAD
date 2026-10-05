---
# https://vitepress.dev/reference/default-theme-home-page
layout: home

hero:
  name: "JellyCAD"
  text: "现代开源可编程 CAD 软件专为程序员、机器人开发者和参数化建模爱好者设计"
#  tagline: My great project tagline
  actions:
    - theme: brand
      text: 开始使用
      link: /guide/install
    - theme: alt
      text: 下载
      link: https://github.com/Jelatine/JellyCAD/releases

features:
  - title: 🌐跨平台支持
    details: 兼容 Windows、Linux 和 macOS 系统
  - title: 📝脚本编程
    details: 使用简洁的 Lua 语言构造三维模型
  - title: 🤖机器人开发
    details: 支持导出URDF和MJCF，方便ROS/ROS2和mujoco开发
  - title: 💾多格式导出
    details: 支持导出 STL、STEP、IGES 格式文件
  - title: 🎨可视化编辑
    details: 深色自定义标题栏，集成 Lua 编辑器与 3D 预览，另提供命令行模式
  - title: 🔧丰富的操作
    details: 支持布尔运算、圆角、倒角、拉伸等多种建模操作
---

## 应用界面

![JellyCAD 自定义标题栏、Lua 编辑器和机械臂基座 3D 预览](./cover.png)

自定义标题栏显示文件名、工作目录及未保存标记，支持拖动窗口、双击最大化／还原和右上角窗口控制按钮；拖动窗口边缘或四角可以调整大小。

截图来自 macOS 上的实际运行界面，模型使用内置 `scripts/0composite.lua` 示例生成。查看[界面交互指南](./guide/interaction.md)，了解窗口、编辑器和 3D 视图操作。
