<div align="center">
  <img src="doc/icon.png" alt="Logo" width="128px" align="center" />
  <p></p>
  <p><strong>JellyCAD</strong>开源可编程CAD软件</p>
</div>
通过脚本语言编程，构造和导出3D模型。

## 特点

- 支持`Windows`和`Linux`系统
- 通过`lua`脚本语言构造三维模型
- 可导出`STL`,`STEP`,`IGES`格式的文件

## 开发环境

- Windows 11 23H2
  - CMake 3.24.0-rc1
  - Visual Studio 17 2022
  - vcpkg(2022.02.02-4297-g78b61582c)[qt5,lua,sol2,opencascade]

- Ubuntu 22.04.5 LTS(WSL)
  - CMake 3.31.0
  - c++ (Ubuntu 11.4.0-1ubuntu1~22.04)
  - vcpkg(2022.02.02-8233-g813a241fb)[qt5,lua,sol2,opencascade]

## 编译

依赖包安装

```bash
vcpkg install qt5:x64-windows
vcpkg install lua:x64-windows
vcpkg install sol2:x64-windows
vcpkg install opencascade:x64-windows
```

生成程序

```bash
cd (your_workspace)/JellyCAD
mkdir build
cd build
cmake .. . -DCMAKE_TOOLCHAIN_FILE=(your_vcpkg_dir)/scripts/buildsystems/vcpkg.cmake
cmake --build
```

## 使用

### 命令行模式

```bash
jellycad -f file.lua
```

### 图形界面模式

#### 鼠标控制3D界面

- 鼠标左键平移
- 鼠标右键旋转
- 鼠标滚轮缩放

#### 快捷键
- 新建:Ctrl+N
- 打开:Ctrl+O
- 保存:Ctrl+S

## 软件定义的类型和函数

### 全局函数

`show`在3D界面显示单个或多个模型，`export_stl`/`export_step`/`export_iges`分别导出对应格式的文件

### 形状实现类

以下8个形状均继承`Shape`基类

- `box`/`cylinder`/`cone`/`sphere`创建类型`SOLID`的三维模型
- `edge`创建边缘，支持`lin`(line),圆弧`circ`(circle),`elips`(ellipse),`hypr`(hyperbola),`parab`(parabola)类型的边缘
- `wire`创建线，`polygon`创建多边形的线
- `face`创建面

### 形状基类

基类`Shape`实现的方法：

- `type`返回形状类型，格式为字符串
- `fuse`融合
- `cut`剪切
- `common`共有部分
- `fillet`圆角
- `chamfer`平角
- `translate`相对平移
- `rotate`相对旋转
- `locate`位置和姿态定位
- `color`设置颜色
- `transparency`设置透明度
- `prism`拉伸操作
  - 拉伸形状变换：`edge->face,face->solid,wire->shell`

## 例程
例1：绘制实体，设置位姿和颜色
```lua
b = box.new(0.1, 1, 1);
b:translate(2, 2, 0);
b:rotate(0, 0, -30);
c = cylinder.new(1, 1, { color = "lightblue", pos = { 2, -2, 0 }, rx = 20 });
n = cone.new(1, 0.2, 2, { color = "#808080", pos = { -2, 2, 0 }, rot = { 90, 0, 0 } });
s = sphere.new(0.5);
s:translate(-2, -2, 0.5);
s:color("red");
show({b,c,n,s});
```

![example1](doc/example1.png)

## Feedback

Jelatine([lijianbinmail@163.com](mailto:lijianbinmail@163.com))

## 参考

> [JellyCAD old version](https://github.com/Jelatine/JellyCAD/tree/master)
>
> [OpenCascade 说明文档](https://dev.opencascade.org/doc/overview/html/index.html)
>
> [布尔运算](https://blog.csdn.net/weixin_45751713/article/details/139399875)
>
> [圆角倒角](https://blog.csdn.net/fcqwin/article/details/17204707)
>
> [平移旋转](https://blog.csdn.net/cfyouling/article/details/136400406)
>
> [访问拓扑边TopoDS_Edge的起末点](https://blog.csdn.net/s634772208/article/details/130101544)
>
> [判断Edge/Wire是直线还是圆弧(wire:BRepAdaptor_CompCurve,Edge:BRepAdaptor_Curve)](https://www.cnblogs.com/occi/p/14619592.html)
>
> [创建实体](https://developer.aliyun.com/article/235775)
>
> [fougue/mayo: 3D CAD viewer and converter based on Qt + OpenCascade](https://github.com/fougue/mayo)

