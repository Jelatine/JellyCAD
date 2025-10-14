# 使用脚本

## 📚 目录

- [🌍 全局函数](#-全局函数)
  - [显示与导出](#显示与导出)
- [📦 形状对象](#-形状对象)
  - [基础形状](#基础形状)
  - [高级形状](#高级形状)
- [⚙️ 形状方法](#️-形状方法)
  - [基础操作](#基础操作)
  - [布尔运算](#布尔运算)
  - [几何变换](#几何变换)
  - [位置与姿态](#位置与姿态)
  - [外观设置](#外观设置)
- [🎨 颜色名称](#-颜色名称)

---

## 🌍 全局函数

### 显示与导出

#### **`show()`** - 显示对象
```lua
show(object)        -- 显示单个对象
show(object_list)   -- 显示对象列表
```
**参数：**
- `object` - *shape* - 要显示的形状对象
- `object_list` - *table* - 形状对象列表 `{obj1, obj2, ...}`

---

#### **`export_stl()`** - 导出 STL 文件
```lua
export_stl(filename)
export_stl(filename, option)
```
**参数：**
- `filename` - *string* - 导出文件路径
- `option` - *table* - 可选参数
  - `type` - *string* - 'ascii' 或 'binary'
  - `radian` - *number* - 曲率值（越小三角形越多）

**示例：**
```lua
export_stl("model.stl")
export_stl("model.stl", {type = "binary", radian = 0.01})
```

---

#### **`export_step()`** - 导出 STEP 文件
```lua
export_step(filename)
```
**参数：**
- `filename` - *string* - 导出文件路径

---

#### **`export_iges()`** - 导出 IGES 文件
```lua
export_iges(filename)
```
**参数：**
- `filename` - *string* - 导出文件路径

---

## 📦 形状对象

### 基础形状

#### **`shape`** - 基础形状类
```lua
shape.new(filename)  -- 从文件加载（支持 *.step, *.stl）
```

所有形状的基类，可从文件加载现有模型。

---

#### **`box`** - 长方体

创建一个从原点开始的长方体

```lua
box.new()              -- 默认: x=y=z=1，从(0,0,0)到(1,1,1)
box.new(x, y, z)       -- 自定义尺寸，从(0,0,0)到(x,y,z)
box.new(other_box)     -- 复制构造
```
**参数：**
- `x, y, z` - *number* - 盒子对角线从 (0,0,0) 到 (x,y,z)

**示例：**

```lua
box.new():show()                -- 单位立方体，从(0,0,0)到(1,1,1)
box.new(2, 1, 0.5):y(3):show()  -- 长方体，从(0,0,0)到(2,1,0.5)
-- 边长信息文本
text.new('x=2', 0.5):x(0.5):y(2.5):color('red'):show()
text.new('y=1', 0.5):x(2.5):y(3):rz(90):color('green'):show()
text.new('z=0.5', 0.2):x(0):y(3):rx(90):ry(-90):color('blue'):show()
```

<img src="../example_box.png" style="zoom: 33%;" />

---

#### **`cylinder`** - 圆柱体

创建一个原点在底部圆心的圆柱体

```lua
cylinder.new()            -- 默认: r=h=1
cylinder.new(r, h)        -- 自定义尺寸
cylinder.new(other_cyl)   -- 复制构造
```
**参数：**
- `r` - *number* - 半径
- `h` - *number* - 高度

**示例：**

```lua
cylinder.new():show()
cylinder.new(0.5, 2):y(3):show()
```

<img src="../example_cylinder.png" style="zoom: 33%;" />

---

#### **`cone`** - 圆锥/圆台

创建一个原点在底部圆心的圆圆锥/圆台

```lua
cone.new()                -- 默认: r1=1,r2=0,h=1
cone.new(r1, r2, h)       -- 自定义尺寸
cone.new(other_cone)      -- 复制构造
```
**参数：**

- `r1` - *number* - 底部半径
- `r2` - *number* - 顶部半径（r2=0 为圆锥）
- `h` - *number* - 高度

**示例：**

```lua
cone.new():show()
cone.new(1, 0.5, 2):y(3):show()
```

<img src="../example_cone.png" style="zoom: 33%;" />

---

#### **`sphere`** - 球体

创建一个原点在球心的球体

```lua
sphere.new()              -- 默认: r=1
sphere.new(r)             -- 自定义半径
sphere.new(other_sphere)  -- 复制构造
```
**参数：**
- `r` - *number* - 半径

#### **`torus`** - 圆环

创建一个原点在环心的圆环

```lua
torus.new()              -- 默认: R1=2,R2=1,angle=360
torus.new(R1, R2, angle)
torus.new(other_torus)
```

**参数：**

- `R1` - *number* - 从管道中心到环面中心的距离
- `R2` - *number* - 管道半径
- `angle` - *number* - 角度(deg)

**示例：**

```lua
torus.new(1, 0.5):show()
torus.new(1, 0.2, 180):y(3):show()
```

<img src="../example_torus.png" style="zoom: 33%;" />

#### **`wedge`** - 楔形

创建一个从原点开始的楔形

```lua
wedge.new()              -- 默认: dx=dy=dz=1,ltx=0
wedge.new(dx, dy, dz, ltx)
wedge.new(dx, dy, dz, xmin, zmin, xmax, zmax)
wedge.new(other_wedge)
```

**参数：**

- `dx, dy, dz` - *number* - 各个方向的长度
- `ltx` - *number* - 楔形中心到X轴的距离
- `xmin, zmin, xmax, zmax` - *number* - 面在`dy`的最大最小值



---

### 高级形状

#### **`edge`** - 边缘
```lua
edge.new(type, vec1, vec2)
edge.new(type, vec1, vec2, r1)
edge.new(type, vec1, vec2, r1, r2)
edge.new(other_edge)
```

**参数：**
- `type` - *string* - 边缘类型
  - `"lin"` - 直线
  - `"circ"` - 圆
  - `"elips"` - 椭圆
  - `"hypr"` - 双曲线
  - `"parab"` - 抛物线
- `vec1` - *table* - 3D点坐标 `{x, y, z}`
- `vec2` - *table* - 3D方向向量 `{x, y, z}`
- `r1` - *number* - 半径（circ/elips/hypr/parab 使用）
- `r2` - *number* - 第二半径（elips/hypr 使用）

---

#### **`wire`** - 线框
```lua
wire.new(list)         -- 从边缘列表创建
wire.new(other_wire)   -- 复制构造
```
**参数：**
- `list` - *table* - 边缘或线框对象列表 `{edge1, edge2, ...}`

---

#### **`polygon`** - 多边形
```lua
polygon.new(point_list)     -- 从点列表创建
polygon.new(other_polygon)  -- 复制构造
```
**参数：**
- `point_list` - *table* - 3D点列表 `{point1, point2, ...}`, 其中point:`{x,y,z}`

**示例：**

```lua
local triangle = polygon.new({
    {0, 0, 0},
    {1, 0, 0},
    {0.5, 1, 0}
})
```

---

#### **`face`** - 面
```lua
face.new(shape_object)  -- 从线框/边缘/多边形创建面
face.new(other_face)    -- 复制构造
```
**参数：**
- `shape_object` - *shape* - wire、edge 或 polygon 对象

#### **`text`** - 文本

```lua
text.new(str)
text.new(str, size)
```

**参数：**

- `str` - *string* - 要显示的文本内容
- `size` - *number* - 字体大小

**示例：**

```lua
text.new('hello', 1):x(2):show()
```

---

## ⚙️ 形状方法

### 基础操作

#### **`shape:type()`** - 获取类型
返回形状类型字符串：
- `"vertex"` - 顶点
- `"edge"` - 边
- `"face"` - 面
- `"shell"` - 壳
- `"wire"` - 线框
- `"solid"` - 实体
- `"compound"` - 复合体

#### **`shape:copy()`** - 复制形状
返回当前形状的副本。

---

### 布尔运算

#### **`shape:fuse()`** - 融合（并集）
```lua
result = shape1:fuse(shape2)
```

#### **`shape:cut()`** - 切割（差集）
```lua
result = shape1:cut(shape2)
```

#### **`shape:common()`** - 交集
```lua
result = shape1:common(shape2)
```

**示例：**
```lua
local box1 = box.new(2, 2, 2)
local box2 = box.new(1, 1, 1):pos(1, 1, 1)
local union = box1:copy():fuse(box2)      -- 并集
local diff = box1:copy():cut(box2)        -- 差集
local inter = box1:copy():common(box2)    -- 交集
```

---

### 几何变换

#### **`shape:fillet()`** - 圆角
```lua
shape:fillet(radius, conditions)
```
**参数：**
- `radius` - *number* - 圆角半径
- `conditions` - *table* - 条件参数
  - `type` - 边缘类型筛选(line/circle/ellipse/hyperbola/parabola/bezier_curve/bspline_curve/offset_curve/other_curve)
  - `dir` - 方向筛选 `{x, y, z}`
  - `min/max` - 位置范围 `{x, y, z}`

---

#### **`shape:chamfer()`** - 倒角
```lua
shape:chamfer(distance, conditions)
```
参数与 `fillet()` 类似。

---

#### **`shape:prism()`** - 拉伸
```lua
shape:prism(x, y, z)
```
沿指定方向拉伸形状。

---

#### **`shape:revol()`** - 旋转体
```lua
shape:revol(pos, dir, angle)
```
**参数：**
- `pos` - *array3* - 旋转轴位置点 `{x, y, z}`
- `dir` - *array3* - 旋转轴方向向量 `{x, y, z}`
- `angle` - *number* - 旋转角度（度）

**示例：**
```lua
local profile = polygon.new({{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0}})
local face = face.new(profile)
local solid = face:revol({0,0,0}, {0,0,1}, 360)  -- 绕Z轴旋转360度
```

---

### 位置与姿态

#### **单轴设置**
```lua
shape:x(value)   -- 设置X坐标
shape:y(value)   -- 设置Y坐标
shape:z(value)   -- 设置Z坐标
shape:rx(angle)  -- 绕X轴旋转（度）
shape:ry(angle)  -- 绕Y轴旋转（度）
shape:rz(angle)  -- 绕Z轴旋转（度）
```

#### **组合设置**
```lua
shape:pos(x, y, z)        -- 设置绝对位置
shape:rot(rx, ry, rz)     -- 设置绝对角度（度）
```

#### **相对移动**
```lua
shape:move("pos", x, y, z)  -- 相对平移
shape:move("rot", rx, ry, rz)  -- 相对旋转（度）
```

**示例：**
```lua
local obj = box.new()
obj:pos(10, 20, 30)           -- 移动到 (10, 20, 30)
obj:rot(0, 0, 45)             -- 绕Z轴旋转45度
obj:move("pos", 5, 0, 0)      -- 沿X轴移动5个单位
obj:move("rot", 0, 90, 0)     -- 绕Y轴再旋转90度
```

---

### 外观设置

#### **`shape:color()`** - 设置颜色
```lua
shape:color(name_or_hex)
```
**参数：**

- `name_or_hex` - *string* - 颜色名称或十六进制值

**示例：**
```lua
shape:color("red")
shape:color("#FF5733")
```

#### **`shape:transparency()`** - 设置透明度
```lua
shape:transparency(value)
```
**参数：**
- `value` - *number* - 透明度值 (0.0 ~ 1.0)
  - `0.0` - 完全不透明
  - `1.0` - 完全透明

---

## 🎨 颜色名称

颜色名称参考 OpenCASCADE 的 `Quantity_NameOfColor` 枚举（移除 `Quantity_NOC_` 前缀）。

### 基础颜色
| 颜色名 | 说明 | 示例值 |
|--------|------|--------|
| `red` | 红色 | #FF0000 |
| `green` | 绿色 | #00FF00 |
| `blue` | 蓝色 | #0000FF |
| `yellow` | 黄色 | #FFFF00 |
| `cyan` | 青色 | #00FFFF |
| `magenta` | 洋红 | #FF00FF |
| `black` | 黑色 | #000000 |
| `white` | 白色 | #FFFFFF |
| `gray` | 灰色 | #808080 |
| `lightgray` | 浅灰 | #D3D3D3 |

> 📖 **更多颜色：** 完整颜色列表请参考 [OpenCASCADE 文档](https://dev.opencascade.org/doc/refman/html/_quantity___name_of_color_8hxx.html)

---

## 📍坐标系

#### **`axes`** - 坐标系类

```lua
axes.new(pose, length)
```

**说明：**

用于URDF导出时，配置关节`joint`位姿

**参数：**

- `pose` - *array* - 位姿数组，6个数据分别为位置`x,y,z`和RPY姿态`rx,ry,rz`(角度)
- `length` - number - 所有坐标轴长度(用于显示效果)

**方法：**

- `show()` - 显示坐标系到界面中

**示例：**

```lua
j1 = axes.new({ 0, 0, 2.5, 90, 0, 0 }, 3)
j1:show()
```

## 🤖URDF导出

有两个相关类`link`和`joint`

#### **`link`** - 连杆类

```lua
link.new(name, shape)
link.new(name, shape_list)
```

**参数：**

- `name` - *string* - 连杆名称
- `shape` - shape - 形状
- `shape_list` - table - 形状列表

**方法：**

- `add(j)` - `j`参数为`joint`对象，增加指定关节到连杆中
- `export(params)` - 生成一个ROS2的URDF包，`params`参数是一个`table`，内容为{name， path}，name[string]为机器人名称，path[string]为导出路径

#### **`joint`** - 关节类

```lua
link.new(name, axes, type, limits)
```

**参数：**

- `name` - *string* - 关节名称
- `axes` - *axes* - 坐标系
- `type` - *string* - 关节类型：`fixed, revolute, continuous, prismatic, floating, planar`
- `limits` - *table* - 关节限制：`lower, upper, effort, velocity`，全为number

**示例：**

```lua
-- 6自由度机械臂URDF建模及URDF导出示例
-- 通用
local r_shell = 32;
local h_motor = 90;
local offset = h_motor / 2 - r_shell
shell = cylinder.new(r_shell, h_motor)
shell:fillet(5, { type = 'circle', min = { r_shell - 1e-2, -1e-2, h_motor - 1e-2 } });
shell:fuse(cylinder.new(r_shell, h_motor / 2):z(h_motor / 2):rx(90)); -- 电机外壳
-- 生成连接柱
function get_pole(r_outer, r2, h)
    local r1 = r_outer - 1
    local h_stair = 2
    local h_cylinder = r1 - r2 + h_stair
    local stair = cylinder.new(r1, h_cylinder):cut(torus.new(r1, r1 - r2):pos(0, 0, h_cylinder))
    stair:fillet(1, { type = 'circle', min = { r1 - 1e-2, -1e-2, h_stair - 1e-2 } }); -- 底部圆柱滑梯台
    local stair_top = stair:copy():rx(180):z(h) -- 顶部圆柱滑梯台
    local pole = cylinder.new(r2, h):fuse(stair):fuse(stair_top)
    return pole
end
-- 基座
local r_base = 50;
local h_base = 35;
base_link = cylinder.new(r_base, h_base);
local R1 = r_base - r_shell
local R0 = R1 + 2
elips = edge.new('elips', { R1 + r_shell, 0, h_base }, { 0, 1, 0 }, R0, R1);
ellipse = face.new(elips);
ellipse:revol({ 0, 0, 0 }, { 0, 0, 1 }, 360)
base_link:cut(ellipse);
base_link:fillet(3, { type = 'bspline_curve', min = { r_base - 1e-2, -1e-2, (h_base - R0) - 1e-2 } });
-- 肩部
sholder = shell:copy():z(h_base);
-- 上臂
local h_upperarm = 150
local r_upperarm = 20
local z_upperarm = h_base + h_motor / 2
upperarm = shell:copy();
upperarm:rot(90, 180, 0);
upperarm:fuse(get_pole(r_shell, r_upperarm, h_upperarm):pos(0, -h_motor / 2, h_motor / 2))
upperarm:fuse(shell:copy():rot(90, 0, 0):pos(0, 0, h_motor + h_upperarm))
upperarm:pos(0, -h_motor / 2, z_upperarm)
-- 前臂
local h_forearm = 120
local r_forearm = 20
local z_forearm = h_base + h_upperarm + r_shell + h_motor + offset
forearm = face.new(edge.new('circ', { 0, 0, 0 }, { 0, 0, 1 }, r_shell));
forearm:revol({ 0, -r_shell, 0 }, { 1, 0, 0 }, -90)
forearm:fuse(get_pole(r_shell, r_forearm, h_forearm))
forearm:fuse(shell:copy():rot(90, 0, 180):pos(0, -h_motor / 2, h_motor / 2 + h_forearm))
forearm:pos(0, -offset, z_forearm + r_shell)
-- 手腕1
local z_wrist1 = z_forearm + h_forearm + h_motor + r_shell
wrist1 = shell:copy():rot(180, 0, 0):pos(0, -h_motor - offset, z_forearm + h_forearm + h_motor + r_shell)
-- 手腕2
local z_wrist2 = z_forearm + h_forearm + 2 * h_motor - offset
wrist2 = shell:copy():rot(90, 0, 0):pos(0, -h_motor / 2 - offset, z_wrist2)
-- 手腕3
local h_flank = 10
wrist3 = cylinder.new(r_shell, h_flank):rot(90, 0, 0):pos(0, -h_motor / 2 - offset + h_flank, z_wrist2)
-- 毫米单位转为米，生成URDF，设置连杆质量
base_link:scale(1e-3):color('#6495ED'):mass(0.1)
sholder:scale(1e-3):color('#8470FF'):mass(0.4)
upperarm:scale(1e-3):color('#FFC1C1'):mass(1)
forearm:scale(1e-3):color('#FFC100'):mass(0.8)
wrist1:scale(1e-3):color('#FF8247'):mass(0.4)
wrist2:scale(1e-3):color('#FFE7BA'):mass(0.4)
wrist3:scale(1e-3):color('#C1CDC1'):mass(0.1)
-- 设置关节坐标轴位置
joint_axes1 = axes.new({ 0, 0, z_upperarm * 1e-3, 0, 0, 0 }, 0.1)
joint_axes2 = axes.new({ 0, -h_motor * 1e-3, z_upperarm * 1e-3, 90, 0, 0 }, 0.1)
joint_axes3 = axes.new({ 0, -h_motor * 1e-3, z_forearm * 1e-3, 90, 0, 0 }, 0.1)
joint_axes4 = axes.new({ 0, -offset * 1e-3, (z_wrist1 - h_motor / 2) * 1e-3, 90, 0, 0 }, 0.2)
joint_axes5 = axes.new({ 0, -(h_motor + offset) * 1e-3, (z_wrist1 - h_motor / 2) * 1e-3, 0, 0, 0 }, 0.2)
joint_axes6 = axes.new({ 0, -(h_motor + offset) * 1e-3, (z_wrist2) * 1e-3, -90, 0, 0 }, 0.1)
-- 生成关节
joint1 = joint.new("joint1", joint_axes1, "revolute")
joint2 = joint.new("joint2", joint_axes2, "revolute")
joint3 = joint.new("joint3", joint_axes3, "revolute")
joint4 = joint.new("joint4", joint_axes4, "revolute")
joint5 = joint.new("joint5", joint_axes5, "revolute")
joint6 = joint.new("joint6", joint_axes6, "revolute")
-- 生成连杆
urdf = link.new("base_link", base_link)
link1 = link.new("link1", sholder)
link2 = link.new("link2", upperarm)
link3 = link.new("link3", forearm)
link4 = link.new("link4", wrist1)
link5 = link.new("link5", wrist2)
link6 = link.new("link6", wrist3)
-- 配置URDF
urdf:add(joint1):next(link1):add(joint2):next(link2):add(joint3):next(link3):add(joint4):next(link4):add(joint5):next(
    link5):add(joint6):next(link6)
-- 显示模型和坐标轴
show({ base_link, sholder, upperarm, forearm, wrist1, wrist2, wrist3 })
show({ joint_axes1, joint_axes2, joint_axes3, joint_axes4, joint_axes5, joint_axes6 })
-- 导出URDF
urdf:export({ name = 'myrobot', path = 'd:/' })
```

<img src="../example_urdf.png" alt="example_urdf" style="zoom:33%;" />

ROS2使用方法

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-urdf-launch
mkdir -p ~/ws_ros2/src
cp -r /mnt/d/myrobot ~/ws_ros2/src/
cd ~/ws_ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch urdf_launch display.launch.py urdf_package:=myrobot urdf_package_path:=urdf/myrobot.urdf
```

## 💡 使用示例

### 创建简单模型
```lua
-- 创建一个带圆角的盒子
local mybox = box.new(10, 10, 5)
mybox:fillet(1, {})
mybox:color("blue")
mybox:transparency(0.3)
show(mybox)

-- 导出模型
export_stl(mybox, "rounded_box.stl")
```

### 布尔运算示例
```lua
-- 创建一个开孔的立方体
local cube = box.new(10, 10, 10)
local hole = cylinder.new(3, 12):pos(5, 5, -1)
local result = cube:cut(hole)
result:color("green")
show(result)
```

### 创建旋转体
```lua
-- 创建一个花瓶轮廓
local profile = polygon.new({
    {0, 0, 0},
    {3, 0, 0},
    {4, 2, 0},
    {3.5, 5, 0},
    {4, 8, 0},
    {0, 8, 0}
})

-- 旋转生成花瓶
local vase = face.new(profile):revol({0,0,0}, {0,1,0}, 360)
vase:color("magenta")
vase:transparency(0.2)
show(vase)
```

---
