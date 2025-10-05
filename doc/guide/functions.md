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
```lua
box.new()              -- 默认: x=y=z=1
box.new(x, y, z)       -- 自定义尺寸
box.new(other_box)     -- 复制构造
```
**参数：**
- `x, y, z` - *number* - 盒子对角线从 (0,0,0) 到 (x,y,z)

**示例：**
```lua
local cube = box.new()           -- 单位立方体
local rect = box.new(2, 1, 0.5)  -- 长方体
```

---

#### **`cylinder`** - 圆柱体
```lua
cylinder.new()            -- 默认: r=h=1
cylinder.new(r, h)        -- 自定义尺寸
cylinder.new(other_cyl)   -- 复制构造
```
**参数：**
- `r` - *number* - 半径
- `h` - *number* - 高度

---

#### **`cone`** - 圆锥/圆台
```lua
cone.new()                -- 默认: r1=1,r2=0,h=1
cone.new(r1, r2, h)       -- 自定义尺寸
cone.new(other_cone)      -- 复制构造
```
**参数：**

- `r1` - *number* - 底部半径
- `r2` - *number* - 顶部半径（r2=0 为圆锥）
- `h` - *number* - 高度

---

#### **`sphere`** - 球体
```lua
sphere.new()              -- 默认: r=1
sphere.new(r)             -- 自定义半径
sphere.new(other_sphere)  -- 复制构造
```
**参数：**
- `r` - *number* - 半径

#### **`torus`** - 圆环

```lua
torus.new()              -- 默认: R1=2,R2=1,angle=360
torus.new(R1, R2, angle)
torus.new(other_torus)
```

**参数：**

- `R1` - *number* - 从管道中心到环面中心的距离
- `R2` - *number* - 管道半径
- `angle` - *number* - 角度(deg)

#### **`wedge`** - 楔形

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
bx = box.new():color('green')
b1 = cylinder.new()
link1 = b1:copy():pos(0, 0, 2):color('#456789')
link2 = b1:copy():pos(0, 0, 3):color('#987654')
j1 = axes.new({ 0, 0, 2.5, 90, 0, 0 }, 3)
j2 = axes.new({ 0, 0, 3.5, 90, 0, 0 }, 3)
j1:show()
j2:show()
bx:mass(1) -- 组合形状时可以设置各自的质量
b1:mass(2)
show({bx,b1,link1,link2})
urdf = link.new("base", { bx, b1 })
urdf:add("joint1", j1, "revolute"):next('link1', link1):add("joint2", j2, "revolute"):next('link2', link2)
urdf:export({name='myrobot', path="d:/"})
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
