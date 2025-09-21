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

#### **`shape.new()`** - 基础形状类
```lua
shape.new(filename)  -- 从文件加载（支持 *.step, *.stl）
```

所有形状的基类，可从文件加载现有模型。

---

#### **`box.new()`** - 长方体
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

#### **`cylinder.new()`** - 圆柱体
```lua
cylinder.new()            -- 默认: r=h=1
cylinder.new(r, h)        -- 自定义尺寸
cylinder.new(other_cyl)   -- 复制构造
```
**参数：**
- `r` - *number* - 半径
- `h` - *number* - 高度

---

#### **`cone.new()`** - 圆锥/圆台
```lua
cone.new()                -- 默认: r1=r2=h=1
cone.new(r1, r2, h)       -- 自定义尺寸
cone.new(other_cone)      -- 复制构造
```
**参数：**
- `r1` - *number* - 底部半径
- `r2` - *number* - 顶部半径（r2=0 为圆锥）
- `h` - *number* - 高度

---

#### **`sphere.new()`** - 球体
```lua
sphere.new()              -- 默认: r=1
sphere.new(r)             -- 自定义半径
sphere.new(other_sphere)  -- 复制构造
```
**参数：**
- `r` - *number* - 半径

---

### 高级形状

#### **`edge.new()`** - 边缘
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

#### **`wire.new()`** - 线框
```lua
wire.new(list)         -- 从边缘列表创建
wire.new(other_wire)   -- 复制构造
```
**参数：**
- `list` - *table* - 边缘或线框对象列表 `{edge1, edge2, ...}`

---

#### **`polygon.new()`** - 多边形
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

#### **`face.new()`** - 面
```lua
face.new(shape_object)  -- 从线框/边缘/多边形创建面
face.new(other_face)    -- 复制构造
```
**参数：**
- `shape_object` - *shape* - wire、edge 或 polygon 对象

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
