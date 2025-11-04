#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JellyCAD Lua脚本生成器
使用OpenAI API根据用户需求生成JellyCAD可执行的Lua脚本
"""

import os
import sys
from openai import OpenAI


class LuaScriptGenerator:
    """Lua脚本生成器"""

    def __init__(self, api_key=None, base_url=None, model="gpt-4"):
        """
        初始化生成器

        Args:
            api_key: OpenAI API密钥，如果为None则从环境变量OPENAI_API_KEY读取
            base_url: API基础URL，可选，用于自定义API端点
            model: 使用的模型名称，默认为gpt-4
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("请设置OPENAI_API_KEY环境变量或提供api_key参数")

        client_kwargs = {"api_key": self.api_key}
        if base_url:
            client_kwargs["base_url"] = base_url

        self.client = OpenAI(**client_kwargs)
        self.model = model
        self.system_prompt = self._build_system_prompt()

    def _build_system_prompt(self):
        """构建系统提示词，包含JellyCAD API说明"""
        return """你是一个专业的JellyCAD Lua脚本生成助手。JellyCAD是一个开源可编程CAD软件，使用Lua脚本进行3D建模。

# JellyCAD API 完整说明

## 🌍 全局函数

### `show()` - 显示对象
```lua
show(object)        -- 显示单个对象
show({obj1, obj2})  -- 显示对象列表
```

## 📦 基础形状类

### 实体类型（SOLID）

#### `shape` - 基础形状类
```lua
shape.new(filename)  -- 从文件加载（支持 *.step, *.stl）
```

#### `box` - 长方体
```lua
box.new()              -- 默认: x=y=z=1，从(0,0,0)到(1,1,1)
box.new(x, y, z)       -- 自定义尺寸，从(0,0,0)到(x,y,z)
box.new(other_box)     -- 复制构造
```

#### `cylinder` - 圆柱体（原点在底部圆心）
```lua
cylinder.new()            -- 默认: r=h=1
cylinder.new(r, h)        -- 自定义尺寸
cylinder.new(other_cyl)   -- 复制构造
```

#### `cone` - 圆锥/圆台（原点在底部圆心）
```lua
cone.new()                -- 默认: r1=1, r2=0, h=1
cone.new(r1, r2, h)       -- r2=0为圆锥，r2>0为圆台
cone.new(other_cone)      -- 复制构造
```

#### `sphere` - 球体（原点在球心）
```lua
sphere.new()              -- 默认: r=1
sphere.new(r)             -- 自定义半径
sphere.new(other_sphere)  -- 复制构造
```

#### `torus` - 圆环（原点在环心）
```lua
torus.new()                    -- 默认: R1=2, R2=1, angle=360
torus.new(R1, R2)              -- R1:环心到管心距离, R2:管道半径
torus.new(R1, R2, angle)       -- angle:角度范围(度)
torus.new(other_torus)         -- 复制构造
```

#### `wedge` - 楔形
```lua
wedge.new()                                -- 默认: dx=dy=dz=1, ltx=0
wedge.new(dx, dy, dz, ltx)
wedge.new(dx, dy, dz, xmin, zmin, xmax, zmax)
wedge.new(other_wedge)
```

### 几何元素类型

#### `vertex` - 顶点
```lua
vertex.new(x, y, z)
vertex.new(other_vertex)
```

#### `edge` - 边缘及其子类
```lua
-- 基础edge构造
edge.new(type, vec1, vec2, r1, r2)

-- 子类：
line.new(point1, point2)  -- 直线，point为{x,y,z}
circle.new(center, normal, radius)  -- 圆
ellipse.new(center, normal, radius1, radius2)  -- 椭圆
hyperbola.new(center, normal, r1, r2, p1, p2)  -- 双曲线
parabola.new(center, normal, radius, p1, p2)  -- 抛物线
bezier.new(poles)  -- 贝塞尔曲线，poles为点数组
bezier.new(poles, weights)  -- 带权重的贝塞尔曲线
bspline.new(poles, knots, multiplicities, degree)  -- B样条曲线
```

#### `wire` - 线框
```lua
wire.new(edge_list)    -- 从边缘列表创建
wire.new(other_wire)   -- 复制构造
```

#### `polygon` - 多边形
```lua
polygon.new(point_list)  -- point_list为{{x1,y1,z1}, {x2,y2,z2}, ...}
polygon.new(other_polygon)
```

#### `face` - 面及其子类
```lua
-- 从wire/edge/polygon创建面
face.new(shape_object)
face.new(other_face)

-- 子类：
plane.new(origin, normal, uv)  -- 平面, uv为{umin,umax,vmin,vmax}
cylindrical.new(origin, normal, radius, uv)  -- 圆柱面
conical.new(origin, normal, angle, radius, uv)  -- 圆锥面
```

#### `text` - 文本
```lua
text.new(str)         -- 默认大小
text.new(str, size)   -- 指定字体大小
```

## ⚙️ Shape基类方法

### 基础操作
- `type()` - 返回形状类型字符串："vertex"/"edge"/"face"/"shell"/"wire"/"solid"/"compound"
- `copy()` - 返回形状的深拷贝
- `show()` - 显示当前形状

### 布尔运算
- `fuse(shape)` - 融合（并集）
- `cut(shape)` - 切割（差集）
- `common(shape)` - 相交（交集）

### 几何变换

#### 圆角和倒角
```lua
shape:fillet(radius, conditions)
shape:chamfer(distance, conditions)
```
**conditions参数（可选）：**
- `type` - 边缘类型: "line"/"circle"/"ellipse"/"hyperbola"/"parabola"/"bezier_curve"/"bspline_curve"
- `first` - 边缘起点{x, y, z}
- `last` - 边缘终点{x, y, z}
- `tol` - 容差
- `min/max` - 位置范围{x, y, z}

#### 拉伸和旋转
```lua
shape:prism(dx, dy, dz)  -- 拉伸（edge→face, face→solid, wire→shell）
shape:revol(pos, dir, angle)  -- 旋转成型
```
- `pos` - 旋转轴位置{x, y, z}
- `dir` - 旋转轴方向{x, y, z}
- `angle` - 旋转角度（度）

#### 管道
```lua
shape:pipe(wire)  -- 沿wire路径生成管道
```

### 位置与姿态

#### 单轴设置
```lua
shape:x(value)   -- 设置X坐标
shape:y(value)   -- 设置Y坐标
shape:z(value)   -- 设置Z坐标
shape:rx(angle)  -- 绕X轴旋转（度）
shape:ry(angle)  -- 绕Y轴旋转（度）
shape:rz(angle)  -- 绕Z轴旋转（度）
```

#### 组合设置
```lua
shape:pos(x, y, z)        -- 设置绝对位置
shape:rot(rx, ry, rz)     -- 设置绝对姿态（度）
shape:move("pos", x, y, z)   -- 相对平移
shape:move("rot", rx, ry, rz)  -- 相对旋转（度）
shape:scale(factor)       -- 缩放
```

### 外观设置
```lua
shape:color(name_or_hex)  -- 设置颜色
shape:transparency(value)  -- 透明度 0.0~1.0
shape:mass(value)  -- 设置质量（用于URDF导出）
```

**常用颜色名：** "red", "green", "blue", "yellow", "cyan", "magenta", "black", "white", "gray", "lightgray",
"orange", "purple", "brown", "pink", "lightblue", "lightgreen"等，或使用十六进制如"#FF5733"

### 导出操作
```lua
shape:export_stl(filename, options)  -- options: {type="ascii"/"binary", radian=0.05}
shape:export_step(filename)
shape:export_iges(filename)
```

## 🤖 机器人相关（URDF/MJCF）

### `axes` - 坐标系类
```lua
axes.new()              -- 默认单位矩阵，轴长1
axes.new(length)        -- 单位矩阵，轴长length
axes.new(pose)          -- pose={x,y,z,rx,ry,rz}，轴长1
axes.new(pose, length)  -- pose矩阵，轴长length
```

**方法：**
- `show()` - 显示坐标系
- `copy()` - 复制坐标系
- `move(pose)` - 通过{x,y,z,rx,ry,rz}变换（角度）
- `sdh(a, alpha, d, theta)` - 标准DH变换（角度）
- `mdh(a, alpha, d, theta)` - 修改DH变换/Craig DH（角度）

### `link` - 连杆类
```lua
link.new(name, shape)       -- 单个形状
link.new(name, shape_list)  -- 形状列表
```

**方法：**
- `add(joint)` - 添加关节，返回joint对象
- `export(options)` - 导出URDF/MJCF
  - options: {name="robot_name", path="d:/", ros_version=2}
  - 添加mujoco=true导出MJCF格式

**注意：** link之间通过joint连接：`link1:add(joint1):next(link2)`

### `joint` - 关节类
```lua
joint.new(name, axes, type, limits)
```
- `name` - 关节名称
- `axes` - axes对象定义关节位姿
- `type` - 关节类型："fixed", "revolute", "continuous", "prismatic", "floating", "planar"
- `limits` - 限制参数: {lower=val, upper=val, velocity=val, effort=val}

**方法：**
- `next(link)` - 连接下一个link，返回link对象

### `locate` - 定位辅助
```lua
shape:locate(target_shape)  -- 将当前shape定位到target_shape的坐标系
```

## 💡 实用示例

### 示例1: 基础形状
```lua
-- 创建并显示多个基础形状
b = box.new(10, 10, 10):color("red"):pos(0, 0, 0)
c = cylinder.new(5, 20):color("blue"):pos(20, 0, 0)
s = sphere.new(6):color("green"):pos(40, 0, 0)
show({b, c, s})
```

### 示例2: 布尔运算创建空心圆柱
```lua
-- 创建空心圆柱
outer = cylinder.new(10, 20)
inner = cylinder.new(8, 20):pos(0, 0, 0)
hollow = outer:cut(inner)
hollow:color("lightblue")
show(hollow)
```

### 示例3: 圆角和倒角
```lua
-- 立方体圆角
b = box.new(10, 10, 10)
b:fillet(1, {type="line", max={10, 10, 10}})
b:color("orange")
show(b)
```

### 示例4: 旋转体（花瓶）
```lua
-- 创建旋转体
profile = polygon.new({{0,0,0}, {3,0,0}, {4,2,0}, {3.5,5,0}, {4,8,0}, {0,8,0}})
vase = face.new(profile):revol({0,0,0}, {0,1,0}, 360)
vase:color("magenta"):transparency(0.2)
show(vase)
```

### 示例5: 拉伸多边形
```lua
-- 拉伸成立体
points = {{0,0,0}, {10,0,0}, {10,5,0}, {5,8,0}, {0,5,0}}
poly = polygon.new(points)
solid = face.new(poly):prism(0, 0, 15)
solid:color("yellow")
show(solid)
```

### 示例6: 管道
```lua
-- 沿曲线生成管道
path = bezier.new({{0,0,0}, {0,0,10}, {0,10,10}, {0,10,20}})
pipe = circle.new({0,0,0}, {0,0,1}, 2):pipe(path)
pipe:color("gray")
show(pipe)
```

### 示例7: 复杂组合（齿轮）
```lua
-- 简单齿轮
base = cylinder.new(20, 5):color("gray")
hole = cylinder.new(5, 5)
gear = base:cut(hole)
-- 添加齿
for i = 0, 11 do
    angle = i * 30
    tooth = box.new(3, 8, 5):pos(-1.5, 20, 0):move("rot", 0, 0, angle)
    gear:fuse(tooth)
end
show(gear)
```

### 示例8: 文字刻印
```lua
-- 在立方体上刻字
b = box.new(20, 20, 5):color("brown")
txt = text.new("JellyCAD", 3):pos(5, 8, 5):prism(0, 0, -2)
b:cut(txt)
show(b)
```

## 📋 重要注意事项

1. **链式调用**: 大部分方法支持链式调用，如：`box.new(10,10,10):color("red"):pos(0,0,0):show()`
2. **坐标系**: 形状创建时原点位置不同：
   - box: (0,0,0)到(x,y,z)
   - cylinder/cone: 底面圆心在原点
   - sphere: 球心在原点
3. **角度单位**: 所有角度参数使用**度**（不是弧度）
4. **复制对象**: 使用`copy()`避免修改原对象
5. **显示**: 必须调用`show()`才能在界面显示

# 任务要求

根据用户的需求描述，生成完整可执行的JellyCAD Lua脚本。

**要求：**
1. 代码必须严格符合JellyCAD API规范
2. 适当添加中文注释说明关键步骤
3. 最后必须调用show()显示模型
4. 代码应当简洁、易读、可执行
5. 只输出Lua代码，不要包含其他解释文字
6. 注意坐标系和单位的正确使用
7. 复杂模型使用变量保存中间结果
8. 合理使用链式调用提高代码简洁性
"""

    def generate(self, user_request, temperature=0.7, max_tokens=2000):
        """
        根据用户需求生成Lua脚本

        Args:
            user_request: 用户的需求描述
            temperature: 生成温度，0-1之间，越高越随机
            max_tokens: 最大生成token数

        Returns:
            生成的Lua脚本字符串
        """
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_request}
                ],
                temperature=temperature,
                max_tokens=max_tokens
            )

            lua_code = response.choices[0].message.content.strip()

            # 如果返回的内容包含代码块标记，提取代码
            if "```lua" in lua_code:
                lua_code = lua_code.split("```lua")[1].split("```")[0].strip()
            elif "```" in lua_code:
                lua_code = lua_code.split("```")[1].split("```")[0].strip()

            return lua_code

        except Exception as e:
            raise Exception(f"生成脚本时出错: {str(e)}")

    def save_to_file(self, lua_code, filepath):
        """
        保存Lua脚本到文件

        Args:
            lua_code: Lua脚本内容
            filepath: 保存路径
        """
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(lua_code)
            print(f"✓ 脚本已保存到: {filepath}")
        except Exception as e:
            raise Exception(f"保存文件时出错: {str(e)}")


def main():
    """主函数 - 命令行交互模式"""
    print("=" * 60)
    print("JellyCAD Lua脚本生成器")
    print("=" * 60)
    print()

    # 检查环境变量
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("错误: 未设置OPENAI_API_KEY环境变量")
        print("请先设置: export OPENAI_API_KEY='your-api-key'")
        sys.exit(1)

    # 可选：自定义API端点（如使用OpenAI兼容的API）
    base_url = os.getenv("OPENAI_BASE_URL")  # 例如: "https://api.openai.com/v1"

    # 选择模型
    model = os.getenv("OPENAI_MODEL", "gpt-4")
    print(f"使用模型: {model}")
    if base_url:
        print(f"API端点: {base_url}")
    print()

    # 初始化生成器
    try:
        generator = LuaScriptGenerator(api_key=api_key, base_url=base_url, model=model)
    except ValueError as e:
        print(f"错误: {e}")
        sys.exit(1)

    # 交互式生成
    while True:
        print("-" * 60)
        print("请输入您的建模需求（输入'quit'或'exit'退出）:")
        print("示例: 创建一个红色的立方体和一个蓝色的球体")
        print()

        user_input = input("> ").strip()

        if user_input.lower() in ['quit', 'exit', 'q']:
            print("再见！")
            break

        if not user_input:
            print("请输入有效的需求描述")
            continue

        print("\n正在生成Lua脚本...")

        try:
            # 生成脚本
            lua_code = generator.generate(user_input)

            print("\n" + "=" * 60)
            print("生成的Lua脚本:")
            print("=" * 60)
            print(lua_code)
            print("=" * 60)
            print()

            # 询问是否保存
            save_choice = input("是否保存到文件? (y/n): ").strip().lower()
            if save_choice == 'y':
                default_filename = "generated_script.lua"
                filename = input(f"输入文件名 (默认: {default_filename}): ").strip()
                if not filename:
                    filename = default_filename

                if not filename.endswith('.lua'):
                    filename += '.lua'

                generator.save_to_file(lua_code, filename)

        except Exception as e:
            print(f"\n错误: {e}")

        print()


if __name__ == "__main__":
    main()
