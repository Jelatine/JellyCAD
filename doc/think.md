# 思考

## 活动栏

:black_square_button:目录：浏览打开的目录内所有`*.lua`文件

:white_check_mark:编辑器：提供`lua`脚本编辑框，实现语法高亮、代码提示、:black_square_button:语法检测；提供新建和保存按钮；(?:考虑保存时运行脚本还是多一个运行按钮；是否要实现调试功能)

:white_check_mark:终端：显示脚本运行历史结果，以及单行输入脚本测试的功能

:black_square_button:代码管理：使用git查看当前打开目录仓库的修改

:black_square_button:设置：外观主题，更新检测

:white_check_mark:帮助：版本信息，第三方依赖，版权信息及脚本参考文档



## 脚本内容

?:思考模型是否要有名称

模型的通用方法：

```lua
x(value) -- 设置x绝对位置value(number)
y(value) -- 设置y绝对位置value(number)
z(value) -- 设置z绝对位置value(number)
rx(deg) -- 设置绕x轴旋转绝对角度deg(number)
ry(deg) -- 设置绕y轴旋转绝对角度deg(number)
rz(deg) -- 设置绕z轴旋转绝对角度deg(number)
pos(x,y,z) -- 设置绝对位置x(number),y(number),z(number)
rot(rx,ry,rz) -- 设置旋转绝对角度rx(number),ry(number),rz(number)
fuse(shape) -- Union（并集）合并shape [BRepAlgoAPI_Fuse]
cut(shape) -- Subtraction（差集）删除与shape相交部分 [BRepAlgoAPI_Cut]
common(shape) -- Intersection（交集）只保留与shape相交部分 [BRepAlgoAPI_Common]
prism(x,y,z) -- 拉伸 输入x(number),y(number),z(number)表示各方向拉伸距离 [BRepPrimAPI_MakePrism]
fillet(radius,table) -- 倒圆 输入radius(number),conditions [BRepFilletAPI_MakeFillet]
-- conditions: table - {type[string](line/circle/ellipse/hyperbola/parabola/bezier_curve), dir[string](x/y/z), min/max[table]({x,y,z})}
chamfer(distance,table) -- 倒角 输入distance(number),conditions [BRepFilletAPI_MakeChamfer]
```

全局方法：

```lua
show(shape) -- 显示一个形状
show(list_shape)  -- 显示多个形状
export_stl(filename) -- 导出stl文件到filename(string)文件
export_stl(filename,option) -- option(table): {type[string](ascii/binary), radian[number](The smaller the curvature, the more triangles there are)}
export_step(filename) -- 导出step文件到filename(string)文件
export_iges(filename) -- 导出iges文件到filename(string)文件
```



