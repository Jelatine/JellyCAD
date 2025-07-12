# 内置函数

## Global Functions

```lua
`show(object)`
`show(object_list) `
show the object in the scene
object: shape object - the shape to be shown
object_list: table - the list of shape objects to be shown{obj1, obj2,...}

`export_stl(filename)`
`export_stl(filename,option)`
export the shape to the stl file
filename: string - the path of the file to be exported
option: table - {type[string](ascii/binary), radian[number](The smaller the curvature, the more triangles there are)}

`export_step(filename)`
export the shape to the step file
filename: string - the path of the file to be exported

`export_iges(filename)`
export the shape to the iges file
filename: string - the path of the file to be exported
```

## Shape Objects

```lua
`shape.new(filename)`
Base class for all shape
filename: string - the path of the file to be loaded(support: *.step *.stl)

`box.new()`
`box.new(x, y, z)`
`box.new(x, y, z, options)`
`box.new(other_box_obj)`
solid box, default x=y=z=1
x/y/z: number - The diagonal of the box is from 0,0,0 to x, y, z
options: table - see Shape Options

`cylinder.new()`
`cylinder.new(r, h)`
`cylinder.new(r, h, options)`
`cylinder.new(other_cylinder_obj)`
solid cylinder, default r=h=1
r: number - radius
h: number - height
options: table - see Shape Options

`cone.new()`
`cone.new(r1, r2, h)`
`cone.new(r1, r2, h, options)`
`cone.new(other_cone_obj)`
solid cone, default r1=r2=h=1
r1: number - bottom radius
r2: number - top radius
h: number - height
options: table - see Shape Options

`sphere.new()`
`sphere.new(r)`
`sphere.new(r, options)`
`sphere.new(other_sphere_obj)`
solid sphere, default r=1
r: number - radius
options: table - see Shape Options

edge.new(type, vec1, vec2)
edge.new(type, vec1, vec2, r1)
edge.new(type, vec1, vec2, r1, r2)
edge.new(other_edge_obj)
--type: string - lin/circ/elips/hypr/parab
--vec1: table - {x[number],y[number],z[number]}, point3d, all used
--vec2: table - {x[number],y[number],z[number]}, dir3d, all used
--r1: number - radius, circ/elips/hypr/parab used
--r2: number - radius, elips/hypr used
--other_edge_obj: edge - clone a edge

`wire.new(list)`
`wire.new(other_wire_obj)`
list: table - {wire_or_edge_obj1, wire_or_edge_obj2, ...}
other_wire_obj: wire - clone a wire

`polygon.new(list_point_3d)`
`polygon.new(other_polygon_obj)`
A polygon wire
list_point_3d: table - {{x1[number],y1[number],z1[number]}, {x2,y2,z2}, {x3,y3,z3}, ...}
other_polygon_obj: polygon - clone a polygon

`face.new(shape_object)`
`face.new(other_face_obj)`
shape_object: shape - make wire, edge or polygon to face
other_face_obj: face - clone a face
```

### Shape Options

```lua
color: string - color name or hex string, such as 'red','#FF0000'
pos: table - position {x[number],y[number],z[number]}
rot: table - orientation {rx[number],ry[number],rz[number]}
rx/ry/rz: number - rotation angle in degrees
x/y/z: number - position
```

#### Color Names

```lua
color names: remove Quantity_NOC_ from the enum Quantity_NameOfColor
see https://dev.opencascade.org/doc/refman/html/_quantity___name_of_color_8hxx.html
`red, green, blue, yellow, cyan, magenta, black, white, gray, lightgray, ...`
```

### Shape Functions

```lua
`shape:type()`
return the type of the shape, such as 'vertex', 'edge', 'face', 'shell', 'wire', 'solid', 'compound'
shape:fuse(other_shape_object)
shape:cut(other_shape_object)
shape:common(other_shape_object)
`shape:fillet(radius[number], conditions[table])`
conditions: table - {type[string] `line / circle / ellipse / hyperbola / parabola / bezier_curve`, dir[string]`x / y / z`, min/max[table]`{x,y,z}`}
shape:chamfer(distance[number], conditions[table])
shape:translate(x[number], y[number], z[number])
shape:rotate(rx[number], ry[number], rz[number])
`shape:locate(pose[table])`
pose: table - {x[number],y[number],z[number], rx[number], ry[number], rz[number]} or {pos={x,y,z},rot={rx,ry,rz}}

`shape:color(name_or_hex[string])`
name_or_hex: string - see Color Names

`shape:transparency(value[number])`
value: number - 0.0-1.0
shape:prism(x[number], y[number], z[number])
```