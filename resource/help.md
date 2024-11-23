<p>JellyCAD is a free and open-source CAD application, developed by Li Jianbin.</p>
<p>It is licensed under the MIT License.</p>
<p>The source code is available on <a href="https://github.com/Jelatine/JellyCAD">GitHub</a>.</p>
<hr>
<h1>Reference</h1>

<h2>Global Functions</h2>
<p>
    <code>show(object)</code><br>
    <code>show(object_list) </code><br>
    show the object in the scene<br>
    object: shape object - the shape to be shown<br>
    object_list: table - the list of shape objects to be shown{obj1, obj2,...}
</p>
<p>
    <code>export_stl(filename)</code><br>
    <code>export_stl(filename,option)</code><br>
    export the shape to the stl file<br>
    filename: string - the path of the file to be exported<br>
    option: table - {type[string](ascii/binary), radius[number]}
</p>
<p>
    <code>export_step(filename)</code><br>
    export the shape to the step file<br>
    filename: string - the path of the file to be exported<br>
</p>
<p>
    <code>export_iges(filename)</code><br>
    export the shape to the iges file<br>
    filename: string - the path of the file to be exported<br>
</p>

<h2>Shape Objects</h2>
<p>
    <code>shape.new(filename)</code><br>
    Base class for all shape<br>
    filename: string - the path of the file to be loaded(support: *.step *.stl)
</p>
<p>
    <code>box.new()</code><br>
    <code>box.new(x, y, z)</code><br>
    <code>box.new(x, y, z, options)</code><br>
    <code>box.new(other_box_obj)</code><br>
    solid box, default x=y=z=1<br>
    x/y/z: number - The diagonal of the box is from 0,0,0 to x, y, z<br>
    options: table - see Shape Options
</p>
<p>
    <code>cylinder.new()</code><br>
    <code>cylinder.new(r, h)</code><br>
    <code>cylinder.new(r, h, options)</code><br>
    <code>cylinder.new(other_cylinder_obj)</code><br>
    solid cylinder, default r=h=1<br>
    r: number - radius<br>
    h: number - height<br>
    options: table - see Shape Options
</p>
<p>
    <code>cone.new()</code><br>
    <code>cone.new(r1, r2, h)</code><br>
    <code>cone.new(r1, r2, h, options)</code><br>
    <code>cone.new(other_cone_obj)</code><br>
    solid cone, default r1=r2=h=1<br>
    r1: number - bottom radius<br>
    r2: number - top radius<br>
    h: number - height<br>
    options: table - see Shape Options
</p>
<p>
    <code>sphere.new()</code><br>
    <code>sphere.new(r)</code><br>
    <code>sphere.new(r, options)</code><br>
    <code>sphere.new(other_sphere_obj)</code><br>
    solid sphere, default r=1<br>
    r: number - radius<br>
    options: table - see Shape Options
</p>
<p>
    <code>edge.new(type, vec1, vec2)</code><br>
    <code>edge.new(type, vec1, vec2, r1)</code><br>
    <code>edge.new(type, vec1, vec2, r1, r2)</code><br>
    <code>edge.new(other_edge_obj)</code><br>
    type: string - lin/circ/elips/hypr/parab<br>
    vec1: table - {x[number],y[number],z[number]}, point3d, all used<br>
    vec2: table - {x[number],y[number],z[number]}, dir3d, all used<br>
    r1: number - radius, circ/elips/hypr/parab used<br>
    r2: number - radius, elips/hypr used<br>
    other_edge_obj: edge - clone a edge
</p>
<p>
    <code>wire.new(list)</code><br>
    <code>wire.new(other_wire_obj)</code><br>
    list: table - {wire_or_edge_obj1, wire_or_edge_obj2, ...}<br>
    other_wire_obj: wire - clone a wire
</p>
<p>
    <code>polygon.new(list_point_3d)</code><br>
    <code>polygon.new(other_polygon_obj)</code><br>
    A polygon wire<br>
    list_point_3d: table - {{x1[number],y1[number],z1[number]}, {x2,y2,z2}, {x3,y3,z3}, ...}<br>
    other_polygon_obj: polygon - clone a polygon
</p>
<p>
    <code>face.new(shape_object)</code><br>
    <code>face.new(other_face_obj)</code><br>
    shape_object: shape - make wire, edge or polygon to face<br>
    other_face_obj: face - clone a face
</p>

<h3>Shape Options</h3>
<p>
    color: string - color name or hex string, such as 'red','#FF0000'<br>
    pos: table - position {x[number],y[number],z[number]}<br>
    rot: table - orientation {rx[number],ry[number],rz[number]}<br>
    rx/ry/rz: number - rotation angle in degrees<br>
    x/y/z: number - position<br>
</p>
<h4>Color Names</h4>
<p>
    color names: remove Quantity_NOC_ from the enum Quantity_NameOfColor<br>
    see https://dev.opencascade.org/doc/refman/html/_quantity___name_of_color_8hxx.html<br>
    <code>red, green, blue, yellow, cyan, magenta, black, white, gray, lightgray, ...</code><br>
</p>

<h3>Shape Functions</h3>

<p>
    <code>shape:type()</code><br>
    return the type of the shape, such as 'vertex', 'edge', 'face', 'shell', 'wire', 'solid', 'compound'<br>
</p>
<p>
    <code>shape:fuse(other_shape_object)</code><br>
</p>
<p>
    <code>shape:cut(other_shape_object)</code><br>
</p>
<p>
    <code>shape:common(other_shape_object)</code><br>
</p>
<p>
    <code>shape:fillet(radius[number], conditions[table])</code><br>
    conditions: table - {type[string](line/circle/ellipse/hyperbola/parabola/bezier_curve), dir[string](x/y/z), min/max[table]({x,y,z})}<br>
</p>
<p>
    <code>shape:chamfer(distance[number], conditions[table])</code><br>
</p>
<p>
    <code>shape:translate(x[number], y[number], z[number])</code><br>
</p>
<p>
    <code>shape:rotate(rx[number], ry[number], rz[number])</code><br>
</p>
<p>
    <code>shape:locate(pose[table])</code><br>
    pose: table - {x[number],y[number],z[number], rx[number], ry[number], rz[number]} or {pos={x,y,z},rot={rx,ry,rz}}<br>
</p>
<p>
    <code>shape:color(name_or_hex[string])</code><br>
    name_or_hex: string - see Color Names<br>
</p>
<p>
    <code>shape:transparency(value[number])</code><br>
    value: number - 0.0-1.0<br>
</p>
<p>
    <code>shape:prism(x[number], y[number], z[number])</code><br>
</p>