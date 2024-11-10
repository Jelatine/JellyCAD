print("Hello, World!");
b = box.new(0.1, 1, 1); -- create a box with dimensions 0.1 x 1 x 1
b:translate(2, 2, 0); -- translate the box by 2 units in the x, y
b:rotate(0, 0, -30); -- rotate the box by -30 degrees around the z axis
-- create a cylinder with radius 1, height 1, color lightblue, position {2, -2, 0}, rotate 20 degrees around the x axis
c = cylinder.new(1, 1, { color = "lightblue", pos = { 2, -2, 0 }, rx = 20 });
-- create a cone with radius 1, height 0.2, color gray, position {-2, 2, 0}, roll 90 degrees(RPY)
n = cone.new(1, 0.2, 2, { color = "#808080", pos = { -2, 2, 0 }, rot = { 90, 0, 0 } });
s = sphere.new(0.5); -- create a sphere with radius 0.5
s:locate({pos = { -2, -2, 0.5 }, rot = { 0, 0, 0 }}); -- set the position and rotation of the sphere
s:color("red"); -- set the color of the sphere to red
show({b,c,n,s});  -- display the objects