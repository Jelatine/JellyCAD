b = box.new(0.1, 1, 1);
b:translate(2, 2, 0);
b:rotate(0, 0, -30);
c = cylinder.new(1, 1, { color = "lightblue", pos = { 2, -2, 0 }, rx = 20 });
n = cone.new(1, 0.2, 2, { color = "#808080", pos = { -2, 2, 0 }, rot = { 90, 0, 0 } });
s = sphere.new(0.5);
s:translate(-2, -2, 0.5);
s:color("red");
show({b,c,n,s}); 