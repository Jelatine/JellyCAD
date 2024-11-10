p = polygon.new({ { 0, 0, 0 }, { 0, 1, 1 }, { 1, 1, 1 }, { 1, 0, 0 } });
p:color("#FFF")
show(p);
f = face.new(p);
f:prism(0, 0, 1);
show(f);