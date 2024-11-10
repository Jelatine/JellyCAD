b1=box.new(1,1,1,{x=2,y=2});
b1:fillet(0.2,{dir='z'});
b1:show();

b2=box.new(1,1,1,{x=2,y=-2});
b2:fillet(0.2,{max={3,3,3}});
b2:show();

c=cylinder.new(0.5,1,{x=-2,y=-2});
c:fillet(0.2,{type='circle'});
c:show();

b3=box.new(1,1,1);
b3:chamfer(0.3,{min={0.5,-1,0.5},max={9,9,9}});
b3:show();