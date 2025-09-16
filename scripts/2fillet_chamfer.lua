print("Fillet OR Chamfer");
b1=box.new(1,1,1):color('red3'):pos(2, 2, 0);
b1:fillet(0.2,{dir='z'}); -- 圆角 r=0.2 限制条件为边缘与基坐标系的Z重合

b2=box.new(1,1,1):color('green3'):pos(2, -2, 0);
b2:fillet(0.2,{max={3,3,3}}); -- 圆角 r=0.2 边缘始末点同时小于 3,3,3

c=cylinder.new(0.5,1):color('gray'):pos(-2, -2, 0);
c:fillet(0.2,{type='circle'}); -- 圆角 r=0.2 限制条件为边缘类型是圆形

b3=box.new(1,1,1):color('lightblue');
b3:chamfer(0.3,{min={0.5,-1,0.5},max={9,9,9}}); -- 倒角 r=0.3 边缘始末点同时大于 0.5,-1,0.5 且小于 9,9,9
show({b1,b2,b3,c});