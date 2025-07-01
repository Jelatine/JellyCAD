c=cylinder.new(12,10);
c:color('orange3');
c:cut(cylinder.new(10,10,{z=0.2}));
for deg=60,360,60 do
  rad=deg*math.pi/180;
  x0=6*math.sin(rad);
  y0=6*math.cos(rad);
  c:cut(cylinder.new(1,3,{pos={x0,y0,0}}));
end
show(c);
