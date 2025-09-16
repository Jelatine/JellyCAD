print("Export");
cy=cylinder.new(10,10);
s=sphere.new(10);
c=cone.new(10,5,20):color('green4');
export_stl(cy,'cylinder.stl',{type='ascii',radian=0.05});
export_step(s,'sphere.step');
export_iges(c,'cone.iges');