local r_outer = 57 / 2 + 3.2;
local h_flank_thin = 4;
local r_base = 45;
local r_base_screw = 40;
local h_base_top = 16.6;
local h_base = (8 + h_base_top);
local d_flank_outer = (2 * r_outer - 3);
local r_flank_screw_pos = (d_flank_outer / 2 - 5.3);
local r_cut = r_base - r_outer;

base = cylinder.new(r_base, h_base + h_flank_thin);
elips = edge.new('elips', { r_base, 0, h_base + h_flank_thin }, { 0, 1, 0 }, h_base_top + h_flank_thin, r_cut);
ellipse = face.new(elips);
ellipse:revol({ 0, 0, 0 }, { 0, 0, 1 }, 360)
print(ellipse:type())
base:cut(ellipse)
base:fillet(3, { type = 'bspline_curve', max = { r_base + 1, 1, h_base - r_cut - h_flank_thin + 1 } });
base:cut(cylinder.new(r_outer - 2, h_flank_thin):z(h_base));
for deg = 60, 360, 60 do
  rad = deg * math.pi / 180;
  x0 = r_flank_screw_pos * math.sin(rad);
  y0 = r_flank_screw_pos * math.cos(rad);
  base:cut(cylinder.new(1.9, 50):pos(x0, y0, 0));
  x1 = r_base_screw * math.sin(rad);
  y1 = r_base_screw * math.cos(rad);
  base:cut(cylinder.new(4.2 / 2, 20):pos(x1, y1, 0));
  base:cut(cylinder.new(8 / 2, 10):pos(x1, y1, 0):z(8));
end
base:cut(cylinder.new(0.7, 3 * r_outer):x(-1.5 * r_outer):z(h_base + h_flank_thin / 2):ry(90));
base:cut(cylinder.new(4, 50):z(8 + h_base_top / 2):ry(90));
base:cut(cylinder.new(25 / 2, 50));
base:show()
