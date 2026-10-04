#include "jy_bindings.h"
#include "jy_make_shapes.h"
#include "jy_urdf_generator.h"

namespace {
EdgeFilter edgeFilter(const sol::table &t) {
    EdgeFilter filter;
    if (!t.valid()) return filter;
    filter.tolerance = t["tol"].get_or(1e-3);
    if (t["type"].is<std::string>()) filter.type = t["type"].get<std::string>();
    if (t["first"].is<sol::table>()) filter.first = t["first"].get<std::array<double, 3>>();
    if (t["last"].is<sol::table>()) filter.last = t["last"].get<std::array<double, 3>>();
    if (t["min"].is<sol::table>()) filter.minimum = t["min"].get<std::array<double, 3>>();
    if (t["max"].is<sol::table>()) filter.maximum = t["max"].get<std::array<double, 3>>();
    return filter;
}
StlOptions stlOptions(const sol::table &t) {
    StlOptions options;
    if (!t.valid()) return options;
    const auto type = t["type"].get_or(std::string("binary"));
    if (type != "ascii" && type != "binary") throw std::runtime_error("Invalid STL type");
    options.ascii = type == "ascii";
    options.deflection = t["deflection"].get_or(t["radian"].get_or(0.01));
    return options;
}
Link::ExportOptions robotOptions(const sol::table &t) {
    Link::ExportOptions options;
    options.name = t["name"].get<std::string>();
    options.path = t["path"].get<std::string>();
    if (t["ros_version"].valid()) options.format = t["ros_version"].get<int>() == 1 ? Link::Format::ROS1 : Link::Format::ROS2;
    if (t["mujoco"].get_or(false)) options.format = Link::Format::MUJOCO;
    return options;
}
}

sol::usertype<JyShape> bindShape(sol::state &lua) {
    auto shape_user = lua.new_usertype<JyShape>("shape", sol::constructors<JyShape(),
                                                                           JyShape(const std::string &)>());
    shape_user["copy"] = [](const JyShape &self) { return JyShape(self); };
    shape_user["type"] = &JyShape::type;
    shape_user["empty"] = &JyShape::empty;
    shape_user["get_edge"] = [](const JyShape &self, const sol::table &t) { return self.get_edge(edgeFilter(t)); };
    shape_user["get_face"] = &JyShape::get_face;
    // 布尔运算
    shape_user["fuse"] = &JyShape::fuse;
    shape_user["cut"] = &JyShape::cut;
    shape_user["common"] = &JyShape::common;
    // 几何变换
    shape_user["fillet"] = sol::overload(
            static_cast<JyShape &(JyShape::*) (const double &)>(&JyShape::fillet),
            static_cast<JyShape &(JyShape::*) (const double &, const JyShape &)>(&JyShape::fillet),
            [](JyShape &self, double value, const sol::table &t) -> JyShape & { return self.fillet(value, edgeFilter(t)); });
    shape_user["chamfer"] = sol::overload(
            static_cast<JyShape &(JyShape::*) (const double &)>(&JyShape::chamfer),
            [](JyShape &self, double value, const sol::table &t) -> JyShape & { return self.chamfer(value, edgeFilter(t)); });
    shape_user["prism"] = &JyShape::prism;
    shape_user["revol"] = &JyShape::revol;
    shape_user["pipe"] = &JyShape::pipe;
    shape_user["thick"] = &JyShape::thick;
    shape_user["scale"] = &JyShape::scale;
    shape_user["mirror"] = &JyShape::mirror;
    // 位置姿态调整
    shape_user["x"] = &JyShape::x;
    shape_user["y"] = &JyShape::y;
    shape_user["z"] = &JyShape::z;
    shape_user["rx"] = &JyShape::rx;
    shape_user["ry"] = &JyShape::ry;
    shape_user["rz"] = &JyShape::rz;
    shape_user["pos"] = &JyShape::pos;
    shape_user["rot"] = &JyShape::rot;
    shape_user["move"] = sol::overload(
            static_cast<JyShape &(JyShape::*) (const std::string &, const double &, const double &, const double &)>(&JyShape::move),
            static_cast<JyShape &(JyShape::*) (const std::string &, const double &)>(&JyShape::move));
    shape_user["zero"] = &JyShape::zero;
    shape_user["locate"] = sol::overload(
            static_cast<JyShape &(JyShape::*) (const JyShape &)>(&JyShape::locate),
            static_cast<JyShape &(JyShape::*) (const double &, const double &, const double &, const double &, const double &, const double &)>(&JyShape::locate));
    // 属性设置
    shape_user["color"] = &JyShape::color;
    shape_user["transparency"] = &JyShape::transparency;
    shape_user["mass"] = &JyShape::mass;
    shape_user["export_step"] = &JyShape::export_step;
    shape_user["export_iges"] = &JyShape::export_iges;
    const auto overload_export_stl = sol::overload(
            static_cast<JyShape &(JyShape::*) (const std::string &_filename)>(&JyShape::export_stl),
            [](JyShape &self, const std::string &file, const sol::table &t) -> JyShape & { return self.export_stl(file, stlOptions(t)); });
    shape_user["export_stl"] = overload_export_stl;
    return shape_user;
}

sol::usertype<JyAxes> bindAxes(sol::state &lua) {
    auto axes_user = lua.new_usertype<JyAxes>("axes", sol::constructors<JyAxes(),
                                                                        JyAxes(const double &),
                                                                        JyAxes(const std::array<double, 6>),
                                                                        JyAxes(const std::array<double, 6>, const double &),
                                                                        JyAxes(const JyAxes &)>());
    axes_user["copy"] = [](const JyAxes &self) { return JyAxes(self); };
    axes_user["move"] = &JyAxes::move;
    axes_user["sdh"] = &JyAxes::sdh;
    axes_user["mdh"] = &JyAxes::mdh;
    return axes_user;
}

void bindEdges(sol::state &lua) {
    const auto edge_ctor = sol::constructors<JyEdge(const JyEdge &),
                                             JyEdge(const std::string &, const std::array<double, 3>, const std::array<double, 3>),
                                             JyEdge(const std::string &, const std::array<double, 3>, const std::array<double, 3>, const double &),
                                             JyEdge(const std::string &, const std::array<double, 3>, const std::array<double, 3>, const double &, const double &)>();
    lua.new_usertype<JyEdge>("edge", edge_ctor, sol::base_classes, sol::bases<JyShape>());
    const auto line_ctor = sol::constructors<JyLine(const JyLine &),
                                             JyLine(const std::array<double, 3>, const std::array<double, 3>)>();
    lua.new_usertype<JyLine>("line", line_ctor, sol::base_classes, sol::bases<JyEdge, JyShape>());
    const auto circle_ctor = sol::constructors<JyCircle(const JyCircle &),
                                               JyCircle(const std::array<double, 3>, const std::array<double, 3>, const double &)>();
    lua.new_usertype<JyCircle>("circle", circle_ctor, sol::base_classes, sol::bases<JyEdge, JyShape>());
    const auto ellipse_ctor = sol::constructors<JyEllipse(const JyEllipse &),
                                                JyEllipse(const std::array<double, 3>, const std::array<double, 3>, const double &, const double &)>();
    lua.new_usertype<JyEllipse>("ellipse", ellipse_ctor, sol::base_classes, sol::bases<JyEdge, JyShape>());
    const auto hyperbola_ctor = sol::constructors<JyHyperbola(const JyHyperbola &),
                                                  JyHyperbola(const std::array<double, 3>, const std::array<double, 3>, const double &, const double &, const double &, const double &)>();
    lua.new_usertype<JyHyperbola>("hyperbola", hyperbola_ctor, sol::base_classes, sol::bases<JyEdge, JyShape>());
    const auto parabola_ctor = sol::constructors<JyParabola(const JyParabola &),
                                                 JyParabola(const std::array<double, 3>, const std::array<double, 3>, const double &, const double &, const double &)>();
    lua.new_usertype<JyParabola>("parabola", parabola_ctor, sol::base_classes, sol::bases<JyEdge, JyShape>());
    const auto bezier_ctor = sol::constructors<JyBezier(const JyBezier &),
                                               JyBezier(const std::vector<std::array<double, 3>>),
                                               JyBezier(const std::vector<std::array<double, 3>>, const std::vector<double>)>();
    lua.new_usertype<JyBezier>("bezier", bezier_ctor, sol::base_classes, sol::bases<JyEdge, JyShape>());
    const auto bspline_ctor = sol::constructors<JyBSpline(const JyBSpline &),
                                                JyBSpline(const std::vector<std::array<double, 3>>),
                                                JyBSpline(const std::vector<std::array<double, 3>>,
                                                          const std::vector<double>,
                                                          const std::vector<int>,
                                                          const int &)>();
    lua.new_usertype<JyBSpline>("bspline", bspline_ctor, sol::base_classes, sol::bases<JyEdge, JyShape>());
    const auto arc_ctor = sol::constructors<JyArc(const JyArc &),
                                            JyArc(const std::array<double, 3>, const std::array<double, 3>, const std::array<double, 3>)>();
    lua.new_usertype<JyArc>("arc", arc_ctor, sol::base_classes, sol::bases<JyEdge, JyShape>());
}

void bindFaces(sol::state &lua) {
    const auto face_ctor = sol::constructors<JyFace(),
                                             JyFace(const JyFace &),
                                             JyFace(const JyShape &)>();
    lua.new_usertype<JyFace>("face", face_ctor, sol::base_classes, sol::bases<JyShape>());
    const auto plane_ctor = sol::constructors<JyPlane(const JyPlane &),
                                              JyPlane(const std::array<double, 3>, const std::array<double, 3>, const std::array<double, 4>)>();
    lua.new_usertype<JyPlane>("plane", plane_ctor, sol::base_classes, sol::bases<JyFace, JyShape>());
    const auto cylindrical_ctor = sol::constructors<JyCylindrical(const JyCylindrical &),
                                                    JyCylindrical(const std::array<double, 3>, const std::array<double, 3>, const double &, const std::array<double, 4>),
                                                    JyCylindrical(const std::array<double, 3>, const std::array<double, 3>, const double &, const double &)>();
    lua.new_usertype<JyCylindrical>("cylindrical", cylindrical_ctor, sol::base_classes, sol::bases<JyFace, JyShape>());
    const auto conical_ctor = sol::constructors<JyConical(const JyConical &),
                                                JyConical(const std::array<double, 3>, const std::array<double, 3>, const double &, const double &, const std::array<double, 4>)>();
    lua.new_usertype<JyConical>("conical", conical_ctor, sol::base_classes, sol::bases<JyFace, JyShape>());
}

void bindPrimitives(sol::state &lua) {
    const auto box_ctor = sol::constructors<JyShapeBox(),
                                            JyShapeBox(const JyShapeBox &),
                                            JyShapeBox(const std::array<double, 3>, const std::array<double, 3>),
                                            JyShapeBox(const double &, const double &, const double &)>();
    const auto cylinder_ctor = sol::constructors<JyCylinder(),
                                                 JyCylinder(const JyCylinder &),
                                                 JyCylinder(const std::array<double, 3>, const std::array<double, 3>, const double &, const double &),
                                                 JyCylinder(const double &, const double &)>();
    const auto cone_ctor = sol::constructors<JyCone(),
                                             JyCone(const JyCone &),
                                             JyCone(const double &, const double &, const double &)>();
    const auto sphere_ctor = sol::constructors<JySphere(),
                                               JySphere(const JySphere &),
                                               JySphere(const double &)>();
    const auto torus_ctor = sol::constructors<JyTorus(),
                                              JyTorus(const JyTorus &),
                                              JyTorus(const double &, const double &),
                                              JyTorus(const double &, const double &, const double &)>();
    const auto wedge_ctor = sol::constructors<JyWedge(),
                                              JyWedge(const JyWedge &),
                                              JyWedge(const double &, const double &, const double &, const double &),
                                              JyWedge(const double &, const double &, const double &, const double &, const double &, const double &, const double &)>();
    lua.new_usertype<JyShapeBox>("box", box_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyCylinder>("cylinder", cylinder_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyCone>("cone", cone_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JySphere>("sphere", sphere_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyTorus>("torus", torus_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyWedge>("wedge", wedge_ctor, sol::base_classes, sol::bases<JyShape>());

    const auto vertex_ctor = sol::constructors<JyVertex(const JyVertex &),
                                               JyVertex(const double &, const double &, const double &)>();
    lua.new_usertype<JyVertex>("vertex", vertex_ctor, sol::base_classes, sol::bases<JyShape>());

    const auto wire_ctor = sol::constructors<JyWire(),
                                             JyWire(const JyWire &),
                                             JyWire(const JyEdge &),
                                             JyWire(const std::vector<JyShape>)>();
    lua.new_usertype<JyWire>("wire", wire_ctor, sol::base_classes, sol::bases<JyShape>());
    const auto polygon_ctor = sol::constructors<JyPolygon(),
                                                JyPolygon(const JyPolygon &),
                                                JyPolygon(const std::vector<std::array<double, 3>>)>();
    lua.new_usertype<JyPolygon>("polygon", polygon_ctor, sol::base_classes, sol::bases<JyWire, JyShape>());

    const auto text_ctor = sol::constructors<JyText(),
                                             JyText(const std::string &),
                                             JyText(const std::string &, const double &),
                                             JyText(const std::string &, const double &, const std::string &),
                                             JyText(const JyText &)>();
    lua.new_usertype<JyText>("text", text_ctor, sol::base_classes, sol::bases<JyShape>());
}

void bindRobot(sol::state &lua) {
    auto link_user = lua.new_usertype<Link>("link", sol::factories([](const std::string &name, const JyShape &s) { return Link(name, s); }, [](const std::string &name, const sol::table &t) { std::vector<JyShape> shapes; for (size_t i = 1; i <= t.size(); ++i) shapes.push_back(t[i].get<JyShape>()); return Link(name, shapes); }));
    link_user["export"] = sol::overload(
            static_cast<void (Link::*)(const std::string &robot_name) const>(&Link::export_urdf),
            [](const Link &self, const sol::table &t) { self.export_urdf(robotOptions(t)); });
    link_user["add"] = &Link::add;
    auto joint_user = lua.new_usertype<Joint>("joint", sol::constructors<Joint(const std::string &, const JyAxes &, const std::string &),
        Joint(const std::string &, const JyAxes &, const std::string &, std::unordered_map<std::string, double>)>());
    joint_user["next"] = &Joint::next;
}
