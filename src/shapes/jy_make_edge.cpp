/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_make_edge.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Hypr.hxx>
#include <gp_Parab.hxx>

void JyEdge::configure_usertype(sol::state &lua) {
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

JyEdge::JyEdge(const std::string &_type, const std::array<double, 3> _vec1, const std::array<double, 3> _vec2,
               const double &_r1, const double &_r2) {
    const auto pos = gp_Pnt(_vec1[0], _vec1[1], _vec1[2]);
    const auto dir = gp_Dir(_vec2[0], _vec2[1], _vec2[2]);
    if (_type == "lin") {
        const auto end = gp_Pnt(_vec2[0], _vec2[1], _vec2[2]);
        BRepBuilderAPI_MakeEdge make_edge_lin(pos, end);
        s_ = make_edge_lin;
    } else if (_type == "circ") {
        // 示例：edge.new('circ', { 0, 0, 0 }, { 0, 0, 1 }, 1);
        if (_r1 <= 0) { throw std::invalid_argument("Invalid circle radius!"); }
        gp_Circ circ({pos, dir}, _r1);
        BRepBuilderAPI_MakeEdge make_edge_circ(circ);
        s_ = make_edge_circ;
    } else if (_type == "elips") {
        // R1: 大半轴  R2: 小半轴，R1必须大于R2
        if (_r1 <= 0 || _r2 <= 0 || _r1 < _r2) {
            throw std::invalid_argument("Invalid ellipse parameters! R1 > R2 > 0");
        }
        // 示例：edge.new('elips', { 0, 0, 0 }, { 0, 0, 1 }, 4, 2);
        gp_Elips elips({pos, dir}, _r1, _r2);
        BRepBuilderAPI_MakeEdge make_edge_elips(elips);
        s_ = make_edge_elips;
    } else if (_type == "hypr") {
        gp_Hypr hypr({pos, dir}, _r1, _r2);
        BRepBuilderAPI_MakeEdge make_edge_hypr(hypr);
        s_ = make_edge_hypr;
    } else if (_type == "parab") {
        gp_Parab parab({pos, dir}, _r1);
        BRepBuilderAPI_MakeEdge make_edge_parab(parab);
        s_ = make_edge_parab;
    } else {
        throw std::runtime_error("Edge: wrong type!");
    }
}

/*
// 用于测试，查看是否会调用析构函数
class MyGeom_BezierCurve : public Geom_BezierCurve {
public:
    MyGeom_BezierCurve(const TColgp_Array1OfPnt &poles) : Geom_BezierCurve(poles) {
        std::cout << "construct MyGeom_BezierCurve: " << poles.Size() << std::endl;
    }
    ~MyGeom_BezierCurve() {
        std::cout << "destroy MyGeom_BezierCurve" << std::endl;
    }
};
*/

JyBezier::JyBezier(const std::vector<std::array<double, 3>> poles) {
    // 示例：bezier.new({ { 0, 0, 0 }, { 1, 1, 1 }, { 0, 2, 3 } }):show()
    if ((poles.size() < 2) || (poles.size() > Geom_BezierCurve::MaxDegree() + 1)) {
        throw std::invalid_argument("Invalid bezier poles! The number of poles must be between 2 and " + std::to_string(Geom_BezierCurve::MaxDegree() + 1));
    }
    TColgp_Array1OfPnt CurvePoles(1, static_cast<Standard_Integer>(poles.size()));
    for (int i = 0; i < poles.size(); i++) {
        CurvePoles.SetValue(i + 1, gp_Pnt(poles[i][0], poles[i][1], poles[i][2]));
    }
    Handle(Geom_BezierCurve) bezier_curve = new Geom_BezierCurve(CurvePoles);
    BRepBuilderAPI_MakeEdge make_edge_bezier(bezier_curve);
    s_ = make_edge_bezier;
}
JyBezier::JyBezier(const std::vector<std::array<double, 3>> poles, const std::vector<double> weights) {
    // 示例：bezier.new({ { 0, 0, 0 }, { 1, 1, 1 }, { 0, 2, 3 } }, { 1, 0.2, 1 }):show()
    if ((poles.size() < 2) || (poles.size() > Geom_BezierCurve::MaxDegree() + 1)) {
        throw std::invalid_argument("Invalid bezier poles! The number of poles must be between 2 and " + std::to_string(Geom_BezierCurve::MaxDegree() + 1));
    }
    if (poles.size() != weights.size()) {
        throw std::invalid_argument("Invalid bezier weights!");
    }
    TColgp_Array1OfPnt CurvePoles(1, static_cast<Standard_Integer>(poles.size()));
    for (int i = 0; i < poles.size(); i++) {
        CurvePoles.SetValue(i + 1, gp_Pnt(poles[i][0], poles[i][1], poles[i][2]));
    }
    TColStd_Array1OfReal CurveWeights(1, static_cast<Standard_Integer>(weights.size()));
    for (int i = 0; i < weights.size(); i++) {
        CurveWeights.SetValue(i + 1, weights[i]);
    }
    Handle(Geom_BezierCurve) bezier_curve = new Geom_BezierCurve(CurvePoles, CurveWeights);
    BRepBuilderAPI_MakeEdge make_edge_bezier(bezier_curve);
    s_ = make_edge_bezier;
}


JyBSpline::JyBSpline(const std::vector<std::array<double, 3>> poles,
                     const std::vector<double> knots,
                     const std::vector<int> multiplicities,
                     const int &degree) {
    // 示例：bspline.new({ { 0, 0, 0 }, { 1, 2, 1 }, { 2, 2, 2 }, { 3, 0, 3 } }, { 0, 1 }, { 4, 4 }, 3):show()
    if (!(degree > 0 && degree <= Geom_BSplineCurve::MaxDegree())) {
        throw std::invalid_argument("Invalid bspline degree!");
    }
    if (!(knots.size() == multiplicities.size() && knots.size() >= 2)) {
        throw std::invalid_argument("Invalid bspline knots!");
    }
    TColgp_Array1OfPnt CurvePoles(1, static_cast<Standard_Integer>(poles.size()));
    for (int i = 0; i < poles.size(); i++) {
        CurvePoles.SetValue(i + 1, gp_Pnt(poles[i][0], poles[i][1], poles[i][2]));
    }
    TColStd_Array1OfReal CurveKnots(1, static_cast<Standard_Integer>(knots.size()));
    for (int i = 0; i < knots.size(); i++) {
        CurveKnots.SetValue(i + 1, knots[i]);
    }
    TColStd_Array1OfInteger CurveMults(1, static_cast<Standard_Integer>(multiplicities.size()));
    for (int i = 0; i < multiplicities.size(); i++) {
        CurveMults.SetValue(i + 1, multiplicities[i]);
    }
    Handle(Geom_BSplineCurve) bspline_curve = new Geom_BSplineCurve(CurvePoles, CurveKnots, CurveMults, degree);
    BRepBuilderAPI_MakeEdge make_edge_bspline(bspline_curve);
    s_ = make_edge_bspline;
}

JyBSpline::JyBSpline(const std::vector<std::array<double, 3>> points) {
    /*
    -- 示例：创建一个螺旋线样条曲线
    local radius = 1
    local pitch = 2
    local height = 10
    local numTurns = height / pitch
    local numPoints = (numTurns * 100) + 1
    local points = {}
    for i = 0, numPoints - 1 do
        local t = i / (numPoints - 1)
        local angle = 2.0 * math.pi * numTurns * t
        table.insert(points, { radius * math.cos(angle), radius * math.sin(angle), height * t })
    end
    bspline.new(points):show()
    */
    if (points.size() < 2) {
        throw std::invalid_argument("Invalid bspline points!");
    }
    TColgp_Array1OfPnt CurvePoles(1, static_cast<Standard_Integer>(points.size()));
    for (int i = 0; i < points.size(); i++) {
        CurvePoles.SetValue(i + 1, gp_Pnt(points[i][0], points[i][1], points[i][2]));
    }
    GeomAPI_PointsToBSpline splineBuilder(CurvePoles);
    Handle(Geom_BSplineCurve) spline = splineBuilder.Curve();
    TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(spline);
    s_ = edge;
}

JyLine::JyLine(const std::array<double, 3> p1, const std::array<double, 3> p2) {
    // 示例：line.new({ 0, 0, 0 }, { 1, 1, 1 }):show()
    const auto pnt1 = gp_Pnt(p1[0], p1[1], p1[2]);
    const auto pnt2 = gp_Pnt(p2[0], p2[1], p2[2]);
    BRepBuilderAPI_MakeEdge make_edge_lin(pnt1, pnt2);
    s_ = make_edge_lin;
}

JyCircle::JyCircle(const std::array<double, 3> center, const std::array<double, 3> normal, const double &radius) {
    // 示例：circle.new({ 1, 1, 1 }, { 1, 1, 1 }, 3):show()
    const auto pnt = gp_Pnt(center[0], center[1], center[2]);
    const auto dir = gp_Dir(normal[0], normal[1], normal[2]);
    gp_Circ circ({pnt, dir}, radius);
    BRepBuilderAPI_MakeEdge make_edge_circ(circ);
    s_ = make_edge_circ;
}
JyEllipse::JyEllipse(const std::array<double, 3> center, const std::array<double, 3> normal, const double &radius1, const double &radius2) {
    // 示例：ellipse.new({ 1, 1, 1 }, { 1, 1, 1 }, 4, 2):show()
    if (radius1 <= 0 || radius2 <= 0 || radius1 < radius2) {
        throw std::invalid_argument("Invalid ellipse parameters! R1 > R2 > 0");
    }
    const auto pnt = gp_Pnt(center[0], center[1], center[2]);
    const auto dir = gp_Dir(normal[0], normal[1], normal[2]);
    gp_Elips elips({pnt, dir}, radius1, radius2);
    BRepBuilderAPI_MakeEdge make_edge_elips(elips);
    s_ = make_edge_elips;
}

JyHyperbola::JyHyperbola(const std::array<double, 3> center, const std::array<double, 3> normal,
                         const double &radius1, const double &radius2, const double &p1, const double &p2) {
    // 示例：hyperbola.new({ 0, 0, 0 }, { 1, 1, 1 }, 4, 2, -2, 2):show()
    if (radius1 <= 0 || radius2 <= 0 || radius1 < radius2) {
        throw std::invalid_argument("Invalid hyperbola parameters! R1 > R2 > 0");
    }
    if (p1 >= p2) {
        throw std::invalid_argument("Invalid hyperbola parameters! P1 < P2");
    }
    const auto pnt = gp_Pnt(center[0], center[1], center[2]);
    const auto dir = gp_Dir(normal[0], normal[1], normal[2]);
    gp_Hypr hypr({pnt, dir}, radius1, radius2);
    BRepBuilderAPI_MakeEdge make_edge_hypr(hypr, p1, p2);
    s_ = make_edge_hypr;
}
JyParabola::JyParabola(const std::array<double, 3> center, const std::array<double, 3> normal,
                       const double &radius, const double &p1, const double &p2) {
    // 示例：parabola.new({ 2, 1, 3 }, { 1, 1, 1 }, 3, -2, 2):show()
    if (radius <= 0) {
        throw std::invalid_argument("Invalid parabola parameters! R > 0");
    }
    const auto pnt = gp_Pnt(center[0], center[1], center[2]);
    const auto dir = gp_Dir(normal[0], normal[1], normal[2]);
    gp_Parab parab({pnt, dir}, radius);
    BRepBuilderAPI_MakeEdge make_edge_parab(parab, p1, p2);
    s_ = make_edge_parab;
}

JyArc::JyArc(const std::array<double, 3> p1, const std::array<double, 3> p2, const std::array<double, 3> p3) {
    // 示例：arc.new({ 0, 0, 0 }, { 1, 1, 1 }, { 0, 0, 2 }):show()
    const auto pnt1 = gp_Pnt(p1[0], p1[1], p1[2]);
    const auto pnt2 = gp_Pnt(p2[0], p2[1], p2[2]);
    const auto pnt3 = gp_Pnt(p3[0], p3[1], p3[2]);
    const auto make_arc = GC_MakeArcOfCircle(pnt1, pnt2, pnt3);
    const auto status = make_arc.Status();
    if (status != gce_Done) {
        static const std::unordered_map<gce_ErrorType, std::string> error_messages = {
                {gce_ConfusedPoints, "Two points are coincident"},
                {gce_NegativeRadius, "Radius value is negative"},
                {gce_ColinearPoints, "Three points are collinear"},
                {gce_IntersectionError, "Intersection cannot be computed"},
                {gce_NullAxis, "Axis is undefined"},
                {gce_NullAngle, "Angle value is invalid (usually null)"},
                {gce_NullRadius, "Radius is null"},
                {gce_InvertAxis, "Axis value is invalid"},
                {gce_BadAngle, "Angle value is invalid"},
                {gce_InvertRadius, "Radius value is incorrect (usually with respect to another radius)"},
                {gce_NullFocusLength, "Focal distance is null"},
                {gce_NullVector, "Vector is null"},
                {gce_BadEquation, "Coefficients are incorrect"}};
        auto it = error_messages.find(status);
        if (it != error_messages.end()) {
            throw std::runtime_error("Arc construction error: " + it->second + ".");
        }
        throw std::runtime_error("Arc construction error: Unknown error.");
    }
    Handle(Geom_TrimmedCurve) anArcOfCircle = make_arc.Value();
    BRepBuilderAPI_MakeEdge make_edge_arc(anArcOfCircle);
    s_ = make_edge_arc;
}
