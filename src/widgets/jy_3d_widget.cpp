/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_3d_widget.h"
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Geom_Line.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <QApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <StdSelect_BRepOwner.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <V3d_View.hxx>

#ifdef _WIN32

#include <WNT_Window.hxx>

#elif defined(__APPLE__)
#include <Cocoa_Window.hxx>
#else
#include <Xw_Window.hxx>
#endif

Jy3DWidget::Jy3DWidget(QWidget *parent) : QWidget(parent) {
    //    resize(425, 400);
    //配置QWidget
    setBackgroundRole(QPalette::NoRole);//无背景
    setMouseTracking(true);             //开启鼠标位置追踪
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setFocusPolicy(Qt::StrongFocus);
    createContextMenu();
}

void Jy3DWidget::onDisplayShape(const JyShape &theIObj) {
    if (theIObj.data().IsNull()) { return; }
    // 更新网格数据，保证网格的XY能覆盖模型
    const auto shape = theIObj.data();
    // 模型包围盒计算
    Bnd_Box bounding;
    BRepBndLib::Add(shape, bounding);
    if (!bounding.IsOpen()) {
        std::array<Standard_Real, 4> bndXY;
        Standard_Real bndMinZ, bndMaxinZ;
        bounding.Get(bndXY[0], bndXY[1], bndMinZ, bndXY[2], bndXY[3], bndMaxinZ);
        std::transform(bndXY.begin(), bndXY.end(), bndXY.begin(), [](double x) { return std::abs(x); });
        const auto max = *std::max_element(bndXY.begin(), bndXY.end());
        Standard_Real x_size_old, y_size_old, offset_old;
        m_viewer->RectangularGridGraphicValues(x_size_old, y_size_old, offset_old);
        if (max > x_size_old) {
            const double logValue = std::log10(std::abs(max));
            const auto step = std::pow(10.0, std::floor(logValue));
            const auto x = std::ceil(max / step) * step + 1e-3;
            m_viewer->SetRectangularGridValues(0, 0, step, step, 0);
            m_viewer->SetRectangularGridGraphicValues(x, x, 0);
        }
    } else {
        qDebug() << "Warning: bounding is open";
    }
    const auto ais_shape = new AIS_Shape(shape);
    ais_shape->SetColor(theIObj.color_);
    ais_shape->SetTransparency(theIObj.transparency_);
    ais_shape->SetMaterial(Graphic3d_NameOfMaterial_Stone);
    m_context->Display(ais_shape, Standard_True);
    m_view->FitAll();
}


void Jy3DWidget::onDisplayAxes(const JyAxes &theAxes) {
    gp_Ax2 ax2;
    ax2.Transform(theAxes.data());
    const auto trihedron = new AIS_Trihedron(new Geom_Axis2Placement(ax2));
    trihedron->SetDatumDisplayMode(Prs3d_DM_WireFrame);
    trihedron->SetDrawArrows(false);
    trihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_XAxis)->SetWidth(2.5);
    trihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_YAxis)->SetWidth(2.5);
    trihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_ZAxis)->SetWidth(2.5);
    trihedron->SetDatumPartColor(Prs3d_DatumParts_XAxis, Quantity_NOC_RED2);
    trihedron->SetDatumPartColor(Prs3d_DatumParts_YAxis, Quantity_NOC_GREEN2);
    trihedron->SetDatumPartColor(Prs3d_DatumParts_ZAxis, Quantity_NOC_BLUE2);
    trihedron->SetLabel(Prs3d_DatumParts_XAxis, "");
    trihedron->SetLabel(Prs3d_DatumParts_YAxis, "");
    trihedron->SetLabel(Prs3d_DatumParts_ZAxis, "");
    trihedron->SetSize(theAxes.length());
    m_context->Display(trihedron, Standard_True);
}

void Jy3DWidget::remove_all() {
    m_viewer->SetRectangularGridValues(0, 0, 1, 1, 0);
    m_viewer->SetRectangularGridGraphicValues(2.01, 2.01, 0);
#if 1
    AIS_ListOfInteractive objects;
    m_context->DisplayedObjects(objects);
    for (const auto &x: objects) {
        if ((x == view_cube) || (x == origin_coord)) { continue; }
        m_context->Remove(x, Standard_True);
    }
#else
    m_context->RemoveAll(Standard_True);
    m_context->Display(view_cube, Standard_True);
    m_context->Display(origin_coord, Standard_True);
#endif
    update();
}

void Jy3DWidget::initialize_context() {
    //此对象提供与X server的连接，在Windows和Mac OS中不起作用
    Handle(Aspect_DisplayConnection) display_connection = new Aspect_DisplayConnection();
    //获取QWidget的窗口系统标识符
    WId window_handle = (WId) winId();
#ifdef _WIN32
    // 创建Windows NT 窗口
    Handle(WNT_Window) wind = new WNT_Window((Aspect_Handle) window_handle);
#elif defined(__APPLE__)
    Handle(Cocoa_Window) wind = new Cocoa_Window(reinterpret_cast<NSView *>(winId()));
#else
    // 创建XLib window 窗口
    Handle(Xw_Window) wind = new Xw_Window(display_connection, window_handle);
#endif
    //创建3D查看器,使用OpenGl图形驱动
    m_viewer = new V3d_Viewer(new OpenGl_GraphicDriver(display_connection));
    //创建视图
    m_view = m_viewer->CreateView();
    m_view->SetWindow(wind);
    //打开窗口
    if (!wind->IsMapped()) {
        wind->Map();
    }
    m_context = new AIS_InteractiveContext(m_viewer);//创建交互式上下文
    //配置查看器的光照
    light_direction = new V3d_Light(Graphic3d_TypeOfLightSource_Directional);
    light_direction->SetDirection(m_view->Camera()->Direction());
    m_viewer->AddLight(new V3d_Light(Graphic3d_TypeOfLightSource_Ambient));
    m_viewer->AddLight(light_direction);
    m_viewer->SetLightOn();
    //设置视图的背景颜色为灰色
    Quantity_Color background_color;
    Quantity_Color::ColorFromHex("#3f3f3f", background_color);
    m_view->SetBackgroundColor(background_color);
    m_view->MustBeResized();

    create_view_cube();   // 创建视方体
    create_origin_coord();// 创建基坐标系

    //设置显示模式
    m_context->SetDisplayMode(AIS_Shaded, Standard_True);
    // 设置模型高亮的风格
    Handle(Prs3d_Drawer) highlight_style = m_context->HighlightStyle();// 获取高亮风格
    highlight_style->SetMethod(Aspect_TOHM_COLOR);                     // 颜色显示方式
    highlight_style->SetColor(Quantity_NOC_LIGHTYELLOW);               // 设置高亮颜色
    highlight_style->SetDisplayMode(1);                                // 整体高亮
    highlight_style->SetTransparency(0.2f);                            // 设置透明度
    // 设置选择模型的风格
    Handle(Prs3d_Drawer) t_select_style = m_context->SelectionStyle();// 获取选择风格
    t_select_style->SetMethod(Aspect_TOHM_COLOR);                     // 颜色显示方式
    t_select_style->SetColor(Quantity_NOC_LIGHTSEAGREEN);             // 设置选择后颜色
    t_select_style->SetDisplayMode(1);                                // 整体高亮
    t_select_style->SetTransparency(0.4f);                            // 设置透明度
    m_view->SetZoom(100);                                             // 放大
    // 激活二维网格
    m_viewer->SetRectangularGridValues(0, 0, 1, 1, 0);
    m_viewer->SetRectangularGridGraphicValues(2.01, 2.01, 0);
    m_viewer->ActivateGrid(Aspect_GT_Rectangular, Aspect_GDM_Lines);
}


void Jy3DWidget::create_view_cube() {
    view_cube = new AIS_ViewCube();
    const auto &vc_attributes = view_cube->Attributes();
    //设置视方体基准线
    vc_attributes->SetDatumAspect(new Prs3d_DatumAspect());//设置的预前工作（十分重要）
    const Handle(Prs3d_DatumAspect) &datumAspect = view_cube->Attributes()->DatumAspect();
    //设置轴颜色
    datumAspect->ShadingAspect(Prs3d_DatumParts_XAxis)->SetColor(Quantity_NOC_RED);
    datumAspect->ShadingAspect(Prs3d_DatumParts_YAxis)->SetColor(Quantity_NOC_GREEN);
    datumAspect->ShadingAspect(Prs3d_DatumParts_ZAxis)->SetColor(Quantity_NOC_BLUE);
    //设置X,Y,Z文本颜色
    datumAspect->TextAspect(Prs3d_DatumParts_XAxis)->SetColor(Quantity_NOC_RED);
    datumAspect->TextAspect(Prs3d_DatumParts_YAxis)->SetColor(Quantity_NOC_GREEN);
    datumAspect->TextAspect(Prs3d_DatumParts_ZAxis)->SetColor(Quantity_NOC_BLUE);
    Handle(Graphic3d_TransformPers) transform_pers = new Graphic3d_TransformPers(Graphic3d_TMF_TriedronPers, Aspect_TOTP_LEFT_LOWER, Graphic3d_Vec2i(85, 85));
    view_cube->SetTransformPersistence(transform_pers);
    view_cube->SetSize(50);
    view_cube->SetAxesPadding(3);                                  // 坐标轴与立方体的距离
    view_cube->SetBoxColor(Quantity_Color(Quantity_NOC_MATRAGRAY));// 立方体颜色
    view_cube->SetFontHeight(12);
    m_context->Display(view_cube, Standard_True);
}

void Jy3DWidget::create_origin_coord() {
    Handle(Geom_Axis2Placement) axis = new Geom_Axis2Placement(gp::XOY());
    origin_coord = new AIS_Trihedron(axis);
    origin_coord->SetDatumDisplayMode(Prs3d_DM_WireFrame);
    origin_coord->SetDrawArrows(false);
    origin_coord->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_XAxis)->SetWidth(2.5);
    origin_coord->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_YAxis)->SetWidth(2.5);
    origin_coord->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_ZAxis)->SetWidth(2.5);
    origin_coord->SetDatumPartColor(Prs3d_DatumParts_XAxis, Quantity_NOC_RED2);
    origin_coord->SetDatumPartColor(Prs3d_DatumParts_YAxis, Quantity_NOC_GREEN2);
    origin_coord->SetDatumPartColor(Prs3d_DatumParts_ZAxis, Quantity_NOC_BLUE2);
    origin_coord->SetLabel(Prs3d_DatumParts_XAxis, "");
    origin_coord->SetLabel(Prs3d_DatumParts_YAxis, "");
    origin_coord->SetLabel(Prs3d_DatumParts_ZAxis, "");
    origin_coord->SetSize(60);
    origin_coord->SetTransformPersistence(
            new Graphic3d_TransformPers(Graphic3d_TMF_ZoomPers, axis->Ax2().Location()));
    origin_coord->Attributes()->SetZLayer(Graphic3d_ZLayerId_Topmost);
    origin_coord->SetInfiniteState(true);
    // 基坐标系无法被选中，REF:https://www.cnblogs.com/zekexiao/p/17623421.html
    m_context->Display(origin_coord, AIS_WireFrame, -1, Standard_True);
}

void Jy3DWidget::paintEvent(QPaintEvent *event) {
    m_view->Redraw();
    QWidget::paintEvent(event);
}

void Jy3DWidget::resizeEvent(QResizeEvent *event) {
    if (!m_context) { initialize_context(); }// 初始化交互环境
    if (!m_view.IsNull()) { m_view->MustBeResized(); }
    QWidget::resizeEvent(event);
}

void Jy3DWidget::mousePressEvent(QMouseEvent *event) {
    const qreal ratio = devicePixelRatioF();
    if (event->buttons() & Qt::LeftButton) {
        // 鼠标左右键齐按：初始化平移
        m_x_max = event->x();
        m_y_max = event->y();
        // 点击前，将鼠标位置传递到交互环境（需要乘以设备像素比以处理高DPI屏幕）
        m_context->MoveTo(event->pos().x() * ratio, event->pos().y() * ratio, m_view, Standard_True);
        // 鼠标左键：选择模型
        if (QApplication::keyboardModifiers() == Qt::ControlModifier) {
            m_context->SelectDetected(AIS_SelectionScheme_Add);// 多选
        } else {
            const auto pick_status = m_context->SelectDetected();// 单选
            if (pick_status == AIS_SOP_OneSelected) {
                Handle(SelectMgr_EntityOwner) owner = m_context->DetectedOwner();
                if (owner) {
                    Handle(StdSelect_BRepOwner) brepOwner = Handle(StdSelect_BRepOwner)::DownCast(owner);
                    if (!brepOwner.IsNull()) {
                        TopoDS_Shape shape = brepOwner->Shape();
                        handleSelectedShape(shape);
                    }
                }
            }
        }
        m_view->Update();
    } else if (event->buttons() & Qt::RightButton) {
        m_isDragging = false;
        // 鼠标滚轮键：初始化平移
        m_x_max = event->x();
        m_y_max = event->y();
        // 鼠标滚轮键：初始化旋转（需要乘以设备像素比）
        m_view->StartRotation(event->x() * ratio, event->y() * ratio);
    }
}

void Jy3DWidget::mouseReleaseEvent(QMouseEvent *event) {
    const qreal ratio = devicePixelRatioF();
    // 将鼠标位置传递到交互环境（需要乘以设备像素比）
    m_context->MoveTo(event->pos().x() * ratio, event->pos().y() * ratio, m_view, Standard_True);
    if (!m_isDragging && event->button() == Qt::RightButton) {
        m_contextMenu->exec(event->globalPos());
    }
}

void Jy3DWidget::mouseMoveEvent(QMouseEvent *event) {
    const qreal ratio = devicePixelRatioF();
    if ((event->buttons() & Qt::LeftButton)) {
        // 鼠标左右键齐按：执行平移（平移差值需要乘以设备像素比）
        m_view->Pan((event->pos().x() - m_x_max) * ratio, (m_y_max - event->pos().y()) * ratio);
        m_x_max = event->x();
        m_y_max = event->y();
    } else if (event->buttons() & Qt::RightButton) {
        // 鼠标滚轮键：执行旋转（需要乘以设备像素比）
        m_view->Rotation(event->x() * ratio, event->y() * ratio);
        light_direction->SetDirection(m_view->Camera()->Direction());
        m_isDragging = true;
    } else {
        // 将鼠标位置传递到交互环境（需要乘以设备像素比）
        m_context->MoveTo(event->pos().x() * ratio, event->pos().y() * ratio, m_view, Standard_True);
    }
}
void Jy3DWidget::wheelEvent(QWheelEvent *event) {
    const qreal ratio = devicePixelRatioF();
    // 缩放操作需要乘以设备像素比以处理高DPI屏幕
    m_view->StartZoomAtPoint((int) (event->position().x() * ratio), (int) (event->position().y() * ratio));
    m_view->ZoomAtPoint(0, 0, event->angleDelta().y(), 0);//执行缩放
}

void Jy3DWidget::createContextMenu() {
    m_contextMenu = new QMenu(this);
    m_selectionModeGroup = new QActionGroup(this);
    const std::unordered_map<int, QString> shape_type_map = {
            {static_cast<int>(TopAbs_SHAPE), "Shapes"},
            {static_cast<int>(TopAbs_VERTEX), "Vertices"},
            {static_cast<int>(TopAbs_EDGE), "Edges"},
            {static_cast<int>(TopAbs_WIRE), "Wires"},
            {static_cast<int>(TopAbs_FACE), "Faces"},
            {static_cast<int>(TopAbs_SHELL), "Shells"},
            {static_cast<int>(TopAbs_SOLID), "Solids"},
            {static_cast<int>(TopAbs_COMPSOLID), "CompSolids"},
            {static_cast<int>(TopAbs_COMPOUND), "Compounds"},
    };
    QAction *selectVector = nullptr;
    for (auto it = shape_type_map.cbegin(); it != shape_type_map.cend(); ++it) {
        const auto action = m_selectionModeGroup->addAction(tr("Select %1").arg(it->second));
        action->setData(it->first);
        action->setCheckable(true);
        action->setChecked(it->first == static_cast<int>(TopAbs_SHAPE));
        if (it->first == static_cast<int>(TopAbs_VERTEX)) { selectVector = action; }
    }
    m_contextMenu->addActions(m_selectionModeGroup->actions());
    if (selectVector) m_contextMenu->insertSeparator(selectVector);
    connect(m_contextMenu, &QMenu::triggered, this, &Jy3DWidget::selectActionTriggered);
}

void Jy3DWidget::setSelectionMode(TopAbs_ShapeEnum mode) {
    if (!m_context.IsNull()) {
        // 清除当前选择
        m_context->ClearSelected(Standard_False);
        // 设置新的选择模式
        m_context->Deactivate();
        m_context->Activate(AIS_Shape::SelectionMode(mode));
        m_view->Redraw();
        qDebug() << "选择模式已切换到:" << static_cast<int>(mode);
    }
}

void Jy3DWidget::handleSelectedShape(const TopoDS_Shape &shape) {
    QJsonDocument json_doc;

    // 根据形状类型调用相应的转换函数
    switch (shape.ShapeType()) {
        case TopAbs_VERTEX:
            json_doc = vertexToJson(TopoDS::Vertex(shape));
            break;
        case TopAbs_EDGE:
            json_doc = edgeToJson(TopoDS::Edge(shape));
            break;
        case TopAbs_WIRE:
            json_doc = wireToJson(shape);
            break;
        case TopAbs_FACE:
            json_doc = faceToJson(TopoDS::Face(shape));
            break;
        case TopAbs_SHELL:
            json_doc = shellToJson(shape);
            break;
        case TopAbs_SOLID:
        case TopAbs_COMPSOLID:
            json_doc = solidToJson(shape);
            break;
        case TopAbs_COMPOUND:
            json_doc = compoundToJson(shape);
            break;
        default:
            return;
    }

    emit selectedShapeInfo(json_doc);
}

QJsonDocument Jy3DWidget::vertexToJson(const TopoDS_Vertex &vertex) {
    // 获取顶点的坐标和容差
    gp_Pnt point = BRep_Tool::Pnt(vertex);
    Standard_Real tolerance = BRep_Tool::Tolerance(vertex);

    QJsonObject jsonObj{
            {"vertex", QJsonObject({
                               {"x", point.X()},
                               {"y", point.Y()},
                               {"z", point.Z()},
                               {"tolerance", tolerance},
                       })},
    };
    return QJsonDocument(jsonObj);
}

QJsonDocument Jy3DWidget::edgeToJson(const TopoDS_Edge &edge) {
    static const std::unordered_map<int, std::string> type_map = {
            {GeomAbs_Line, "line"},
            {GeomAbs_Circle, "circle"},
            {GeomAbs_Ellipse, "ellipse"},
            {GeomAbs_Hyperbola, "hyperbola"},
            {GeomAbs_Parabola, "parabola"},
            {GeomAbs_BezierCurve, "bezier_curve"},
            {GeomAbs_BSplineCurve, "bspline_curve"},
            {GeomAbs_OffsetCurve, "offset_curve"},
            {GeomAbs_OtherCurve, "other_curve"}};

    BRepAdaptor_Curve curve(edge);
    const std::string type_str = type_map.find(curve.GetType()) != type_map.end() ? type_map.at(curve.GetType()) : "unknown";
    QString type = QString::fromStdString(type_str);

    // 获取边的起点和终点
    const gp_Pnt &start_point = BRep_Tool::Pnt(TopExp::FirstVertex(edge));
    const gp_Pnt &end_point = BRep_Tool::Pnt(TopExp::LastVertex(edge));

    // 计算边的长度
    GProp_GProps linear_props;
    BRepGProp::LinearProperties(edge, linear_props);
    Standard_Real length = linear_props.Mass();

    QJsonObject jsonObj{{
            {"edge", QJsonObject({
                             {"type", type},
                             {"length", length},
                             {"first", QJsonObject({
                                               {"x", start_point.X()},
                                               {"y", start_point.Y()},
                                               {"z", start_point.Z()},
                                       })},
                             {"last", QJsonObject({
                                              {"x", end_point.X()},
                                              {"y", end_point.Y()},
                                              {"z", end_point.Z()},
                                      })},
                     })},
    }};
    return QJsonDocument(jsonObj);
}

QJsonDocument Jy3DWidget::faceToJson(const TopoDS_Face &face) {
    static const std::unordered_map<int, std::string> type_map = {
            {GeomAbs_Plane, "plane"},
            {GeomAbs_Cylinder, "cylinder"},
            {GeomAbs_Cone, "cone"},
            {GeomAbs_Sphere, "sphere"},
            {GeomAbs_Torus, "torus"},
            {GeomAbs_BezierSurface, "bezier_surface"},
            {GeomAbs_BSplineSurface, "bspline_surface"},
            {GeomAbs_SurfaceOfRevolution, "surface_of_revolution"},
            {GeomAbs_SurfaceOfExtrusion, "surface_of_extrusion"},
            {GeomAbs_OffsetSurface, "offset_surface"},
            {GeomAbs_OtherSurface, "other_surface"}};

    BRepAdaptor_Surface surface(face);
    const std::string type_str = type_map.find(surface.GetType()) != type_map.end() ? type_map.at(surface.GetType()) : "unknown";
    QString type = QString::fromStdString(type_str);

    // 计算面积
    GProp_GProps surface_props;
    BRepGProp::SurfaceProperties(face, surface_props);
    Standard_Real area = surface_props.Mass();

    // 获取重心
    gp_Pnt center = surface_props.CentreOfMass();

    // 统计边的数量
    int edge_count = 0;
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
        edge_count++;
    }

    QJsonObject jsonObj{{
            {"face", QJsonObject({
                             {"type", type},
                             {"area", area},
                             {"edge_count", edge_count},
                             {"center", QJsonObject({
                                                {"x", center.X()},
                                                {"y", center.Y()},
                                                {"z", center.Z()},
                                        })},
                     })},
    }};
    return QJsonDocument(jsonObj);
}

QJsonDocument Jy3DWidget::wireToJson(const TopoDS_Shape &shape) {
    // 统计边的数量
    int edge_count = 0;
    for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next()) { edge_count++; }

    // 计算线框的总长度
    GProp_GProps linear_props;
    BRepGProp::LinearProperties(shape, linear_props);
    Standard_Real length = linear_props.Mass();

    const TopoDS_Wire &wire = TopoDS::Wire(shape);
    // 检查是否闭合
    bool is_closed = shape.Closed();

    QJsonObject jsonObj{{
            {"wire", QJsonObject({
                             {"edge_count", edge_count},
                             {"length", length},
                             {"is_closed", is_closed},
                     })},
    }};
    return QJsonDocument(jsonObj);
}

QJsonDocument Jy3DWidget::shellToJson(const TopoDS_Shape &shell) {
    // 统计面的数量
    int face_count = 0;
    for (TopExp_Explorer exp(shell, TopAbs_FACE); exp.More(); exp.Next()) { face_count++; }
    // 计算表面积
    GProp_GProps surface_props;
    BRepGProp::SurfaceProperties(shell, surface_props);
    Standard_Real area = surface_props.Mass();
    // 检查是否闭合
    bool is_closed = shell.Closed();
    QJsonObject jsonObj{{
            {"shell", QJsonObject({
                              {"face_count", face_count},
                              {"area", area},
                              {"is_closed", is_closed},
                      })},
    }};
    return QJsonDocument(jsonObj);
}

QJsonDocument Jy3DWidget::solidToJson(const TopoDS_Shape &solid) {
    // 计算体积
    GProp_GProps volume_props;
    BRepGProp::VolumeProperties(solid, volume_props);
    Standard_Real volume = volume_props.Mass();
    // 获取重心
    gp_Pnt center = volume_props.CentreOfMass();
    // 计算表面积
    GProp_GProps surface_props;
    BRepGProp::SurfaceProperties(solid, surface_props);
    Standard_Real area = surface_props.Mass();
    // 统计面的数量
    int face_count = 0;
    for (TopExp_Explorer exp(solid, TopAbs_FACE); exp.More(); exp.Next()) { face_count++; }
    QJsonObject jsonObj{{
            {"solid", QJsonObject({
                              {"volume", volume},
                              {"area", area},
                              {"face_count", face_count},
                              {"center", QJsonObject({
                                                 {"x", center.X()},
                                                 {"y", center.Y()},
                                                 {"z", center.Z()},
                                         })},
                      })},
    }};
    return QJsonDocument(jsonObj);
}

QJsonDocument Jy3DWidget::compoundToJson(const TopoDS_Shape &compound) {
    // 统计各种类型的子形状数量
    int vertex_count = 0, edge_count = 0, wire_count = 0;
    int face_count = 0, shell_count = 0, solid_count = 0;
    for (TopExp_Explorer exp(compound, TopAbs_VERTEX, TopAbs_EDGE); exp.More(); exp.Next()) vertex_count++;
    for (TopExp_Explorer exp(compound, TopAbs_EDGE, TopAbs_WIRE); exp.More(); exp.Next()) edge_count++;
    for (TopExp_Explorer exp(compound, TopAbs_WIRE, TopAbs_FACE); exp.More(); exp.Next()) wire_count++;
    for (TopExp_Explorer exp(compound, TopAbs_FACE, TopAbs_SHELL); exp.More(); exp.Next()) face_count++;
    for (TopExp_Explorer exp(compound, TopAbs_SHELL, TopAbs_SOLID); exp.More(); exp.Next()) shell_count++;
    for (TopExp_Explorer exp(compound, TopAbs_SOLID); exp.More(); exp.Next()) solid_count++;

    QJsonObject jsonObj{{
            {"compound", QJsonObject({
                                 {"vertex_count", vertex_count},
                                 {"edge_count", edge_count},
                                 {"wire_count", wire_count},
                                 {"face_count", face_count},
                                 {"shell_count", shell_count},
                                 {"solid_count", solid_count},
                         })},
    }};
    return QJsonDocument(jsonObj);
}