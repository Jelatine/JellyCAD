/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_3d_widget.h"
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <BRepBndLib.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <QApplication>
#include <QMouseEvent>
#include <QDebug>
#include <StdSelect_BRepOwner.hxx>
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
                        emit selectedShape(JyShape(shape));
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
