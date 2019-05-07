/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : c3dwidget.cpp
#   Last Modified : 2019-04-21 15:00
#   Describe      : 3D Widget
#
# ====================================================*/

#include "c3dwidget.h"

C3DWidget::C3DWidget(QWidget *parent) : QWidget(parent),
    m_shift_key_pressed(false)
{
    //若交互式上下文为空，则创建对象
    if (m_context.IsNull())
    {
        //此对象提供与X server的连接，在Windows和Mac OS中不起作用
        Handle(Aspect_DisplayConnection) m_display_donnection = new Aspect_DisplayConnection();
        //创建OpenGl图形驱动
        if (m_graphic_driver.IsNull())
        {
            m_graphic_driver = new OpenGl_GraphicDriver(m_display_donnection);
        }
        //获取QWidget的窗口系统标识符
        WId window_handle = (WId) winId();
        //创建Windows NT 窗口
        Handle(WNT_Window) wind = new WNT_Window((Aspect_Handle) window_handle);
        //创建3D查看器
        m_viewer = new V3d_Viewer(m_graphic_driver, Standard_ExtString("viewer3d"));
        //创建视图
        m_view = m_viewer->CreateView();
        m_view->SetWindow(wind);
        //打开窗口
        if (!wind->IsMapped())
        {
            wind->Map();
        }
        m_context = new AIS_InteractiveContext(m_viewer);  //创建交互式上下文
        //配置查看器的光照
        m_viewer->SetDefaultLights();
        m_viewer->SetLightOn();
        //设置视图的背景颜色为灰色
        m_view->SetBackgroundColor(Quantity_NOC_GRAY60);
        m_view->MustBeResized();
        //显示直角坐标系，可以配置在窗口显示位置、文字颜色、大小、样式
        m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_GOLD, 0.08, V3d_ZBUFFER);
        //设置显示模式
        m_context->SetDisplayMode(AIS_Shaded, Standard_True);
    }
    //配置QWidget
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setBackgroundRole( QPalette::NoRole );  //无背景
    setFocusPolicy( Qt::StrongFocus );
    setAttribute( Qt::WA_PaintOnScreen );
    setAttribute( Qt::WA_NoSystemBackground );
    setMouseTracking( true );   //开启鼠标位置追踪

    // 创建一个立方体作测试
    TopoDS_Shape t_topo_box = BRepPrimAPI_MakeBox(3.0, 4.0, 5.0).Shape();
    Handle(AIS_Shape) t_ais_box = new AIS_Shape(t_topo_box);
    t_ais_box->SetColor(Quantity_NOC_AZURE);
    m_context->Display(t_ais_box, Standard_True);
    m_view->FitAll();
}

void C3DWidget::paintEvent(QPaintEvent *)
{
    m_view->Redraw();
}

void C3DWidget::resizeEvent(QResizeEvent *)
{
    if( !m_view.IsNull() )
    {
        m_view->MustBeResized();
    }
}

QPaintEngine *C3DWidget::paintEngine() const
{
    return 0;
}

void C3DWidget::keyPressEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_Shift)
    {
        m_shift_key_pressed = true;
    }
}

void C3DWidget::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_Shift)
    {
        m_shift_key_pressed = false;
    }
}

void C3DWidget::mousePressEvent(QMouseEvent *event)
{
    if( ((event->buttons() & Qt::MidButton) && m_shift_key_pressed)  //平移方式1
        ||((event->buttons()&Qt::LeftButton)&&(event->buttons()&Qt::RightButton)))//平移方式2
    {
        m_current_mode = CurAction3d_DynamicPanning;
        m_x_max = event->pos().x(); //记录起始X位置
        m_y_max = event->pos().y(); //记录起始Y位置
    }
    else if(event->buttons() & Qt::MidButton)  //旋转
    {
        m_current_mode = CurAction3d_DynamicRotation;
        m_view->StartRotation(event->pos().x(), event->pos().y());
    }
    else
    {
        m_current_mode = CurAction3d_Nothing;
    }
}

void C3DWidget::mouseReleaseEvent(QMouseEvent *)
{
    m_current_mode = CurAction3d_Nothing;
}

void C3DWidget::mouseMoveEvent(QMouseEvent *event)
{
    switch (m_current_mode)
    {
    case CurAction3d_DynamicPanning:
        //执行平移
        m_view->Pan(event->pos().x() - m_x_max, m_y_max - event->pos().y());
        m_x_max = event->pos().x();
        m_y_max = event->pos().y();
        break;
    case CurAction3d_DynamicRotation:
        //执行旋转
        m_view->Rotation(event->pos().x(), event->pos().y());
        break;
    default:
        break;
    }
}

void C3DWidget::wheelEvent(QWheelEvent *event)
{
    m_view->Zoom(0, 0, event->angleDelta().y(), 0); //执行缩放
}
