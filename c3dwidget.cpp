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

C3DWidget::C3DWidget(QWidget *parent) : QWidget(parent)
{
    if (m_context.IsNull())
    {
        Handle(Aspect_DisplayConnection) m_display_donnection = new Aspect_DisplayConnection();

        if (m_graphic_driver.IsNull())
        {
            m_graphic_driver = new OpenGl_GraphicDriver(m_display_donnection);
        }
        WId window_handle = (WId) winId();
        Handle(WNT_Window) wind = new WNT_Window((Aspect_Handle) window_handle);
        m_viewer = new V3d_Viewer(m_graphic_driver, Standard_ExtString("viewer3d"));
        m_view = m_viewer->CreateView();
        m_view->SetWindow(wind);
        if (!wind->IsMapped()) wind->Map();

        m_context = new AIS_InteractiveContext(m_viewer);

        m_viewer->SetDefaultLights();
        m_viewer->SetLightOn();

        m_view->SetBackgroundColor(Quantity_NOC_BLACK);
        m_view->MustBeResized();
        m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_GOLD, 0.08, V3d_ZBUFFER);

        m_context->SetDisplayMode(AIS_Shaded, Standard_True);
    }
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setBackgroundRole( QPalette::NoRole );
    setFocusPolicy( Qt::StrongFocus );
    setAttribute( Qt::WA_PaintOnScreen );
    setAttribute( Qt::WA_NoSystemBackground );
    setMouseTracking( true );
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
