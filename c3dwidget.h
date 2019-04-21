/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : c3dwidget.h
#   Last Modified : 2019-04-21 15:00
#   Describe      : 3D Widget
#
# ====================================================*/

#ifndef C3DWIDGET_H
#define C3DWIDGET_H

#include <QWidget>
#include <AIS_InteractiveContext.hxx>

#include <OpenGl_GraphicDriver.hxx>
#include <V3d_View.hxx>
#include <Aspect_Handle.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <WNT_Window.hxx>
class C3DWidget : public QWidget
{
    Q_OBJECT
public:
    explicit C3DWidget(QWidget *parent = nullptr);
private:
    Handle(AIS_InteractiveContext) m_context;
    Handle(V3d_Viewer) m_viewer;
    Handle(V3d_View) m_view;
    Handle(Graphic3d_GraphicDriver) m_graphic_driver;
protected:
    void paintEvent(QPaintEvent *);
    void resizeEvent(QResizeEvent *);
    QPaintEngine *paintEngine() const;
signals:

public slots:
};

#endif // C3DWIDGET_H
