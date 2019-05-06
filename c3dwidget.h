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
///
/// \brief 三维显示窗口
///
class C3DWidget : public QWidget
{
    Q_OBJECT
public:
    explicit C3DWidget(QWidget *parent = nullptr);
private:
    //!交互式上下文能够管理一个或多个查看器(viewer)中的图形行为和交互式对象的选择
    Handle(AIS_InteractiveContext) m_context;
    //!定义查看器(viewer)类型对象上的服务
    Handle(V3d_Viewer) m_viewer;
    //!创建一个视图
    Handle(V3d_View) m_view;
    //!创建3d接口定义图形驱动程序
    Handle(Graphic3d_GraphicDriver) m_graphic_driver;
protected:
    //!覆写绘图事件
    void paintEvent(QPaintEvent *);
    //!覆写窗口尺寸变化事件
    void resizeEvent(QResizeEvent *);
    //!返回窗口的绘制引擎
    QPaintEngine *paintEngine() const;
signals:

public slots:
};

#endif // C3DWIDGET_H
