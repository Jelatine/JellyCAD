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
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

#include <AIS_InteractiveContext.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_View.hxx>
#include <Aspect_Handle.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <WNT_Window.hxx>

#include <BRepPrimAPI_MakeBox.hxx>
#include <AIS_Shape.hxx>
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
    //!覆写键盘按键按下事件
    void keyPressEvent(QKeyEvent *event);
    //!覆写键盘按键释放事件
    void keyReleaseEvent(QKeyEvent *event);
    //!覆写鼠标按键按下事件
    void mousePressEvent(QMouseEvent *event);
    //!覆写鼠标按键释放事件
    void mouseReleaseEvent(QMouseEvent *event);
    //!覆写鼠标移动事件
    void mouseMoveEvent(QMouseEvent *event);
    //!覆写鼠标滚轮事件
    void wheelEvent(QWheelEvent *event);

protected:
    //!三维场景转换模式
    enum CurrentAction3d
    {
        CurAction3d_Nothing,
        CurAction3d_DynamicPanning, //平移
        CurAction3d_DynamicZooming, //缩放
        CurAction3d_DynamicRotation //旋转
    };
private:
    Standard_Integer m_x_max;    //!记录鼠标平移坐标X
    Standard_Integer m_y_max;    //!记录鼠标平移坐标Y
    CurrentAction3d m_current_mode; //!三维场景转换模式
    bool m_shift_key_pressed;   //!记录Shift键是否被按下


signals:

public slots:
};

#endif // C3DWIDGET_H
