/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_3D_WIDGET_H
#define JY_3D_WIDGET_H

#include <QApplication>
#include <QWidget>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ViewCube.hxx>
#include <AIS_Trihedron.hxx>
#include <V3d_Light.hxx>
#include "jy_shape.h"

class Jy3DWidget : public QWidget {
Q_OBJECT
    Handle(AIS_InteractiveContext) m_context; //!交互式上下文能够管理一个或多个查看器(viewer)中的图形行为和交互式对象的选择
    Handle(V3d_Viewer) m_viewer; //!定义查看器(viewer)类型对象上的服务
    Handle(V3d_View) m_view; //!创建一个视图
    Handle(AIS_ViewCube) view_cube; //!视方体
    Handle(AIS_Trihedron) origin_coord; //!基坐标系
    Handle(V3d_Light) light_direction; //! 定向光源
    Standard_Integer m_x_max{};    //!记录鼠标平移坐标X
    Standard_Integer m_y_max{};    //!记录鼠标平移坐标Y
public:
    explicit Jy3DWidget(QWidget *parent = nullptr);

    void remove_all();

    //!初始化交互环境
    void initialize_context();

private:
    void create_view_cube(); //!< 创建视方体

    void create_origin_coord(); //!< 创建基坐标系

public slots:

    void display(const JyShape &theIObj, const bool &with_coord);

protected:
    //!覆写绘图事件
    void paintEvent(QPaintEvent *event) override;

    //!覆写窗口尺寸变化事件
    void resizeEvent(QResizeEvent *event) override;

    //! 返回窗口的绘制引擎
    [[nodiscard]] QPaintEngine *paintEngine() const override { return nullptr; }

    //!覆写鼠标按键按下事件
    void mousePressEvent(QMouseEvent *event) override;

    //!覆写鼠标按键释放事件
    void mouseReleaseEvent(QMouseEvent *event) override;

    //!覆写鼠标移动事件
    void mouseMoveEvent(QMouseEvent *event) override;

    //!覆写鼠标滚轮事件
    void wheelEvent(QWheelEvent *event) override;
};

#endif //JY_3D_WIDGET_H
