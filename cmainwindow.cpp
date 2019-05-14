/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : cmakewindow.cpp
#   Last Modified : 2019-04-21 11:00
#   Describe      : Main Window
#
# ====================================================*/

#include "cmainwindow.h"
#include "ui_cmainwindow.h"

CMainWindow::CMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CMainWindow)
{
    ui->setupUi(this);
    m_3d_widget = new C3DWidget(this);
    setCentralWidget(m_3d_widget);
}

CMainWindow::~CMainWindow()
{
    delete ui;
}

void CMainWindow::on_actionMkCube_triggered()
{
    m_3d_widget->make_cube();
}

void CMainWindow::on_actionMkCylinder_triggered()
{
    m_3d_widget->make_cylinder();
}

void CMainWindow::on_actionMkSphere_triggered()
{
    m_3d_widget->make_sphere();
}

void CMainWindow::on_actionMkCone_triggered()
{
    m_3d_widget->make_cone();
}

void CMainWindow::on_actionMkTorus_triggered()
{
    m_3d_widget->make_torus();
}
