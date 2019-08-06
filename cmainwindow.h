/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : cmakewindow.h
#   Last Modified : 2019-04-21 11:00
#   Describe      : Main Window
#
# ====================================================*/

#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>

#include <TopTools_HSequenceOfShape.hxx>

#include "c3dwidget.h"
namespace Ui {
class CMainWindow;
}

class CMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit CMainWindow(QWidget *parent = 0);
    ~CMainWindow();
    C3DWidget* m_3d_widget;
private slots:
    void on_actionMkCube_triggered();

    void on_actionMkCylinder_triggered();

    void on_actionMkSphere_triggered();

    void on_actionMkCone_triggered();

    void on_actionMkTorus_triggered();

    void on_actionImport_triggered();

    void on_actionExport_triggered();

private:
    Ui::CMainWindow *ui;
};

#endif // CMAINWINDOW_H
