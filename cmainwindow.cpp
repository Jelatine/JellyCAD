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

void CMainWindow::on_actionImport_triggered()
{
    // 获取ASSIMP支持的导入格式
    Importer t_importer;
    std::string szOut;
    t_importer.GetExtensionList(szOut); // ASSIMP支持的导入格式
    // 筛选文件格式
    QString t_assimp=tr("ASSIMP (") +QString::fromStdString(szOut) +tr(")");
    QString all_filter;
    all_filter+=t_assimp;
    // 获取被打开的文件路径
    QString filename = QFileDialog::getOpenFileName(this,tr("open file"),"D:/models",all_filter);
    if(filename.isEmpty())  // 若文件名为空，则不执行操作
    {
        return;
    }
    CModel *t_model = new CModel(filename); // 创建模型
    m_3d_widget->m_get_context()->Display(t_model,true);  // 显示模型
    // 选择整个导入的模型
    for(PrsMgr_ListOfPresentableObjectsIter i(t_model->Children());i.More();i.Next())
    {
        // 获取当前模型
        Handle(PrsMgr_PresentableObject) t_object = i.Value();
        // 若为CModel类型则转换为CModel
        if(t_object->IsKind(STANDARD_TYPE(CModel)))
        {
            Handle_CModel t_child_model =  Handle(CModel)::DownCast(t_object);
            // 选择该模型（注：不选择过模型就AddSelect可能会出错）
            m_3d_widget->m_get_context()->SetSelected(t_child_model,true);
        }
    }
    // 选择该模型
    m_3d_widget->m_get_context()->SetSelected(t_model,true);
    // 三维界面显示该模型的全部
    m_3d_widget->m_get_view()->FitAll(t_model->m_get_full_aabb());
}

void CMainWindow::on_actionExport_triggered()
{// 触发导出模型文档
    // 判断模型是否被选择
    if(m_3d_widget->m_get_context()->NbSelected() == 0)
    {
        // 无选择则弹出提示框
        QMessageBox::warning(this,tr("Export Error"),tr("There is no object selected!"));
        return; // 不执行操作
    }

    QString t_all_filter; // 所有文件过滤器
    QHash<QString,const char *> t_hash_format;  // 格式过滤器与文档描述哈希表
    Exporter t_export;  // 导出器
    for(int i=0; i<t_export.GetExportFormatCount(); i++)    // 遍历ASSIMP允许导出的格式
    {
        const aiExportFormatDesc *t_format_desc = t_export.GetExportFormatDescription(i);   // 获取每个文档描述
        // 文档过滤器
        QString t_single_format = QString(t_format_desc->description)+QString("(*.%1)").arg(t_format_desc->fileExtension);
        t_hash_format.insert(t_single_format,t_format_desc->id);    // 插入格式过滤器与文档描述哈希表
        t_all_filter += t_single_format;    // 添加单个文档过滤到整体过滤器
        if(i != t_export.GetExportFormatCount()-1)  // 最后一个不添加分行
        {
            t_all_filter+=";;"; // 分行
        }
    }
    QString t_selected_filter; // 被选择的过滤器
    // 打开文件保存提示框
    QString filename = QFileDialog::getSaveFileName(this,tr("Save"),".",t_all_filter,&t_selected_filter);
    if(filename.isEmpty())  // 若文件名为空，则不执行操作
    {
        return; // 不执行操作
    }
    // 导出模型文件
    CModel::m_export_model(filename,t_hash_format.value(t_selected_filter),m_3d_widget->m_get_context());
}
