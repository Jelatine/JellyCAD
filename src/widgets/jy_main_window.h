/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_MAIN_WINDOW_H
#define JY_MAIN_WINDOW_H

#include "jy_3d_widget.h"
#include "jy_activity_bar.h"
#include "jy_code_editor.h"
#include "jy_lua_virtual_machine.h"
#include "jy_shape.h"
#include <QDebug>
#include <QFileDialog>
#include <QFileSystemWatcher>
#include <QMainWindow>
#include <QSplitter>
#include <QTextBrowser>
#include <QProgressDialog>
#include <QTreeWidget>
#include <QPushButton>

class QSettings;
class JySearchWidget;

class JyMainWindow : public QMainWindow {
    Q_OBJECT
    Jy3DWidget *jy_3d_widget;
    QFileSystemWatcher *watcher;
    JyLuaVirtualMachine *lvm;
    JyCodeEditor *code_editor;
    JySearchWidget *search_widget;
    bool is_save_from_editor{false};//!< 从编辑器保存的标志, true 从本编辑器保存, false 外部保存
    QString current_file_dir;       //!< 当前文件的路径
    QTextBrowser *text_lua_message;
    QSettings *settings;
    QProgressDialog *m_progressDialog;
    QTreeWidget *treeShapeInfo;
    QPushButton *button_edge_info;
    QString current_edge_info;
public:
    explicit JyMainWindow(QWidget *parent = nullptr);
    
public slots:

    void slot_file_changed(const QString &path);

    void slot_button_new_clicked();

    void slot_button_open_clicked();

    void slot_button_save_clicked();

    void slot_shape_info(const QJsonDocument &doc);

    void slot_button_edge_info_clicked();

private slots:
    void showSearchWidget();
    void hideSearchWidget();
    void findNext();
    void findPrevious();
    void onSearchTextChanged(const QString &text);

    void onScriptStarted(const QString &fileName);
    void onScriptFinished(const QString &message);
    void onScriptError(const QString &error);
    void onScriptOutput(const QString &output);
    void onStopScript();


private:
    int ask_whether_to_save();//!< 询问是否保存已修改的文件 return 0 if save, 1 if not save, 2 if cancel

    void performSearch(bool backward = false);
    QString lastSearchText;

    void addJsonValue(QTreeWidgetItem *parent, const QString &key, const QJsonValue &value);

protected:
    void closeEvent(QCloseEvent *event) override;
};

#endif//JY_MAIN_WINDOW_H
