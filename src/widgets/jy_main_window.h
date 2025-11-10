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
#include "jy_shape_info_widget.h"
#include <QDebug>
#include <QFileDialog>
#include <QFileSystemWatcher>
#include <QMainWindow>
#include <QProgressDialog>
#include <QSplitter>
#include <QTextBrowser>

class JySearchWidget;
class JyFileManager;

class JyMainWindow : public QMainWindow {
    Q_OBJECT
    Jy3DWidget *jy_3d_widget;
    QFileSystemWatcher *watcher;
    JyLuaVirtualMachine *lvm;
    JyCodeEditor *code_editor;
    JySearchWidget *search_widget;
    JyShapeInfoWidget *shape_info_widget;
    JyFileManager *file_manager;
    bool is_save_from_editor{false};//!< 从编辑器保存的标志, true 从本编辑器保存, false 外部保存
    QTextBrowser *text_lua_message;
    QProgressDialog *m_progressDialog;

public:
    explicit JyMainWindow(QWidget *parent = nullptr);

public slots:

    void slot_file_changed(const QString &path);

    void slot_button_save_clicked();

    void slot_button_run_clicked();

    void onInsertEdgeInfo(const QString &edgeInfo);

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
    bool saveFile();//!< 保存文件 return true if saved successfully, false if cancelled or failed

    void performSearch(bool backward = false);
    QString lastSearchText;

protected:
    void closeEvent(QCloseEvent *event) override;
};

#endif//JY_MAIN_WINDOW_H
