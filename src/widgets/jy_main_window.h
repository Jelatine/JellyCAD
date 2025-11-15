/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_MAIN_WINDOW_H
#define JY_MAIN_WINDOW_H

#include "jy_3d_widget.h"
#include "jy_activity_bar.h"
#include "jy_editor_widget.h"
#include "jy_lua_virtual_machine.h"
#include "jy_shape.h"
#include "jy_shape_info_widget.h"
#include <QDebug>
#include <QFileDialog>
#include <QMainWindow>
#include <QProgressDialog>
#include <QSplitter>
#include <QTextBrowser>

class JyFileManager;
class JyGitManager;

class JyMainWindow : public QMainWindow {
    Q_OBJECT
    Jy3DWidget *jy_3d_widget;
    JyLuaVirtualMachine *lvm;
    JyEditorWidget *m_editorWidget;
    JyShapeInfoWidget *shape_info_widget;
    JyFileManager *file_manager;
    JyGitManager *git_manager;
    JyActivityBar *m_activity_bar;
    QTextBrowser *text_lua_message;
    QProgressDialog *m_progressDialog;
    bool m_isStoppingScript;

public:
    explicit JyMainWindow(QWidget *parent = nullptr);

public slots:

    void slot_file_changed(const QString &path);

    void slot_button_save_clicked();

    void slot_button_run_clicked();

    void onInsertEdgeInfo(const QString &edgeInfo);

    void onFileOpenRequested(const QString &filePath);

private slots:
    void onScriptStarted();
    void onScriptFinished(const QString &message);
    void onScriptError(const QString &error);
    void onScriptOutput(const QString &output);
    void onStopScript();
    void onResetWorkspace();


private:
    int ask_whether_to_save();//!< 询问是否保存已修改的文件 return 0 if save, 1 if not save, 2 if cancel
    bool saveFile();          //!< 保存文件 return true if saved successfully, false if cancelled or failed
    void runScript(const QString &path);

protected:
    void closeEvent(QCloseEvent *event) override;
};

#endif//JY_MAIN_WINDOW_H
