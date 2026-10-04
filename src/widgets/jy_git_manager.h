/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_GIT_MANAGER_H
#define JY_GIT_MANAGER_H

#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include "services/jy_git_service.h"
#include <QPushButton>
#include <QQueue>
#include <QTextEdit>
#include <QTreeWidget>
#include <QWidget>

class JyGitManager : public QWidget {
    Q_OBJECT

public:
    explicit JyGitManager(QWidget *parent = nullptr);
    ~JyGitManager() override;

    void setWorkingDirectory(const QString &path);
    void refreshStatus();

signals:
    void statusChanged(const QString &status);
    void branchChanged(const QString &branch);
    void errorOccurred(const QString &error);
    void fileDiscarded(const QString &filePath);  // 文件修改被放弃，需要重新加载

private slots:
    void onRefreshClicked();
    void onInitRepoClicked();
    void onCommitClicked();
    void onPullClicked();
    void onPushClicked();
    void onBranchChanged(int index);
    void onAddRemoteClicked();
    void onRemoveRemoteClicked();
    void onFileItemDoubleClicked(QTreeWidgetItem *item, int column);
    void onFileTreeContextMenu(const QPoint &pos);
    void onStageAllClicked();
    void onUnstageAllClicked();
    void onProcessFinished(const QString &command, int exitCode, QProcess::ExitStatus exitStatus, const QString &output, const QString &errorOutput);

private:
    void setupUi();
    void checkGitInstallation();
    void checkRepositoryStatus();
    void loadBranches();
    void loadFileChanges();
    void loadCommitHistory();
    void loadRemotes();
    void enqueueCommand(const QString &command, const QStringList &args, const QString &commandType);
    void showDiffForFile(const QString &filePath);
    void updateStatusLabel();

    // UI组件
    QLabel *m_statusLabel;
    QComboBox *m_branchComboBox;
    QComboBox *m_remoteComboBox;
    QPushButton *m_refreshButton;
    QPushButton *m_initRepoButton;
    QPushButton *m_commitButton;
    QPushButton *m_pullButton;
    QPushButton *m_pushButton;
    QPushButton *m_addRemoteButton;
    QPushButton *m_removeRemoteButton;
    QPushButton *m_menuButton;
    QTreeWidget *m_fileChangesTree;
    QListWidget *m_commitHistoryList;
    QTextEdit *m_diffViewer;
    QLineEdit *m_commitMessageEdit;

    // GroupBox容器
    QGroupBox *m_diffGroup;
    QGroupBox *m_historyGroup;
    QWidget *m_branchWidget;
    QWidget *m_remoteWidget;

    // 菜单
    QMenu *m_operationsMenu;
    QMenu *m_branchMenu;
    QMenu *m_remoteMenu;

    // 数据
    QString m_workingDirectory;
    QString m_gitRepositoryRoot;// Git仓库根目录
    QString m_currentBranch;
    bool m_isGitInstalled;
    bool m_isGitRepository;
    JyGitService *m_service;
};
#endif
