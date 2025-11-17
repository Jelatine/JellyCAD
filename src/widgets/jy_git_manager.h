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
#include <QProcess>
#include <QPushButton>
#include <QQueue>
#include <QTextEdit>
#include <QTreeWidget>
#include <QWidget>

// Git命令结构体
struct GitCommand {
    QString command;
    QStringList args;
    QString commandType;// 用于识别命令类型
};

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
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onProcessError(QProcess::ProcessError error);

private:
    void setupUi();
    void checkGitInstallation();
    void checkRepositoryStatus();
    void loadBranches();
    void loadFileChanges();
    void loadCommitHistory();
    void loadRemotes();
    void enqueueCommand(const QString &command, const QStringList &args, const QString &commandType);
    void executeNextCommand();
    void executeGitCommand(const QString &command, const QStringList &args);
    void showDiffForFile(const QString &filePath);
    void updateStatusLabel();
    void updateFileOperationButtons();

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
    QPushButton *m_stageFileButton;
    QPushButton *m_unstageFileButton;
    QPushButton *m_stageAllButton;
    QPushButton *m_unstageAllButton;
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
    QProcess *m_gitProcess;
    QString m_currentCommand;         // 记录当前执行的命令类型
    QQueue<GitCommand> m_commandQueue;// 命令队列
    bool m_isProcessing;              // 是否正在处理命令
};
#endif// JY_GIT_MANAGER_H