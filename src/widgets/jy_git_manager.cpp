/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_git_manager.h"
#include <QDebug>
#include <QDesktopServices>
#include <QDir>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QSplitter>
#include <QUrl>
#include <QVBoxLayout>

namespace {
    // 格式化 Git Diff 输出，添加语法高亮
    QString formatDiffOutput(const QString &diffText) {
        if (diffText.isEmpty()) {
            return "<pre style='color: #888888;'>No changes to display</pre>";
        }

        QString html = "<pre style='font-family: \"Courier New\", monospace; font-size: 9pt; line-height: 1.4;'>";

        QStringList lines = diffText.split('\n');
        for (const QString &line: lines) {
            QString escapedLine = line.toHtmlEscaped();

            if (line.startsWith("diff --git")) {
                // Diff header - 黄色
                html += QString("<span style='color: #FFD700; font-weight: bold;'>%1</span>\n").arg(escapedLine);
            } else if (line.startsWith("index ") || line.startsWith("new file mode") ||
                       line.startsWith("deleted file mode") || line.startsWith("similarity index") ||
                       line.startsWith("rename from") || line.startsWith("rename to")) {
                // Metadata - 灰色
                html += QString("<span style='color: #888888;'>%1</span>\n").arg(escapedLine);
            } else if (line.startsWith("+++")) {
                // New file marker - 亮绿色，粗体
                html += QString("<span style='color: #00FF88; font-weight: bold;'>%1</span>\n").arg(escapedLine);
            } else if (line.startsWith("---")) {
                // Old file marker - 亮红色，粗体
                html += QString("<span style='color: #FF6B6B; font-weight: bold;'>%1</span>\n").arg(escapedLine);
            } else if (line.startsWith("@@")) {
                // Hunk header - 青色，粗体
                html += QString("<span style='color: #4FC3F7; font-weight: bold;'>%1</span>\n").arg(escapedLine);
            } else if (line.startsWith("+")) {
                // Added lines - 绿色背景
                html += QString("<span style='color: #A5D6A7; background-color: rgba(76, 175, 80, 0.2);'>%1</span>\n").arg(escapedLine);
            } else if (line.startsWith("-")) {
                // Removed lines - 红色背景
                html += QString("<span style='color: #EF9A9A; background-color: rgba(244, 67, 54, 0.2);'>%1</span>\n").arg(escapedLine);
            } else if (line.trimmed().isEmpty()) {
                // Empty lines
                html += "\n";
            } else {
                // Context lines - 浅灰色
                html += QString("<span style='color: #B8BCC5;'>%1</span>\n").arg(escapedLine);
            }
        }

        html += "</pre>";
        return html;
    }
}// anonymous namespace

JyGitManager::JyGitManager(QWidget *parent)
    : QWidget(parent), m_isGitInstalled(false), m_isGitRepository(false), m_gitProcess(nullptr), m_isProcessing(false) {
    setupUi();
    checkGitInstallation();
}

JyGitManager::~JyGitManager() {
    if (m_gitProcess) {
        m_gitProcess->kill();
        m_gitProcess->waitForFinished();
        delete m_gitProcess;
    }
}

void JyGitManager::setupUi() {
    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(5, 5, 5, 5);
    mainLayout->setSpacing(5);

    // 1. 顶部：状态栏（始终显示）
    m_statusLabel = new QLabel(tr("Checking Git status..."));
    m_statusLabel->setWordWrap(true);
    mainLayout->addWidget(m_statusLabel);

    // 2. Initialize Repository按钮（非git仓库时显示）
    m_initRepoButton = new QPushButton(tr("Initialize Repository"));
    mainLayout->addWidget(m_initRepoButton);

    // 3. Commit区域（不使用 GroupBox）
    // 提交消息输入
    m_commitMessageEdit = new QLineEdit();
    m_commitMessageEdit->setPlaceholderText(tr("Enter commit message..."));
    mainLayout->addWidget(m_commitMessageEdit);

    // Commit 按钮和 Operations 按钮放在同一行
    auto buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(5);

    // 提交按钮
    m_commitButton = new QPushButton(tr("Commit"));
    buttonLayout->addWidget(m_commitButton);

    // 操作菜单按钮（只显示图标，最窄布局）
    m_menuButton = new QPushButton(tr("⚙"));
    m_menuButton->setMaximumWidth(30);// 设置为最窄
    m_menuButton->setMinimumWidth(30);
    m_menuButton->setStyleSheet("min-width:42px;padding-left:8;padding-right:0;font-size: 18px;");// 紧凑的左右内边距
    m_menuButton->setToolTip(tr("Git Operations"));
    buttonLayout->addWidget(m_menuButton);

    mainLayout->addLayout(buttonLayout);

    // 创建操作菜单
    m_operationsMenu = new QMenu(this);

    // 添加 Refresh 操作
    m_operationsMenu->addAction(tr("🔄 Refresh"), this, &JyGitManager::onRefreshClicked);
    m_operationsMenu->addSeparator();

    // 添加 Pull 和 Push 操作
    m_operationsMenu->addAction(tr("⬇ Pull"), this, &JyGitManager::onPullClicked);
    m_operationsMenu->addAction(tr("⬆ Push"), this, &JyGitManager::onPushClicked);
    m_operationsMenu->addSeparator();

    // 创建 Branch 子菜单
    m_branchMenu = new QMenu(tr("Branch"), this);
    m_operationsMenu->addMenu(m_branchMenu);

    // 创建 Remote 子菜单
    m_remoteMenu = new QMenu(tr("Remote"), this);
    m_remoteMenu->addAction(tr("Add Remote..."), this, &JyGitManager::onAddRemoteClicked);
    m_remoteMenu->addAction(tr("Remove Remote..."), this, &JyGitManager::onRemoveRemoteClicked);
    m_operationsMenu->addMenu(m_remoteMenu);

    // 绑定菜单到按钮
    m_menuButton->setMenu(m_operationsMenu);

    // 4. 文件变更列表（放在 Commit 按钮下方）
    m_fileChangesTree = new QTreeWidget();
    m_fileChangesTree->setHeaderLabels({tr("File"), tr("Status")});
    m_fileChangesTree->setRootIsDecorated(false);
    m_fileChangesTree->setAlternatingRowColors(true);
    m_fileChangesTree->header()->setStretchLastSection(false);
    m_fileChangesTree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_fileChangesTree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_fileChangesTree->setMinimumHeight(100);  // 设置最小高度，确保始终可见
    m_fileChangesTree->setContextMenuPolicy(Qt::CustomContextMenu);
    // 设置大小策略，允许垂直扩展
    m_fileChangesTree->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    mainLayout->addWidget(m_fileChangesTree);

    // 4. Diff Viewer（可折叠，默认隐藏）
    m_diffGroup = new QGroupBox(tr("Diff Viewer"));
    m_diffGroup->setCheckable(true);
    m_diffGroup->setChecked(false);// 默认隐藏
    auto diffLayout = new QVBoxLayout(m_diffGroup);
    m_diffViewer = new QTextEdit();
    m_diffViewer->setReadOnly(true);
    m_diffViewer->setFont(QFont("Courier New", 9));
    m_diffViewer->setMinimumHeight(150);
    m_diffViewer->setVisible(false);
    diffLayout->addWidget(m_diffViewer);
    mainLayout->addWidget(m_diffGroup);

    // 6. Commit History（可折叠，默认隐藏）
    m_historyGroup = new QGroupBox(tr("Commit History"));
    m_historyGroup->setCheckable(true);
    m_historyGroup->setChecked(false);// 默认隐藏
    auto historyLayout = new QVBoxLayout(m_historyGroup);
    m_commitHistoryList = new QListWidget();
    m_commitHistoryList->setMinimumHeight(150);
    m_commitHistoryList->setVisible(false);
    historyLayout->addWidget(m_commitHistoryList);
    mainLayout->addWidget(m_historyGroup);

    // 创建隐藏的ComboBox（用于内部数据管理，不显示在UI上）
    m_branchComboBox = new QComboBox();
    m_branchComboBox->setVisible(false);
    m_remoteComboBox = new QComboBox();
    m_remoteComboBox->setVisible(false);

    // 创建隐藏的按钮（保持兼容性）
    m_refreshButton = new QPushButton();
    m_refreshButton->setVisible(false);
    m_pullButton = new QPushButton();
    m_pullButton->setVisible(false);
    m_pushButton = new QPushButton();
    m_pushButton->setVisible(false);
    m_addRemoteButton = new QPushButton();
    m_addRemoteButton->setVisible(false);
    m_removeRemoteButton = new QPushButton();
    m_removeRemoteButton->setVisible(false);

    // 连接信号槽
    connect(m_initRepoButton, &QPushButton::clicked, this, &JyGitManager::onInitRepoClicked);
    connect(m_commitButton, &QPushButton::clicked, this, &JyGitManager::onCommitClicked);
    connect(m_fileChangesTree, &QTreeWidget::itemClicked,
            this, &JyGitManager::onFileItemDoubleClicked);
    connect(m_fileChangesTree, &QTreeWidget::customContextMenuRequested,
            this, &JyGitManager::onFileTreeContextMenu);

    // 连接可折叠GroupBox的toggled信号，控制内容显示/隐藏
    connect(m_diffGroup, &QGroupBox::toggled, m_diffViewer, &QWidget::setVisible);
    connect(m_historyGroup, &QGroupBox::toggled, m_commitHistoryList, &QWidget::setVisible);

    // 初始状态：禁用操作按钮
    m_commitButton->setEnabled(false);
    m_commitMessageEdit->setEnabled(false);
    m_initRepoButton->setEnabled(false);
    m_menuButton->setEnabled(false);
}

void JyGitManager::checkGitInstallation() {
    m_currentCommand = "check_git";

    if (!m_gitProcess) {
        m_gitProcess = new QProcess(this);
        connect(m_gitProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, &JyGitManager::onProcessFinished);
        connect(m_gitProcess, &QProcess::errorOccurred,
                this, &JyGitManager::onProcessError);
    }

    m_gitProcess->start("git", QStringList() << "--version");
}

void JyGitManager::checkRepositoryStatus() {
    if (!m_isGitInstalled) {
        return;
    }

    m_currentCommand = "check_repo";

    QDir dir(m_workingDirectory);
    if (!dir.exists()) {
        m_statusLabel->setText(tr("Working directory does not exist"));
        return;
    }

    m_gitProcess->setWorkingDirectory(m_workingDirectory);
    m_gitProcess->start("git", QStringList() << "rev-parse" << "--is-inside-work-tree");
}

void JyGitManager::setWorkingDirectory(const QString &path) {
    m_workingDirectory = path;
    m_gitRepositoryRoot.clear();// 清空旧的仓库根目录
    if (m_isGitInstalled) {
        checkRepositoryStatus();
    }
}

void JyGitManager::refreshStatus() {
    if (!m_isGitRepository) {
        return;
    }

    loadBranches();
    loadFileChanges();
    loadCommitHistory();
    loadRemotes();
}

void JyGitManager::loadBranches() {
    enqueueCommand("git", QStringList() << "branch" << "--list", "list_branches");
}

void JyGitManager::loadFileChanges() {
    enqueueCommand("git", QStringList() << "status" << "--porcelain", "status");
}

void JyGitManager::loadCommitHistory() {
    enqueueCommand("git", QStringList() << "log" << "--oneline" << "-20", "log");
}

void JyGitManager::executeGitCommand(const QString &command, const QStringList &args) {
    if (!m_isGitInstalled) {
        QMessageBox::warning(this, tr("Git Not Found"),
                             tr("Git is not installed on your system."));
        return;
    }

    // 使用Git仓库根目录（如果可用），否则使用工作目录
    QString workDir = m_gitRepositoryRoot.isEmpty() ? m_workingDirectory : m_gitRepositoryRoot;
    m_gitProcess->setWorkingDirectory(workDir);
    m_gitProcess->start(command, args);
}

void JyGitManager::showDiffForFile(const QString &filePath) {
    qDebug() << "Showing diff for file:" << filePath;
    qDebug() << "Repository root:" << m_gitRepositoryRoot;
    // 使用 -- 分隔符确保文件路径被正确识别
    enqueueCommand("git", QStringList() << "diff" << "HEAD" << "--" << filePath, "diff:" + filePath);
}

void JyGitManager::updateStatusLabel() {
    if (!m_isGitInstalled) {
        // Git未安装：显示提示和下载链接
        m_statusLabel->setText(tr("⚠ Git not installed - <a href=\"https://git-scm.com/downloads\">Download Git</a>"));
        m_statusLabel->setTextFormat(Qt::RichText);
        m_statusLabel->setOpenExternalLinks(true);

        // 隐藏所有git相关内容
        m_initRepoButton->setVisible(false);
        m_menuButton->setVisible(false);
        m_commitButton->setVisible(false);
        m_commitMessageEdit->setVisible(false);
        m_fileChangesTree->setVisible(false);
        m_diffGroup->setVisible(false);
        m_historyGroup->setVisible(false);
    } else if (!m_isGitRepository) {
        // 不是Git仓库：只显示初始化按钮
        m_statusLabel->setText(tr("📁 Not a Git repository"));
        m_statusLabel->setTextFormat(Qt::PlainText);

        // 显示初始化按钮
        m_initRepoButton->setVisible(true);
        m_initRepoButton->setEnabled(true);

        // 隐藏所有git相关内容
        m_menuButton->setVisible(false);
        m_commitButton->setVisible(false);
        m_commitMessageEdit->setVisible(false);
        m_fileChangesTree->setVisible(false);
        m_diffGroup->setVisible(false);
        m_historyGroup->setVisible(false);
    } else {
        // 是Git仓库：显示所有git相关内容
        m_statusLabel->setText(tr("✓ Git repository - Branch: %1").arg(m_currentBranch));
        m_statusLabel->setTextFormat(Qt::PlainText);

        // 隐藏初始化按钮
        m_initRepoButton->setVisible(false);

        // 显示所有git相关内容
        m_menuButton->setVisible(true);
        m_menuButton->setEnabled(true);
        m_commitButton->setVisible(true);
        m_commitMessageEdit->setVisible(true);
        m_fileChangesTree->setVisible(true);
        m_diffGroup->setVisible(true);
        m_historyGroup->setVisible(true);

        // 启用git操作
        m_commitButton->setEnabled(true);
        m_commitMessageEdit->setEnabled(true);
    }
}

void JyGitManager::onRefreshClicked() {
    if (m_isGitInstalled) {
        checkRepositoryStatus();
    } else {
        checkGitInstallation();
    }
}

void JyGitManager::onInitRepoClicked() {
    if (!m_isGitInstalled) {
        QMessageBox::warning(this, tr("Git Not Found"),
                             tr("Please install Git first: https://git-scm.com/downloads"));
        return;
    }

    if (m_workingDirectory.isEmpty()) {
        QMessageBox::warning(this, tr("No Working Directory"),
                             tr("Please set a working directory first"));
        return;
    }

    auto reply = QMessageBox::question(this, tr("Initialize Repository"),
                                       tr("Initialize a new Git repository in:\n%1").arg(m_workingDirectory),
                                       QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        enqueueCommand("git", QStringList() << "init", "init");
    }
}

void JyGitManager::onCommitClicked() {
    QString message = m_commitMessageEdit->text().trimmed();
    if (message.isEmpty()) {
        QMessageBox::warning(this, tr("Empty Message"),
                             tr("Please enter a commit message"));
        return;
    }

    // 检查是否有暂存的文件
    bool hasStagedFiles = false;
    for (int i = 0; i < m_fileChangesTree->topLevelItemCount(); ++i) {
        QTreeWidgetItem *item = m_fileChangesTree->topLevelItem(i);
        QString statusCode = item->data(0, Qt::UserRole).toString();
        if (!statusCode.isEmpty() && statusCode.at(0) != ' ' && statusCode.at(0) != '?') {
            hasStagedFiles = true;
            break;
        }
    }

    if (!hasStagedFiles) {
        QMessageBox::information(this, tr("No Staged Files"),
                                 tr("No files are staged for commit.\n\n"
                                    "Please stage files first by:\n"
                                    "- Right-clicking on files and selecting 'Stage'\n"
                                    "- Or selecting 'Stage All Files' from the menu"));
        return;
    }

    // 直接提交暂存的文件，不再自动 add -A
    enqueueCommand("git", QStringList() << "commit" << "-m" << message, "commit");
}

void JyGitManager::onPullClicked() {
    auto reply = QMessageBox::question(this, tr("Pull Changes"),
                                       tr("Pull changes from remote repository?"),
                                       QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        enqueueCommand("git", QStringList() << "pull", "pull");
    }
}

void JyGitManager::onPushClicked() {
    auto reply = QMessageBox::question(this, tr("Push Changes"),
                                       tr("Push changes to remote repository?"),
                                       QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        enqueueCommand("git", QStringList() << "push", "push");
    }
}

void JyGitManager::onBranchChanged(int index) {
    if (index < 0) return;

    QString branchName = m_branchComboBox->currentText();
    if (branchName.isEmpty()) return;

    // 移除可能的星号标记
    if (branchName.startsWith("★ ")) {
        branchName = branchName.mid(2);
    }

    // 如果选择的就是当前分支，不需要切换
    if (branchName == m_currentBranch) {
        return;
    }

    auto reply = QMessageBox::question(this, tr("Switch Branch"),
                                       tr("Switch to branch '%1'?").arg(branchName),
                                       QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        enqueueCommand("git", QStringList() << "checkout" << branchName, "checkout");
    } else {
        // 用户点击了 No，恢复到当前分支的选择
        m_branchComboBox->blockSignals(true);

        // 首先尝试查找带星号的当前分支
        int currentIndex = m_branchComboBox->findText("★ " + m_currentBranch);

        // 如果找不到，遍历所有项查找当前分支
        if (currentIndex < 0) {
            for (int i = 0; i < m_branchComboBox->count(); ++i) {
                QString itemText = m_branchComboBox->itemText(i);
                QString itemBranch = itemText;
                if (itemBranch.startsWith("★ ")) {
                    itemBranch = itemBranch.mid(2);
                }
                if (itemBranch == m_currentBranch) {
                    currentIndex = i;
                    break;
                }
            }
        }

        if (currentIndex >= 0) {
            m_branchComboBox->setCurrentIndex(currentIndex);
        }

        m_branchComboBox->blockSignals(false);
    }
}

void JyGitManager::onFileItemDoubleClicked(QTreeWidgetItem *item, int column) {
    if (!item) return;

    QString filePath = item->text(0);
    showDiffForFile(filePath);

    // 自动展开Diff Viewer（如果它是折叠的）
    if (!m_diffGroup->isChecked()) {
        m_diffGroup->setChecked(true);
    }
}

void JyGitManager::onFileTreeContextMenu(const QPoint &pos) {
    QTreeWidgetItem *item = m_fileChangesTree->itemAt(pos);
    QMenu menu(this);

    if (item) {
        // 获取文件状态
        QString statusCode = item->data(0, Qt::UserRole).toString();
        QChar stagedStatus = statusCode.isEmpty() ? QChar(' ') : statusCode.at(0);
        QChar workStatus = statusCode.length() > 1 ? statusCode.at(1) : QChar(' ');
        QString filePath = item->text(0);

        // 根据文件状态显示不同的菜单项
        if (stagedStatus != ' ' && stagedStatus != '?') {
            // 文件已暂存，提供 Unstage 选项
            QAction *unstageAction = menu.addAction(tr("📤 Unstage This File"));
            connect(unstageAction, &QAction::triggered, this, [this, filePath]() {
                enqueueCommand("git", QStringList() << "reset" << "HEAD" << filePath, "unstage_file");
            });
        }

        if (workStatus != ' ' || stagedStatus == '?') {
            // 文件有未暂存的修改或未跟踪，提供 Stage 选项
            QAction *stageAction = menu.addAction(tr("📥 Stage This File"));
            connect(stageAction, &QAction::triggered, this, [this, filePath]() {
                enqueueCommand("git", QStringList() << "add" << filePath, "stage_file");
            });
        }

        // 如果文件在工作区有未暂存的修改（不是未跟踪文件），提供放弃修改选项
        if (workStatus != ' ' && stagedStatus != '?') {
            QAction *discardAction = menu.addAction(tr("🗑 Discard Changes"));
            discardAction->setToolTip(tr("Discard uncommitted changes in working directory"));
            connect(discardAction, &QAction::triggered, this, [this, filePath]() {
                auto reply = QMessageBox::warning(this, tr("Discard Changes"),
                                                  tr("Are you sure you want to discard changes to:\n%1\n\n"
                                                     "This action cannot be undone!")
                                                          .arg(filePath),
                                                  QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
                if (reply == QMessageBox::Yes) {
                    // 使用 "discard_file:" + filePath 格式，以便在完成后能识别具体文件
                    enqueueCommand("git", QStringList() << "checkout" << "--" << filePath, "discard_file:" + filePath);
                }
            });
        }

        menu.addSeparator();
    }

    // 批量操作（始终显示）
    QAction *stageAllAction = menu.addAction(tr("📥 Stage All Files"));
    connect(stageAllAction, &QAction::triggered, this, &JyGitManager::onStageAllClicked);

    QAction *unstageAllAction = menu.addAction(tr("📤 Unstage All Files"));
    connect(unstageAllAction, &QAction::triggered, this, &JyGitManager::onUnstageAllClicked);

    menu.exec(m_fileChangesTree->mapToGlobal(pos));
}

void JyGitManager::onStageAllClicked() {
    auto reply = QMessageBox::question(this, tr("Stage All Files"),
                                       tr("Stage all modified files?"),
                                       QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        enqueueCommand("git", QStringList() << "add" << "-A", "stage_all");
    }
}

void JyGitManager::onUnstageAllClicked() {
    auto reply = QMessageBox::question(this, tr("Unstage All Files"),
                                       tr("Unstage all files?"),
                                       QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        enqueueCommand("git", QStringList() << "reset" << "HEAD", "unstage_all");
    }
}

void JyGitManager::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    QString output = QString::fromUtf8(m_gitProcess->readAllStandardOutput());
    QString errorOutput = QString::fromUtf8(m_gitProcess->readAllStandardError());

    if (m_currentCommand == "check_git") {
        if (exitCode == 0) {
            m_isGitInstalled = true;
            m_statusLabel->setText(tr("Git installed: %1").arg(output.trimmed()));
            // Git安装成功后，检查仓库状态
            if (!m_workingDirectory.isEmpty()) {
                checkRepositoryStatus();
            }
        } else {
            m_isGitInstalled = false;
            updateStatusLabel();
        }
    } else if (m_currentCommand == "check_repo") {
        if (exitCode == 0 && output.trimmed() == "true") {
            m_isGitRepository = true;
            // 获取Git仓库根目录
            enqueueCommand("git", QStringList() << "rev-parse" << "--show-toplevel", "get_repo_root");
        } else {
            m_isGitRepository = false;
            m_gitRepositoryRoot.clear();
            updateStatusLabel();
            m_fileChangesTree->clear();
            m_commitHistoryList->clear();
            m_diffViewer->clear();
        }
    } else if (m_currentCommand == "list_branches") {
        // 阻塞信号，避免在重新填充时触发 currentIndexChanged
        m_branchComboBox->blockSignals(true);
        m_branchComboBox->clear();

        // 清空分支菜单
        m_branchMenu->clear();

        QStringList branches = output.split('\n', Qt::SkipEmptyParts);
        for (const QString &branch: branches) {
            QString branchName = branch.trimmed();
            if (branchName.startsWith("* ")) {
                branchName = branchName.mid(2);
                m_currentBranch = branchName;
                m_branchComboBox->addItem("★ " + branchName);

                // 添加到菜单，当前分支显示勾选标记
                QAction *action = m_branchMenu->addAction("✓ " + branchName);
                action->setData(branchName);
                action->setEnabled(false);// 当前分支不可点击

                updateStatusLabel();
            } else {
                m_branchComboBox->addItem(branchName);

                // 添加到菜单
                QAction *action = m_branchMenu->addAction(branchName);
                action->setData(branchName);
                connect(action, &QAction::triggered, this, [this, branchName]() {
                    auto reply = QMessageBox::question(this, tr("Switch Branch"),
                                                       tr("Switch to branch '%1'?").arg(branchName),
                                                       QMessageBox::Yes | QMessageBox::No);
                    if (reply == QMessageBox::Yes) {
                        enqueueCommand("git", QStringList() << "checkout" << branchName, "checkout");
                    }
                });
            }
        }

        // 选中当前分支（内部数据管理）
        int currentIndex = m_branchComboBox->findText("★ " + m_currentBranch);
        if (currentIndex >= 0) {
            m_branchComboBox->setCurrentIndex(currentIndex);
        }
        // 恢复信号
        m_branchComboBox->blockSignals(false);
    } else if (m_currentCommand == "status") {
        m_fileChangesTree->clear();
        QStringList lines = output.split('\n', Qt::SkipEmptyParts);
        for (const QString &line: lines) {
            if (line.length() < 3) continue;

            // Git status --porcelain 格式: XY filename
            // X = 暂存区状态, Y = 工作区状态
            QChar stagedStatus = line.at(0);
            QChar workStatus = line.at(1);
            QString filePath = line.mid(3);

            QString statusText;
            // 优先显示暂存区状态
            if (stagedStatus == 'M') statusText = tr("Staged (Modified)");
            else if (stagedStatus == 'A')
                statusText = tr("Staged (Added)");
            else if (stagedStatus == 'D')
                statusText = tr("Staged (Deleted)");
            else if (stagedStatus == 'R')
                statusText = tr("Staged (Renamed)");
            else if (stagedStatus == 'C')
                statusText = tr("Staged (Copied)");
            // 如果暂存区没有变化，显示工作区状态
            else if (workStatus == 'M')
                statusText = tr("Modified");
            else if (workStatus == 'D')
                statusText = tr("Deleted");
            else if (stagedStatus == '?' && workStatus == '?')
                statusText = tr("Untracked");
            else
                statusText = QString("%1%2").arg(stagedStatus).arg(workStatus);

            auto item = new QTreeWidgetItem(m_fileChangesTree);
            item->setText(0, filePath);
            item->setText(1, statusText);
            // 存储原始状态用于后续判断
            item->setData(0, Qt::UserRole, QString("%1%2").arg(stagedStatus).arg(workStatus));
        }
    } else if (m_currentCommand == "log") {
        m_commitHistoryList->clear();
        QStringList lines = output.split('\n', Qt::SkipEmptyParts);
        for (const QString &line: lines) {
            m_commitHistoryList->addItem(line);
        }
    } else if (m_currentCommand.startsWith("diff:")) {
        // 格式化并高亮显示 diff 输出
        QString formattedDiff = formatDiffOutput(output);
        m_diffViewer->setHtml(formattedDiff);
    } else if (m_currentCommand == "init") {
        if (exitCode == 0) {
            QMessageBox::information(this, tr("Success"),
                                     tr("Git repository initialized successfully"));
            m_isGitRepository = true;
            updateStatusLabel();
            refreshStatus();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to initialize repository:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "stage_file" || m_currentCommand == "stage_all") {
        if (exitCode == 0) {
            // 暂存成功，刷新状态
            refreshStatus();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to stage files:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "unstage_file" || m_currentCommand == "unstage_all") {
        if (exitCode == 0) {
            // 取消暂存成功，刷新状态
            refreshStatus();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to unstage files:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand.startsWith("discard_file:")) {
        if (exitCode == 0) {
            // 提取文件路径（格式为 "discard_file:<filePath>"）
            QString filePath = m_currentCommand.mid(13);// 跳过 "discard_file:" 前缀

            // 发送信号通知文件已被放弃修改，需要重新加载
            emit fileDiscarded(filePath);

            // 放弃修改成功，刷新状态
            refreshStatus();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to discard changes:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "commit") {
        if (exitCode == 0) {
            QMessageBox::information(this, tr("Success"),
                                     tr("Changes committed successfully"));
            m_commitMessageEdit->clear();
            refreshStatus();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to commit:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "pull") {
        if (exitCode == 0) {
            QMessageBox::information(this, tr("Success"),
                                     tr("Changes pulled successfully:\n%1").arg(output));
            refreshStatus();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to pull:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "push") {
        if (exitCode == 0) {
            QMessageBox::information(this, tr("Success"),
                                     tr("Changes pushed successfully:\n%1").arg(output));
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to push:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "checkout") {
        if (exitCode == 0) {
            QMessageBox::information(this, tr("Success"),
                                     tr("Branch switched successfully"));
            refreshStatus();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to switch branch:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "list_remotes") {
        // 清空ComboBox（用于内部数据管理）
        m_remoteComboBox->clear();

        // 重建Remote菜单
        m_remoteMenu->clear();

        if (exitCode == 0 && !output.isEmpty()) {
            QStringList lines = output.split('\n', Qt::SkipEmptyParts);
            QSet<QString> remoteNames;       // 用于去重
            QMap<QString, QString> remoteMap;// 存储remote名称和URL的映射

            for (const QString &line: lines) {
                // 格式: origin  https://github.com/user/repo.git (fetch)
                QStringList parts = line.split('\t', Qt::SkipEmptyParts);
                if (parts.size() >= 2) {
                    QString remoteName = parts[0];
                    QString remoteUrl = parts[1].split(' ').first();// 移除 (fetch) 或 (push)
                    if (!remoteNames.contains(remoteName)) {
                        m_remoteComboBox->addItem(remoteName + " " + remoteUrl);
                        remoteNames.insert(remoteName);
                        remoteMap[remoteName] = remoteUrl;
                    }
                }
            }

            // 在菜单中显示现有的remote（只读信息）
            if (!remoteMap.isEmpty()) {
                for (auto it = remoteMap.constBegin(); it != remoteMap.constEnd(); ++it) {
                    QAction *infoAction = m_remoteMenu->addAction(QString("📍 %1: %2").arg(it.key(), it.value()));
                    infoAction->setEnabled(false);// 只是信息展示，不可点击
                }
                m_remoteMenu->addSeparator();
            }
        }

        // 添加操作菜单项
        m_remoteMenu->addAction(tr("➕ Add Remote..."), this, &JyGitManager::onAddRemoteClicked);
        m_remoteMenu->addAction(tr("➖ Remove Remote..."), this, &JyGitManager::onRemoveRemoteClicked);
    } else if (m_currentCommand == "add_remote") {
        if (exitCode == 0) {
            QMessageBox::information(this, tr("Success"),
                                     tr("Remote added successfully"));
            loadRemotes();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to add remote:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "remove_remote") {
        if (exitCode == 0) {
            QMessageBox::information(this, tr("Success"),
                                     tr("Remote removed successfully"));
            loadRemotes();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to remove remote:\n%1").arg(errorOutput));
        }
    } else if (m_currentCommand == "get_repo_root") {
        if (exitCode == 0) {
            // 保存Git仓库根目录（转换为本地路径分隔符）
            m_gitRepositoryRoot = QDir::fromNativeSeparators(output.trimmed());
            qDebug() << "Git repository root:" << m_gitRepositoryRoot;
            updateStatusLabel();
            refreshStatus();
        } else {
            qDebug() << "Failed to get repository root:" << errorOutput;
            m_gitRepositoryRoot = m_workingDirectory;// 回退到工作目录
            updateStatusLabel();
            refreshStatus();
        }
    }

    // 命令执行完成，标记为不在处理中，并执行队列中的下一个命令
    m_isProcessing = false;
    executeNextCommand();
}

void JyGitManager::onProcessError(QProcess::ProcessError error) {
    if (m_currentCommand == "check_git") {
        m_isGitInstalled = false;
        updateStatusLabel();
    } else {
        QString errorMsg;
        switch (error) {
            case QProcess::FailedToStart:
                errorMsg = tr("Failed to start Git process. Make sure Git is installed.");
                break;
            case QProcess::Crashed:
                errorMsg = tr("Git process crashed.");
                break;
            case QProcess::Timedout:
                errorMsg = tr("Git process timed out.");
                break;
            default:
                errorMsg = tr("Git process error: %1").arg(static_cast<int>(error));
                break;
        }

        emit errorOccurred(errorMsg);

        if (m_currentCommand != "check_git" && m_currentCommand != "check_repo") {
            QMessageBox::warning(this, tr("Process Error"), errorMsg);
        }
    }

    m_isProcessing = false;
    executeNextCommand();
}

void JyGitManager::enqueueCommand(const QString &command, const QStringList &args, const QString &commandType) {
    GitCommand cmd;
    cmd.command = command;
    cmd.args = args;
    cmd.commandType = commandType;
    m_commandQueue.enqueue(cmd);

    // 如果当前没有正在执行的命令，立即执行
    if (!m_isProcessing) {
        executeNextCommand();
    }
}

void JyGitManager::executeNextCommand() {
    if (m_isProcessing || m_commandQueue.isEmpty()) {
        return;
    }

    if (!m_gitProcess || m_gitProcess->state() == QProcess::Running) {
        return;
    }

    GitCommand cmd = m_commandQueue.dequeue();
    m_currentCommand = cmd.commandType;
    m_isProcessing = true;

    // 使用Git仓库根目录（如果可用），否则使用工作目录
    QString workDir = m_gitRepositoryRoot.isEmpty() ? m_workingDirectory : m_gitRepositoryRoot;
    m_gitProcess->setWorkingDirectory(workDir);
    m_gitProcess->start(cmd.command, cmd.args);
}

void JyGitManager::loadRemotes() {
    enqueueCommand("git", QStringList() << "remote" << "-v", "list_remotes");
}

void JyGitManager::onAddRemoteClicked() {
    if (!m_isGitRepository) {
        QMessageBox::warning(this, tr("Not a Repository"),
                             tr("Please initialize a Git repository first"));
        return;
    }

    bool ok;
    QString remoteName = QInputDialog::getText(this, tr("Add Remote"),
                                               tr("Remote name (e.g., origin):"),
                                               QLineEdit::Normal, "origin", &ok, Qt::WindowCloseButtonHint | Qt::MSWindowsFixedSizeDialogHint);
    if (!ok || remoteName.isEmpty()) {
        return;
    }

    QString remoteUrl = QInputDialog::getText(this, tr("Add Remote"),
                                              tr("Remote URL:"),
                                              QLineEdit::Normal, "", &ok, Qt::WindowCloseButtonHint | Qt::MSWindowsFixedSizeDialogHint);
    if (!ok || remoteUrl.isEmpty()) {
        return;
    }

    enqueueCommand("git", QStringList() << "remote" << "add" << remoteName << remoteUrl, "add_remote");
}

void JyGitManager::onRemoveRemoteClicked() {
    if (!m_isGitRepository) {
        return;
    }

    // 从内部ComboBox获取remote列表（如果有的话）
    QStringList remoteList;
    for (int i = 0; i < m_remoteComboBox->count(); ++i) {
        QString item = m_remoteComboBox->itemText(i);
        QString remoteName = item.split(" ").first();
        remoteList.append(remoteName);
    }

    QString remoteName;
    if (remoteList.isEmpty()) {
        QMessageBox::warning(this, tr("No Remote"),
                             tr("No remote repository found"));
        return;
    } else if (remoteList.size() == 1) {
        // 只有一个remote，直接使用
        remoteName = remoteList.first();
    } else {
        // 多个remote，让用户选择
        bool ok;
        remoteName = QInputDialog::getItem(this, tr("Remove Remote"),
                                           tr("Select remote to remove:"),
                                           remoteList, 0, false, &ok);
        if (!ok || remoteName.isEmpty()) {
            return;
        }
    }

    auto reply = QMessageBox::question(this, tr("Remove Remote"),
                                       tr("Remove remote '%1'?").arg(remoteName),
                                       QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        enqueueCommand("git", QStringList() << "remote" << "remove" << remoteName, "remove_remote");
    }
}
