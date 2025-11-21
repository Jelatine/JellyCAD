/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_file_manager.h"
#include <QApplication>
#include <QClipboard>
#include <QDebug>
#include <QFile>
#include <QIcon>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QProcess>
#include <QSettings>
#include <QStyle>

JyFileManager::JyFileManager(QWidget *parent) : QWidget(parent), m_newFileItem(nullptr), m_newFileEditor(nullptr), m_renameFileItem(nullptr), m_renameFileEditor(nullptr), m_watcher(new QFileSystemWatcher(this)) {

    const QString default_dir = QApplication::applicationDirPath() + "/scripts";// 默认当前文件目录为应用程序目录下的scripts文件夹
    QDir dir_current(default_dir);
    if (!dir_current.exists()) { QDir(QApplication::applicationDirPath()).mkdir("scripts"); }
    // 读取软件配置
    settings = new QSettings("Jelatine", "JellyCAD", this);
    m_workingDirectory = settings->value("lastDirectory", default_dir).toString();
    if (!QDir(m_workingDirectory).exists()) { m_workingDirectory = default_dir; }

    // Create layout
    auto layout = new QVBoxLayout(this);

    // Create open folder button
    m_openFolderButton = new QPushButton("Open Folder", this);
    m_openFolderButton->setToolTip("Select working directory");
    layout->addWidget(m_openFolderButton);

    // Create file list widget
    m_fileList = new QListWidget(this);
    m_fileList->setContextMenuPolicy(Qt::CustomContextMenu);
    m_fileList->installEventFilter(this);
    m_fileList->viewport()->installEventFilter(this);
    layout->addWidget(m_fileList);

    // Connect signals
    connect(m_openFolderButton, &QPushButton::clicked, this, &JyFileManager::onOpenFolderClicked);
    connect(m_fileList, &QListWidget::itemDoubleClicked, this, &JyFileManager::onFileDoubleClicked);
    connect(m_fileList, &QListWidget::customContextMenuRequested, this, &JyFileManager::onFileListContextMenu);
    connect(m_watcher, &QFileSystemWatcher::fileChanged, this, &JyFileManager::onFileChanged);

    refreshFileList();
    updateWatcher();
}

void JyFileManager::onOpenFolderClicked() {
    QString dir = QFileDialog::getExistingDirectory(this, tr("Select Working Directory"),
                                                    m_workingDirectory,
                                                    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (dir.isEmpty()) return;
    if (dir == m_workingDirectory) {
        // 相同目录无需重置工作区
        refreshFileList();
        updateWatcher();
    } else {
        m_workingDirectory = dir;
        m_openedFilePath.clear();
        settings->setValue("lastDirectory", m_workingDirectory);
        settings->sync();
        refreshFileList();
        updateWatcher();
        // 发出工作目录已改变的信号
        emit resetWorkspace();
    }
}

void JyFileManager::onFileDoubleClicked(QListWidgetItem *item) {
    if (!item) return;

    QString fileName = item->text();
    QString filePath = m_workingDirectory + "/" + fileName;

    QFileInfo fileInfo(filePath);
    if (fileInfo.exists() && fileInfo.isFile()) {
        emit fileOpenRequested(filePath);
    }
}

void JyFileManager::onFileListContextMenu(const QPoint &pos) {
    QListWidgetItem *item = m_fileList->itemAt(pos);
    QMenu menu(this);

    if (item) {
        // File is selected - show file operations menu
        QAction *openAction = menu.addAction(tr("Open"));
        QAction *renameAction = menu.addAction(tr("Rename"));
        QAction *deleteAction = menu.addAction(tr("Delete"));
        QAction *copyAction = menu.addAction(tr("Copy"));
        menu.addSeparator();

        connect(openAction, &QAction::triggered, this, &JyFileManager::openSelectedFile);
        connect(renameAction, &QAction::triggered, this, &JyFileManager::renameSelectedFile);
        connect(deleteAction, &QAction::triggered, this, &JyFileManager::deleteSelectedFile);
        connect(copyAction, &QAction::triggered, this, &JyFileManager::copySelectedFile);
    } else {
        // Empty space - show new file option
        QAction *newFileAction = menu.addAction(tr("New File"));
        menu.addSeparator();

        connect(newFileAction, &QAction::triggered, this, &JyFileManager::createNewFile);
    }
    QAction *locationAction = menu.addAction(tr("Open File Location"));
    connect(locationAction, &QAction::triggered, this, &JyFileManager::openWorkingDirectory);

    menu.exec(m_fileList->mapToGlobal(pos));
}

void JyFileManager::refreshFileList() {
    m_fileList->clear();

    QDir dir(m_workingDirectory);
    if (!dir.exists()) {
        return;
    }

    // Filter for lua files
    QStringList filters;
    filters << "*.lua";
    dir.setNameFilters(filters);
    dir.setFilter(QDir::Files);
    dir.setSorting(QDir::Name);

    QStringList fileList = dir.entryList();

    // Add items with icon
    QIcon fileIcon = style()->standardIcon(QStyle::SP_FileIcon);
    for (const QString &fileName: fileList) {
        QListWidgetItem *item = new QListWidgetItem(fileIcon, fileName);
        m_fileList->addItem(item);
    }
}

void JyFileManager::createNewFile() {
    // If already creating a file, cancel it first
    if (m_newFileItem != nullptr) {
        cancelFileCreation();
    }

    // Create a new list item
    m_newFileItem = new QListWidgetItem();
    m_fileList->addItem(m_newFileItem);

    // Create an inline editor
    m_newFileEditor = new QLineEdit(m_fileList);
    m_newFileEditor->setText("");
    m_newFileEditor->installEventFilter(this);

    // Connect signals
    connect(m_newFileEditor, &QLineEdit::editingFinished, this, &JyFileManager::onEditorFinished);

    // Set the editor as the item widget
    m_fileList->setItemWidget(m_newFileItem, m_newFileEditor);
    m_newFileEditor->setFocus();
}

void JyFileManager::openSelectedFile() {
    onFileDoubleClicked(m_fileList->currentItem());
}

void JyFileManager::deleteSelectedFile() {
    QListWidgetItem *item = m_fileList->currentItem();
    if (!item) return;

    QString fileName = item->text();
    QString filePath = m_workingDirectory + "/" + fileName;

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, tr("Delete File"),
                                  tr("Are you sure you want to delete %1?").arg(fileName),
                                  QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        // Check if deleting the currently opened file
        bool isDeletingOpenedFile = (filePath == m_openedFilePath);

        // Remove from watcher before deleting
        if (m_watcher->files().contains(filePath)) {
            m_watcher->removePath(filePath);
        }

        QFile file(filePath);
        if (file.remove()) {
            // If deleted file was the opened file, notify and clear
            if (isDeletingOpenedFile) {
                m_openedFilePath.clear();
                emit resetWorkspace();
            }

            refreshFileList();
        } else {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to delete file!"));
        }
    }
}

void JyFileManager::copySelectedFile() {
    QListWidgetItem *item = m_fileList->currentItem();
    if (!item) return;

    QString fileName = item->text();
    QString filePath = m_workingDirectory + "/" + fileName;

    QFileInfo fileInfo(filePath);
    QString baseName = fileInfo.completeBaseName();
    QString extension = fileInfo.suffix();

    // Find a unique name
    int counter = 1;
    QString newFileName;
    QString newFilePath;
    do {
        newFileName = QString("%1_copy%2.%3").arg(baseName).arg(counter).arg(extension);
        newFilePath = m_workingDirectory + "/" + newFileName;
        counter++;
    } while (QFile::exists(newFilePath));

    if (QFile::copy(filePath, newFilePath)) {
        // Add new file to watcher
        m_watcher->addPath(newFilePath);

        refreshFileList();

        // Select the copied file
        QList<QListWidgetItem *> items = m_fileList->findItems(newFileName, Qt::MatchExactly);
        if (!items.isEmpty()) {
            m_fileList->setCurrentItem(items.first());
        }
    } else {
        QMessageBox::warning(this, tr("Error"),
                             tr("Failed to copy file!"));
    }
}

void JyFileManager::renameSelectedFile() {
    QListWidgetItem *item = m_fileList->currentItem();
    if (!item) return;

    // If already renaming a file, cancel it first
    if (m_renameFileItem != nullptr) {
        cancelFileRename();
    }

    // Store the original file name
    m_originalFileName = item->text();
    m_renameFileItem = item;

    // Create an inline editor
    m_renameFileEditor = new QLineEdit(m_fileList);

    // Set the current file name without extension for editing
    QFileInfo fileInfo(m_originalFileName);
    QString baseName = fileInfo.completeBaseName();
    m_renameFileEditor->setText(baseName);
    m_renameFileEditor->selectAll();
    m_renameFileEditor->installEventFilter(this);

    // Set the editor as the item widget
    m_fileList->setItemWidget(m_renameFileItem, m_renameFileEditor);
    m_renameFileEditor->setFocus();
}

void JyFileManager::finishFileRename() {
    if (!m_renameFileItem || !m_renameFileEditor) {
        return;
    }

    QString newBaseName = m_renameFileEditor->text().trimmed();

    // Remove the editor
    m_renameFileEditor->removeEventFilter(this);
    m_fileList->removeItemWidget(m_renameFileItem);
    delete m_renameFileEditor;
    m_renameFileEditor = nullptr;

    // If name is empty or unchanged, cancel rename
    if (newBaseName.isEmpty()) {
        m_renameFileItem = nullptr;
        m_originalFileName.clear();
        return;
    }

    // Ensure .lua extension
    QString newFileName = newBaseName;
    if (!newFileName.endsWith(".lua", Qt::CaseInsensitive)) {
        newFileName += ".lua";
    }

    // If name unchanged, just cancel
    if (newFileName == m_originalFileName) {
        m_renameFileItem = nullptr;
        m_originalFileName.clear();
        return;
    }

    QString oldFilePath = m_workingDirectory + "/" + m_originalFileName;
    QString newFilePath = m_workingDirectory + "/" + newFileName;

    // Check if new file name already exists
    if (QFile::exists(newFilePath)) {
        QMessageBox::warning(this, tr("File Exists"),
                             tr("A file with name '%1' already exists!").arg(newFileName));
        m_renameFileItem = nullptr;
        m_originalFileName.clear();
        return;
    }

    // Check if renaming the currently opened file
    bool isRenamingOpenedFile = (oldFilePath == m_openedFilePath);

    // Remove from watcher before renaming
    if (m_watcher->files().contains(oldFilePath)) {
        m_watcher->removePath(oldFilePath);
    }

    // Perform the rename
    QFile file(oldFilePath);
    if (file.rename(newFilePath)) {
        // Add new path to watcher
        m_watcher->addPath(newFilePath);

        // If renamed file was the opened file, update the path and notify
        if (isRenamingOpenedFile) {
            m_openedFilePath = newFilePath;
            // Emit signal to notify that the file path has changed
            emit fileOpenRequested(newFilePath);
        }

        refreshFileList();

        // Select the renamed file
        QList<QListWidgetItem *> items = m_fileList->findItems(newFileName, Qt::MatchExactly);
        if (!items.isEmpty()) {
            m_fileList->setCurrentItem(items.first());
        }
    } else {
        QMessageBox::warning(this, tr("Error"),
                             tr("Failed to rename file!"));
    }

    m_renameFileItem = nullptr;
    m_originalFileName.clear();
}

void JyFileManager::cancelFileRename() {
    if (!m_renameFileItem || !m_renameFileEditor) {
        return;
    }

    // Remove the editor
    m_renameFileEditor->removeEventFilter(this);
    m_fileList->removeItemWidget(m_renameFileItem);
    delete m_renameFileEditor;
    m_renameFileEditor = nullptr;
    m_renameFileItem = nullptr;
    m_originalFileName.clear();
}

void JyFileManager::onEditorFinished() {
    if (m_newFileEditor && m_newFileEditor->hasFocus()) {
        // editingFinished can be triggered multiple times, only process when editor has focus
        return;
    }
    finishFileCreation();
}

void JyFileManager::finishFileCreation() {
    if (!m_newFileItem || !m_newFileEditor) {
        return;
    }

    QString fileName = m_newFileEditor->text().trimmed();

    // Disconnect signals before cleanup
    disconnect(m_newFileEditor, &QLineEdit::editingFinished, this, &JyFileManager::onEditorFinished);
    m_newFileEditor->removeEventFilter(this);

    // Remove the temporary item
    m_fileList->removeItemWidget(m_newFileItem);
    delete m_newFileItem;
    m_newFileItem = nullptr;
    m_newFileEditor = nullptr;

    // If fileName is empty, don't create file
    if (fileName.isEmpty()) {
        return;
    }

    // Ensure .lua extension
    if (!fileName.endsWith(".lua", Qt::CaseInsensitive)) {
        fileName += ".lua";
    }

    QString filePath = m_workingDirectory + "/" + fileName;
    QFile file(filePath);

    if (file.exists()) {
        QMessageBox::warning(this, tr("File Exists"),
                             tr("File already exists!"));
        return;
    }

    if (file.open(QIODevice::WriteOnly)) {
        file.close();

        // Add new file to watcher
        m_watcher->addPath(filePath);

        refreshFileList();

        // Select the newly created file
        QList<QListWidgetItem *> items = m_fileList->findItems(fileName, Qt::MatchExactly);
        if (!items.isEmpty()) {
            m_fileList->setCurrentItem(items.first());
        }
    } else {
        QMessageBox::warning(this, tr("Error"),
                             tr("Failed to create file!"));
    }
}

void JyFileManager::cancelFileCreation() {
    if (!m_newFileItem || !m_newFileEditor) {
        return;
    }

    // Disconnect signals before cleanup
    disconnect(m_newFileEditor, &QLineEdit::editingFinished, this, &JyFileManager::onEditorFinished);
    m_newFileEditor->removeEventFilter(this);

    // Remove the temporary item
    m_fileList->removeItemWidget(m_newFileItem);
    delete m_newFileItem;
    m_newFileItem = nullptr;
    m_newFileEditor = nullptr;
}

bool JyFileManager::eventFilter(QObject *obj, QEvent *event) {
    if (obj == m_newFileEditor && event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Escape) {
            // Cancel file creation on ESC
            cancelFileCreation();
            return true;
        } else if (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter) {
            // Finish file creation on Enter
            finishFileCreation();
            return true;
        }
    } else if (obj == m_renameFileEditor && event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Escape) {
            // Cancel file rename on ESC
            cancelFileRename();
            return true;
        } else if (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter) {
            // Finish file rename on Enter
            finishFileRename();
            return true;
        }
    } else if (obj == m_fileList && event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Delete) {
            // Delete selected file when Delete key is pressed
            QListWidgetItem *item = m_fileList->currentItem();
            if (item && item != m_newFileItem && item != m_renameFileItem) {
                // Only delete if it's not the new file editor item or rename editor item
                deleteSelectedFile();
                return true;
            }
        } else if (keyEvent->key() == Qt::Key_F2) {
            // Rename selected file when F2 is pressed
            QListWidgetItem *item = m_fileList->currentItem();
            if (item && item != m_newFileItem && item != m_renameFileItem) {
                // Only rename if it's not the new file editor item or already renaming
                renameSelectedFile();
                return true;
            }
        } else if (keyEvent->key() == Qt::Key_N && keyEvent->modifiers() == Qt::ControlModifier) {
            // Create new file when Ctrl+N is pressed
            createNewFile();
            return true;
        }
    } else if (obj == m_fileList->viewport() && event->type() == QEvent::MouseButtonDblClick) {
        // Handle double click on list widget viewport
        QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
        QListWidgetItem *item = m_fileList->itemAt(mouseEvent->pos());
        if (!item) {
            // Double click on empty space - create new file
            createNewFile();
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

void JyFileManager::openWorkingDirectory() {
    QDir dir(m_workingDirectory);
    if (dir.exists()) {
#ifdef Q_OS_WIN
        QProcess::startDetached("explorer.exe", QStringList() << QDir::toNativeSeparators(m_workingDirectory));
#elif defined(Q_OS_MAC)
        QProcess::startDetached("open", QStringList() << m_workingDirectory);
#else
        QDesktopServices::openUrl(QUrl::fromLocalFile(m_workingDirectory));
#endif
    }
}

void JyFileManager::updateWatcher() {
    // Clear all watched files
    if (!m_watcher->files().isEmpty()) {
        m_watcher->removePaths(m_watcher->files());
    }

    // Add all lua files in working directory
    QDir dir(m_workingDirectory);
    if (!dir.exists()) {
        return;
    }

    QStringList filters;
    filters << "*.lua";
    dir.setNameFilters(filters);
    dir.setFilter(QDir::Files);

    QStringList fileList = dir.entryList();
    for (const QString &fileName: fileList) {
        QString filePath = m_workingDirectory + "/" + fileName;
        m_watcher->addPath(filePath);
    }
}

void JyFileManager::setOpenedFile(const QString &filePath) {
    m_openedFilePath = filePath;
}

void JyFileManager::onFileChanged(const QString &path) {
    qDebug() << "File changed in JyFileManager:" << path;

    // If the changed file is the opened file, emit signal
    if (!m_openedFilePath.isEmpty()) {
        emit openedFileChanged(path);
    }
}
