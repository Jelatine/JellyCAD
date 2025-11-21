/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_FILE_MANAGER_H
#define JY_FILE_MANAGER_H

#include <QDesktopServices>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFileSystemWatcher>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QUrl>
#include <QVBoxLayout>
#include <QWidget>

class QSettings;

class JyFileManager : public QWidget {
    Q_OBJECT
public:
    explicit JyFileManager(QWidget *parent = nullptr);

    QString getWorkingDirectory() const { return m_workingDirectory; }
    void setOpenedFile(const QString &filePath);
    QString getOpenedFile() const { return m_openedFilePath; }

signals:
    void fileOpenRequested(const QString &filePath);
    void resetWorkspace();
    void openedFileChanged(const QString &filePath);

private slots:
    void onOpenFolderClicked();
    void onFileDoubleClicked(QListWidgetItem *item);
    void onFileListContextMenu(const QPoint &pos);
    void onEditorFinished();
    void onFileChanged(const QString &path);

private:
    void refreshFileList();
    void createNewFile();
    void openSelectedFile();
    void deleteSelectedFile();
    void copySelectedFile();
    void renameSelectedFile();
    void openWorkingDirectory();
    void finishFileCreation();
    void cancelFileCreation();
    void finishFileRename();
    void cancelFileRename();
    void updateWatcher();
    bool eventFilter(QObject *obj, QEvent *event) override;

    QString m_workingDirectory;
    QString m_openedFilePath;
    QListWidget *m_fileList;
    QPushButton *m_openFolderButton;
    QListWidgetItem *m_newFileItem;
    QLineEdit *m_newFileEditor;
    QListWidgetItem *m_renameFileItem;
    QLineEdit *m_renameFileEditor;
    QString m_originalFileName;
    QSettings *settings;
    QFileSystemWatcher *m_watcher;
};

#endif// JY_FILE_MANAGER_H
