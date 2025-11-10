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

    void setWorkingDirectory(const QString &path);
    QString getWorkingDirectory() const { return m_workingDirectory; }

signals:
    void fileOpenRequested(const QString &filePath);
    void switchToEditor();
    void workingDirectoryChanged(const QString &newPath);

private slots:
    void onOpenFolderClicked();
    void onFileDoubleClicked(QListWidgetItem *item);
    void onFileListContextMenu(const QPoint &pos);
    void onRefreshFileList();
    void onEditorFinished();

private:
    void refreshFileList();
    void createNewFile();
    void openSelectedFile();
    void deleteSelectedFile();
    void copySelectedFile();
    void openWorkingDirectory();
    void finishFileCreation();
    void cancelFileCreation();
    bool eventFilter(QObject *obj, QEvent *event) override;

    QString m_workingDirectory;
    QListWidget *m_fileList;
    QPushButton *m_openFolderButton;
    QListWidgetItem *m_newFileItem;
    QLineEdit *m_newFileEditor;
    QSettings *settings;
};

#endif// JY_FILE_MANAGER_H
