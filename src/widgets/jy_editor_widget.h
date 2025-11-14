/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_EDITOR_WIDGET_H
#define JY_EDITOR_WIDGET_H

#include "jy_code_editor.h"
#include "jy_search_widget.h"
#include <QPushButton>
#include <QWidget>

class JyEditorWidget : public QWidget {
    Q_OBJECT

public:
    explicit JyEditorWidget(QWidget *parent = nullptr);

    // Access to code editor
    JyCodeEditor *codeEditor() const { return m_codeEditor; }

    // Access to search widget
    JySearchWidget *searchWidget() const { return m_searchWidget; }

    // File operations
    void loadFile(const QString &filePath);
    void clearEditor();
    bool saveFile();
    QString getFilePath() const;
    void setFilePath(const QString &filePath);
    bool isModified() const;

    // Search operations
    void showSearch();
    void hideSearch();
    void findNext();
    void findPrevious();

signals:
    void saveRequested();
    void runRequested();
    void modificationChanged(bool modified);
    void insertTextRequested(const QString &text);

private slots:
    void onSaveClicked();
    void onRunClicked();
    void onSearchTextChanged(const QString &text);
    void onSearchClosed();

private:
    void setupUi();
    void performSearch(bool backward);

    JyCodeEditor *m_codeEditor;
    JySearchWidget *m_searchWidget;
    QPushButton *m_saveButton;
    QPushButton *m_runButton;
    QString m_lastSearchText;
};

#endif// JY_EDITOR_WIDGET_H
