/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_editor_widget.h"
#include "jy_llm_dialog.h"
#include <QFile>
#include <QHBoxLayout>
#include <QTextCursor>
#include <QVBoxLayout>

JyEditorWidget::JyEditorWidget(QWidget *parent)
    : QWidget(parent),
      m_codeEditor(new JyCodeEditor(this)),
      m_searchWidget(new JySearchWidget(this)),
      m_saveButton(new QPushButton("Save", this)),
      m_runButton(new QPushButton("Run", this)),
      m_llmButton(new QPushButton("💬", this)) {
    m_llmButton->setMaximumWidth(30);
    m_llmButton->setMinimumWidth(30);
    m_llmButton->setStyleSheet("min-width:42px;padding-left:2;padding-right:2;font-size: 18px;");
    setupUi();
}

void JyEditorWidget::setupUi() {
    // Setup buttons
    m_saveButton->setToolTip("Ctrl+S");
    m_saveButton->setEnabled(false);
    m_runButton->setToolTip("F5");
    m_llmButton->setToolTip("AI Code Assistant");

    // Create layout
    auto mainLayout = new QVBoxLayout(this);

    // Button layout
    auto buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(m_saveButton);
    buttonLayout->addWidget(m_runButton);
    buttonLayout->addWidget(m_llmButton);

    // Add widgets to main layout
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(m_codeEditor);
    mainLayout->addWidget(m_searchWidget);

    // Connect signals
    connect(m_saveButton, &QPushButton::clicked, this, &JyEditorWidget::onSaveClicked);
    connect(m_runButton, &QPushButton::clicked, this, &JyEditorWidget::onRunClicked);
    connect(m_llmButton, &QPushButton::clicked, this, &JyEditorWidget::onLlmClicked);
    connect(m_codeEditor, &QPlainTextEdit::modificationChanged, m_saveButton, &QPushButton::setEnabled);
    connect(m_codeEditor, &QPlainTextEdit::modificationChanged, this, &JyEditorWidget::modificationChanged);

    // Search widget signals
    connect(m_searchWidget, &JySearchWidget::findNext, this, &JyEditorWidget::findNext);
    connect(m_searchWidget, &JySearchWidget::findPrevious, this, &JyEditorWidget::findPrevious);
    connect(m_searchWidget, &JySearchWidget::searchTextChanged, this, &JyEditorWidget::onSearchTextChanged);
    connect(m_searchWidget, &JySearchWidget::closed, this, &JyEditorWidget::onSearchClosed);
}

void JyEditorWidget::loadFile(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return;
    }

    m_codeEditor->set_text(file.readAll());
    file.close();

    m_codeEditor->document()->setModified(false);
    m_codeEditor->modificationChanged(false);
    m_codeEditor->setFilePath(filePath);
}

void JyEditorWidget::clearEditor() {
    m_codeEditor->clear();
    m_codeEditor->setFilePath("");
    m_codeEditor->document()->setModified(false);
}

bool JyEditorWidget::saveFile() {
    QString filePath = m_codeEditor->getFilePath();
    if (filePath.isEmpty()) {
        return false;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }

    file.write(m_codeEditor->get_text().toUtf8());
    file.close();

    m_codeEditor->document()->setModified(false);
    m_codeEditor->setFilePath(filePath);

    return true;
}

QString JyEditorWidget::getFilePath() const {
    return m_codeEditor->getFilePath();
}

void JyEditorWidget::setFilePath(const QString &filePath) {
    m_codeEditor->setFilePath(filePath);
}

bool JyEditorWidget::isModified() const {
    return m_codeEditor->document()->isModified();
}

void JyEditorWidget::showSearch() {
    if (m_searchWidget->isVisible()) {
        hideSearch();
    } else {
        m_searchWidget->show();
        m_searchWidget->focusSearchBox();

        // If there's selected text, use it as search keyword
        QTextCursor cursor = m_codeEditor->textCursor();
        if (cursor.hasSelection()) {
            m_searchWidget->setSearchText(cursor.selectedText());
        }
    }
}

void JyEditorWidget::hideSearch() {
    m_searchWidget->hide();
    m_codeEditor->setFocus();

    // Clear selection
    QTextCursor cursor = m_codeEditor->textCursor();
    cursor.clearSelection();
    m_codeEditor->setTextCursor(cursor);
}

void JyEditorWidget::findNext() {
    performSearch(false);
}

void JyEditorWidget::findPrevious() {
    performSearch(true);
}

void JyEditorWidget::onSaveClicked() {
    emit saveRequested();
}

void JyEditorWidget::onRunClicked() {
    emit runRequested();
}

void JyEditorWidget::onLlmClicked() {
    // Create and show LLM dialog
    auto dialog = new JyLlmDialog(this);

    // Set current code
    QString currentCode = m_codeEditor->get_text();
    dialog->setCurrentCode(currentCode);

    // Flag to track first stream update
    bool isFirstUpdate = true;

    // Connect stream updates to editor
    connect(dialog, &JyLlmDialog::codeStreamUpdate, this, [this, &isFirstUpdate](const QString &deltaText) {
        // On first update, clear the editor to replace old code
        if (isFirstUpdate) {
            m_codeEditor->clear();
            isFirstUpdate = false;
        }

        // Append delta text to editor in real-time
        QTextCursor cursor = m_codeEditor->textCursor();
        cursor.movePosition(QTextCursor::End);
        cursor.insertText(deltaText);
        m_codeEditor->setTextCursor(cursor);
        m_codeEditor->ensureCursorVisible();
    });

    // Connect generation finished - no need to do anything since we've been streaming
    connect(dialog, &JyLlmDialog::codeGenerationFinished, this, [this](const QString &fullCode) {
        Q_UNUSED(fullCode);
        // Code has already been streamed to the editor
    });

    dialog->exec();
    dialog->deleteLater();
}

void JyEditorWidget::onSearchTextChanged(const QString &text) {
    m_lastSearchText = text;
    if (!text.isEmpty()) {
        performSearch(false);
    } else {
        // Clear selection
        m_searchWidget->setFoundStatus(true);
        QTextCursor cursor = m_codeEditor->textCursor();
        cursor.clearSelection();
        m_codeEditor->setTextCursor(cursor);
    }
}

void JyEditorWidget::onSearchClosed() {
    hideSearch();
}

void JyEditorWidget::performSearch(bool backward) {
    if (m_lastSearchText.isEmpty()) {
        return;
    }

    QTextDocument::FindFlags flags;
    if (backward) {
        flags |= QTextDocument::FindBackward;
    }

    QTextCursor cursor = m_codeEditor->textCursor();
    QTextCursor newCursor = m_codeEditor->document()->find(m_lastSearchText, cursor, flags);

    if (newCursor.isNull()) {
        // If not found, search from beginning/end
        if (backward) {
            newCursor = m_codeEditor->document()->find(m_lastSearchText,
                                                       m_codeEditor->document()->characterCount(),
                                                       flags);
        } else {
            newCursor = m_codeEditor->document()->find(m_lastSearchText, 0, flags);
        }
    }

    if (!newCursor.isNull()) {
        m_codeEditor->setTextCursor(newCursor);
        m_searchWidget->setFoundStatus(true);
    } else {
        m_searchWidget->setFoundStatus(false);
    }
}
