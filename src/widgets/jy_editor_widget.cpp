/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_editor_widget.h"
#include "jy_llm_dialog.h"
#include "jy_theme.h"
#include <QFile>
#include <QSaveFile>
#include <memory>
#include <QHBoxLayout>
#include <QTextCursor>
#include <QVBoxLayout>
#include <QToolButton>
#include <QMenu>
#include <QTabBar>

JyEditorWidget::JyEditorWidget(QWidget *parent)
    : QWidget(parent),
      m_codeEditor(new JyCodeEditor(this)),
      m_searchWidget(new JySearchWidget(this)),
      m_saveButton(new QPushButton(JyTheme::icon("save"), "Save", this)),
      m_runButton(new QPushButton(JyTheme::icon("play", JyTheme::color("text-on-accent")), "Run", this)),
      m_llmButton(new QPushButton(JyTheme::icon("sparkles"), "", this)) {
    m_runButton->setProperty("primary", true);
    m_llmButton->setFlat(true);
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
    buttonLayout->addWidget(m_runButton);
    buttonLayout->addWidget(m_saveButton);
    auto *codeButton = new QToolButton(this);
    codeButton->setObjectName("luaCodeButton");
    codeButton->setText(tr("Code"));
    codeButton->setIcon(JyTheme::icon("file-code"));
    codeButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    codeButton->setIconSize(QSize(16, 16));
    codeButton->setPopupMode(QToolButton::InstantPopup);
    auto *codeMenu = new QMenu(codeButton);
    codeMenu->addActions(m_codeEditor->codeActions());
    codeButton->setMenu(codeMenu);
    buttonLayout->addWidget(codeButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_llmButton);

    // Add widgets to main layout
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(m_codeEditor);
    mainLayout->addWidget(m_searchWidget);
    m_results = new QTabWidget(this);
    m_results->setObjectName("luaResults");
    m_results->setMaximumHeight(160);
    m_results->setDocumentMode(true);
    m_results->tabBar()->setDrawBase(false);
    m_results->setElideMode(Qt::ElideRight);
    m_problems = new QListWidget(m_results);
    m_problems->setObjectName("luaProblems");
    m_references = new QListWidget(m_results);
    m_references->setObjectName("luaReferences");
    m_results->addTab(m_problems, tr("Problems"));
    m_results->addTab(m_references, tr("References"));
    auto *closeResults = new QToolButton(m_results);
    closeResults->setObjectName("luaResultsClose");
    closeResults->setIcon(JyTheme::icon("x"));
    closeResults->setIconSize(QSize(16, 16));
    closeResults->setAutoRaise(true);
    closeResults->setToolTip(tr("Hide results"));
    m_results->setCornerWidget(closeResults);
    connect(closeResults, &QToolButton::clicked, m_results, &QWidget::hide);
    mainLayout->addWidget(m_results);
    m_results->hide();
    m_languageStatus = new QLabel(this);
    m_languageStatus->setObjectName("luaLanguageStatus");
    m_languageStatus->setProperty("class", "caption");
    m_languageStatus->setTextFormat(Qt::RichText);
    mainLayout->addWidget(m_languageStatus);
    connect(m_languageStatus, &QLabel::linkActivated, this, [this] {
        m_results->setCurrentWidget(m_problems); m_results->setVisible(!m_results->isVisible());
    });
    for (auto *list : {m_problems, m_references}) {
        list->setIconSize(QSize(14, 14));
        list->setUniformItemSizes(true);
        list->setTextElideMode(Qt::ElideRight);
        list->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        connect(list, &QListWidget::itemActivated, this, [this](QListWidgetItem *item) {
            m_codeEditor->goToPosition(item->data(Qt::UserRole).toInt());
        });
        connect(list, &QListWidget::itemClicked, this, [this](QListWidgetItem *item) {
            m_codeEditor->goToPosition(item->data(Qt::UserRole).toInt());
        });
    }
    connect(m_codeEditor, &JyCodeEditor::analysisUpdated, this, [this] {
        m_problems->clear();
        int errors = 0, warnings = 0;
        for (const auto &diagnostic : m_codeEditor->analysis().diagnostics) {
            const auto block = m_codeEditor->document()->findBlock(diagnostic.start);
            auto *item = new QListWidgetItem(tr("%1 · Line %2: %3")
                .arg(diagnostic.warning ? tr("Warning") : tr("Error")).arg(block.blockNumber() + 1).arg(diagnostic.message), m_problems);
            item->setIcon(JyTheme::icon("info", JyTheme::color(diagnostic.warning ? "warning" : "danger")));
            item->setData(Qt::UserRole, diagnostic.start);
            item->setToolTip(diagnostic.message);
            if (diagnostic.warning) ++warnings; else ++errors;
        }
        m_results->setTabText(0, tr("Problems (%1)").arg(errors + warnings));
        m_languageStatus->setText(
            QStringLiteral("<a href=\"problems\" style=\"color:%1; text-decoration:none;\">%2</a> · UTF-8 · 4 spaces")
                .arg(JyTheme::color(errors ? "danger" : warnings ? "warning" : "text-muted").name(),
                     tr("Lua: %1 errors, %2 warnings").arg(errors).arg(warnings).toHtmlEscaped()));
    });
    connect(m_codeEditor, &QPlainTextEdit::textChanged, this, [this] {
        m_references->clear(); // Stored offsets are invalid as soon as the document changes.
        m_problems->clear();
        m_languageStatus->setText(tr("Checking Lua…"));
    });
    connect(m_codeEditor, &JyCodeEditor::referencesFound, this, [this](const QVector<int> &positions) {
        m_references->clear();
        for (int position : positions) {
            const auto block = m_codeEditor->document()->findBlock(position);
            auto *item = new QListWidgetItem(tr("Line %1: %2").arg(block.blockNumber() + 1).arg(block.text().trimmed()), m_references);
            item->setIcon(JyTheme::icon("file-code"));
            item->setToolTip(block.text().trimmed());
            item->setData(Qt::UserRole, position);
        }
        m_results->setTabText(1, tr("References (%1)").arg(positions.size()));
        m_results->setCurrentWidget(m_references); m_results->show();
    });
    connect(m_codeEditor, &JyCodeEditor::editorMessage, this, [this](const QString &message) {
        m_languageStatus->setText(message.toHtmlEscaped());
    });

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
    const auto path = m_codeEditor->getFilePath();
    if (path.isEmpty()) return false;
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) return false;
    const auto bytes = m_codeEditor->get_text().toUtf8();
    if (file.write(bytes) != bytes.size() || !file.commit()) return false;
    m_codeEditor->document()->setModified(false);
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

    connect(dialog, &JyLlmDialog::codeGenerationFinished, this, [this](const QString &fullCode) {
        QTextCursor cursor(m_codeEditor->document());
        cursor.beginEditBlock();
        cursor.select(QTextCursor::Document);
        cursor.insertText(fullCode);
        cursor.endEditBlock();
        m_codeEditor->setTextCursor(cursor);
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
