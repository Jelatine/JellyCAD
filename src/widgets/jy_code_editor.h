/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_CODE_EDITOR_H
#define JY_CODE_EDITOR_H

#include "jy_lua_analysis.h"
#include <QCompleter>
#include <QTimer>
#include <QSet>
#include <memory>
#include <QPainter>
#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QSyntaxHighlighter>
#include <QTextBlock>

class JyCodeEditor : public QPlainTextEdit {
    Q_OBJECT
    QWidget *number_area_{nullptr};


    // 行号显示子界面
    class NumberArea : public QWidget {
    public:
        explicit NumberArea(JyCodeEditor *_editor) : QWidget(_editor), editor_(_editor) {}

        [[nodiscard]] QSize sizeHint() const override { return {editor_->number_area_width(), 0}; }

    protected:
        void paintEvent(QPaintEvent *event) override { editor_->paint_line_number(event); }
        void mousePressEvent(QMouseEvent *event) override;

    private:
        JyCodeEditor *editor_{nullptr};
    };

    class Highlighter : public QSyntaxHighlighter {
    public:
        explicit Highlighter(QTextDocument *parent = nullptr) : QSyntaxHighlighter(parent) {}
        struct HighlightingRule {
            QRegularExpression pattern;
            QTextCharFormat format;
        };
        void addRule(const HighlightingRule &rule) { highlightingRules.append(rule); }

    protected:
        void highlightBlock(const QString &text) override;

    private:
        QList<HighlightingRule> highlightingRules;
    };

    Highlighter *highlighter_;

    QStringList keyword_list_;

    void init_highlighter();

public:
    explicit JyCodeEditor(QWidget *parent = nullptr);

    void set_text(const QString &text);
    const JyLuaAnalysis &analysis();
    void refreshAnalysis();
    void formatCode();
    void goToDefinition();
    void findReferences();
    void goToPosition(int position);
    void toggleFold(int line);
    void unfoldAll();
    void requestCompletion();
    void showSignature();
    QList<QAction *> codeActions() const { return code_actions_; }

signals:
    void analysisUpdated();
    void referencesFound(const QVector<int> &positions);
    void editorMessage(const QString &message);


public:
    QString get_text() const;

    void setFilePath(const QString &filePath = {}) { m_filePath = filePath; }

    QString getFilePath() const { return m_filePath; }
    
    [[nodiscard]] QStringList keyword_list() const { return keyword_list_; }

private:

    int number_area_width();

    void paint_line_number(QPaintEvent *event);

    void resizeEvent(QResizeEvent *event) override;

public slots:

    void slot_update_number_width(int) { setViewportMargins(number_area_width() + 8, 0, 0, 0); }

    void slot_update_number_area(const QRect &rect, int dy);

protected:
    // 重写右键菜单事件
    void contextMenuEvent(QContextMenuEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;


private slots:
    // 在文件资源管理器中显示
    void showInExplorer();

private:
    bool is_CRLF{false};

    QString m_filePath;
    QString m_vscodeCmd;

    void toggleComment();
    void updateSelections();
    void applyFolds();
    void indentSelection(bool outdent);
    void autoOutdent();
    QString completionPrefix() const;
    void insertCompletion(const QString &text);
    int symbolPosition() const;

    std::unique_ptr<JyLuaAnalysis> analysis_;
    QTimer analysis_timer_;
    bool analysis_dirty_ = true;
    bool refreshing_ = false;
    QCompleter *completer_ = nullptr;
    QSet<int> folded_lines_;
    QList<QAction *> code_actions_;

};

#endif//JY_CODE_EDITOR_H
