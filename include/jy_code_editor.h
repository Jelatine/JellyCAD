/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_CODE_EDITOR_H
#define JY_CODE_EDITOR_H

#include <QPlainTextEdit>
#include <QTextBlock>
#include <QPainter>
#include <QSyntaxHighlighter>
#include <QRegularExpression>

class JyCodeEditor : public QPlainTextEdit {
    QWidget *number_area_{nullptr};


// 行号显示子界面
    class NumberArea : public QWidget {
    public:
        explicit NumberArea(JyCodeEditor *_editor) : QWidget(_editor), editor_(_editor) {}

        [[nodiscard]] QSize sizeHint() const override { return {editor_->number_area_width(), 0}; }

    protected:
        void paintEvent(QPaintEvent *event) override { editor_->paint_line_number(event); }

    private:
        JyCodeEditor *editor_{nullptr};
    };

    class Highlighter : public QSyntaxHighlighter {
    public:
        explicit Highlighter(QTextDocument *parent = nullptr);

    private:
        struct HighlightingRule {
            QRegularExpression pattern;
            QTextCharFormat format;
        };
        QVector<HighlightingRule> highlightingRules;

        void m_append_keyword_list(const QStringList &_list, const QColor &_color);

        QRegularExpression m_comment_start_expression;
        QRegularExpression m_comment_end_expression;
        QTextCharFormat m_multi_line_comment_format;

    protected:
        void highlightBlock(const QString &text) override;
    };

public:
    explicit JyCodeEditor(QWidget *parent = nullptr);

    int number_area_width();

    void paint_line_number(QPaintEvent *event);

    void resizeEvent(QResizeEvent *event) override;

public slots:

    void slot_update_number_width(int) { setViewportMargins(number_area_width() + 8, 0, 0, 0); }

    void slot_update_number_area(const QRect &rect, int dy);
};

#endif //JY_CODE_EDITOR_H
