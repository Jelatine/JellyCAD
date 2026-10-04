#pragma once

#include <QHash>
#include <QStringList>
#include <QVector>

// Document-local Lua language services. Offsets are UTF-16 QTextCursor positions.
class JyLuaAnalysis {
public:
    enum class Kind { Name, Keyword, Number, String, Comment, Symbol };
    struct Token {
        QString text;
        int start = 0, end = 0, line = 0;
        Kind kind = Kind::Symbol;
        int symbol = -1;
        int pair = -1;
    };
    struct Symbol {
        QString name, signature;
        int token = -1, scope = 0, visibleFrom = 0;
        bool local = false, parameter = false;
        QVector<int> references;
    };
    struct Scope { int parent = -1, start = 0, end = 0; };
    struct Fold { int first = 0, last = 0; };
    struct Diagnostic { int start = 0, length = 1; QString message; bool warning = false; };

    explicit JyLuaAnalysis(const QString &source = {});
    QString source;
    QVector<Token> tokens;
    QVector<Symbol> symbols;
    QVector<Scope> scopes;
    QVector<Fold> folds;
    QVector<Diagnostic> diagnostics;

    int tokenAt(int position) const;
    int definitionAt(int position) const;
    QVector<int> referencesAt(int position) const;
    QStringList completions(int position) const;
    QString signatureAt(int position, int *argument = nullptr) const;
    QString formatted() const;
    int indentationAfter(int position) const;
    bool isCode(int position) const;
    static QStringList keywords();
    static const QHash<QString, QString> &builtins();
    static const QHash<QString, QString> &methods();

private:
    QVector<int> code;
    int current = 0, scope = 0, parseDepth = 0;
    QHash<QString, int> globals;
    QHash<int, QString> memberNames;
    struct Use { int token, scope; QString name; };
    QVector<Use> unresolved;
    void lex();
    QString peek(int offset = 0) const;
    int take();
    bool accept(const QString &text);
    void block(const QStringList &ends = {});
    void statement();
    void expression(int minimum = 0);
    int primary();
    void expressions();
    void functionBody(int declaration, int opener, bool method = false);
    int declare(int token, bool local, bool parameter = false, const QString &name = {});
    void use(int token, const QString &name = {});
    int resolve(const QString &name, int atScope, int position) const;
    int enterScope(int position);
    void leaveScope(int end);
    void fold(int opener, int closer);
    void checkSyntax();
    QString bindingKey(const QString &name, int atScope, int position) const;
};
