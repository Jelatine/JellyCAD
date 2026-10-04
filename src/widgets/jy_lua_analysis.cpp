#include "jy_lua_analysis.h"
#include <QRegularExpression>
#include <QSet>
#include <QScopedValueRollback>
#include <algorithm>
extern "C" {
#include <lua.h>
#include <lauxlib.h>
}

QStringList JyLuaAnalysis::keywords() {
    return QStringLiteral("and break do else elseif end false for function goto if in local nil not or repeat return then true until while").split(' ');
}

const QHash<QString, QString> &JyLuaAnalysis::builtins() {
    static const QHash<QString, QString> values = {
        {"assert", "assert(value, message)"}, {"error", "error(message, level)"},
        {"print", "print(...)"}, {"pairs", "pairs(table)"}, {"ipairs", "ipairs(table)"},
        {"next", "next(table, index)"}, {"type", "type(value)"}, {"tostring", "tostring(value)"},
        {"tonumber", "tonumber(value, base)"}, {"require", "require(module)"},
        {"pcall", "pcall(function, ...)"}, {"xpcall", "xpcall(function, handler, ...)"},
        {"select", "select(index, ...)"}, {"setmetatable", "setmetatable(table, metatable)"},
        {"getmetatable", "getmetatable(value)"}, {"rawget", "rawget(table, index)"},
        {"rawset", "rawset(table, index, value)"}, {"rawequal", "rawequal(a, b)"},
        {"rawlen", "rawlen(value)"}, {"load", "load(chunk, chunkname, mode, env)"},
        {"loadfile", "loadfile(filename, mode, env)"}, {"dofile", "dofile(filename)"},
        {"collectgarbage", "collectgarbage(option, arg)"},
        {"math.abs", "math.abs(x)"}, {"math.sin", "math.sin(x)"}, {"math.cos", "math.cos(x)"},
        {"math.tan", "math.tan(x)"}, {"math.sqrt", "math.sqrt(x)"}, {"math.floor", "math.floor(x)"},
        {"math.ceil", "math.ceil(x)"}, {"math.min", "math.min(...)"}, {"math.max", "math.max(...)"},
        {"math.rad", "math.rad(degrees)"}, {"math.deg", "math.deg(radians)"},
        {"math.random", "math.random(m, n)"}, {"math.pi", {}}, {"math.huge", {}},
        {"string.format", "string.format(format, ...)"}, {"string.sub", "string.sub(s, i, j)"},
        {"string.find", "string.find(s, pattern, init, plain)"}, {"string.gsub", "string.gsub(s, pattern, replacement, n)"},
        {"string.match", "string.match(s, pattern, init)"}, {"string.len", "string.len(s)"},
        {"string.rep", "string.rep(s, n, separator)"}, {"string.lower", "string.lower(s)"},
        {"string.upper", "string.upper(s)"}, {"table.insert", "table.insert(list, [position,] value)"},
        {"table.remove", "table.remove(list, position)"}, {"table.concat", "table.concat(list, separator, i, j)"},
        {"table.sort", "table.sort(list, compare)"}, {"table.unpack", "table.unpack(list, i, j)"},
        {"io.open", "io.open(filename, mode)"}, {"io.lines", "io.lines(filename, ...)"},
        {"os.clock", "os.clock()"}, {"os.time", "os.time(table)"}, {"os.date", "os.date(format, time)"},
        {"show", "show(object)"}, {"box.new", "box.new([x, y, z])"},
        {"sphere.new", "sphere.new([radius])"}, {"cylinder.new", "cylinder.new([radius, height])"},
        {"cone.new", "cone.new([r1, r2, height])"}, {"torus.new", "torus.new([majorRadius, minorRadius, angle])"},
        {"shape.new", "shape.new([filename])"}, {"axes.new", "axes.new([pose, length])"},
        {"vertex.new", "vertex.new(x, y, z)"}, {"line.new", "line.new(first, last)"},
        {"circle.new", "circle.new(position, direction, radius)"},
        {"ellipse.new", "ellipse.new(position, direction, majorRadius, minorRadius)"},
        {"arc.new", "arc.new(first, middle, last)"}, {"bezier.new", "bezier.new(points [, weights])"},
        {"bspline.new", "bspline.new(points [, knots, multiplicities, degree])"},
        {"wire.new", "wire.new([edges])"}, {"polygon.new", "polygon.new([points])"},
        {"face.new", "face.new([shape])"}, {"text.new", "text.new([text, height, font])"},
        {"plane.new", "plane.new(position, direction, uv)"},
        {"edge.new", "edge.new(kind, position, direction [, r1, r2])"},
        {"wedge.new", "wedge.new([dx, dy, dz, ltx])"},
        {"hyperbola.new", "hyperbola.new(position, direction, majorRadius, minorRadius, first, last)"},
        {"parabola.new", "parabola.new(position, direction, focal, first, last)"},
        {"cylindrical.new", "cylindrical.new(position, direction, radius, uv)"},
        {"conical.new", "conical.new(position, direction, angle, radius, uv)"},
        {"link.new", "link.new(name, shapes)"}, {"joint.new", "joint.new(name, axes, type [, limits])"}
    };
    return values;
}

const QHash<QString, QString> &JyLuaAnalysis::methods() {
    // Method suggestions are an API catalogue, not inferred receiver types.
    static const QHash<QString, QString> values = {
        {"copy", "copy()"}, {"type", "type()"}, {"empty", "empty()"},
        {"show", "show()"}, {"get_edge", "get_edge(filter)"}, {"get_face", "get_face(type, area, center, uv)"},
        {"fuse", "fuse(other)"}, {"cut", "cut(other)"}, {"common", "common(other)"},
        {"fillet", "fillet(radius [, edgesOrFilter])"}, {"chamfer", "chamfer(distance [, filter])"},
        {"prism", "prism(x, y, z)"}, {"revol", "revol(position, direction, angle)"},
        {"pipe", "pipe(wire)"}, {"thick", "thick(face, offset)"},
        {"scale", "scale(factor)"}, {"mirror", "mirror(position, direction)"},
        {"x", "x(value)"}, {"y", "y(value)"}, {"z", "z(value)"},
        {"rx", "rx(degrees)"}, {"ry", "ry(degrees)"}, {"rz", "rz(degrees)"},
        {"pos", "pos(x, y, z)"}, {"rot", "rot(rx, ry, rz)"},
        {"move", "move(kind, x [, y, z])"}, {"zero", "zero()"},
        {"locate", "locate(x, y, z, rx, ry, rz) / locate(shape)"},
        {"color", "color(color)"}, {"transparency", "transparency(value)"}, {"mass", "mass(value)"},
        {"export_step", "export_step(filename)"}, {"export_iges", "export_iges(filename)"},
        {"export_stl", "export_stl(filename [, options])"},
        {"add", "add(joint)"}, {"next", "next(link)"}, {"export", "export(nameOrOptions)"},
        {"sdh", "sdh(alpha, a, d, theta)"}, {"mdh", "mdh(alpha, a, d, theta)"}
    };
    return values;
}

JyLuaAnalysis::JyLuaAnalysis(const QString &text) : source(text) {
    scopes.append({-1, 0, int(source.size())});
    lex();
    block();
    for (const auto &item : unresolved) {
        const int id = resolve(item.name, item.scope, tokens[item.token].start);
        if (id >= 0) { tokens[item.token].symbol = id; symbols[id].references.append(item.token); }
    }
    checkSyntax();
    // Avoid misleading secondary warnings while the document is incomplete.
    if (diagnostics.isEmpty()) {
        for (const auto &s : symbols) {
            if (s.local && !s.parameter && s.references.isEmpty() && !s.name.startsWith('_'))
                diagnostics.append({tokens[s.token].start, tokens[s.token].end - tokens[s.token].start,
                                    QStringLiteral("Unused local '%1'").arg(s.name), true});
        }
    }
    std::sort(folds.begin(), folds.end(), [](const Fold &a, const Fold &b) {
        return a.first == b.first ? a.last > b.last : a.first < b.first;
    });
    // One gutter control per line, using the outermost range.
    for (int i = folds.size() - 1; i > 0; --i)
        if (folds[i].first == folds[i - 1].first) folds.removeAt(i);
}

void JyLuaAnalysis::lex() {
    const auto words = keywords();
    const QSet<QString> keywordSet(words.begin(), words.end());
    int line = 0;
    QVector<int> delimiters;
    for (int i = 0; i < source.size();) {
        if (source[i].isSpace()) { if (source[i] == '\n') ++line; ++i; continue; }
        const int start = i, firstLine = line;
        Kind kind = Kind::Symbol;
        bool comment = source.mid(i, 2) == "--" || (i == 0 && source.mid(i, 2) == "#!");
        if (comment) i += 2;
        int equals = 0;
        if (i < source.size() && source[i] == '[') {
            int p = i + 1;
            while (p < source.size() && source[p] == '=') { ++p; ++equals; }
            if (p < source.size() && source[p] == '[') {
                const auto close = "]" + QString(equals, '=') + "]";
                const int end = source.indexOf(close, p + 1);
                i = end < 0 ? source.size() : end + close.size();
                kind = comment ? Kind::Comment : Kind::String;
            }
        }
        if (i == start || (comment && i == start + 2)) {
            if (comment) {
                while (i < source.size() && source[i] != '\n') ++i;
                kind = Kind::Comment;
            } else if (source[i] == '\'' || source[i] == '"') {
                const auto quote = source[i++];
                while (i < source.size()) {
                    if (source[i] == '\\') {
                        ++i;
                        if (i < source.size() && source[i] == 'z') {
                            ++i; while (i < source.size() && source[i].isSpace()) ++i;
                        } else if (i < source.size()) {
                            if (source[i] == '\r' && i + 1 < source.size() && source[i + 1] == '\n') ++i;
                            ++i;
                        }
                    } else if (source[i++] == quote) break;
                    else if (source[i - 1] == '\n') break;
                }
                kind = Kind::String;
            } else if (source[i].isLetter() || source[i] == '_') {
                while (i < source.size() && (source[i].isLetterOrNumber() || source[i] == '_')) ++i;
                kind = keywordSet.contains(source.mid(start, i - start)) ? Kind::Keyword : Kind::Name;
            } else if (source[i].isDigit() || (source[i] == '.' && i + 1 < source.size() && source[i + 1].isDigit())) {
                static const QRegularExpression number("(?:0[xX][0-9a-fA-F]+(?:\\.[0-9a-fA-F]*)?(?:[pP][+-]?[0-9]+)?|(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?)");
                const auto match = number.match(source, i, QRegularExpression::NormalMatch, QRegularExpression::AnchorAtOffsetMatchOption);
                i += qMax(1, int(match.capturedLength()));
                kind = Kind::Number;
            } else {
                static const QSet<QString> two = {"==", "~=", "<=", ">=", "//", "<<", ">>", "..", "::"};
                i += source.mid(i, 3) == "..." ? 3 : two.contains(source.mid(i, 2)) ? 2 : 1;
            }
        }
        const QString value = source.mid(start, i - start);
        tokens.append({value, start, i, firstLine, kind});
        const int t = tokens.size() - 1;
        line += value.count('\n');
        if (kind == Kind::Comment || kind == Kind::String) {
            if (line > firstLine) folds.append({firstLine, line});
        }
        if (kind != Kind::Comment) code.append(t);
        if (kind == Kind::Symbol) {
            if (value == "(" || value == "[" || value == "{") delimiters.append(t);
            else if ((value == ")" || value == "]" || value == "}") && !delimiters.isEmpty()) {
                const auto opening = tokens[delimiters.last()].text;
                if ((opening == "(" && value == ")") || (opening == "[" && value == "]") || (opening == "{" && value == "}")) {
                    const int other = delimiters.takeLast();
                    tokens[other].pair = t; tokens[t].pair = other;
                    if (opening == "{") fold(other, t);
                }
            }
        }
    }
}
QString JyLuaAnalysis::peek(int offset) const {
    return current + offset < code.size() ? tokens[code[current + offset]].text : QString();
}
int JyLuaAnalysis::take() { return current < code.size() ? code[current++] : -1; }
bool JyLuaAnalysis::accept(const QString &text) { if (peek() != text) return false; take(); return true; }
int JyLuaAnalysis::enterScope(int position) {
    scopes.append({scope, position, int(source.size())}); scope = scopes.size() - 1; return scope;
}
void JyLuaAnalysis::leaveScope(int end) { scopes[scope].end = end; scope = qMax(0, scopes[scope].parent); }
void JyLuaAnalysis::fold(int opener, int closer) {
    if (opener >= 0 && closer >= 0 && tokens[closer].line > tokens[opener].line)
        folds.append({tokens[opener].line, tokens[closer].line});
}
int JyLuaAnalysis::declare(int token, bool local, bool parameter, const QString &name) {
    if (token < 0 || tokens[token].kind != Kind::Name) return -1;
    const auto value = name.isEmpty() ? tokens[token].text : name;
    const auto key = bindingKey(value, scope, tokens[token].start);
    if (!local && globals.contains(key)) { use(token, value); return globals[key]; }
    int id = symbols.size();
    symbols.append({value, {}, token, local ? scope : 0, tokens[token].end, local, parameter, {}});
    tokens[token].symbol = id;
    if (!local) globals.insert(key, id);
    return id;
}
int JyLuaAnalysis::resolve(const QString &name, int atScope, int position) const {
    for (int s = atScope; s >= 0; s = scopes[s].parent) {
        for (int i = symbols.size() - 1; i >= 0; --i)
            if (symbols[i].local && symbols[i].scope == s && symbols[i].name == name && symbols[i].visibleFrom <= position) return i;
    }
    return globals.value(bindingKey(name, atScope, position), -1);
}
QString JyLuaAnalysis::bindingKey(const QString &name, int atScope, int position) const {
    QString key = name;
    key.replace(':', '.');
    const int dot = key.indexOf('.');
    if (dot > 0) {
        const int receiver = resolve(key.left(dot), atScope, position);
        if (receiver >= 0 && symbols[receiver].local) key = QString("#%1").arg(receiver) + key.mid(dot);
    }
    return key;
}
void JyLuaAnalysis::use(int token, const QString &name) {
    if (token < 0 || tokens[token].kind != Kind::Name) return;
    const QString value = name.isEmpty() ? tokens[token].text : name;
    const int id = resolve(value, scope, tokens[token].start);
    if (id < 0) unresolved.append({token, scope, value});
    else { tokens[token].symbol = id; symbols[id].references.append(token); }
}
void JyLuaAnalysis::block(const QStringList &ends) {
    if (parseDepth >= 200) { take(); return; }
    QScopedValueRollback<int> depthGuard(parseDepth, parseDepth + 1);
    while (current < code.size() && !ends.contains(peek())) {
        const int before = current;
        statement();
        if (current == before) take(); // Error recovery must always advance.
    }
}

void JyLuaAnalysis::functionBody(int declaration, int opener, bool method) {
    if (!accept("(")) return;
    enterScope(tokens[opener].start);
    QStringList parameters;
    Q_UNUSED(method); // Implicit self has no source declaration to navigate to.
    while (current < code.size() && peek() != ")") {
        int t = take();
        if (tokens[t].kind == Kind::Name) { declare(t, true, true); parameters.append(tokens[t].text); }
        else if (tokens[t].text == "...") parameters.append("...");
        else if (tokens[t].text != ",") break;
    }
    accept(")");
    if (declaration >= 0) symbols[declaration].signature = symbols[declaration].name + "(" + parameters.join(", ") + ")";
    block({"end"});
    const int end = peek() == "end" ? take() : -1;
    if (end >= 0) fold(opener, end);
    leaveScope(end >= 0 ? tokens[end].end : source.size());
}

void JyLuaAnalysis::statement() {
    const QString word = peek();
    const int opener = current < code.size() ? code[current] : -1;
    if (accept("local")) {
        if (accept("function")) {
            int id = declare(take(), true);
            functionBody(id, opener); return;
        }
        QVector<int> declarations;
        do {
            int t = take();
            if (t < 0 || tokens[t].kind != Kind::Name) break;
            declarations.append(t);
            if (accept("<")) { take(); accept(">"); } // Lua 5.4 attributes
        } while (accept(","));
        // Initializers see the enclosing locals, not the declarations being initialized.
        const int firstValue = current;
        if (accept("=")) expressions();
        QString signature;
        if (declarations.size() == 1 && firstValue + 2 < code.size() && tokens[code[firstValue + 1]].text == "function") {
            const int paren = code[firstValue + 2];
            if (tokens[paren].pair >= 0)
                signature = tokens[declarations[0]].text + source.mid(tokens[paren].start, tokens[tokens[paren].pair].end - tokens[paren].start);
        }
        const int visible = current < code.size() ? tokens[code[current]].start : source.size();
        for (int t : declarations) { int id = declare(t, true); symbols[id].visibleFrom = visible; symbols[id].signature = signature; }
    } else if (accept("function")) {
        int t = take();
        if (t < 0) return;
        QString name = tokens[t].text;
        bool member = false, method = false;
        while (peek() == "." || peek() == ":") {
            if (!member) use(t);
            const QString separator = tokens[take()].text;
            method = separator == ":"; member = true;
            t = take(); if (t < 0) break;
            name += separator + tokens[t].text;
        }
        if (t < 0) return;
        int id = member ? -1 : resolve(name, scope, tokens[t].start);
        if (id >= 0) use(t, name); else id = declare(t, false, false, name);
        functionBody(id, opener, method);
    } else if (accept("if")) {
        expression(); accept("then");
        enterScope(opener >= 0 ? tokens[opener].start : 0); block({"elseif", "else", "end"});
        leaveScope(current < code.size() ? tokens[code[current]].start : source.size());
        while (accept("elseif")) {
            expression(); accept("then"); enterScope(current < code.size() ? tokens[code[current]].start : source.size());
            block({"elseif", "else", "end"}); leaveScope(current < code.size() ? tokens[code[current]].start : source.size());
        }
        if (accept("else")) {
            enterScope(current < code.size() ? tokens[code[current]].start : source.size());
            block({"end"}); leaveScope(current < code.size() ? tokens[code[current]].start : source.size());
        }
        if (peek() == "end") fold(opener, take());
    } else if (accept("for")) {
        QVector<int> variables;
        do { if (current < code.size()) variables.append(take()); } while (accept(","));
        if (accept("=") || accept("in")) expressions();
        accept("do"); enterScope(current < code.size() ? tokens[code[current]].start : source.size());
        for (int t : variables) declare(t, true, true);
        block({"end"}); int end = peek() == "end" ? take() : -1; fold(opener, end);
        leaveScope(end >= 0 ? tokens[end].end : source.size());
    } else if (word == "while" || word == "do" || word == "repeat") {
        take();
        if (word == "while") { expression(); accept("do"); }
        enterScope(tokens[opener].start);
        block({word == "repeat" ? "until" : "end"});
        int end = current < code.size() ? take() : -1;
        if (word == "repeat") expression(); // Repeat locals remain visible in the condition.
        fold(opener, end);
        leaveScope(current < code.size() ? tokens[code[current]].start : source.size());
    } else if (accept("return")) {
        expressions();
    } else if (accept("goto")) {
        take();
    } else if (accept("::")) {
        take(); accept("::");
    } else if (accept("break") || accept(";")) {
    } else {
        QVector<int> targets;
        do { int before = current; int target = primary(); if (before == current) break; targets.append(target); } while (accept(","));
        if (accept("=")) {
            int declaration = -1;
            for (int t : targets) {
                if (t < 0 || tokens[t].kind != Kind::Name) continue;
                if (tokens[t].symbol < 0) {
                    unresolved.erase(std::remove_if(unresolved.begin(), unresolved.end(), [t](const Use &u) { return u.token == t; }), unresolved.end());
                    declaration = declare(t, false, false, memberNames.value(t));
                } else declaration = tokens[t].symbol;
            }
            if (targets.size() == 1 && peek() == "function") { int function = take(); functionBody(declaration, function); }
            else expressions();
        }
    }
}
void JyLuaAnalysis::expressions() { expression(); while (accept(",")) expression(); }
void JyLuaAnalysis::expression(int minimum) {
    if (parseDepth >= 200) { take(); return; }
    QScopedValueRollback<int> depthGuard(parseDepth, parseDepth + 1);
    if (peek() == "not" || peek() == "#" || peek() == "-" || peek() == "~") { take(); expression(11); }
    else primary();
    static const QHash<QString, int> precedence = {{"or",1},{"and",2},{"<",3},{">",3},{"<=",3},{">=",3},{"~=",3},{"==",3},
        {"|",4},{"~",5},{"&",6},{"<<",7},{">>",7},{"..",8},{"+",9},{"-",9},{"*",10},{"/",10},{"//",10},{"%",10},{"^",12}};
    while (precedence.value(peek(), -1) >= minimum) {
        const auto op = peek(); take();
        expression(precedence[op] + ((op == "^" || op == "..") ? 0 : 1));
    }
}
int JyLuaAnalysis::primary() {
    if (parseDepth >= 200) { take(); return -1; }
    QScopedValueRollback<int> depthGuard(parseDepth, parseDepth + 1);
    if (current >= code.size()) return -1;
    int t = code[current], result = -1;
    QString name;
    if (tokens[t].kind == Kind::Name) { take(); use(t); result = t; name = tokens[t].text; }
    else if (accept("function")) { functionBody(-1, t); }
    else if (accept("(")) { expression(); accept(")"); }
    else if (accept("{")) {
        while (current < code.size() && peek() != "}") {
            int before = current;
            if (accept("[")) { expression(); accept("]"); accept("="); expression(); }
            else if (tokens[code[current]].kind == Kind::Name && peek(1) == "=") { take(); take(); expression(); }
            else expression();
            if (!accept(",")) accept(";");
            if (before == current) take();
        }
        accept("}");
    } else if (tokens[t].kind == Kind::Number || tokens[t].kind == Kind::String || peek() == "nil" || peek() == "true" || peek() == "false" || peek() == "...") take();
    else return -1;
    while (current < code.size()) {
        if (peek() == "." || peek() == ":") {
            QString sep = tokens[take()].text;
            if (current >= code.size() || tokens[code[current]].kind != Kind::Name) break;
            int field = take(); name += sep + tokens[field].text; use(field, name);
            memberNames.insert(field, name);
            result = field;
        } else if (accept("[")) { expression(); accept("]"); result = -1; name.clear(); }
        else if (accept("(")) { expressions(); accept(")"); result = -1; name.clear(); }
        else if (peek() == "{" || tokens[code[current]].kind == Kind::String) { primary(); result = -1; name.clear(); }
        else break;
    }
    return result;
}

void JyLuaAnalysis::checkSyntax() {
    lua_State *state = luaL_newstate();
    if (!state) { diagnostics.append({0, 1, "Unable to allocate Lua syntax checker", false}); return; }
    auto bytes = source.toUtf8();
    // luaL_loadbufferx doesn't skip a script's shebang as luaL_loadfilex does.
    if (bytes.startsWith("#!")) { int end = bytes.indexOf('\n'); bytes.replace(0, end < 0 ? bytes.size() : end, QByteArray(end < 0 ? bytes.size() : end, ' ')); }
    if (luaL_loadbufferx(state, bytes.constData(), size_t(bytes.size()), "editor", "t") != LUA_OK) {
        QString message = QString::fromUtf8(lua_tostring(state, -1));
        static const QRegularExpression location(":(\\d+):\\s*(.*)");
        const auto match = location.match(message);
        const int line = match.hasMatch() ? match.captured(1).toInt() : 1;
        int start = 0;
        for (int n = 1; n < line && start < source.size(); ++n) { int next = source.indexOf('\n', start); start = next < 0 ? source.size() : next + 1; }
        int end = source.indexOf('\n', start); if (end < 0) end = source.size();
        diagnostics.append({start, qMax(1, end - start), match.hasMatch() ? match.captured(2) : message, false});
    }
    lua_close(state); // Compilation only: no libraries opened and no code executed.
}
int JyLuaAnalysis::tokenAt(int position) const {
    auto it = std::upper_bound(tokens.begin(), tokens.end(), position, [](int p, const Token &t) { return p < t.start; });
    if (it == tokens.begin()) return -1;
    --it; return position < it->end ? int(it - tokens.begin()) : -1;
}
bool JyLuaAnalysis::isCode(int position) const {
    const int i = tokenAt(position);
    return i < 0 || (tokens[i].kind != Kind::Comment && tokens[i].kind != Kind::String);
}
int JyLuaAnalysis::definitionAt(int position) const {
    int t = tokenAt(position); if (t < 0 || tokens[t].symbol < 0) return -1;
    return tokens[symbols[tokens[t].symbol].token].start;
}
QVector<int> JyLuaAnalysis::referencesAt(int position) const {
    QVector<int> result;
    int t = tokenAt(position); if (t < 0 || tokens[t].symbol < 0) return result;
    const auto &s = symbols[tokens[t].symbol];
    result.append(tokens[s.token].start);
    for (int reference : s.references) result.append(tokens[reference].start);
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}
QStringList JyLuaAnalysis::completions(int position) const {
    QStringList result = keywords(); result.append(builtins().keys());
    int atScope = 0;
    for (int i = 1; i < scopes.size(); ++i) if (scopes[i].start <= position && position < scopes[i].end) atScope = i;
    for (const auto &s : symbols) if (resolve(s.name, atScope, position) >= 0) result.append(s.name);
    int start = position;
    while (start > 0 && (source[start - 1].isLetterOrNumber() || QString("_.:").contains(source[start - 1]))) --start;
    const auto prefix = source.mid(start, position - start);
    const int colon = prefix.lastIndexOf(':');
    if (colon >= 0) for (auto it = methods().cbegin(); it != methods().cend(); ++it)
        result.append(prefix.left(colon + 1) + it.key());
    result.removeDuplicates(); result.sort(); return result;
}
QString JyLuaAnalysis::signatureAt(int position, int *argument) const {
    QVector<int> stack;
    for (int i : code) {
        if (tokens[i].start >= position) break;
        const auto &t = tokens[i];
        if (t.kind != Kind::Symbol) continue;
        if (t.text == "(" || t.text == "[" || t.text == "{") stack.append(i);
        else if ((t.text == ")" || t.text == "]" || t.text == "}") && !stack.isEmpty()) stack.removeLast();
    }
    for (int n = stack.size() - 1; n >= 0; --n) {
        const int opening = stack[n];
        if (tokens[opening].text != "(") continue;
        const int at = code.indexOf(opening);
        if (at < 1) continue;
        const int nameToken = code[at - 1];
        if (tokens[nameToken].kind != Kind::Name) continue;
        QString name = tokens[nameToken].text;
        for (int p = at - 2; p >= 1 && (tokens[code[p]].text == "." || tokens[code[p]].text == ":"); p -= 2)
            name.prepend(tokens[code[p - 1]].text + tokens[code[p]].text);
        QString signature;
        int id = tokens[nameToken].symbol;
        if (id >= 0) signature = symbols[id].signature;
        else {
            signature = builtins().value(name);
            if (signature.isEmpty() && name.contains(':')) {
                const auto hint = methods().value(name.section(':', -1));
                if (!hint.isEmpty()) signature = QStringLiteral("JellyCAD · ") + hint;
            }
        }
        if (signature.isEmpty()) continue;
        int arg = 0, depth = 0;
        for (int p = at + 1; p < code.size() && tokens[code[p]].start < position; ++p) {
            const auto &t = tokens[code[p]];
            if (t.kind != Kind::Symbol) continue;
            if (t.text == "(" || t.text == "[" || t.text == "{") ++depth;
            else if (t.text == ")" || t.text == "]" || t.text == "}") --depth;
            else if (t.text == "," && depth == 0) ++arg;
        }
        if (argument) *argument = arg;
        return signature;
    }
    return {};
}

QString JyLuaAnalysis::formatted() const {
    auto lines = source.split('\n');
    int depth = 0, token = 0, offset = 0;
    // Only rewrite indentation outside literals/comments. This is intentionally a
    // conservative formatter: expression spacing and literal bytes are preserved.
    for (int line = 0; line < lines.size(); ++line) {
        const auto original = lines[line];
        const int end = offset + original.size();
        while (token < tokens.size() && tokens[token].end <= offset) ++token;
        const bool continuation = token < tokens.size() && tokens[token].start < offset &&
            (tokens[token].kind == Kind::String || tokens[token].kind == Kind::Comment);
        int leadingClose = 0;
        bool beginning = true;
        int delta = 0;
        for (int t = token; t < tokens.size() && tokens[t].start <= end; ++t) {
            const auto &item = tokens[t];
            if (item.start < offset || item.kind == Kind::Comment) continue;
            const auto &v = item.text;
            bool close = item.kind == Kind::Keyword && (v == "end" || v == "until" || v == "else" || v == "elseif");
            close |= item.kind == Kind::Symbol && (v == "}" || v == ")" || v == "]");
            if (close) { --delta; if (beginning) ++leadingClose; }
            // `elseif` closes a branch; its `then` opens the next one.
            bool open = item.kind == Kind::Keyword && (v == "function" || v == "then" || v == "do" || v == "repeat" || v == "else");
            open |= item.kind == Kind::Symbol && (v == "{" || v == "(" || v == "[");
            if (open) ++delta;
            if (!close) beginning = false;
        }
        if (!continuation) {
            int indent = 0;
            while (indent < original.size() && original[indent].isSpace()) ++indent;
            const auto content = original.mid(indent);
            lines[line] = content.isEmpty() ? QString() : QString(qMax(0, depth - leadingClose) * 4, ' ') + content;
        }
        depth = qMax(0, depth + delta);
        offset = end + 1;
    }
    return lines.join('\n');
}
int JyLuaAnalysis::indentationAfter(int position) const {
    // Formatting a synthetic trailing line reuses exactly the same block rules.
    JyLuaAnalysis prefix(source.left(position) + "\n__indent_probe");
    const QString last = prefix.formatted().section('\n', -1);
    return last.size() - last.trimmed().size();
}
