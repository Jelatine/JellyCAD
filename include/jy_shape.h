/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_SHAPE_H
#define JY_SHAPE_H

#include <AIS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <sol/table.hpp>
#include <QDebug>

class JyShape {
public:
    [[nodiscard]] Handle(AIS_Shape) data() const { return s_; }

    Handle(AIS_Shape) s_;
public:

    explicit JyShape() = default;

    explicit JyShape(const std::string &_filename);

    //!< 拷贝构造函数
    JyShape(const JyShape &other) {
        if (other.s_) {
            s_ = new AIS_Shape(other.s_->Shape());
            Quantity_Color color;
            other.s_->Color(color);
            s_->SetColor(color);
            s_->SetTransparency(other.s_->Transparency());
        }
    }

    ~JyShape() = default;

    [[nodiscard]] std::string type() const;

    [[nodiscard]] bool fuse(const JyShape &_other) const;

    [[nodiscard]] bool cut(const JyShape &_other) const;

    [[nodiscard]] bool common(const JyShape &_other) const;

    [[nodiscard]] bool fillet(const double &_r, const sol::table &_cond) const;

    [[nodiscard]] bool chamfer(const double &_dis, const sol::table &_cond) const;

    void prism(const double &_x, const double &_y, const double &_z) const;  //!< 拉伸

    void translate(const double &_x, const double &_y, const double &_z) const; //!< 相对当前坐标系平移

    void rotate(const double &_rx, const double &_ry, const double &_rz) const; //!< 相对当前坐标系旋转

    void locate(const sol::table &_t) const; //!< 设置绝对位置和姿态

    void color(const std::string &_name_or_hex) const;

    void transparency(const double &_value) const;

    void set_stl_radian(const sol::table &_opt) const;

    void export_stl(const std::string &_filename, const sol::table &_opt) const;

    void export_step(const std::string &_filename) const;

    void export_iges(const std::string &_filename) const;

private:
    static bool get_double_vector(const sol::table &_t, std::vector<double> &_v);

    template<typename T>
    [[nodiscard]] bool algo(const JyShape &_other) const {
        if (!s_) { return false; }
        if (!_other.s_) { return false; }
        T algo_fuse(s_->Shape(), _other.s_->Shape());
        s_->SetShape(algo_fuse.Shape());
        return true;
    }

    enum class LocateType : int {
        TRANSLATE_ALL = 0,
        TRANSLATE_X,
        TRANSLATE_Y,
        TRANSLATE_Z,
        ROTATE_ALL = 20,
        ROTATE_X,
        ROTATE_Y,
        ROTATE_Z,
    };

    void locate_base(const LocateType &_type, const double &_x, const double &_y, const double &_z,
                     const bool &_abs = true) const;

    static bool edge_filter(const TopoDS_Edge &_edge, const sol::table &_cond);

protected:
    void process_opt(const sol::table &_opt) const;

    static gp_Pnt get_point_3d(const sol::table &_t);

    static gp_Dir get_dir_3d(const sol::table &_t);
};

/************ 三维基本形状 ************/
class JyShapeBox : public JyShape {
public:
    explicit JyShapeBox(const double &_x = 1, const double &_y = 1, const double &_z = 1, const sol::table &_opt = {});
};

class JyCylinder : public JyShape {
public:
    explicit JyCylinder(const double &_r = 1, const double &_h = 1, const sol::table &_opt = {});
};

class JyCone : public JyShape {
public:
    explicit JyCone(const double &R1 = 1, const double &R2 = 1, const double &H = 1, const sol::table &_opt = {});
};

class JySphere : public JyShape {
public:
    explicit JySphere(const double &_r = 1, const sol::table &_opt = {});
};

class JyEdge : public JyShape {
public:
    explicit JyEdge(const std::string &_type, const sol::table &_vec1, const sol::table &_vec2,
                    const double &_r1 = -1, const double &_r2 = -1);
};

class JyWire : public JyShape {
public:
    explicit JyWire() = default;

    explicit JyWire(const sol::table &_param);
};

class JyPolygon : public JyWire {
public:
    explicit JyPolygon(const sol::table &_param = {});
};

class JyFace : public JyShape {
public:
    JyFace() = default;

    explicit JyFace(const JyShape &_shape);
};

#endif //JY_SHAPE_H
