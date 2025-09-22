/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_SHAPE_H
#define JY_SHAPE_H

#include <AIS_Shape.hxx>
#include <QDebug>
#include <TopoDS_Edge.hxx>
#include <sol/table.hpp>

class JyTopoShape : public TopoDS_Shape {
public:
    // 默认构造函数
    JyTopoShape() {}

    // 从基类构造
    JyTopoShape(const TopoDS_Shape &shape) : TopoDS_Shape(shape) {}

    // 拷贝构造函数（如果需要特殊处理）
    JyTopoShape(const JyTopoShape &other) : TopoDS_Shape(other) {}

    // 赋值操作符 - 从基类赋值
    JyTopoShape &operator=(const TopoDS_Shape &shape) {
        if (this != &shape) {              // 自赋值检查
            TopoDS_Shape::operator=(shape);// 调用基类赋值操作符
        }
        return *this;
    }

    // 赋值操作符 - 从同类赋值
    JyTopoShape &operator=(const JyTopoShape &other) {
        if (this != &other) {              // 自赋值检查
            TopoDS_Shape::operator=(other);// 调用基类赋值操作符
        }
        return *this;
    }

    // 如果基类的析构函数不是虚函数，可能需要虚析构函数
    // virtual ~JyTopoShape() {}
};

struct InertialProperties {
    double mass;
    std::array<double, 3> center_of_mass;
    std::array<double, 6> inertia_tensor;
};

/**
 * @brief 三维形状基类
 * 
 * JyShape类是所有三维几何形状的基类，提供了形状的基本操作功能，
 * 包括布尔运算、几何变换、位置姿态调整、颜色透明度设置等。
 */
class JyShape {
public:
    /**
     * @brief 获取形状数据
     * @return TopoDS_Shape 形状数据
     */
    [[nodiscard]] JyTopoShape data() const { return s_; }

    JyTopoShape s_;//!< OpenCASCADE形状对象

    Quantity_Color color_{Quantity_NOC_ORANGE3};//!< 形状颜色，默认为橙色
    Standard_Real transparency_{0};             //!< 透明度，0为不透明，1为完全透明
    Standard_Real density_{1.0};                //!< 密度，用于计算质量，质量=体积*密度

public:
    // ==================== 构造与析构 ====================
    /**
     * @brief 默认构造函数
     */
    explicit JyShape() = default;

    /**
     * @brief 从文件构造形状
     * @param _filename 文件路径
     */
    explicit JyShape(const std::string &_filename);

    /**
     * @brief 析构函数
     */
    ~JyShape() = default;

    /**
     * @brief 获取形状类型
     * @return std::string 形状类型字符串
     */
    [[nodiscard]] std::string type() const;

    // ==================== 布尔运算 ====================
    /**
     * @brief 融合操作（并集）
     * @param _other 要融合的另一个形状
     * @return JyShape& 当前形状引用
     */
    JyShape &fuse(const JyShape &_other);

    /**
     * @brief 切割操作（差集）
     * @param _other 用于切割的形状
     * @return JyShape& 当前形状引用
     */
    JyShape &cut(const JyShape &_other);

    /**
     * @brief 交集操作
     * @param _other 用于求交的形状
     * @return JyShape& 当前形状引用
     */
    JyShape &common(const JyShape &_other);

    // ==================== 几何变换 ====================
    /**
     * @brief 倒圆角操作
     * @param _r 圆角半径
     * @param _cond 条件表，指定要倒圆角的边
     * @return JyShape& 当前形状引用
     */
    JyShape &fillet(const double &_r, const sol::table &_cond);

    /**
     * @brief 倒角操作
     * @param _dis 倒角距离
     * @param _cond 条件表，指定要倒角的边
     * @return JyShape& 当前形状引用
     */
    JyShape &chamfer(const double &_dis, const sol::table &_cond);

    /**
     * @brief 拉伸操作
     * @param _x X方向拉伸距离
     * @param _y Y方向拉伸距离
     * @param _z Z方向拉伸距离
     * @return JyShape& 当前形状引用
     */
    JyShape &prism(const double &_x, const double &_y, const double &_z);//!< 拉伸

    /**
     * @brief 旋转体生成操作
     * @param _pos 旋转轴位置点
     * @param _dir 旋转轴方向向量
     * @param _angle 旋转角度（度）
     * @return JyShape& 当前形状引用
     */
    JyShape &revol(const std::array<double, 3> _pos, const std::array<double, 3> _dir, const double &_angle);//!< 旋转面

    //!< 按比例缩放
    JyShape &scale(const double &factor);

    // ==================== 位置姿态调整 ====================
    /**
     * @param x 绝对位置
     * @param y 绝对位置
     * @param z 绝对位置
     * @param rx 姿态角度（度）
     * @param ry 姿态角度（度）
     * @param rz 姿态角度（度）
     */
    // 沿X轴平移
    JyShape &x(const double &x) { return locate_base(LocateType::TRANSLATE_X, x, 0, 0); }
    // 沿Y轴平移
    JyShape &y(const double &y) { return locate_base(LocateType::TRANSLATE_Y, 0, y, 0); }
    // 沿Z轴平移
    JyShape &z(const double &z) { return locate_base(LocateType::TRANSLATE_Z, 0, 0, z); }
    // 绕X轴旋转
    JyShape &rx(const double &rx) { return locate_base(LocateType::ROTATE_X, rx, 0, 0); }
    // 绕Y轴旋转
    JyShape &ry(const double &ry) { return locate_base(LocateType::ROTATE_Y, 0, ry, 0); }
    // 绕Z轴旋转
    JyShape &rz(const double &rz) { return locate_base(LocateType::ROTATE_Z, 0, 0, rz); }
    // 设置绝对位置
    JyShape &pos(const double &x, const double &y, const double &z) { return locate_base(LocateType::TRANSLATE_ALL, x, y, z); }
    // 设置绝对姿态（欧拉角，单位度）
    JyShape &rot(const double &rx, const double &ry, const double &rz) { return locate_base(LocateType::ROTATE_ALL, rx, ry, rz); }
    /**
     * @brief 平移和旋转（相对于当前位置和姿态）
     * @param _move_type 移动类型："pos"或"rot"
     * @param _x 沿X轴平移距离 或 绕X轴旋转角度（度）
     * @param _y 沿Y轴平移距离 或 绕Y轴旋转角度（度）
     * @param _z 沿Z轴平移距离 或 绕Z轴旋转角度（度）
     * @return JyShape& 当前形状引用
     */
    JyShape &move(const std::string &_move_type, const double &_x, const double &_y, const double &_z);

    JyShape &zero();

    // ==================== 属性设置 ====================
    /**
     * @brief 设置颜色
     * @param _name_or_hex 颜色名称或十六进制值
     * @return JyShape& 当前形状引用
     */
    JyShape &color(const std::string &_name_or_hex);

    /**
     * @brief 设置透明度
     * @param _value 透明度值，0为不透明，1为完全透明
     * @return JyShape& 当前形状引用
     */
    JyShape &transparency(const double &_value);

    /**
     * @brief 设置质量（通过调整密度实现）
     * @param _mass 目标质量
     * @return JyShape& 当前形状引用
     */
    JyShape &mass(const double &_mass);

    // ==================== 导出功能 ====================
    /**
     * @brief 导出为STL格式文件
     * @param _filename 文件路径
     * @param _opt 导出选项表
     */
    void export_stl(const std::string &_filename, const sol::table &_opt) const;
    void export_stl(const std::string &_filename) const { export_stl(_filename, {}); }

    /**
     * @brief 导出为STEP格式文件
     * @param _filename 文件路径
     */
    void export_step(const std::string &_filename) const;

    /**
     * @brief 导出为IGES格式文件
     * @param _filename 文件路径
     */
    void export_iges(const std::string &_filename) const;

    std::array<double, 4> rgba() const;

    static InertialProperties inertial(const JyShape &_shape);

    static InertialProperties inertial(const std::vector<JyShape> &_shapes);

private:
    /**
     * @brief 从Lua表中获取双精度数组
     * @param _t Lua表
     * @param _v 输出的双精度数组
     * @return bool 是否成功
     */
    static bool get_double_vector(const sol::table &_t, std::vector<double> &_v);

    /**
     * @brief 通用算法模板函数
     * @tparam T 算法类型
     * @param _other 另一个形状
     * @return JyShape& 当前形状引用
     */
    template<typename T>
    JyShape &algo(const JyShape &_other) {
        if (s_.IsNull()) { return *this; }
        if (_other.s_.IsNull()) { return *this; }
        T algo_fuse(s_, _other.s_);
        s_ = algo_fuse.Shape();
        return *this;
    }

    /**
     * @brief 位置变换类型枚举
     */
    enum class LocateType : int {
        TRANSLATE_ALL = 0,//!< 完全平移
        TRANSLATE_X,      //!< X轴平移
        TRANSLATE_Y,      //!< Y轴平移
        TRANSLATE_Z,      //!< Z轴平移
        ROTATE_ALL = 20,  //!< 完全旋转
        ROTATE_X,         //!< 绕X轴旋转
        ROTATE_Y,         //!< 绕Y轴旋转
        ROTATE_Z,         //!< 绕Z轴旋转
    };

    /**
     * @brief 基础位置变换函数
     * @param _type 变换类型
     * @param _x X方向参数
     * @param _y Y方向参数
     * @param _z Z方向参数
     * @param _abs 是否绝对坐标
     * @return JyShape& 当前形状引用
     */
    JyShape &locate_base(const LocateType &_type, const double &_x, const double &_y, const double &_z,
                         const bool &_abs = true);

    /**
     * @brief 边过滤函数
     * @param _edge 边对象
     * @param _cond 条件表
     * @return bool 是否符合条件
     */
    static bool edge_filter(const TopoDS_Edge &_edge, const sol::table &_cond);
};

// ==================== 三维基本形状 ====================
/**
 * @brief 长方体类
 */
class JyShapeBox : public JyShape {
public:
    /**
     * @brief 构造指定尺寸的长方体
     * @param _x 长度
     * @param _y 宽度
     * @param _z 高度
     */
    explicit JyShapeBox(const double &_x = 1, const double &_y = 1, const double &_z = 1);
};

/**
 * @brief 圆柱体类
 */
class JyCylinder : public JyShape {
public:
    /**
     * @brief 构造指定半径和高度的圆柱体
     * @param _r 半径
     * @param _h 高度
     */
    explicit JyCylinder(const double &_r = 1, const double &_h = 1);
};

/**
 * @brief 圆锥体类
 */
class JyCone : public JyShape {
public:
    /**
     * @brief 构造指定底部半径、顶部半径和高度的圆锥体
     * @param R1 底部半径
     * @param R2 顶部半径
     * @param H 高度
     */
    explicit JyCone(const double &R1 = 1, const double &R2 = 0, const double &H = 1);
};

/**
 * @brief 球体类
 */
class JySphere : public JyShape {
public:
    /**
     * @brief 构造指定半径的球体
     * @param _r 半径
     */
    explicit JySphere(const double &_r = 1);
};

class JyTorus : public JyShape {
public:
    /**
     * @brief 构造指定主半径、次半径和角度的圆环
     * @param R1 从管道中心到环面中心的距离
     * @param R2 管道半径
     * @param angle 角度(deg)
     */
    explicit JyTorus(const double &R1 = 2, const double &R2 = 1, const double &angle = 360);
};
class JyWedge : public JyShape {
public:
    /**
     * @brief 楔形
     * @param dx X方向长度
     * @param dy Y方向长度
     * @param dz Z方向长度
     * @param ltx 楔形中心到X轴的距离
     */
    explicit JyWedge(const double &dx = 1, const double &dy = 1, const double &dz = 1, const double &ltx = 0);

    explicit JyWedge(const double &dx, const double &dy, const double &dz, const double &xmin, const double &zmin, const double &xmax, const double &zmax);
};

/**
 * @brief 边类
 */
class JyEdge : public JyShape {
public:
    /**
     * @brief 根据类型和参数构造边
     * @param _type 边类型
     * @param _vec1 起点或中心点
     * @param _vec2 终点或方向向量
     * @param _r1 半径1（可选）
     * @param _r2 半径2（可选）
     */
    explicit JyEdge(const std::string &_type, const std::array<double, 3> _vec1, const std::array<double, 3> _vec2,
                    const double &_r1 = -1, const double &_r2 = -1);
};

/**
 * @brief 线框类
 */
class JyWire : public JyShape {
public:
    /**
     * @brief 默认构造函数
     */
    explicit JyWire() = default;

    /**
     * @brief 根据参数表构造线框
     * @param _param 参数表
     */
    explicit JyWire(const sol::table &_param);
};

/**
 * @brief 多边形类
 */
class JyPolygon : public JyWire {
public:
    /**
     * @brief 根据参数表构造多边形
     * @param _param 参数表
     */
    explicit JyPolygon(const sol::table &_param = {});
};

/**
 * @brief 面类
 */
class JyFace : public JyShape {
public:
    /**
     * @brief 默认构造函数
     */
    JyFace() = default;

    /**
     * @brief 根据形状构造面
     * @param _shape 形状对象
     */
    explicit JyFace(const JyShape &_shape);
};

/**
 * @brief 文本形状类
 */
class JyText : public JyShape {
public:
    /**
     * @brief 构造指定文本和字体大小的文本形状
     * @param _text 文本内容
     * @param _size 字体大小
     */
    explicit JyText(const std::string &_text = "", const double &_size = 1);
};

#endif//JY_SHAPE_H
