#pragma once
#include "shapes/jy_urdf_generator.h"
namespace jelly {
class RobotWriter {
public:
    explicit RobotWriter(const Link &root) : m_root(root) {}
    void write(const std::string &robot_name, const std::string &directory, const Link::Format &format) const;
private:
    using ExtendedFile = Link::Format;
    struct CommomData {
        std::string robot_name; // 机器人名称
        std::string path_meshes;// 网格文件路径
    };


    // 深度优先遍历（DFS）
    std::string traverseDFS(const Link &link, const JyAxes &parent_axes, const CommomData &data) const;

    /*
     * 处理关节（Joint）
     * @param joint 关节对象
     * @param parent_link 父连杆对象
     * @param parent_axes 父连杆的轴坐标(前一个关节的轴坐标)
     * @return 生成的URDF字符串
     */
    std::string handleJoint(const Joint &joint, const Link &parent_link, const JyAxes &parent_axes) const;

    /*
     * 处理连杆（Link）
     * @param link 连杆对象
     * @param parent_axes 父连杆的轴坐标(前一个关节的轴坐标)
     * @return 生成的URDF字符串
     */
    std::string handleLink(const Link &link, const JyAxes &parent_axes, const CommomData &data) const;

    // MuJoCo specific handlers
    std::string handleBody_MuJoCo(const Link &link, const Joint &parent_joint, const Joint &grand_joint, const CommomData &data, const int &space = 0) const;
    const Link &m_root;
};
}
