/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : cmodel.h
#   Last Modified : 2019-08-06 14:00
#   Describe      : The Models
#
# ====================================================*/


#ifndef CMODEL_H
#define CMODEL_H

// Qt Include
#include <QObject>
#include <QDebug>
#include <QRegularExpression>
#include <QMatrix4x4>

// OpenCascade Include
#include <AIS_Shape.hxx>
#include <AIS_SequenceOfInteractive.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <AIS_Trihedron.hxx>
#include <Geom_Axis2Placement.hxx>
#include <TopExp_Explorer.hxx>
#include <AIS_InteractiveContext.hxx>
#include <TopoDS.hxx>

// Assimp Inculde
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/ai_assert.h>
#include <assimp/vector3.h>
#include <assimp/Exporter.hpp>
using namespace Assimp;

class CModel : public AIS_Shape
{
      DEFINE_STANDARD_RTTIEXT(CModel, AIS_Shape)
public:
    //! 构造函数
    CModel(const TopoDS_Shape& _shape = TopoDS_Shape());

    //! 构造函数，加载文件
    CModel(const QString _filepath);

    // 模型类型
    enum MODEL_TYPE
    {
        MODEL_COMPONENT = 1,    // 部件
        MODEL_COMP_GROUP = 2,   // 部件组
    };


    //! 设置模型名称
    //! \brief m_set_model_name
    //! \param _name 模型名称
    inline void m_set_model_name(QString _name){m_model_name = _name;}

    //! 获取模型的名称
    //! \brief m_get_model_name
    //! \return 模型名称
    inline QString m_get_model_name(){return m_model_name;}

    //! 使用QMatrix4x4矩阵设置模型位姿
    //! \brief m_set_pose_by_matrix
    //! \param _matrix 齐次矩阵
    void m_set_pose_by_matrix(QMatrix4x4 _matrix);

    //! 设置模型类型
    inline void m_set_model_type(MODEL_TYPE _type){m_model_type = _type;}

    //! 获取模型类型
    inline MODEL_TYPE m_get_model_type(){return m_model_type;}

    //! 获取整个模型的AABB包围盒，包括子模型
    Bnd_Box m_get_full_aabb();

    //! 导出模型
    //! \brief m_export_model
    //! \param _filename 文件名字
    //! \param _format 文件格式
    //! \return
    static bool m_export_model(QString _filename , const char *_format_id,Handle(AIS_InteractiveContext) _context);

private:
    //! 模型名称
    QString m_model_name;

    //! 模型种类
    MODEL_TYPE m_model_type;

    //! 处理所有ASSIMP节点
    //! \brief m_process_nodes
    //! \param _node 节点
    //! \param _scene 场景
    //! \param _is_root 是否为根节点
    void m_process_nodes(const aiNode *_node, const aiScene *_scene, bool _is_root);

    //! 模型材质转换，由ASSIMP转成OCC表示方式
    Graphic3d_MaterialAspect m_material_transfer(aiMaterial *_material);

};

DEFINE_STANDARD_HANDLE(CModel, AIS_Shape)

#endif // CMODEL_H
