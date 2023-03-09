/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : cmodel.cpp
#   Last Modified : 2019-04-21 11:00
#   Describe      : The Models
#
# ====================================================*/

#include "cmodel.h"

IMPLEMENT_STANDARD_RTTIEXT(CModel,AIS_Shape)

CModel::CModel(const TopoDS_Shape &_shape)
    :AIS_Shape(_shape),m_model_name("Unnamed"),m_model_type(MODEL_COMPONENT)
{}

CModel::CModel(const QString _filepath)
    :AIS_Shape(TopoDS_Shape()),m_model_name("Unnamed"),m_model_type(MODEL_COMPONENT)
{
    // 提取文件名字，作为模型名字
    QString t_model_name = _filepath;
    t_model_name.remove(QRegularExpression("(.*/)|([.].+)"));
    m_model_name = t_model_name;
    //从外部文件中加载模型
    Importer t_importer;
    const aiScene *t_scene=t_importer.ReadFile(_filepath.toStdString(),aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);
    if(!t_scene || t_scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !t_scene->mRootNode) // if is Not Zero
    {
        qDebug() << "ERROR::ASSIMP:: " << t_importer.GetErrorString();
        return;
    }
    // 构造根节点及所有子节点
    m_process_nodes(t_scene->mRootNode, t_scene, true);
}

void CModel::m_process_nodes(const aiNode *_node, const aiScene *_scene, bool _is_root)
{
    Graphic3d_MaterialAspect t_occ_material;
    //![0] 模型名字信息
    QString t_node_name = _node->mName.C_Str();
    //![1] 坐标系信息
    aiMatrix4x4 t_matrix=_node->mTransformation;    // 获取转换坐标系
    // 获取旋转矩阵的缩放比例
    ai_real t_scaling_x = aiVector3D(t_matrix.a1,t_matrix.a2,t_matrix.a3).Length();
    ai_real t_scaling_y = aiVector3D(t_matrix.b1,t_matrix.b2,t_matrix.b3).Length();
    ai_real t_scaling_z = aiVector3D(t_matrix.c1,t_matrix.c2,t_matrix.c3).Length();

    // 从 aiMatrix4x4 转换成 gp_Trsf 形式
    gp_Trsf t_transfer;
    t_transfer.SetValues(t_matrix.a1 / t_scaling_x, t_matrix.a2 / t_scaling_x, t_matrix.a3 / t_scaling_x, t_matrix.a4,
                         t_matrix.b1 / t_scaling_y, t_matrix.b2 / t_scaling_y, t_matrix.b3 / t_scaling_y, t_matrix.b4,
                         t_matrix.c1 / t_scaling_z, t_matrix.c2 / t_scaling_z, t_matrix.c3 / t_scaling_z, t_matrix.c4);
    //![2] 拓扑形状信息
    // 构造拓扑形状
    BRepBuilderAPI_Sewing t_sewing_tool;
    t_sewing_tool.Init (1.0e-06, Standard_True);
    // 复合拓扑结构
    TopoDS_Compound t_compound;
    BRep_Builder t_build_tool;
    t_build_tool.MakeCompound (t_compound);
    TopoDS_Face t_topo_face;    // 储存拓扑面
    // 遍历节点的所有网格
    for(unsigned int imesh = 0; imesh < _node->mNumMeshes; imesh++)
    {
        aiMesh* t_mesh = _scene->mMeshes[_node->mMeshes[imesh]];    // 获取当前网格
        // 遍历网格的所有面
        for(unsigned int iface = 0; iface < t_mesh->mNumFaces; iface++)
        {
            t_mesh->mMaterialIndex;
            aiFace t_face = t_mesh->mFaces[iface];
            BRepBuilderAPI_MakePolygon t_polygon;
            // 遍历面的所有顶点
            for(unsigned int ivertex = 0; ivertex < t_face.mNumIndices; ivertex++)
            {
                // 转换顶点储存模式
                gp_Pnt t_pnt=gp_Pnt(t_mesh->mVertices[t_face.mIndices[ivertex]].x,
                                    t_mesh->mVertices[t_face.mIndices[ivertex]].y,
                                    t_mesh->mVertices[t_face.mIndices[ivertex]].z);
                t_polygon.Add(t_pnt);   // 添加顶点
            }
            t_polygon.Close();  // 闭合顶点
            t_topo_face = BRepBuilderAPI_MakeFace (t_polygon); // 通过闭合的线构造面
            if(!t_topo_face.IsNull())
            {
                t_build_tool.Add (t_compound, t_topo_face);  // 将面加入到复合体中
            }
        }
        //! 材质信息
        aiMaterial* material = _scene->mMaterials[t_mesh->mMaterialIndex];   //通过索引获取网格在环境中的材质
        t_occ_material = m_material_transfer(material); // 从ASSIMP格式转换到OCC材质格式
    }
    t_sewing_tool.Load (t_compound);
    t_sewing_tool.Perform();
    TopoDS_Shape t_topo_shape = t_sewing_tool.SewedShape();
    if (t_topo_shape.IsNull())
    {
        t_topo_shape = t_compound;
    }
    bool t_is_next_root = false;    // 下一处理的节点是否根节点
    //! 分别处理根节点和子节点
    if(_is_root)
    {
        // 根节点无形状、只有一个子节点、无孙节点
        if(_node->mNumChildren == 1 && _node->mNumMeshes ==0 && _node->mChildren[0]->mNumChildren == 0)
        {
            t_is_next_root = true;  // 将子节点提升为根节点
            m_set_model_type(MODEL_COMPONENT);
        }
        else if(_node->mNumMeshes == 0)
        {
            TopoDS_Shape t_vertex_topo = BRepBuilderAPI_MakeVertex(t_transfer.TranslationPart());
            SetShape(t_vertex_topo);
            m_set_model_type(MODEL_COMP_GROUP);
        }
        else
        {
            SetShape(t_topo_shape); // 设置拓扑形状
        }
        SetLocalTransformation(t_transfer); // 设置模型位姿
        SetMaterial(t_occ_material);
    }
    else
    {
        if(_node->mNumMeshes>0)
        {
            Handle(CModel) t_child = new CModel(t_topo_shape);  // 设置拓扑形状
            t_child->m_set_model_name(t_node_name); // 设置模型名字
            t_child->SetLocalTransformation(t_transfer);    // 设置模型位姿
            t_child->SetMaterial(t_occ_material);
            t_child->m_set_model_type(MODEL_COMPONENT);
            AddChild(t_child);  // 将以上模型作为本对象的子节点
        }
        else
        {}
    }
    // 遍历子节点
    for(unsigned int i = 0; i < _node->mNumChildren; i++)
    {
        m_process_nodes(_node->mChildren[i], _scene , t_is_next_root);   // 构造子节点
    }
}

Graphic3d_MaterialAspect CModel::m_material_transfer(aiMaterial *_material)
{//! 模型材质转换，由ASSIMP转成OCC表示方式
    // 声明并初始化OCC材质参数
    Graphic3d_MaterialAspect t_result;
    t_result.SetMaterialType(Graphic3d_MATERIAL_PHYSIC);
    Quantity_Color t_occ_colors[Graphic3d_TypeOfReflection_NB];
    t_occ_colors[Graphic3d_TOR_AMBIENT]  = Quantity_Color (Graphic3d_Vec3 (0.2f, 0.2f, 0.2f));
    t_occ_colors[Graphic3d_TOR_DIFFUSE]  = Quantity_Color (Graphic3d_Vec3 (0.2f, 0.2f, 0.2f));
    t_occ_colors[Graphic3d_TOR_SPECULAR] = Quantity_Color (Graphic3d_Vec3 (1.0f, 1.0f, 1.0f));
    Standard_ShortReal t_occ_shininess = 0.039f;

    aiString name;  // 材质名称 原始数据
    if (AI_SUCCESS==aiGetMaterialString(_material,AI_MATKEY_NAME,&name))
    {
        t_result.SetMaterialName(name.C_Str());
    }
    // 环境光
    aiColor4D ambient;      // 环境光 原始数据
    if(AI_SUCCESS ==aiGetMaterialColor(_material, AI_MATKEY_COLOR_AMBIENT, &ambient))
    {
        t_occ_colors[Graphic3d_TOR_AMBIENT]=Quantity_Color(ambient.r,ambient.g,ambient.b,Quantity_TOC_RGB);
        t_result.SetAmbientColor(t_occ_colors[Graphic3d_TOR_AMBIENT]);
    }
    // 漫反射
    aiColor4D diffuse;      // 漫反射 原始数据
    if(AI_SUCCESS ==aiGetMaterialColor(_material, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
    {
        t_occ_colors[Graphic3d_TOR_DIFFUSE]=Quantity_Color(diffuse.r,diffuse.g,diffuse.b,Quantity_TOC_RGB);
        t_result.SetDiffuseColor(t_occ_colors[Graphic3d_TOR_DIFFUSE]);
    }
    // 镜面光
    aiColor4D specular;     // 镜面光 原始数据
    if(AI_SUCCESS ==aiGetMaterialColor(_material, AI_MATKEY_COLOR_SPECULAR, &specular))
    {
        t_occ_colors[Graphic3d_TOR_SPECULAR]=Quantity_Color(specular.r,specular.g,specular.b,Quantity_TOC_RGB);
        t_result.SetSpecularColor(t_occ_colors[Graphic3d_TOR_SPECULAR]);
    }
    // 反光度
    float shininess;        // 反光度 原始数据
    if(AI_SUCCESS ==aiGetMaterialFloat(_material, AI_MATKEY_SHININESS, &shininess))
    {
        t_occ_shininess=shininess/128.0;    // 由OpenGL值转换成VRML97
        // OCC的反光度表示方式只接受0到1之间，否则报错
        t_occ_shininess = t_occ_shininess<1.0 ? t_occ_shininess:1.0;
        t_occ_shininess = t_occ_shininess<0.0 ? 0.0:t_occ_shininess;
        t_result.SetShininess(t_occ_shininess); // 设置反光度
    }
    return t_result;
}

void CModel::m_set_pose_by_matrix(QMatrix4x4 _matrix)
{//! 使用QMatrix4x4矩阵设置模型位姿
    gp_Trsf t_transfer;
    // 从 QMatrix4x4 转换成 gp_Trsf 形式
    t_transfer.SetValues(_matrix(0,0),_matrix(0,1),_matrix(0,2),_matrix(0,3),
                         _matrix(1,0),_matrix(1,1),_matrix(1,2),_matrix(1,3),
                         _matrix(2,0),_matrix(2,1),_matrix(2,2),_matrix(2,3));
    // 设置模型的位姿
    SetLocalTransformation(t_transfer);
}

Bnd_Box CModel::m_get_full_aabb()
{//! 获取整个模型的AABB包围盒，包括子模型
    Bnd_Box t_box(BoundingBox());
     t_box = t_box.Transformed(Transformation());
    for(PrsMgr_ListOfPresentableObjectsIter i(Children());i.More();i.Next())
    {
        // 获取当前模型
        Handle(PrsMgr_PresentableObject) t_object = i.Value();
        // 若为CModel类型则转换为CModel
        if(t_object->IsKind(STANDARD_TYPE(CModel)))
        {
            Handle(CModel) t_child_model =  Handle(CModel)::DownCast(t_object);
            Bnd_Box t_child_box = t_child_model->BoundingBox();
            t_child_box = t_child_box.Transformed(t_child_model->Transformation());
            t_box.Add(t_child_box);
        }
    }
    return t_box;
}

bool CModel::m_export_model(QString _filename, const char *_format_id,Handle(AIS_InteractiveContext) _context)
{
    Exporter exporter;
    // 定义场景
    aiScene *t_scene = new aiScene();
    // 单一材质

    // 创建根节点
    t_scene->mRootNode=new aiNode();
    // 创建子节点
    int t_NumChildrenNode = _context->NbSelected();
    aiNode **t_node_list=new aiNode*[t_NumChildrenNode];

    t_scene->mNumMaterials=t_NumChildrenNode;
    t_scene->mMaterials = new aiMaterial*[t_NumChildrenNode];

    // 定义场景所有网格
    t_scene->mNumMeshes = t_NumChildrenNode;
    t_scene->mMeshes = new aiMesh*[t_NumChildrenNode];

    int t_index = 0;
    for ( _context->InitSelected(); _context->MoreSelected(); _context->NextSelected() )
    {
        aiNode *t_node = t_node_list[t_index] = new aiNode();
        t_node->mNumMeshes=1;   // 一个网格
        t_node->mNumChildren=0; // 无子节点
        t_node->mMeshes = new uint[1];  //一个网格地址
        t_node->mMeshes[0] = t_index; // 网格地址索引

        aiMesh* pMesh = t_scene->mMeshes[t_index] = new aiMesh(); // 创建地址0的网格
        pMesh->mMaterialIndex = t_index;  // 网格材质

        Standard_Integer aNbNodes = 0;
        Standard_Integer aNbTriangles = 0;

        Handle(AIS_InteractiveObject) obj = _context->SelectedInteractive();

//        if ( obj->IsKind( STANDARD_TYPE( AIS_Shape ) ) )
//        {
            Handle(AIS_Shape) ais_shape = Handle(AIS_Shape)::DownCast(obj);

            Graphic3d_MaterialAspect shpae_material(ais_shape->Material());

            aiMaterial* pMaterial = t_scene->mMaterials[t_index] = new aiMaterial();

            Quantity_Color amb=shpae_material.AmbientColor();
            aiColor4D ambient(amb.Red(),amb.Green(),amb.Blue(),1.0);
            pMaterial->AddProperty(&ambient,1,AI_MATKEY_COLOR_AMBIENT);


            Quantity_Color diff=shpae_material.DiffuseColor();
            aiColor4D diffuse(diff.Red(),diff.Green(),diff.Blue(),1.0);
            pMaterial->AddProperty(&diffuse,1,AI_MATKEY_COLOR_DIFFUSE);

            Quantity_Color spec=shpae_material.SpecularColor();
            aiColor4D specular(spec.Red(),spec.Green(),spec.Blue(),1.0);
            pMaterial->AddProperty(&specular,1,AI_MATKEY_COLOR_SPECULAR);

            Standard_ShortReal shin=shpae_material.Shininess();
            pMaterial->AddProperty(&shin,1,AI_MATKEY_SHININESS);

            TopoDS_Shape theShape = ais_shape->Shape();

//        }


        // calculate total number of the nodes and triangles
        for (TopExp_Explorer anExpSF (theShape, TopAbs_FACE); anExpSF.More(); anExpSF.Next())
        {
            TopLoc_Location aLoc;
            Handle(Poly_Triangulation) aTriangulation = BRep_Tool::Triangulation (TopoDS::Face (anExpSF.Current()), aLoc);
            if (! aTriangulation.IsNull())
            {
                aNbNodes += aTriangulation->NbNodes ();
                aNbTriangles += aTriangulation->NbTriangles ();
            }
        }

        pMesh->mNumVertices = aNbNodes;
        pMesh->mNumFaces = aNbTriangles;
        aiVector3D* vp,*vn;
        vp = pMesh->mVertices = new aiVector3D[pMesh->mNumVertices];
        vn = pMesh->mNormals = new aiVector3D[pMesh->mNumVertices];
        pMesh->mFaces = new aiFace[pMesh->mNumFaces];


        int index=0;
        int face_index=0;
        // fill temporary triangulation
        Standard_Integer aNodeOffset = 0;
        for (TopExp_Explorer anExpSF (theShape, TopAbs_FACE); anExpSF.More(); anExpSF.Next())
        {
            TopLoc_Location aLoc;
            Handle(Poly_Triangulation) aTriangulation = BRep_Tool::Triangulation (TopoDS::Face (anExpSF.Current()), aLoc);

            auto aNodes = aTriangulation->InternalNodes();
            auto aTriangles = aTriangulation->InternalTriangles();

            // copy nodes
            gp_Trsf aTrsf = aLoc.Transformation();
            for (Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
            {
                gp_Pnt aPnt = aNodes [aNodeIter];
                aPnt.Transform (aTrsf);
                qDebug()<<"nodes "<<aPnt.X()<<aPnt.Y()<<aPnt.Z();
                vp[index].Set(aPnt.X(),aPnt.Y(),aPnt.Z());
                vn[index].Set(0.0,0.0,1.0);
                index++;
            }

            // copy triangles
            const TopAbs_Orientation anOrientation = anExpSF.Current().Orientation();
            for (Standard_Integer aTriIter = aTriangles.Lower(); aTriIter <= aTriangles.Upper(); ++aTriIter)
            {
                Poly_Triangle aTri = aTriangles (aTriIter);

                Standard_Integer anId[3];
                aTri.Get (anId[0], anId[1], anId[2]);
                if (anOrientation == TopAbs_REVERSED)
                {
                    // Swap 1, 2.
                    Standard_Integer aTmpIdx = anId[1];
                    anId[1] = anId[2];
                    anId[2] = aTmpIdx;
                }
                // Update nodes according to the offset.
                anId[0] += aNodeOffset;
                anId[1] += aNodeOffset;
                anId[2] += aNodeOffset;
                aiFace& face = pMesh->mFaces[face_index++];
                face.mIndices = new unsigned int[face.mNumIndices = 3];
                face.mIndices[0]=anId[0]-1;
                face.mIndices[1]=anId[1]-1;
                face.mIndices[2]=anId[2]-1;
            }
            aNodeOffset += aNodes.Size();
        }

        t_index++;
    }

    // 根节点加入子节点
    t_scene->mRootNode->addChildren(t_NumChildrenNode,t_node_list);

    exporter.Export(t_scene , _format_id , _filename.toStdString());

    return true;

}
