#ifndef MAKEBOTTLE_H
#define MAKEBOTTLE_H
/* ====================================================
#   Copyright (C)2019 Li Jianbin All rights reserved.
#
#   Author        : Li Jianbin
#   Email         : lijianbinmail@163.com
#   File Name     : makebottle.h
#   Last Modified : 2019-05-12 2:00
#   Describe      : Bottle model sample
#
# ====================================================*/
#include <Geom_TrimmedCurve.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopoDS_Wire.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <TopoDS.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <Geom_Plane.hxx>
#include <TopoDS_Edge.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>


TopoDS_Shape MakeBottle(const Standard_Real myWidth, const Standard_Real myHeight,
                        const Standard_Real myThickness)
{
    // 轮廓：定义支持点
    gp_Pnt aPnt1(-myWidth / 2., 0, 0);
    gp_Pnt aPnt2(-myWidth / 2., -myThickness / 4., 0);
    gp_Pnt aPnt3(0, -myThickness / 2., 0);
    gp_Pnt aPnt4(myWidth / 2., -myThickness / 4., 0);
    gp_Pnt aPnt5(myWidth / 2., 0, 0);
    // 轮廓：定义几何图形
    Handle(Geom_TrimmedCurve) anArcOfCircle = GC_MakeArcOfCircle(aPnt2,aPnt3,aPnt4);
    Handle(Geom_TrimmedCurve) aSegment1 = GC_MakeSegment(aPnt1, aPnt2);
    Handle(Geom_TrimmedCurve) aSegment2 = GC_MakeSegment(aPnt4, aPnt5);
    // 轮廓：定义拓扑结构
    TopoDS_Edge anEdge1 = BRepBuilderAPI_MakeEdge(aSegment1);
    TopoDS_Edge anEdge2 = BRepBuilderAPI_MakeEdge(anArcOfCircle);
    TopoDS_Edge anEdge3 = BRepBuilderAPI_MakeEdge(aSegment2);
    TopoDS_Wire aWire  = BRepBuilderAPI_MakeWire(anEdge1, anEdge2, anEdge3);
    // 轮廓：完成轮廓
    gp_Ax1 xAxis = gp::OX();    // 获得X轴
    gp_Trsf aTrsf;
    aTrsf.SetMirror(xAxis);
    BRepBuilderAPI_Transform aBRepTrsf(aWire, aTrsf);
    TopoDS_Shape aMirroredShape = aBRepTrsf.Shape();
    TopoDS_Wire aMirroredWire = TopoDS::Wire(aMirroredShape);
    BRepBuilderAPI_MakeWire mkWire;
    mkWire.Add(aWire);
    mkWire.Add(aMirroredWire);
    TopoDS_Wire myWireProfile = mkWire.Wire();
    // 瓶身：拉伸轮廓
    TopoDS_Face myFaceProfile = BRepBuilderAPI_MakeFace(myWireProfile);
    gp_Vec aPrismVec(0, 0, myHeight);
    TopoDS_Shape myBody = BRepPrimAPI_MakePrism(myFaceProfile, aPrismVec);
    // 瓶身：倒角
    BRepFilletAPI_MakeFillet mkFillet(myBody);
    TopExp_Explorer anEdgeExplorer(myBody, TopAbs_EDGE);
    while(anEdgeExplorer.More()){
        TopoDS_Edge anEdge = TopoDS::Edge(anEdgeExplorer.Current());
        //Add edge to fillet algorithm
        mkFillet.Add(myThickness / 12., anEdge);
        anEdgeExplorer.Next();
    }
    myBody = mkFillet.Shape();
    // 瓶身：添加瓶颈
    gp_Pnt neckLocation(0, 0, myHeight);
    gp_Dir neckAxis = gp::DZ();
    gp_Ax2 neckAx2(neckLocation, neckAxis);
    Standard_Real myNeckRadius = myThickness / 4.;
    Standard_Real myNeckHeight = myHeight / 10.;
    BRepPrimAPI_MakeCylinder MKCylinder(neckAx2, myNeckRadius, myNeckHeight);
    TopoDS_Shape myNeck = MKCylinder.Shape();
    myBody = BRepAlgoAPI_Fuse(myBody, myNeck);
    // 瓶身：创造中空的实体
    TopoDS_Face   faceToRemove;
    Standard_Real zMax = -1;
    for(TopExp_Explorer aFaceExplorer(myBody, TopAbs_FACE); aFaceExplorer.More(); aFaceExplorer.Next()){
        TopoDS_Face aFace = TopoDS::Face(aFaceExplorer.Current());
        // Check if <aFace> is the top face of the bottle's neck
        Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aFace);
        if(aSurface->DynamicType() == STANDARD_TYPE(Geom_Plane)){
            Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(aSurface);
            gp_Pnt aPnt = aPlane->Location();
            Standard_Real aZ   = aPnt.Z();
            if(aZ > zMax){
                zMax = aZ;
                faceToRemove = aFace;
            }
        }
    }
    TopTools_ListOfShape facesToRemove;
    facesToRemove.Append(faceToRemove);
    BRepOffsetAPI_MakeThickSolid BodyMaker;
    BodyMaker.MakeThickSolidByJoin(myBody, facesToRemove, -myThickness / 50, 1.e-3);
    myBody = BodyMaker.Shape();
    // 螺纹：创建表面
    Handle(Geom_CylindricalSurface) aCyl1 = new Geom_CylindricalSurface(neckAx2, myNeckRadius * 0.99);
    Handle(Geom_CylindricalSurface) aCyl2 = new Geom_CylindricalSurface(neckAx2, myNeckRadius * 1.05);
    // 螺纹：定义二维曲线
    gp_Pnt2d aPnt(2. * M_PI, myNeckHeight / 2.);
    gp_Dir2d aDir(2. * M_PI, myNeckHeight / 4.);
    gp_Ax2d anAx2d(aPnt, aDir);
    Standard_Real aMajor = 2. * M_PI;
    Standard_Real aMinor = myNeckHeight / 10;
    Handle(Geom2d_Ellipse) anEllipse1 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor);
    Handle(Geom2d_Ellipse) anEllipse2 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor / 4);
    Handle(Geom2d_TrimmedCurve) anArc1 = new Geom2d_TrimmedCurve(anEllipse1, 0, M_PI);
    Handle(Geom2d_TrimmedCurve) anArc2 = new Geom2d_TrimmedCurve(anEllipse2, 0, M_PI);
    gp_Pnt2d anEllipsePnt1 = anEllipse1->Value(0);
    gp_Pnt2d anEllipsePnt2 = anEllipse1->Value(M_PI);
    Handle(Geom2d_TrimmedCurve) aSegment = GCE2d_MakeSegment(anEllipsePnt1, anEllipsePnt2);
    // 螺纹：创建边和线
    TopoDS_Edge anEdge1OnSurf1 = BRepBuilderAPI_MakeEdge(anArc1, aCyl1);
    TopoDS_Edge anEdge2OnSurf1 = BRepBuilderAPI_MakeEdge(aSegment, aCyl1);
    TopoDS_Edge anEdge1OnSurf2 = BRepBuilderAPI_MakeEdge(anArc2, aCyl2);
    TopoDS_Edge anEdge2OnSurf2 = BRepBuilderAPI_MakeEdge(aSegment, aCyl2);
    TopoDS_Wire threadingWire1 = BRepBuilderAPI_MakeWire(anEdge1OnSurf1, anEdge2OnSurf1);
    TopoDS_Wire threadingWire2 = BRepBuilderAPI_MakeWire(anEdge1OnSurf2, anEdge2OnSurf2);
    BRepLib::BuildCurves3d(threadingWire1);
    BRepLib::BuildCurves3d(threadingWire2);
    // 构建螺纹
    BRepOffsetAPI_ThruSections aTool(Standard_True);
    aTool.AddWire(threadingWire1);
    aTool.AddWire(threadingWire2);
    aTool.CheckCompatibility(Standard_False);
    TopoDS_Shape myThreading = aTool.Shape();
    // 整合模型
    TopoDS_Compound aRes;
    BRep_Builder aBuilder;
    aBuilder.MakeCompound (aRes);
    aBuilder.Add (aRes, myBody);
    aBuilder.Add (aRes, myThreading);
    return aRes;
}
#endif // MAKEBOTTLE_H
