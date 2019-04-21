#-------------------------------------------------
#
# Project created by QtCreator 2019-04-21T01:59:22
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = JellyCAD
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        cmainwindow.cpp \
    c3dwidget.cpp

HEADERS += \
        cmainwindow.h \
    c3dwidget.h

FORMS += \
        cmainwindow.ui

CASROOT = D:/OpenCascade/OpenCASCADE-7.3.0-vc14-64/opencascade-7.3.0
compiler = vc14
INCLUDEPATH +=  $$CASROOT/inc
LIBS += -L$$CASROOT/win64/$$compiler/lib

LIBS +=              \
-lTKBin              \
-lTKBinL             \
-lTKBinTObj          \
-lTKBinXCAF          \
-lTKBO               \
-lTKBool             \
-lTKBRep             \
-lTKCAF              \
-lTKCDF              \
-lTKD3DHost          \
-lTKDCAF             \
-lTKDFBrowser        \
-lTKDraw             \
-lTKernel            \
-lTKFeat             \
-lTKFillet           \
-lTKG2d              \
-lTKG3d              \
-lTKGeomAlgo         \
-lTKGeomBase         \
-lTKHLR              \
-lTKIGES             \
-lTKIVtk             \
-lTKIVtkDraw         \
-lTKLCAF             \
-lTKMath             \
-lTKMesh             \
-lTKMeshVS           \
-lTKOffset           \
-lTKOpenGl           \
-lTKPrim             \
-lTKQADraw           \
-lTKService          \
-lTKShapeView        \
-lTKShHealing        \
-lTKStd              \
-lTKStdL             \
-lTKSTEP             \
-lTKSTEP209          \
-lTKSTEPAttr         \
-lTKSTEPBase         \
-lTKSTL              \
-lTKTInspector       \
-lTKTInspectorAPI    \
-lTKTObj             \
-lTKTObjDRAW         \
-lTKToolsDraw        \
-lTKTopAlgo          \
-lTKTopTest          \
-lTKTreeModel        \
-lTKV3d              \
-lTKVCAF             \
-lTKView             \
-lTKViewerTest       \
-lTKVInspector       \
-lTKVRML             \
-lTKXCAF             \
-lTKXDEDRAW          \
-lTKXDEIGES          \
-lTKXDESTEP          \
-lTKXMesh            \
-lTKXml              \
-lTKXmlL             \
-lTKXmlTObj          \
-lTKXmlXCAF          \
-lTKXSBase           \
-lTKXSDRAW
