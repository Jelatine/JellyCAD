#-------------------------------------------------
#
# Project created by QtCreator 2019-04-21T01:59:22
#
#-------------------------------------------------

QT       += core gui opengl

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
    c3dwidget.h \
    makebottle.h

FORMS += \
        cmainwindow.ui

#Windows系统下编译
win32 {
    #包含路径
    INCLUDEPATH += $$(CASROOT)/inc
    #确定编译器
    win32-msvc2010 {
        compiler=vc10
    }
    win32-msvc2012 {
        compiler=vc11
    }
    win32-msvc2013 {
        compiler=vc12
    }
    win32-msvc2015 {
        compiler=vc14
    }
    #确定64位或32位系统，增加库的路径
    !contains(QMAKE_TARGET.arch, x86_64) {
        CONFIG(debug, debug|release) {  #x86_64 debug
            LIBS += -L$$(CASROOT)/win32/$$compiler/libd
        }
        else {  #x86_64 release
            LIBS += -L$$(CASROOT)/win32/$$compiler/lib
        }
    }
    else {
        CONFIG(debug, debug|release) {  #x86 debug
            LIBS += -L$$(CASROOT)/win64/$$compiler/libd
        }
        else {  #x86 release
            LIBS += -L$$(CASROOT)/win64/$$compiler/lib
        }
    }
}

#Linux环境下编译
linux-g++ {
    INCLUDEPATH += /usr/local/include/opencascade
    LIBS += -L/usr/local/lib
}

#添加OCC库
LIBS += -lTKernel -lTKMath -lTKService -lTKV3d -lTKOpenGl \
        -lTKBRep -lTKIGES -lTKSTL -lTKVRML -lTKSTEP -lTKSTEPAttr -lTKSTEP209 \
        -lTKSTEPBase -lTKGeomBase -lTKGeomAlgo -lTKG3d -lTKG2d \
        -lTKXSBase -lTKShHealing -lTKHLR -lTKTopAlgo -lTKMesh -lTKPrim \
        -lTKCDF -lTKBool -lTKBO -lTKFillet -lTKOffset \

RESOURCES += \
    res.qrc

