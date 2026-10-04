#include <gtest/gtest.h>
#include "robotics/jy_export_transaction.h"
#include "jy_make_shapes.h"
#include "test_support.h"
#include <QTemporaryDir>
#include <QXmlStreamReader>
#include <fstream>
namespace fs = std::filesystem;
TEST(Robotics, SuccessfulExportPreservesUnrelatedFiles) {
    QTemporaryDir temp;
    fs::create_directories((temp.path()+"/robot/meshes").toStdString());
    writeFile(temp.filePath("robot/notes.txt"),"keep");
    writeFile(temp.filePath("robot/meshes/user.stl"),"keep");
    Link root("base", JyShapeBox());
    root.export_urdf({"robot",temp.path().toStdString(),Link::Format::URDF});
    EXPECT_TRUE(QFile::exists(temp.filePath("robot/notes.txt")));
    EXPECT_TRUE(QFile::exists(temp.filePath("robot/meshes/user.stl")));
    QFile urdf(temp.filePath("robot/urdf/robot.urdf"));
    ASSERT_TRUE(urdf.open(QIODevice::ReadOnly));
    QXmlStreamReader xml(urdf.readAll());
    while(!xml.atEnd()) xml.readNext();
    EXPECT_FALSE(xml.hasError()) << xml.errorString().toStdString();
}
TEST(Robotics, GenerationFailureLeavesOldOutput) {
    QTemporaryDir temp;
    fs::path target = temp.filePath("robot").toStdString();
    fs::create_directories(target);
    writeFile(temp.filePath("robot/model.txt"),"old");
    EXPECT_THROW(jelly::exportTransaction(target,[](const fs::path &staging) {
        std::ofstream(staging/"model.txt") << "new";
        throw std::runtime_error("injected generation failure");
    }),std::runtime_error);
    QFile old(temp.filePath("robot/model.txt")); ASSERT_TRUE(old.open(QIODevice::ReadOnly)); EXPECT_EQ(old.readAll(),"old");
}
TEST(Robotics, CommitFailureRollsBackPreviousFiles) {
    QTemporaryDir temp;
    fs::path target = temp.filePath("robot").toStdString();
    fs::create_directories(target);
    writeFile(temp.filePath("robot/a.txt"),"old");
    writeFile(temp.filePath("robot/z"),"blocks-directory");
    EXPECT_THROW(jelly::exportTransaction(target,[](const fs::path &staging) {
        std::ofstream(staging/"a.txt") << "new";
        fs::create_directory(staging/"z");
        std::ofstream(staging/"z/new.txt") << "new";
    }),fs::filesystem_error);
    QFile old(temp.filePath("robot/a.txt")); ASSERT_TRUE(old.open(QIODevice::ReadOnly)); EXPECT_EQ(old.readAll(),"old");
}
TEST(Robotics, InvalidGraphAndNamesRejected) {
    QTemporaryDir temp;
    Link root("base",JyShapeBox());
    EXPECT_THROW(root.export_urdf({"../escape",temp.path().toStdString()}),std::runtime_error);
    root.add(Joint("joint",JyAxes(),"fixed",{})).next(Link("base",JyShapeBox()));
    EXPECT_THROW(root.export_urdf({"robot",temp.path().toStdString()}),std::runtime_error);
    EXPECT_FALSE(QFile::exists(temp.filePath("robot")));
}
TEST(Robotics, InertiaAndJointPose) {
    JyShapeBox box(2,2,2);
    const auto inertia = JyShape::inertial({box});
    EXPECT_NEAR(inertia.mass,8,1e-6);
    EXPECT_NEAR(inertia.inertia_tensor[0],16.0/3.0,1e-6);
    JyAxes parent({1,2,3,0,0,0},1), child({2,4,6,0,0,0},1);
    const auto pose=child.joint2joint(parent);
    EXPECT_NEAR(pose[0],1,1e-6); EXPECT_NEAR(pose[1],2,1e-6); EXPECT_NEAR(pose[2],3,1e-6);
}
