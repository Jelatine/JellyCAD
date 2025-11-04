/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */

#include <gtest/gtest.h>
#include "jy_make_shapes.h"
#include "jy_shape.h"
#include <TopoDS.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

/**
 * @brief Test fixture for basic shape creation tests
 *
 * This fixture provides common setup and utility functions for testing
 * basic shape creation operations.
 */
class BasicShapesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }

    void TearDown() override {
        // Cleanup code if needed
    }

    /**
     * @brief Calculate the volume of a shape
     * @param shape The shape to measure
     * @return Volume in cubic units
     */
    double GetVolume(const JyShape& shape) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(shape.data(), props);
        return props.Mass();
    }

    /**
     * @brief Check if a shape is valid
     * @param shape The shape to check
     * @return true if shape is valid, false otherwise
     */
    bool IsValidShape(const JyShape& shape) {
        return !shape.data().IsNull();
    }
};
// ==================== Box Tests ====================

TEST_F(BasicShapesTest, BoxDefaultConstruction) {
    JyShapeBox box;
    EXPECT_TRUE(IsValidShape(box));
    EXPECT_EQ(box.type(), "solid");

    // Default box is 1x1x1, so volume should be 1
    double volume = GetVolume(box);
    EXPECT_NEAR(volume, 1.0, 0.01);
}

TEST_F(BasicShapesTest, BoxCustomDimensions) {
    JyShapeBox box(2.0, 3.0, 4.0);
    EXPECT_TRUE(IsValidShape(box));

    // Volume should be 2 * 3 * 4 = 24
    double volume = GetVolume(box);
    EXPECT_NEAR(volume, 24.0, 0.01);
}

TEST_F(BasicShapesTest, BoxLargeDimensions) {
    JyShapeBox box(100.0, 100.0, 100.0);
    EXPECT_TRUE(IsValidShape(box));

    double volume = GetVolume(box);
    EXPECT_NEAR(volume, 1000000.0, 1.0);
}

// ==================== Cylinder Tests ====================

TEST_F(BasicShapesTest, CylinderDefaultConstruction) {
    JyCylinder cylinder;
    EXPECT_TRUE(IsValidShape(cylinder));
    EXPECT_EQ(cylinder.type(), "solid");

    // Default cylinder: radius=1, height=1, volume = π * r² * h = π
    double volume = GetVolume(cylinder);
    EXPECT_NEAR(volume, M_PI, 0.01);
}

TEST_F(BasicShapesTest, CylinderCustomDimensions) {
    JyCylinder cylinder(2.0, 5.0);
    EXPECT_TRUE(IsValidShape(cylinder));

    // Volume = π * 2² * 5 = 20π
    double volume = GetVolume(cylinder);
    EXPECT_NEAR(volume, 20.0 * M_PI, 0.1);
}

TEST_F(BasicShapesTest, CylinderSmallRadius) {
    JyCylinder cylinder(0.1, 1.0);
    EXPECT_TRUE(IsValidShape(cylinder));

    // Volume = π * 0.1² * 1 = 0.01π
    double volume = GetVolume(cylinder);
    EXPECT_NEAR(volume, 0.01 * M_PI, 0.001);
}

// ==================== Sphere Tests ====================

TEST_F(BasicShapesTest, SphereDefaultConstruction) {
    JySphere sphere;
    EXPECT_TRUE(IsValidShape(sphere));
    EXPECT_EQ(sphere.type(), "solid");

    // Default sphere: radius=1, volume = (4/3) * π * r³ = (4/3)π
    double volume = GetVolume(sphere);
    EXPECT_NEAR(volume, (4.0 / 3.0) * M_PI, 0.01);
}

TEST_F(BasicShapesTest, SphereCustomRadius) {
    JySphere sphere(3.0);
    EXPECT_TRUE(IsValidShape(sphere));

    // Volume = (4/3) * π * 3³ = 36π
    double volume = GetVolume(sphere);
    EXPECT_NEAR(volume, 36.0 * M_PI, 0.1);
}

TEST_F(BasicShapesTest, SphereSmallRadius) {
    JySphere sphere(0.5);
    EXPECT_TRUE(IsValidShape(sphere));

    // Volume = (4/3) * π * 0.5³ = π/6
    double volume = GetVolume(sphere);
    EXPECT_NEAR(volume, M_PI / 6.0, 0.01);
}

// ==================== Cone Tests ====================

TEST_F(BasicShapesTest, ConeDefaultConstruction) {
    JyCone cone;
    EXPECT_TRUE(IsValidShape(cone));
    EXPECT_EQ(cone.type(), "solid");

    // Default cone: R1=1, R2=0, H=1
    // Volume = (1/3) * π * h * (r1² + r1*r2 + r2²) = π/3
    double volume = GetVolume(cone);
    EXPECT_NEAR(volume, M_PI / 3.0, 0.01);
}

TEST_F(BasicShapesTest, ConeCustomDimensions) {
    JyCone cone(2.0, 1.0, 3.0);
    EXPECT_TRUE(IsValidShape(cone));

    // Truncated cone volume = (1/3) * π * h * (r1² + r1*r2 + r2²)
    // = (1/3) * π * 3 * (4 + 2 + 1) = 7π
    double volume = GetVolume(cone);
    EXPECT_NEAR(volume, 7.0 * M_PI, 0.1);
}

TEST_F(BasicShapesTest, ConePerfectCone) {
    JyCone cone(3.0, 0.0, 6.0);
    EXPECT_TRUE(IsValidShape(cone));

    // Perfect cone: Volume = (1/3) * π * r² * h = (1/3) * π * 9 * 6 = 18π
    double volume = GetVolume(cone);
    EXPECT_NEAR(volume, 18.0 * M_PI, 0.1);
}

// ==================== Torus Tests ====================

TEST_F(BasicShapesTest, TorusDefaultConstruction) {
    JyTorus torus;
    EXPECT_TRUE(IsValidShape(torus));
    EXPECT_EQ(torus.type(), "solid");

    // Default torus: R1=2, R2=1, angle=360
    // Volume = 2 * π² * R1 * R2² = 2 * π² * 2 * 1 = 4π²
    double volume = GetVolume(torus);
    EXPECT_NEAR(volume, 4.0 * M_PI * M_PI, 0.1);
}

TEST_F(BasicShapesTest, TorusCustomDimensions) {
    JyTorus torus(3.0, 1.5, 360.0);
    EXPECT_TRUE(IsValidShape(torus));

    // Volume = 2 * π² * 3 * 1.5² = 13.5π²
    double volume = GetVolume(torus);
    EXPECT_NEAR(volume, 13.5 * M_PI * M_PI, 0.5);
}

TEST_F(BasicShapesTest, TorusPartialAngle) {
    JyTorus torus(2.0, 1.0, 180.0);
    EXPECT_TRUE(IsValidShape(torus));

    // Half torus: Volume should be half of full torus = 2π²
    double volume = GetVolume(torus);
    EXPECT_NEAR(volume, 2.0 * M_PI * M_PI, 0.1);
}

// ==================== Wedge Tests ====================

TEST_F(BasicShapesTest, WedgeDefaultConstruction) {
    JyWedge wedge;
    EXPECT_TRUE(IsValidShape(wedge));
    EXPECT_EQ(wedge.type(), "solid");
}

TEST_F(BasicShapesTest, WedgeCustomDimensions) {
    JyWedge wedge(2.0, 3.0, 4.0, 0.5);
    EXPECT_TRUE(IsValidShape(wedge));

    // Wedge volume calculation is more complex, just verify it's valid
    double volume = GetVolume(wedge);
    EXPECT_GT(volume, 0.0);
}

// ==================== vertex Tests ====================

TEST_F(BasicShapesTest, VertexConstruction) {
    JyVertex vertex(1.0, 2.0, 3.0);
    EXPECT_TRUE(IsValidShape(vertex));
    EXPECT_EQ(vertex.type(), "vertex");
}

TEST_F(BasicShapesTest, VertexAtOrigin) {
    JyVertex vertex(0.0, 0.0, 0.0);
    EXPECT_TRUE(IsValidShape(vertex));
    EXPECT_EQ(vertex.type(), "vertex");
}

TEST_F(BasicShapesTest, VertexNegativeCoordinates) {
    JyVertex vertex(-5.0, -10.0, -15.0);
    EXPECT_TRUE(IsValidShape(vertex));
    EXPECT_EQ(vertex.type(), "vertex");
}

// ==================== Color and Transparency Tests ====================

TEST_F(BasicShapesTest, ColorSetting) {
    JyShapeBox box;

    // Test color setting with hex value
    EXPECT_NO_THROW({
        box.color("#FF0000");
    });

    // Verify color was set (rgba should have red channel)
    auto rgba = box.rgba();
    EXPECT_NEAR(rgba[0], 1.0, 0.01); // Red channel
}

TEST_F(BasicShapesTest, TransparencySetting) {
    JyCylinder cylinder;

    // Test transparency setting
    EXPECT_NO_THROW({
        cylinder.transparency(0.5);
    });

    // Verify transparency was set
    EXPECT_NEAR(cylinder.transparency_, 0.5, 0.01);
}

TEST_F(BasicShapesTest, TransparencyBoundaries) {
    JySphere sphere;

    // Test boundary values
    sphere.transparency(0.0);
    EXPECT_NEAR(sphere.transparency_, 0.0, 0.01);

    sphere.transparency(1.0);
    EXPECT_NEAR(sphere.transparency_, 1.0, 0.01);
}

// ==================== Type Identification Tests ====================

TEST_F(BasicShapesTest, TypeIdentification) {
    JyShapeBox box;
    EXPECT_EQ(box.type(), "solid");

    JyCylinder cylinder;
    EXPECT_EQ(cylinder.type(), "solid");

    JySphere sphere;
    EXPECT_EQ(sphere.type(), "solid");

    JyVertex vertex(0, 0, 0);
    EXPECT_EQ(vertex.type(), "vertex");
}
