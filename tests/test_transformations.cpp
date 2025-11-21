/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */

#include <gtest/gtest.h>
#include "jy_make_shapes.h"
#include "jy_shape.h"
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

/**
 * @brief Test fixture for transformation tests
 *
 * This fixture provides utility functions for testing geometric transformations
 * like translation, rotation, and scaling operations.
 */
class TransformationsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }

    void TearDown() override {
        // Cleanup code if needed
    }

    /**
     * @brief Get the center of mass of a shape
     * @param shape The shape to measure
     * @return Array of [x, y, z] coordinates
     */
    std::array<double, 3> GetCenterOfMass(const JyShape& shape) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(shape.data(), props);
        gp_Pnt center = props.CentreOfMass();
        return {center.X(), center.Y(), center.Z()};
    }

    /**
     * @brief Get bounding box of a shape
     * @param shape The shape to measure
     * @return Array of [xmin, ymin, zmin, xmax, ymax, zmax]
     */
    std::array<double, 6> GetBoundingBox(const JyShape& shape) {
        Bnd_Box box;
        BRepBndLib::Add(shape.data(), box);
        double xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        return {xmin, ymin, zmin, xmax, ymax, zmax};
    }

    /**
     * @brief Calculate volume of a shape
     * @param shape The shape to measure
     * @return Volume in cubic units
     */
    double GetVolume(const JyShape& shape) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(shape.data(), props);
        return props.Mass();
    }
};
// ==================== Translation Tests ====================

TEST_F(TransformationsTest, TranslateAlongX) {
    JyShapeBox box(1.0, 1.0, 1.0);
    auto center_before = GetCenterOfMass(box);

    // Translate 5 units along X axis
    box.x(5.0);

    auto center_after = GetCenterOfMass(box);

    // X coordinate should increase by 5
    EXPECT_NEAR(center_after[0], center_before[0] + 5.0, 0.01);
    // Y and Z should remain unchanged
    EXPECT_NEAR(center_after[1], center_before[1], 0.01);
    EXPECT_NEAR(center_after[2], center_before[2], 0.01);
}

TEST_F(TransformationsTest, TranslateAlongY) {
    JyCylinder cylinder(1.0, 2.0);
    auto center_before = GetCenterOfMass(cylinder);

    // Translate 3 units along Y axis
    cylinder.y(3.0);

    auto center_after = GetCenterOfMass(cylinder);

    // Y coordinate should increase by 3
    EXPECT_NEAR(center_after[0], center_before[0], 0.01);
    EXPECT_NEAR(center_after[1], center_before[1] + 3.0, 0.01);
    EXPECT_NEAR(center_after[2], center_before[2], 0.01);
}

TEST_F(TransformationsTest, TranslateAlongZ) {
    JySphere sphere(2.0);
    auto center_before = GetCenterOfMass(sphere);

    // Translate 4 units along Z axis
    sphere.z(4.0);

    auto center_after = GetCenterOfMass(sphere);

    // Z coordinate should increase by 4
    EXPECT_NEAR(center_after[0], center_before[0], 0.01);
    EXPECT_NEAR(center_after[1], center_before[1], 0.01);
    EXPECT_NEAR(center_after[2], center_before[2] + 4.0, 0.01);
}

TEST_F(TransformationsTest, TranslateAllAxes) {
    JyShapeBox box(1.0, 1.0, 1.0);
    auto center_before = GetCenterOfMass(box);
    EXPECT_NEAR(center_before[0], 0, 0.01);
    EXPECT_NEAR(center_before[1], 0, 0.01);
    EXPECT_NEAR(center_before[2], 0.5, 0.01);

    // Translate along all axes
    box.pos(10.0, 20.0, 30.0);

    auto center_after = GetCenterOfMass(box);

    // All coordinates should be at the specified position
    EXPECT_NEAR(center_after[0], 10.5, 0.01);
    EXPECT_NEAR(center_after[1], 20.5, 0.01);
    EXPECT_NEAR(center_after[2], 30.5, 0.01);
}

TEST_F(TransformationsTest, ChainedTranslations) {
    JyShapeBox box(1.0, 1.0, 1.0);

    // Chain multiple translations
    box.x(5.0).y(3.0).z(2.0);

    auto center = GetCenterOfMass(box);

    EXPECT_NEAR(center[0], 5.5, 0.01);  // 0.5 (initial) + 5.0
    EXPECT_NEAR(center[1], 3.5, 0.01);  // 0.5 (initial) + 3.0
    EXPECT_NEAR(center[2], 2.5, 0.01);  // 0.5 (initial) + 2.0
}

TEST_F(TransformationsTest, NegativeTranslation) {
    JyShapeBox box(1.0, 1.0, 1.0);

    // Translate with negative values
    box.x(-5.0).y(-3.0).z(-2.0);

    auto center = GetCenterOfMass(box);

    EXPECT_NEAR(center[0], -4.5, 0.01);
    EXPECT_NEAR(center[1], -2.5, 0.01);
    EXPECT_NEAR(center[2], -1.5, 0.01);
}

// ==================== Rotation Tests ====================

TEST_F(TransformationsTest, RotateAroundX) {
    JyShapeBox box(1.0, 1.0, 1.0);

    // Rotate 90 degrees around X axis
    EXPECT_NO_THROW({
        box.rx(90.0);
    });

    // Volume should remain constant after rotation
    double volume = GetVolume(box);
    EXPECT_NEAR(volume, 1.0, 0.01);
}

TEST_F(TransformationsTest, RotateAroundY) {
    JyCylinder cylinder(1.0, 2.0);

    // Rotate 45 degrees around Y axis
    EXPECT_NO_THROW({
        cylinder.ry(45.0);
    });

    // Volume should remain constant
    double volume = GetVolume(cylinder);
    EXPECT_NEAR(volume, M_PI * 2.0, 0.01);
}

TEST_F(TransformationsTest, RotateAroundZ) {
    JyShapeBox box(2.0, 1.0, 1.0);

    // Rotate 180 degrees around Z axis
    EXPECT_NO_THROW({
        box.rz(180.0);
    });

    // Volume should remain constant
    double volume = GetVolume(box);
    EXPECT_NEAR(volume, 2.0, 0.01);
}

TEST_F(TransformationsTest, MultipleRotations) {
    JySphere sphere(1.0);
    double volume_before = GetVolume(sphere);

    // Apply multiple rotations
    sphere.rx(30.0).ry(45.0).rz(60.0);

    double volume_after = GetVolume(sphere);

    // Volume should remain constant for sphere
    EXPECT_NEAR(volume_before, volume_after, 0.01);
}

TEST_F(TransformationsTest, RotationAndTranslation) {
    JyShapeBox box(1.0, 1.0, 1.0);

    // Combine rotation and translation
    box.rx(90.0).x(5.0).y(3.0);

    auto center = GetCenterOfMass(box);

    // Check that translation was applied
    EXPECT_GT(std::abs(center[0]), 4.0);  // Should be around 5.5
}

TEST_F(TransformationsTest, FullRotationPositioning) {
    JyShapeBox box(1.0, 1.0, 1.0);

    // Set absolute rotation
    EXPECT_NO_THROW({
        box.rot(45.0, 30.0, 60.0);
    });

    // Shape should still be valid
    double volume = GetVolume(box);
    EXPECT_NEAR(volume, 1.0, 0.01);
}

// ==================== Scale Tests ====================

TEST_F(TransformationsTest, ScaleUniform) {
    JyShapeBox box(1.0, 1.0, 1.0);
    double volume_before = GetVolume(box);

    // Scale by factor of 2
    box.scale(2.0);

    double volume_after = GetVolume(box);

    // Volume should increase by scale factor cubed (2³ = 8)
    EXPECT_NEAR(volume_after, volume_before * 8.0, 0.1);
}

TEST_F(TransformationsTest, ScaleSmaller) {
    JySphere sphere(2.0);
    double volume_before = GetVolume(sphere);

    // Scale down by factor of 0.5
    sphere.scale(0.5);

    double volume_after = GetVolume(sphere);

    // Volume should decrease by (0.5)³ = 0.125
    EXPECT_NEAR(volume_after, volume_before * 0.125, 0.01);
}

TEST_F(TransformationsTest, ScaleLarge) {
    JyCylinder cylinder(1.0, 1.0);
    double volume_before = GetVolume(cylinder);

    // Scale up by factor of 10
    cylinder.scale(10.0);

    double volume_after = GetVolume(cylinder);

    // Volume should increase by 10³ = 1000
    EXPECT_NEAR(volume_after, volume_before * 1000.0, 1.0);
}
