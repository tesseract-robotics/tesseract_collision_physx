#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_box_sphere_unit.hpp>
#include <tesseract_collision_physx/physx_discrete_manager.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, PhysxDiscreteCollisionBoxSphereUnit)  // NOLINT
{
  PhysxDiscreteManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, PhysxDiscreteCollisionBoxSphereConvexHullUnit)  // NOLINT
{
  PhysxDiscreteManager checker;
  test_suite::runTest(checker, true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
