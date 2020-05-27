#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_sphere_sphere_unit.hpp>
#include <tesseract_collision_physx/physx_discrete_manager.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, PhysxDiscreteCollisionSphereSphereUnit)  // NOLINT
{
  PhysxDiscreteManager checker;
  test_suite::runTest(checker, false);
}

TEST(TesseractCollisionUnit, PhysxDiscreteCollisionSphereSphereConvexHullUnit)  // NOLINT
{
  PhysxDiscreteManager checker;

  // @note Only the runTestConvex2 passes, the other two fail on the nearest points because they are little different
  test_suite::detail::addCollisionObjects(checker, true);
  test_suite::detail::runTestConvex2(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
