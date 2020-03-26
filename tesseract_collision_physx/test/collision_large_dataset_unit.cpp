#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_large_dataset_unit.hpp>
#include <tesseract_collision_physx/physx_discrete_manager.h>

using namespace tesseract_collision;

TEST(TesseractCollisionLargeDataSetUnit, PhysxDiscreteCollisionLargeDataSetConvexHullUnit)  // NOLINT
{
  PhysxDiscreteManager checker;
  test_suite::runTest(checker, true);
}

TEST(TesseractCollisionLargeDataSetUnit, PhysxDiscreteCollisionLargeDataSetUnit)  // NOLINT
{
  PhysxDiscreteManager checker;
  test_suite::runTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
