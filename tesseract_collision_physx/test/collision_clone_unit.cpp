#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/collision_clone_unit.hpp>
#include <tesseract_collision_physx/physx_discrete_manager.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, PhysxDiscreteCollisionCloneUnit)  // NOLINT
{
  PhysxDiscreteManager checker;
  test_suite::runTest(checker, 0.001, 0.001, 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
