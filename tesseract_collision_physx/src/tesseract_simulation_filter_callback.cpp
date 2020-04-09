/**
 * @file tesseract_simulation_filter_callback.h
 * @brief Tesseract Physx simulation filter callback.
 *
 * @author Levi Armstrong
 * @date March 07, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (LGPLv3)
 */

#include <tesseract_collision_physx/tesseract_simulation_filter_callback.h>

namespace tesseract_collision
{

TesseractSimulationFilterCallback::TesseractSimulationFilterCallback(const ContactTestData& contact_data)
  : contact_data_(contact_data) {}

physx::PxFilterFlags TesseractSimulationFilterCallback::pairFound(physx::PxU32 /*pairID*/,
                                                                  physx::PxFilterObjectAttributes /*attributes0*/,
                                                                  physx::PxFilterData /*filterData0*/,
                                                                  const physx::PxActor* a0,
                                                                  const physx::PxShape* /*s0*/,
                                                                  physx::PxFilterObjectAttributes /*attributes1*/,
                                                                  physx::PxFilterData /*filterData1*/,
                                                                  const physx::PxActor* a1,
                                                                  const physx::PxShape* /*s1*/,
                                                                  physx::PxPairFlags& pairFlags)
{
  // let triggers through
//  if(physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1))
//  {
//      pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
//      return physx::PxFilterFlag::eDEFAULT;
//  }

  // generate contacts for all that were not filtered above
  pairFlags = physx::PxPairFlag::eNOTIFY_CONTACT_POINTS | physx::PxPairFlag::eDETECT_DISCRETE_CONTACT;

  // trigger the contact callback for pairs (A,B) where
  // the filtermask of A contains the ID of B and vice versa.
  if(!contact_data_.done)
  {
    if (fn_ == nullptr || !fn_(a0->getName(), a1->getName()))
    {
      pairFlags |= (physx::PxPairFlag::eNOTIFY_TOUCH_FOUND | physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS);
    }
    else if (!fn_(a0->getName(), a1->getName()))
    {
      pairFlags |= (physx::PxPairFlag::eNOTIFY_TOUCH_FOUND | physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS);
    }
    else
    {
      // Ignore the collision pair as long as the bounding volumes of the pair objects overlap.
      //
      // Killed pairs will be ignored by the simulation and won't run through the filter again until one
      // of the following occurs:
      //
      //   - The bounding volumes of the two objects overlap again (after being separated)
      //   - The user enforces a re-filtering (see #PxScene::resetFiltering()) @see PxScene::resetFiltering()
      return physx::PxFilterFlag::eKILL;
    }
  }

  return physx::PxFilterFlag::eDEFAULT;
}

void TesseractSimulationFilterCallback::pairLost(physx::PxU32 /*pairID*/,
                                                 physx::PxFilterObjectAttributes /*attributes0*/,
                                                 physx::PxFilterData /*filterData0*/,
                                                 physx::PxFilterObjectAttributes /*attributes1*/,
                                                 physx::PxFilterData /*filterData1*/,
                                                 bool /*objectRemoved*/) {}


bool TesseractSimulationFilterCallback::statusChange(physx::PxU32& /*pairID*/,
                                                     physx::PxPairFlags& /*pairFlags*/,
                                                     physx::PxFilterFlags& /*filterFlags*/) { return false; }

void TesseractSimulationFilterCallback::setIsContactAllowedFn(IsContactAllowedFn fn) { fn_ = fn; }
}
