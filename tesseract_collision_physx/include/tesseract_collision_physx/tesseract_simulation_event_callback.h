/**
 * @file tesseract_simulation_event_callback.h
 * @brief Tesseract Physx simulation event callback.
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
#ifndef TESSERACT_COLLISION_PHYSX_TESSERACT_SIMULATION_EVENT_CALLBACK_H
#define TESSERACT_COLLISION_PHYSX_TESSERACT_SIMULATION_EVENT_CALLBACK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <set>
#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>

namespace tesseract_collision
{

class TesseractPhysx;

class TesseractSimulationEventCallback : public physx::PxSimulationEventCallback
{
public:
  using Ptr = std::shared_ptr<TesseractSimulationEventCallback>;

  TesseractSimulationEventCallback(TesseractPhysx *physx);
  ~TesseractSimulationEventCallback() override = default;
  TesseractSimulationEventCallback(const TesseractSimulationEventCallback&) = delete;
  TesseractSimulationEventCallback& operator=(const TesseractSimulationEventCallback&) = delete;
  TesseractSimulationEventCallback(TesseractSimulationEventCallback&&) = delete;
  TesseractSimulationEventCallback& operator=(TesseractSimulationEventCallback&&) = delete;

  void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override;
  void onWake(physx::PxActor** actors, physx::PxU32 count) override;
  void onSleep(physx::PxActor** actors, physx::PxU32 count) override;
  void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override;
  void onAdvance(const physx::PxRigidBody*const* bodyBuffer,
                 const physx::PxTransform* poseBuffer,
                 physx::PxU32 count) override;

  // https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/AdvancedCollisionDetection.html#extracting-contact-information
  void onContact(const physx::PxContactPairHeader& pairHeader,
                 const physx::PxContactPair* pairs,
                 physx::PxU32 nbPairs) override;

private:
  TesseractPhysx *physx_;
};
}

#endif // TESSERACT_COLLISION_PHYSX_TESSERACT_SIMULATION_EVENT_CALLBACK_H
