/**
 * @file tesseract_physx_scene.h
 * @brief Tesseract Physx Scene Container.
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
#ifndef TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_SCENE_H
#define TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_SCENE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <thread>
#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision_physx/tesseract_physx.h>
#include <tesseract_collision_physx/tesseract_simulation_event_callback.h>
#include <tesseract_collision_physx/tesseract_simulation_filter_callback.h>
#include <tesseract_collision/core/types.h>

namespace tesseract_collision
{

/**
 * @brief This contains a single PhysX scene.
 *
 * Each PhysX Contact Manager will have its own scene but will share the TesseractPhysx objects. It is unclear if each
 * scene is operating on a different thread but sharing a TesseractPhysx is an issue.
 */
class TesseractPhysxScene
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<TesseractPhysxScene>;
  using ConstPtr = std::shared_ptr<const TesseractPhysxScene>;

  TesseractPhysxScene(TesseractPhysx::Ptr tesseract_physx);
  virtual ~TesseractPhysxScene();

  TesseractPhysxScene(const TesseractPhysxScene&) = delete;
  TesseractPhysxScene& operator=(const TesseractPhysxScene&) = delete;
  TesseractPhysxScene(TesseractPhysxScene&&) = delete;
  TesseractPhysxScene& operator=(TesseractPhysxScene&&) = delete;

  physx::PxScene* getScene();
  const TesseractPhysx::Ptr& getTesseractPhysx();

  ContactTestData& getContactTestData();

  void setupFiltering(physx::PxRigidActor* actor,
                      const physx::PxFilterData& filter_data);

  void setIsContactAllowedFn(IsContactAllowedFn fn);

private:
  physx::PxDefaultCpuDispatcher* dispatcher_{nullptr};
  physx::PxScene*                scene_ {nullptr};

  TesseractPhysx::Ptr physx_; /**< @brief The tesseract physx container the scene is associated with */
  ContactTestData contact_data_;

  TesseractSimulationEventCallback::Ptr event_cb_;
  TesseractSimulationFilterCallback::Ptr filter_cb_;
};
}

#endif // TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_SCENE_H
