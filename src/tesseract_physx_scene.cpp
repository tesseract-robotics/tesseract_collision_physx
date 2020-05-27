/**
 * @file tesseract_physx_scene.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision_physx/tesseract_physx_scene.h>
#include <tesseract_collision_physx/types.h>

namespace tesseract_collision
{

physx::PxFilterFlags contactReportFilterShader(physx::PxFilterObjectAttributes attributes0,
                                               physx::PxFilterData filterData0,
                                               physx::PxFilterObjectAttributes attributes1,
                                               physx::PxFilterData filterData1,
                                               physx::PxPairFlags& pairFlags,
                                               const void* constantBlock,
                                               physx::PxU32 constantBlockSize)
{
  UNUSED(constantBlockSize);
  UNUSED(constantBlock);

  // let triggers through
  if(physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1))
  {
      pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
      return physx::PxFilterFlag::eDEFAULT;
  }

  // trigger the contact callback for pairs (A,B) where
  // the filtermask of A contains the ID of B and vice versa.
  if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
    return physx::PxFilterFlag::eCALLBACK; // To call the is collision allowed function

  // Ignore the collision pair as long as the bounding volumes of the pair objects overlap.
  //
  // Killed pairs will be ignored by the simulation and won't run through the filter again until one
  // of the following occurs:
  //
  //   - The bounding volumes of the two objects overlap again (after being separated)
  //   - The user enforces a re-filtering (see #PxScene::resetFiltering()) @see PxScene::resetFiltering()
  return physx::PxFilterFlag::eKILL;
}

TesseractPhysxScene::TesseractPhysxScene(TesseractPhysx::Ptr tesseract_physx)
  : physx_(std::move(tesseract_physx))
{
  // Define event callback function for processing contacts
  event_cb_ = std::make_shared<TesseractSimulationEventCallback>(this);
  filter_cb_ = std::make_shared<TesseractSimulationFilterCallback>(contact_data_);

  physx::PxSceneDesc scene_desc(physx_->getPhysics()->getTolerancesScale());
  scene_desc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
  dispatcher_ = physx::PxDefaultCpuDispatcherCreate(static_cast<physx::PxU32>(physx_->getDescription().worker_threads));
  scene_desc.cpuDispatcher	= dispatcher_;
  scene_desc.kineKineFilteringMode = physx::PxPairFilteringMode::eKEEP; // So kin-kin contacts with be reported
  scene_desc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP; // So static-kin constacts will be reported
  scene_desc.simulationEventCallback = event_cb_.get();
  scene_desc.filterShader	= contactReportFilterShader;
  scene_desc.filterCallback = filter_cb_.get();
  scene_desc.broadPhaseType = physx_->getDescription().broad_phase_algorithm;
  scene_desc.dynamicTreeRebuildRateHint = physx_->getDescription().dynamic_tree_rebuild_rate_hint;

  if (physx_->getDescription().enable_gpu)
  {
    scene_desc.cudaContextManager = physx_->getCudaContextManager();
    scene_desc.flags |= physx::PxSceneFlag::eENABLE_GPU_DYNAMICS;
    scene_desc.flags |= physx::PxSceneFlag::eENABLE_PCM;
    scene_desc.broadPhaseType = physx::PxBroadPhaseType::eGPU;
    scene_desc.gpuDynamicsConfig = physx_->getDescription().cuda_dynamics_config;
  }

  scene_ = physx_->getPhysics()->createScene(scene_desc);

  physx::PxPvdSceneClient* pvdClient = scene_->getScenePvdClient();
  if(pvdClient)
  {
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
  }
}

TesseractPhysxScene::~TesseractPhysxScene()
{
  PX_RELEASE(scene_);
  PX_RELEASE(dispatcher_);
}

physx::PxScene* TesseractPhysxScene::getScene()  { return scene_; }

const TesseractPhysx::Ptr& TesseractPhysxScene::getTesseractPhysx() { return physx_; }

ContactTestData& TesseractPhysxScene::getContactTestData() { return contact_data_; }

void TesseractPhysxScene::setupFiltering(physx::PxRigidActor* actor,
                                         const physx::PxFilterData& filter_data)
{
  // If filtering changes must reset filtering because we are using the eKill feature in the filter callbacks
  // and shapes that were once allowed to be incollision may not be anymore.
  scene_->resetFiltering(*actor);

  const physx::PxU32 numShapes = actor->getNbShapes();
  physx::PxShape** shapes = (physx::PxShape**)physx_->getAllocator().allocate(sizeof(physx::PxShape*)*numShapes, nullptr, __FILE__, __LINE__);
  actor->getShapes(shapes, numShapes);
  for(physx::PxU32 i = 0; i < numShapes; i++)
  {
      physx::PxShape* shape = shapes[i];
      shape->setSimulationFilterData(filter_data);
  }
  physx_->getAllocator().deallocate(shapes);
}

void TesseractPhysxScene::setIsContactAllowedFn(IsContactAllowedFn fn)
{
  filter_cb_->setIsContactAllowedFn(fn);
}

}
