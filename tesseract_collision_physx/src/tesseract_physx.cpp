/**
 * @file tesseract_Physx.cpp
 * @brief Tesseract Physx Container.
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

#include <tesseract_collision_physx/tesseract_physx.h>
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

  // generate contacts for all that were not filtered above
  pairFlags = physx::PxPairFlag::eNOTIFY_CONTACT_POINTS | physx::PxPairFlag::eDETECT_DISCRETE_CONTACT;

  // trigger the contact callback for pairs (A,B) where
  // the filtermask of A contains the ID of B and vice versa.
  if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
      pairFlags |= (physx::PxPairFlag::eNOTIFY_TOUCH_FOUND | physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS);

  return physx::PxFilterFlag::eDEFAULT;
}


TesseractPhysx::TesseractPhysx() { initialize(); }
TesseractPhysx::~TesseractPhysx()
{
  PX_RELEASE(scene_);
  PX_RELEASE(dispatcher_);
  PX_RELEASE(physics_);
  PX_RELEASE(cooking_);

  if(pvd_)
  {
    physx::PxPvdTransport* transport = pvd_->getTransport();
    PX_RELEASE(pvd_);
    PX_RELEASE(transport);
  }

  PxCloseExtensions();

  PX_RELEASE(foundation_);
}

physx::PxFoundation* TesseractPhysx::getFoundation() { return foundation_; }
physx::PxPhysics* TesseractPhysx::getPhysics() { return physics_; }
physx::PxCooking* TesseractPhysx::getCooking() { return cooking_; }
physx::PxScene* TesseractPhysx::getScene()  { return scene_; }
physx::PxMaterial* TesseractPhysx::getMaterial() const { return material_; }

physx::PxDefaultAllocator& TesseractPhysx::getAllocator() { return default_allocator_; }
const physx::PxDefaultErrorCallback& TesseractPhysx::getErrorCallback() { return error_callback_; }

ContactTestData& TesseractPhysx::getContactTestData() { return contact_data_; }

void TesseractPhysx::setupFiltering(physx::PxRigidActor* actor,
                                    const physx::PxFilterData& filter_data)
{
  const physx::PxU32 numShapes = actor->getNbShapes();
  physx::PxShape** shapes = (physx::PxShape**)default_allocator_.allocate(sizeof(physx::PxShape*)*numShapes, nullptr, __FILE__, __LINE__);
  actor->getShapes(shapes, numShapes);
  for(physx::PxU32 i = 0; i < numShapes; i++)
  {
      physx::PxShape* shape = shapes[i];
      shape->setSimulationFilterData(filter_data);
  }
  default_allocator_.deallocate(shapes);
}

void TesseractPhysx::initialize()
{
  foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, default_allocator_, error_callback_);
  if(!foundation_)
      CONSOLE_BRIDGE_logError("PxCreateFoundation failed!");

  pvd_ = physx::PxCreatePvd(*foundation_);
  physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate(pvd_host_.c_str(), 5425, 10);
  pvd_->connect(*transport,physx::PxPvdInstrumentationFlag::eALL);

  physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, physx::PxTolerancesScale(), true, pvd_);
  if(!physics_)
    CONSOLE_BRIDGE_logError("PxCreatePhysics failed!");

  // The PhysX cooking library provides utilities for creating, converting, and serializing bulk data. Depending on
  // your application, you may wish to link to the cooking library in order to process such data at runtime.
  // Alternatively you may be able to process all such data in advance and just load it into memory as required.
  // Initialize the cooking library as follows:
  cooking_ = PxCreateCooking(PX_PHYSICS_VERSION, *foundation_, physx::PxCookingParams(physx::PxTolerancesScale()));
  if (!cooking_)
      CONSOLE_BRIDGE_logError("PxCreateCooking failed!");

  // The extensions library contains many functions that may be useful to a large class of users, but which some users
  // may prefer to omit from their application either for code size reasons or to avoid use of certain subsystems,
  // such as those pertaining to networking. Initializing the extensions library requires the PxPhysics object:
  if (!PxInitExtensions(*physics_, pvd_))
    CONSOLE_BRIDGE_logError("PxInitExtensions failed!");

  // Define event callback function for processing contacts
  event_cb_ = std::make_shared<TesseractSimulationEventCallback>(this);

  physx::PxSceneDesc scene_desc(physics_->getTolerancesScale());
  scene_desc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
  dispatcher_ = physx::PxDefaultCpuDispatcherCreate(2);
  scene_desc.cpuDispatcher	= dispatcher_;
  scene_desc.kineKineFilteringMode = physx::PxPairFilteringMode::eKEEP; // So kin-kin contacts with be reported
  scene_desc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP; // So static-kin constacts will be reported
  scene_desc.simulationEventCallback = event_cb_.get();
  scene_desc.filterShader	= contactReportFilterShader;
  scene_ = physics_->createScene(scene_desc);

  physx::PxPvdSceneClient* pvdClient = scene_->getScenePvdClient();
  if(pvdClient)
  {
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
  }

  material_ = physics_->createMaterial(0.5f, 0.5f, 0.6f);
}
}
