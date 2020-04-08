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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision_physx/tesseract_physx.h>
#include <tesseract_collision_physx/types.h>

namespace tesseract_collision
{

TesseractPhysx::TesseractPhysx(TesseractPhysxDesc desc)
  : thread_id_(std::this_thread::get_id())
  , desc_(desc)
{
  foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, default_allocator_, error_callback_);
  if(!foundation_)
      CONSOLE_BRIDGE_logError("PxCreateFoundation failed!");

  if (desc_.debug)
  {
    pvd_ = physx::PxCreatePvd(*foundation_);
    physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate(desc_.pvd_host.c_str(), desc_.pvd_port, 10);
    pvd_->connect(*transport,physx::PxPvdInstrumentationFlag::eALL);
  }

  physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, physx::PxTolerancesScale(), true, pvd_);
  if(!physics_)
    CONSOLE_BRIDGE_logError("PxCreatePhysics failed!");

#ifdef PX_SUPPORT_GPU_PHYSX
  if (desc_.enable_gpu)
    cuda_ = PxCreateCudaContextManager(*foundation_, desc_.cuda_desc);
#else
  UNUSED(enable_gpu);
#endif

  // If cuda setup failed disable gpu
  desc_.enable_gpu = (cuda_ != nullptr);

  // The PhysX cooking library provides utilities for creating, converting, and serializing bulk data. Depending on
  // your application, you may wish to link to the cooking library in order to process such data at runtime.
  // Alternatively you may be able to process all such data in advance and just load it into memory as required.
  // Initialize the cooking library as follows:
  physx::PxCookingParams cooking_desc((physx::PxTolerancesScale()));
  cooking_desc.buildGPUData = (cuda_ != nullptr);
  cooking_ = PxCreateCooking(PX_PHYSICS_VERSION, *foundation_, cooking_desc);
  if (!cooking_)
      CONSOLE_BRIDGE_logError("PxCreateCooking failed!");

  // The extensions library contains many functions that may be useful to a large class of users, but which some users
  // may prefer to omit from their application either for code size reasons or to avoid use of certain subsystems,
  // such as those pertaining to networking. Initializing the extensions library requires the PxPhysics object:
  if (!PxInitExtensions(*physics_, pvd_))
    CONSOLE_BRIDGE_logError("PxInitExtensions failed!");

  material_ = physics_->createMaterial(0.5f, 0.5f, 0.6f);
}

TesseractPhysx::~TesseractPhysx()
{
  PX_RELEASE(physics_);
  PX_RELEASE(cooking_);

  if(pvd_)
  {
    physx::PxPvdTransport* transport = pvd_->getTransport();
    PX_RELEASE(pvd_);
    PX_RELEASE(transport);
  }

  PxCloseExtensions();

  PX_RELEASE(cuda_);

  PX_RELEASE(foundation_);
}

physx::PxFoundation* TesseractPhysx::getFoundation() { return foundation_; }
physx::PxPhysics* TesseractPhysx::getPhysics() { return physics_; }
physx::PxCooking* TesseractPhysx::getCooking() { return cooking_; }
physx::PxMaterial* TesseractPhysx::getMaterial() const { return material_; }
physx::PxCudaContextManager* TesseractPhysx::getCudaContextManager() { return cuda_; }
physx::PxDefaultAllocator& TesseractPhysx::getAllocator() { return default_allocator_; }
const physx::PxDefaultErrorCallback& TesseractPhysx::getErrorCallback() { return error_callback_; }
const TesseractPhysxDesc& TesseractPhysx::getDescription() { return desc_; }
}
