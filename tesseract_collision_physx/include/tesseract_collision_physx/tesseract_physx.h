/**
 * @file tesseract_Physx.h
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
#ifndef TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_H
#define TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision_physx/tesseract_simulation_event_callback.h>
#include <tesseract_collision_physx/tesseract_simulation_filter_callback.h>
#include <tesseract_collision/core/types.h>

namespace tesseract_collision
{

class TesseractPhysx
{
public:
  using Ptr = std::shared_ptr<TesseractPhysx>;
  using ConstPtr = std::shared_ptr<const TesseractPhysx>;

  TesseractPhysx();
  virtual ~TesseractPhysx();
  TesseractPhysx(const TesseractPhysx&) = delete;
  TesseractPhysx& operator=(const TesseractPhysx&) = delete;
  TesseractPhysx(TesseractPhysx&&) = delete;
  TesseractPhysx& operator=(TesseractPhysx&&) = delete;

  physx::PxFoundation* getFoundation();
  physx::PxPhysics* getPhysics();
  physx::PxCooking* getCooking();
  physx::PxScene* getScene();
  physx::PxMaterial* getMaterial() const;

  physx::PxDefaultAllocator& getAllocator();
  const physx::PxDefaultErrorCallback& getErrorCallback();

  ContactTestData& getContactTestData();

  void setupFiltering(physx::PxRigidActor* actor,
                      const physx::PxFilterData& filter_data);

  void setIsContactAllowedFn(IsContactAllowedFn fn);

private:
  physx::PxDefaultAllocator		default_allocator_;
  physx::PxDefaultErrorCallback	error_callback_;

  physx::PxFoundation*           foundation_ {nullptr};
  physx::PxPhysics*              physics_ {nullptr};
  physx::PxCooking*              cooking_ {nullptr};
  physx::PxScene*                scene_ {nullptr};
  physx::PxDefaultCpuDispatcher* dispatcher_{nullptr};
  physx::PxMaterial*				     material_{nullptr};
  physx::PxPvd*                  pvd_{nullptr};
  std::string                    pvd_host_ {"127.0.0.1"};

  ContactTestData contact_data_;

  TesseractSimulationEventCallback::Ptr event_cb_;
  TesseractSimulationFilterCallback::Ptr filter_cb_;

  void initialize();
};
}
#endif // TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_H
