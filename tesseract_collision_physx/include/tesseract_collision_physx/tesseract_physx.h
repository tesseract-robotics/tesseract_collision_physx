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
#include <thread>
#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_collision
{

/**
 * @brief Tesseract Physx only allowrf one copy per thread.
 *
 * This contains PhysX components that are only allowed once per thread. It is currently unclear if this is allowed to
 * be shared between threads.
 */
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
  physx::PxMaterial* getMaterial() const;

  physx::PxDefaultAllocator& getAllocator();
  const physx::PxDefaultErrorCallback& getErrorCallback();

private:
  physx::PxDefaultAllocator		default_allocator_;
  physx::PxDefaultErrorCallback	error_callback_;

  physx::PxFoundation*           foundation_ {nullptr};
  physx::PxPhysics*              physics_ {nullptr};
  physx::PxCooking*              cooking_ {nullptr};
  physx::PxPvd*                  pvd_ {nullptr};
  std::string                    pvd_host_ {"127.0.0.1"};
  physx::PxMaterial*				     material_{nullptr};
  std::thread::id                thread_id_;
};
}
#endif // TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_H
