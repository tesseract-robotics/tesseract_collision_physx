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

  /**
   * @brief TesseractPhysx
   * @param worker_threads The number of worker threads created for each scene.
   * @param enable_gpu Enable GPU functionality
   */
  TesseractPhysx(int worker_threads = 2,
                 bool enable_gpu = false,
                 bool debug = false,
                 std::string pvd_host = "127.0.0.1",
                 int pvd_port = 5425);

  virtual ~TesseractPhysx();

  TesseractPhysx(const TesseractPhysx&) = delete;
  TesseractPhysx& operator=(const TesseractPhysx&) = delete;
  TesseractPhysx(TesseractPhysx&&) = delete;
  TesseractPhysx& operator=(TesseractPhysx&&) = delete;

  physx::PxFoundation* getFoundation();
  physx::PxPhysics* getPhysics();
  physx::PxCooking* getCooking();
  physx::PxMaterial* getMaterial() const;
  physx::PxCudaContextManager* getCudaContextManager();

  /**
   * @brief Check if GPU is enable
   * @return True if GPU is being used, otherwise false
   */
  bool useGPU() const;

  /**
   * @brief The number of worker threads per scene
   * @return number of worker threads per scene
   */
  int getWorkerThreadCount() const;

  physx::PxDefaultAllocator& getAllocator();
  const physx::PxDefaultErrorCallback& getErrorCallback();

private:
  physx::PxDefaultAllocator		default_allocator_;
  physx::PxDefaultErrorCallback	error_callback_;

  physx::PxFoundation*           foundation_ {nullptr};
  physx::PxPhysics*              physics_ {nullptr};
  physx::PxCooking*              cooking_ {nullptr};
  physx::PxPvd*                  pvd_ {nullptr};
  physx::PxCudaContextManager*   cuda_ {nullptr};
  physx::PxMaterial*				     material_{nullptr};
  std::thread::id                thread_id_;
  int                            worker_threads_ {2};
};
}
#endif // TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_H
