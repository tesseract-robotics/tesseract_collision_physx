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

struct TesseractPhysxDesc
{
  /** @brief The number of worker threads created for each scene. */
  int worker_threads {2};

  /** @brief Enable GPU functionality */
  bool enable_gpu {false};

  /** @brief Data structure used to initialize cuda functionality */
  physx::PxCudaContextManagerDesc cuda_desc;

  /** @brief GPU Memory configuration for Dynamic Actors */
  physx::PxgDynamicsMemoryConfig cuda_dynamics_config;

  /** @brief Enable Physx Debug functionality */
  bool debug {false};
  std::string pvd_host {"127.0.0.1"};
  int pvd_port {5425};

  /** @brief PxCookingParams::midphaseDesc can be used to select the desired mid-phase structure. It is a good idea to try the different options and see which one works best for you. Generally speaking the new PxMeshMidPhase::eBVH34 introduced in PhysX 3.4 has better performance for scene queries against large triangle meshes. */
  physx::PxMidphaseDesc mid_phase_desc;

  /** @brief PhysX also supports three different broad-phase implementations, selected with PxSceneDesc::broadPhaseType. The different implementations have various performance characteristics, and it is a good idea to experiment with them and find which one works best for you. */
  physx::PxBroadPhaseType::Enum broad_phase_algorithm { physx::PxBroadPhaseType::eABP };

  /** @brief If the PxScene::fetchResults call takes a significant amount of time in scenes containing a lot of dynamic objects, try to increase the PxSceneDesc::dynamicTreeRebuildRateHint parameter. */
  int dynamic_tree_rebuild_rate_hint { 100 };
};

/**
 * @brief Tesseract Physx only allow one copy per thread.
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
  TesseractPhysx(TesseractPhysxDesc desc = TesseractPhysxDesc());

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
   * @brief Get the description information used during setup
   * @return TesseractPhysxDesc
   */
  const TesseractPhysxDesc& getDescription();

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
  TesseractPhysxDesc             desc_;
};
}
#endif // TESSERACT_COLLISION_PHYSX_TESSERACT_PHYSX_H
