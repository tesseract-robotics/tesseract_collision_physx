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
#ifndef TESSERACT_COLLISION_PHYSX_COLLISION_OBJECT_WRAPPER_H
#define TESSERACT_COLLISION_PHYSX_COLLISION_OBJECT_WRAPPER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <map>
#include <string>
#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision_physx/tesseract_physx.h>
#include <tesseract_collision_physx/types.h>

namespace tesseract_collision
{

class PhysxCollisionObjectWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<PhysxCollisionObjectWrapper>;
  using ConstPtr = std::shared_ptr<const PhysxCollisionObjectWrapper>;

  PhysxCollisionObjectWrapper(std::string name,
                              const int& type_id,
                              CollisionShapesConst shapes,
                              tesseract_common::VectorIsometry3d shape_poses,
                              TesseractPhysx::Ptr physx);

  ~PhysxCollisionObjectWrapper();

  physx::PxFilterData filter_data;
  bool enabled;

  const std::string& getName() const;
  const int& getTypeID() const;

  /** \brief Check if two objects point to the same source object */
  bool sameObject(const PhysxCollisionObjectWrapper& other) const;

  const CollisionShapesConst& getCollisionGeometries() const;

  const tesseract_common::VectorIsometry3d& getCollisionGeometriesTransforms() const;

  void setWorldTransform(const Eigen::Isometry3d& pose);
  void setContactDistance(physx::PxReal dist);
  void updateFilterData();

  const Eigen::Isometry3d& getWorldTransform() const;
  const std::vector<physx::PxRigidDynamic*>& getCollisionObjects() const;
  std::vector<physx::PxRigidDynamic*>& getCollisionObjects();
  std::shared_ptr<PhysxCollisionObjectWrapper> clone(TesseractPhysx::Ptr tesseract_physx) const;



  /**
   * @brief Given Physx collision shape get the index to the links collision shape
   * @param co Physx collision shape
   * @return links collision shape index
   */
  int getShapeIndex(const physx::PxRigidDynamic* co) const;

protected:
  PhysxCollisionObjectWrapper(std::string name,
                              const int& type_id,
                              CollisionShapesConst shapes,
                              tesseract_common::VectorIsometry3d shape_poses,
                              std::vector<std::vector<TesseractPhysxGeometryHolder>> collision_geometries,
                              const std::vector<physx::PxRigidDynamic*>& collision_objects);

  std::string name_;             // name of the collision object
  int type_id_;                  // user defined type id
  Eigen::Isometry3d world_pose_; /**< @brief Collision Object World Transformation */
  CollisionShapesConst shapes_;
  tesseract_common::VectorIsometry3d shape_poses_;
  std::vector<std::vector<TesseractPhysxGeometryHolder>> collision_geometries_;
  std::vector<physx::PxRigidDynamic*> collision_objects_;
  TesseractPhysx::Ptr physx_;
};

using PhysxCOW = PhysxCollisionObjectWrapper;
using Link2PhysxCOW = std::map<std::string, PhysxCOW::Ptr>;
using Link2ConstPhysxCOW = std::map<std::string, PhysxCOW::ConstPtr>;

PhysxCOW::Ptr createPhysxCollisionObject(const std::string& name,
                                         const int& type_id,
                                         const CollisionShapesConst& shapes,
                                         const tesseract_common::VectorIsometry3d& shape_poses,
                                         bool enabled,
                                         TesseractPhysx::Ptr physx);
}

#endif // TESSERACT_COLLISION_PHYSX_COLLISION_OBJECT_WRAPPER_H
