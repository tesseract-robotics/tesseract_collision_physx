/**
 * @file physx_utils.h
 * @brief Tesseract Physx utils.
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
#ifndef TESSERACT_COLLISION_PHYSX_UTILS_H
#define TESSERACT_COLLISION_PHYSX_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>

#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_collision_physx/tesseract_physx_scene.h>
#include <tesseract_collision_physx/types.h>

namespace tesseract_collision
{

physx::PxTransform convertEigenToPhysx(const Eigen::Isometry3d& tf);
Eigen::Isometry3d convertPhysxToEigen(const physx::PxTransform& tf);
Eigen::Vector3d convertPhysxToEigen(const physx::PxVec3& p);
Eigen::Quaterniond convertPhysxToEigen(const physx::PxQuat& q);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::Plane::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::Box::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::Sphere::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::Cylinder::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::Cone::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::Capsule::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::Mesh::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::ConvexMesh::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::Octree::ConstPtr& geom);

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const CollisionShapeConstPtr& geom);

///**
// * @brief Update collision objects filters
// * @param active The active collision objects
// * @param cow The collision object to update
// */
//inline void updateCollisionObjectFilters(const std::vector<std::string>& active, COW& cow)
//{
//  // For descrete checks we can check static to kinematic and kinematic to
//  // kinematic
//  cow.m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
//  if (!isLinkActive(active, cow.getName()))
//  {
//    cow.m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
//  }

//  if (cow.m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
//  {
//    cow.m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
//  }
//  else
//  {
//    cow.m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
//  }
//}

}  // namespace tesseract_collision
#endif // TESSERACT_COLLISION_PHYSX_UTILS_H
