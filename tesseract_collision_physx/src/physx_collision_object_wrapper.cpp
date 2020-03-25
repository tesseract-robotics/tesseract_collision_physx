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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision_physx/physx_collision_object_wrapper.h>
#include <tesseract_collision_physx/utils.h>

namespace tesseract_collision
{

PhysxCollisionObjectWrapper::PhysxCollisionObjectWrapper(std::string name,
                                                         const int& type_id,
                                                         CollisionShapesConst shapes,
                                                         tesseract_common::VectorIsometry3d shape_poses,
                                                         TesseractPhysx::Ptr physx)
  : name_(std::move(name))
  , type_id_(type_id)
  , shapes_(std::move(shapes))
  , shape_poses_(std::move(shape_poses))
  , physx_(std::move(physx))
{
  assert(!shapes_.empty());
  assert(!shape_poses_.empty());
  assert(!name_.empty());
  assert(shapes_.size() == shape_poses_.size());
  assert(physx_ != nullptr);

  collision_geometries_.reserve(shapes_.size());
  collision_objects_.reserve(shapes_.size());

  // TODO: Should all objects be added to a single kinematic object?
  // TODO: Should aggregate be used?
  // TODO: Should these be different for GPU and CPU? Documentation suggest that they should be.
  for (std::size_t i = 0; i < shapes_.size(); ++i)
  {
    physx::PxGeometryHolder subshape = createShapePrimitive(*physx_, shapes_[i]);
    if (subshape.getType() != physx::PxGeometryType::Enum::eINVALID)
    {
      collision_geometries_.push_back(subshape);

      physx::PxTransform shape_tf = convertEigenToPhysx(shape_poses_[i]);
      physx::PxTransform world_tf = convertEigenToPhysx(Eigen::Isometry3d::Identity());
      physx::PxRigidDynamic* dyn = physx::PxCreateKinematic(*physx_->getPhysics(),
                                                             world_tf,
                                                             subshape.any(),
                                                             *physx_->getMaterial(),
                                                             physx::PxReal(0.1),
                                                             shape_tf);

      dyn->userData = this;
      dyn->setName((name + "_" + std::to_string(i)).c_str());
      collision_objects_.push_back(dyn);

//      physx_->setupFiltering(dyn, static_cast<physx::PxU32>(PhysxFilterGroup::eKINEMATIC), static_cast<physx::PxU32>(PhysxFilterGroup::eSTATIC));
      physx_->getScene()->addActor(*dyn);
    }
  }
}

const std::string& PhysxCollisionObjectWrapper::getName() const
{
  return name_;
}

const int& PhysxCollisionObjectWrapper::getTypeID() const
{
  return type_id_;
}

bool PhysxCollisionObjectWrapper::sameObject(const PhysxCollisionObjectWrapper& other) const
{
  return name_ == other.name_ && type_id_ == other.type_id_ && shapes_.size() == other.shapes_.size() &&
         shape_poses_.size() == other.shape_poses_.size() &&
         std::equal(shapes_.begin(), shapes_.end(), other.shapes_.begin()) &&
         std::equal(shape_poses_.begin(),
                    shape_poses_.end(),
                    other.shape_poses_.begin(),
                    [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
}

const CollisionShapesConst& PhysxCollisionObjectWrapper::getCollisionGeometries() const
{
  return shapes_;
}

const tesseract_common::VectorIsometry3d& PhysxCollisionObjectWrapper::getCollisionGeometriesTransforms() const
{
  return shape_poses_;
}

void PhysxCollisionObjectWrapper::setWorldTransform(const Eigen::Isometry3d& pose)
{
  world_pose_ = pose;
  for (auto co : collision_objects_)
    co->setKinematicTarget(convertEigenToPhysx(pose));
}

void PhysxCollisionObjectWrapper::setContactDistance(physx::PxReal dist)
{
  for (unsigned i = 0; i < collision_objects_.size(); ++i)
  {
    physx::PxRigidDynamic* co = collision_objects_[i];
    physx::PxU32 num_shapes = co->getNbShapes();

    physx::PxShape* shapes[num_shapes];

    co->getShapes(shapes, num_shapes);
    for (physx::PxU32 i = 0; i < num_shapes; ++i)
      shapes[i]->setContactOffset(dist);

  }
}

const Eigen::Isometry3d& PhysxCollisionObjectWrapper::getWorldTransform() const
{
  return world_pose_;
}

const std::vector<physx::PxRigidDynamic*>& PhysxCollisionObjectWrapper::getCollisionObjects() const
{
  return collision_objects_;
}

std::vector<physx::PxRigidDynamic*>& PhysxCollisionObjectWrapper::getCollisionObjects()
{
  return collision_objects_;
}

std::shared_ptr<PhysxCollisionObjectWrapper> PhysxCollisionObjectWrapper::clone(TesseractPhysx::Ptr tesseract_physx) const
{
  // TODO: Need to serialize the Rigid Body, store in this class and use during cloning which will be faster.
  auto clone_cow = std::make_shared<PhysxCollisionObjectWrapper>(name_, type_id_, shapes_, shape_poses_, tesseract_physx);

  clone_cow->filter_data = filter_data;
  clone_cow->enabled = enabled;
  clone_cow->updateFilterData();

  return clone_cow;
}

int PhysxCollisionObjectWrapper::getShapeIndex(const physx::PxRigidDynamic* co) const
{
  auto it = std::find_if(collision_objects_.begin(), collision_objects_.end(), [&co](const physx::PxRigidDynamic* c) {
    return c == co;
  });

  if (it != collision_objects_.end())
    return static_cast<int>(std::distance(collision_objects_.begin(), it));

  return -1;
}

void PhysxCollisionObjectWrapper::updateFilterData()
{
  for (auto co : collision_objects_)
    physx_->setupFiltering(co, filter_data);
}



PhysxCollisionObjectWrapper::PhysxCollisionObjectWrapper(std::string name,
                                                         const int& type_id,
                                                         CollisionShapesConst shapes,
                                                         tesseract_common::VectorIsometry3d shape_poses,
                                                         std::vector<physx::PxGeometryHolder> collision_geometries,
                                                         const std::vector<physx::PxRigidDynamic*>& collision_objects)
  : name_(std::move(name))
  , type_id_(type_id)
  , shapes_(std::move(shapes))
  , shape_poses_(std::move(shape_poses))
  , collision_geometries_(std::move(collision_geometries))
{
  UNUSED(collision_objects);
  assert(false);
//  collision_objects_.reserve(collision_objects.size());
//  for (const auto& co : collision_objects)
//  {
//    CollisionObjectPtr collObj(new fcl::CollisionObjectd(*co));
//    collObj->setUserData(this);
//    collision_objects_.push_back(collObj);
//  }
}

PhysxCOW::Ptr createPhysxCollisionObject(const std::string& name,
                                         const int& type_id,
                                         const CollisionShapesConst& shapes,
                                         const tesseract_common::VectorIsometry3d& shape_poses,
                                         bool enabled,
                                         TesseractPhysx::Ptr physx)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    CONSOLE_BRIDGE_logDebug("ignoring link %s", name.c_str());
    return nullptr;
  }

  PhysxCOW::Ptr new_cow = std::make_shared<PhysxCOW>(name, type_id, shapes, shape_poses, physx);

  new_cow->enabled = enabled;
  CONSOLE_BRIDGE_logDebug("Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}
}
