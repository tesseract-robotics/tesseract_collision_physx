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
#include <console_bridge/console.h>
#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision_physx/physx_collision_object_wrapper.h>
#include <tesseract_collision_physx/utils.h>
#include <tesseract_collision_physx/types.h>

namespace tesseract_collision
{

PhysxCollisionObjectWrapper::PhysxCollisionObjectWrapper(std::string name,
                                                         const int& type_id,
                                                         CollisionShapesConst shapes,
                                                         tesseract_common::VectorIsometry3d shape_poses,
                                                         TesseractPhysxScene::Ptr physx_scene)
  : name_(std::move(name))
  , type_id_(type_id)
  , shapes_(std::move(shapes))
  , shape_poses_(std::move(shape_poses))
  , physx_scene_(std::move(physx_scene))
{
  assert(!shapes_.empty());
  assert(!shape_poses_.empty());
  assert(!name_.empty());
  assert(shapes_.size() == shape_poses_.size());
  assert(physx_scene_ != nullptr);

  collision_geometries_.reserve(shapes_.size());
  collision_objects_.reserve(shapes_.size());

  for (std::size_t i = 0; i < shapes_.size(); ++i)
  {
    bool has_valid_shapes = true;
    std::vector<TesseractPhysxGeometryHolder> subshapes = createShapePrimitive(*physx_scene_, shapes_[i]);
    physx::PxTransform shape_tf = convertEigenToPhysx(shape_poses_[i]);

    for (auto& shape : subshapes)
    {
      physx::PxRigidDynamic* actor = physx_scene_->getTesseractPhysx()->getPhysics()->createRigidDynamic(physx::PxTransform(physx::PxIdentity));
      actor->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);

      if (shape.first.getType() != physx::PxGeometryType::Enum::eINVALID)
      {
        physx::PxShapeFlags shapeFlags;
        if (physx_scene_->getTesseractPhysx()->getDescription().debug)
          shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE;
        else
          shapeFlags = physx::PxShapeFlag::eSIMULATION_SHAPE;

        physx::PxShape* s = physx_scene_->getTesseractPhysx()->getPhysics()->createShape(shape.first.any(), *physx_scene_->getTesseractPhysx()->getMaterial(), true);
        if(!s)
        {
          has_valid_shapes = false;
          s->release();
          PX_RELEASE(actor);
          break;
        }

        s->setLocalPose(shape_tf * shape.second);

        actor->attachShape(*s);

        s->release();
      }
      else
      {
        has_valid_shapes = false;
        PX_RELEASE(actor);
        break;
      }

      actor->setMass(1.f);
      actor->setMassSpaceInertiaTensor(physx::PxVec3(1.f,1.f,1.f));
      actor->userData = this;
      actor->setName(name_.c_str()); // Must use a member variable because physx only stores a pointer to a string
      collision_objects_.push_back(actor);
      collision_geometries_.push_back(shape);
    }

    if (!has_valid_shapes)
    {
      CONSOLE_BRIDGE_logError("Link was unable to add shape to PhysX for link: %s", name.c_str());
      return;
    }
  }

  /** @note For GPU it is best to not use aggregate */
  if (physx_scene_->getTesseractPhysx()->getDescription().enable_gpu)
  {
    for (auto* co : collision_objects_)
      physx_scene_->getScene()->addActor(*co);
  }
  else /** @note For CPU it is best to use aggregate */
  {
    collision_aggregate_ = physx_scene_->getTesseractPhysx()->getPhysics()->createAggregate(static_cast<physx::PxU32>(collision_objects_.size()), false);
    for (auto* co : collision_objects_)
      collision_aggregate_->addActor(*co);

    physx_scene_->getScene()->addAggregate(*collision_aggregate_);
  }
}

PhysxCollisionObjectWrapper::~PhysxCollisionObjectWrapper()
{
  for(std::size_t c = 0; c < collision_objects_.size(); ++c)
  {
    PX_RELEASE(collision_objects_[c]);
  }
  collision_objects_.clear();

  PX_RELEASE(collision_aggregate_);
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
  for (auto& co : collision_objects_)
  {
    auto t = convertEigenToPhysx(world_pose_);
    // Using setGlobalPose dose not wake up the actor so no contacts are reports. Adding setKinematicTarget with the
    // same transform wakes up the actor so contacts are reported. If you were to only use setKinematicTarget it would
    // require two calls to simulation to get the contacts at the final stage. Having both solve all the isues and
    // simplifies the code.
    co->setGlobalPose(t); 
    co->setKinematicTarget(t);
  }
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

std::shared_ptr<PhysxCollisionObjectWrapper> PhysxCollisionObjectWrapper::clone(TesseractPhysxScene::Ptr physx_scene) const
{
  // TODO: Need to serialize the Rigid Body, store in this class and use during cloning which will be faster.
  auto clone_cow = std::make_shared<PhysxCollisionObjectWrapper>(name_, type_id_, shapes_, shape_poses_, physx_scene);

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
    physx_scene_->setupFiltering(co, filter_data);
}



PhysxCollisionObjectWrapper::PhysxCollisionObjectWrapper(std::string name,
                                                         const int& type_id,
                                                         CollisionShapesConst shapes,
                                                         tesseract_common::VectorIsometry3d shape_poses,
                                                         std::vector<TesseractPhysxGeometryHolder> collision_geometries,
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
                                         const TesseractPhysxScene::Ptr& physx_scene)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    CONSOLE_BRIDGE_logDebug("ignoring link %s", name.c_str());
    return nullptr;
  }

  PhysxCOW::Ptr new_cow = std::make_shared<PhysxCOW>(name, type_id, shapes, shape_poses, physx_scene);

  new_cow->enabled = enabled;
  CONSOLE_BRIDGE_logDebug("Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}
}
