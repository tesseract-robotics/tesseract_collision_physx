/**
 * @file physx_discrete_manager.cpp
 * @brief Tesseract ROS Physx Discrete Manager implementation.
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

#include <tesseract_collision_physx/physx_discrete_manager.h>
#include <tesseract_collision_physx/physx_collision_object_wrapper.h>
#include <tesseract_collision_physx/types.h>

static const physx::PxReal SIMULATION_TIME = physx::PxReal(0.0001); // TODO: How small can this be?

namespace tesseract_collision
{

static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract_common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

PhysxDiscreteManager::PhysxDiscreteManager() : physx_(std::make_shared<TesseractPhysx>())
{
}

DiscreteContactManager::Ptr PhysxDiscreteManager::clone() const
{
  auto manager= std::make_shared<PhysxDiscreteManager>();

//  for (const auto& cow : link2cow_)
//  {
//    PhysxCOW::Ptr new_cow = cow.second->clone();

//    assert(new_cow->getCollisionShape());
//    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

//    new_cow->setWorldTransform(cow.second->getWorldTransform());

//    new_cow->setContactDistance(contact_distance_);
//    manager->addCollisionObject(new_cow);
//  }

//  manager->setActiveCollisionObjects(active_);
//  manager->setContactDistanceThreshold(contact_distance_);
//  manager->setIsContactAllowedFn(fn_);

  return std::move(manager);
}

bool PhysxDiscreteManager::addCollisionObject(const std::string& name,
                                              const int& mask_id,
                                              const CollisionShapesConst& shapes,
                                              const tesseract_common::VectorIsometry3d& shape_poses,
                                              bool enabled)
{
  PhysxCOW::Ptr new_cow = createPhysxCollisionObject(name, mask_id, shapes, shape_poses, enabled, physx_);
  if (new_cow != nullptr)
  {
    link2cow_[new_cow->getName()] = new_cow;
    dirty_ = true;
    return true;
  }

  return false;
}

const CollisionShapesConst& PhysxDiscreteManager::getCollisionObjectGeometries(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometries() :
                                                     EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract_common::VectorIsometry3d&
PhysxDiscreteManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() :
                                                     EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool PhysxDiscreteManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool PhysxDiscreteManager::removeCollisionObject(const std::string& name)
{
  UNUSED(name);
  assert(false);

  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    for (auto co : it->second->getCollisionObjects())
      physx_->getScene()->removeActor(*co);

    link2cow_.erase(name);
    return true;
  }

  return false;
}

bool PhysxDiscreteManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    PhysxCOW::Ptr& cow = it->second;
    cow->enabled = true;

    if (!isLinkActive(active_, cow->getName()))
    {
      cow->filter_data.word0 = static_cast<physx::PxU32>(PhysxFilterGroup::STATIC);
      cow->filter_data.word1 = static_cast<physx::PxU32>(PhysxFilterGroup::KINEMATIC);
    }
    else
    {
      cow->filter_data.word0 = static_cast<physx::PxU32>(PhysxFilterGroup::KINEMATIC);
      cow->filter_data.word1 = static_cast<physx::PxU32>(PhysxFilterGroup::STATIC) | static_cast<physx::PxU32>(PhysxFilterGroup::KINEMATIC);
    }

    cow->updateFilterData();

    return true;
  }
  return false;
}

bool PhysxDiscreteManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    PhysxCOW::Ptr& cow = it->second;
    cow->enabled = false;
    cow->filter_data.word0 = static_cast<physx::PxU32>(PhysxFilterGroup::DISABLED);
    cow->filter_data.word1 = static_cast<physx::PxU32>(PhysxFilterGroup::NONE);
    cow->updateFilterData();

    return true;
  }
  return false;
}

void PhysxDiscreteManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    PhysxCOW::Ptr& cow = it->second;
    cow->setWorldTransform(pose);
    dirty_ = true;
  }
}

void PhysxDiscreteManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                            const tesseract_common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void PhysxDiscreteManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void PhysxDiscreteManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  for (auto& co : link2cow_)
  {
    PhysxCOW::Ptr& cow = co.second;

    // For descrete checks we can check static to kinematic and kinematic to  kinematic
    if (cow->enabled)
    {
      if (!isLinkActive(active_, cow->getName()))
      {
        cow->filter_data.word0 = static_cast<physx::PxU32>(PhysxFilterGroup::STATIC);
        cow->filter_data.word1 = static_cast<physx::PxU32>(PhysxFilterGroup::KINEMATIC);
      }
      else
      {
        cow->filter_data.word0 = static_cast<physx::PxU32>(PhysxFilterGroup::KINEMATIC);
        cow->filter_data.word1 = static_cast<physx::PxU32>(PhysxFilterGroup::STATIC) | static_cast<physx::PxU32>(PhysxFilterGroup::KINEMATIC);
      }
      cow->updateFilterData();
    }
  }
}

const std::vector<std::string>& PhysxDiscreteManager::getActiveCollisionObjects() const { return active_; }
void PhysxDiscreteManager::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = static_cast<physx::PxReal>(contact_distance);
  physx_->getContactTestData().contact_distance = contact_distance;

  for (auto& co : link2cow_)
    co.second->setContactDistance(contact_distance_);
}

double PhysxDiscreteManager::getContactDistanceThreshold() const { return static_cast<double>(contact_distance_); }
void PhysxDiscreteManager::setIsContactAllowedFn(IsContactAllowedFn fn) { fn_ = fn; }
IsContactAllowedFn PhysxDiscreteManager::getIsContactAllowedFn() const { return fn_; }
void PhysxDiscreteManager::contactTest(ContactResultMap& collisions, const ContactTestType& type)
{
  // If you add a new collision object or set its transform you must call simulation twice for some reason.
  if (dirty_)
  {
    dummy_.clear();
    ContactTestData& cd = physx_->getContactTestData();
    cd.fn = fn_;
    cd.active = &active_;
    cd.type = tesseract_collision::ContactTestType::CLOSEST; // TODO: tesseract_collision::ContactTestType::FIRST
    cd.res = &dummy_;
    cd.done = false;

    physx_->getScene()->simulate(SIMULATION_TIME);
    physx_->getScene()->fetchResults(true);
    dirty_ = false;
  }

  ContactTestData& cd = physx_->getContactTestData();
  cd.fn = fn_;
  cd.active = &active_;
  cd.type = type;
  cd.res = &collisions;
  cd.done = false;

  physx_->getScene()->simulate(SIMULATION_TIME);
  physx_->getScene()->fetchResults(true);
}

}  // namespace tesseract_collision
