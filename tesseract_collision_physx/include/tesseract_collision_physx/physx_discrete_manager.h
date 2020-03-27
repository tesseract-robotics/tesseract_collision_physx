/**
 * @file physx_discrete_manager.h
 * @brief Tesseract Physx discrete simple collision manager.
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
#ifndef TESSERACT_COLLISION_PHYSX_DISCRETE_MANAGER_H
#define TESSERACT_COLLISION_PHYSX_DISCRETE_MANAGER_H

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision_physx/tesseract_physx.h>
#include <tesseract_collision_physx/physx_collision_object_wrapper.h>

namespace tesseract_collision
{

/** @brief A simple implementaiton of a bullet manager which does not use BHV */
class PhysxDiscreteManager : public DiscreteContactManager
{
public:
  using Ptr = std::shared_ptr<PhysxDiscreteManager>;
  using ConstPtr = std::shared_ptr<const PhysxDiscreteManager>;

  PhysxDiscreteManager();
  ~PhysxDiscreteManager() override = default;
  PhysxDiscreteManager(const PhysxDiscreteManager&) = delete;
  PhysxDiscreteManager& operator=(const PhysxDiscreteManager&) = delete;
  PhysxDiscreteManager(PhysxDiscreteManager&&) = delete;
  PhysxDiscreteManager& operator=(PhysxDiscreteManager&&) = delete;

  static std::string name() { return "PhysxDiscreteManager"; }
  static PhysxDiscreteManager::Ptr create() { return std::make_shared<PhysxDiscreteManager>(); }

  DiscreteContactManager::Ptr clone() const override;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const CollisionShapesConst& shapes,
                          const tesseract_common::VectorIsometry3d& shape_poses,
                          bool enabled = true) override;

  const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const override;

  const tesseract_common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const override;

  bool hasCollisionObject(const std::string& name) const override;

  bool removeCollisionObject(const std::string& name) override;

  bool enableCollisionObject(const std::string& name) override;

  bool disableCollisionObject(const std::string& name) override;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) override;

  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const tesseract_common::VectorIsometry3d& poses) override;

  void setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms) override;

  void setActiveCollisionObjects(const std::vector<std::string>& names) override;

  const std::vector<std::string>& getActiveCollisionObjects() const override;

  void setContactDistanceThreshold(double contact_distance) override;

  double getContactDistanceThreshold() const override;

  void setIsContactAllowedFn(IsContactAllowedFn fn) override;

  IsContactAllowedFn getIsContactAllowedFn() const override;

  void contactTest(ContactResultMap& collisions, const ContactTestType& type) override;

private:
  std::vector<std::string> active_; /**< @brief A list of the active collision objects */
  physx::PxReal contact_distance_;  /**< @brief The contact distance threshold */
  IsContactAllowedFn fn_;           /**< @brief The is allowed collision function */
  TesseractPhysx::Ptr physx_;       /**< @brief The tesseract physx container */
  Link2PhysxCOW link2cow_;          /**< @brief A map of all (static and active) physx collision objects managed */
  bool dirty_ {false};              /**< @brief Indicates that simulation must be ran twice */
  ContactResultMap dummy_;          /**< @brief A dummy contact results used when simulation need to be ran twice */
};

}  // namespace tesseract_collision

#endif // TESSERACT_COLLISION_PHYSX_DISCRETE_SIMPLE_MANAGER_H
