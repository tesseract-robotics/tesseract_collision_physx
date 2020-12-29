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
#include <tesseract_collision_physx/tesseract_physx_scene.h>
#include <tesseract_collision_physx/physx_collision_object_wrapper.h>

namespace tesseract_collision
{

/** @brief An implementaiton of a PhysX discrete contact manager with both CPU and GPU support */
class PhysxDiscreteManager : public DiscreteContactManager
{
public:
  using Ptr = std::shared_ptr<PhysxDiscreteManager>;
  using ConstPtr = std::shared_ptr<const PhysxDiscreteManager>;

  PhysxDiscreteManager(TesseractPhysxDesc desc = TesseractPhysxDesc());
  PhysxDiscreteManager(TesseractPhysx::Ptr tesseract_physx);

  ~PhysxDiscreteManager() override = default;
  PhysxDiscreteManager(const PhysxDiscreteManager&) = delete;
  PhysxDiscreteManager& operator=(const PhysxDiscreteManager&) = delete;
  PhysxDiscreteManager(PhysxDiscreteManager&&) = delete;
  PhysxDiscreteManager& operator=(PhysxDiscreteManager&&) = delete;

  static std::string name() { return "PhysxDiscreteManager"; }
  static DiscreteContactManager::Ptr create() { return std::make_shared<PhysxDiscreteManager>(); }

  /**
   * @brief Clone the PhysxDiscreteManager.
   *
   * Currently the clone creates a new PhysX Scene, but share the same TesseractPhysx object. It is unclear if this is
   * an issue when scenes are being operated on different threads.
   *
   * @return Clone of PhysxDiscreteManager
   */
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

  const std::vector<std::string>& getCollisionObjects() const override;

  void setActiveCollisionObjects(const std::vector<std::string>& names) override;

  const std::vector<std::string>& getActiveCollisionObjects() const override;

  void setCollisionMarginData(CollisionMarginData collision_margin_data) override;

  void setDefaultCollisionMarginData(double default_collision_margin) override;

  void setPairCollisionMarginData(const std::string& name1,
                                  const std::string& name2,
                                  double collision_margin) override;

  const CollisionMarginData& getCollisionMarginData() const override;

  void setIsContactAllowedFn(IsContactAllowedFn fn) override;

  IsContactAllowedFn getIsContactAllowedFn() const override;

  void contactTest(ContactResultMap& collisions, const ContactRequest& request) override;

  /**
   * @brief Add a collision object to the manager
   * @param cow The tesseract physx collision object
   */
  void addCollisionObject(const PhysxCOW::Ptr& cow);

private:
  std::vector<std::string> active_;            /**< @brief A list of the active collision objects */
  std::vector<std::string> collision_objects_; /**< @brief A list of the collision objects */
  CollisionMarginData collision_margin_data_;  /**< @brief The contact distance threshold */
  IsContactAllowedFn fn_;                      /**< @brief The is allowed collision function */
  TesseractPhysxScene::Ptr physx_scene_;       /**< @brief The tesseract physx scene container */
  Link2PhysxCOW link2cow_;                     /**< @brief A map of all (static and active) physx collision objects managed */

  /** @brief This function will update internal data when margin data has changed */
  void onCollisionMarginDataChanged();

};

}  // namespace tesseract_collision

#endif // TESSERACT_COLLISION_PHYSX_DISCRETE_SIMPLE_MANAGER_H
