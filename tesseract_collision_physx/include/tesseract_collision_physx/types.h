/**
 * @file types.h
 * @brief Tesseract Physx types.
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
#ifndef TESSERACT_COLLISION_PHYSX_TYPES_H
#define TESSERACT_COLLISION_PHYSX_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <PxPhysicsAPI.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_collision
{

#define PX_RELEASE(x)	if(x)	{ (x)->release(); (x) = nullptr;}

using TesseractPhysxGeometryHolder = std::pair<physx::PxGeometryHolder, physx::PxTransform>;

enum class PhysxFilterGroup
{
  NONE       = (1 << 0),
  STATIC     = (1 << 1),
  KINEMATIC  = (1 << 2),
  DISABLED   = (1 << 3)
};

//enum CollisionFilterGroups
//{
//  DefaultFilter = 1,
//  StaticFilter = 2,
//  KinematicFilter = 4,
//  AllFilter = -1  // all bits sets: DefaultFilter | StaticFilter | KinematicFilter
//};

}

#endif // TESSERACT_COLLISION_PHYSX_TYPES_H
