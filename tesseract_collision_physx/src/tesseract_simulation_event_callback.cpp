/**
 * @file tesseract_simulation_event_callback.h
 * @brief Tesseract Physx simulation event callback.
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

#include <tesseract_collision_physx/tesseract_simulation_event_callback.h>
#include <tesseract_collision_physx/physx_collision_object_wrapper.h>
#include <tesseract_collision_physx/tesseract_physx_scene.h>
#include <tesseract_collision_physx/utils.h>

namespace tesseract_collision
{

TesseractSimulationEventCallback::TesseractSimulationEventCallback(TesseractPhysxScene *physx_scene)
  : physx_scene_(physx_scene)
{
}

void TesseractSimulationEventCallback::onConstraintBreak(physx::PxConstraintInfo* /*constraints*/,
                                                         physx::PxU32 /*count*/)
{
  assert(false);
}

void TesseractSimulationEventCallback::onWake(physx::PxActor** /*actors*/, physx::PxU32 /*count*/)
{
  assert(false);
}

void TesseractSimulationEventCallback::onSleep(physx::PxActor** /*actors*/, physx::PxU32 /*count*/)
{
  assert(false);
}

void TesseractSimulationEventCallback::onTrigger(physx::PxTriggerPair* /*pairs*/, physx::PxU32 /*count*/)
{
  assert(false);
}

void TesseractSimulationEventCallback::onAdvance(const physx::PxRigidBody*const* /*bodyBuffer*/,
                                                 const physx::PxTransform* /*poseBuffer*/,
                                                 physx::PxU32 /*count*/)
{
  assert(false);
}

// https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/AdvancedCollisionDetection.html#extracting-contact-information
void TesseractSimulationEventCallback::onContact(const physx::PxContactPairHeader& pairHeader,
                                                 const physx::PxContactPair* pairs,
                                                 physx::PxU32 nbPairs)
{
  UNUSED(pairHeader);
  ContactTestData& cdata = physx_scene_->getContactTestData();
  if (cdata.done)
    return;

  const physx::PxU32 bufferSize = 64;
  physx::PxContactPairPoint contacts[bufferSize];
  for(physx::PxU32 i=0; i < nbPairs; i++)
  {
    const physx::PxContactPair& cp = pairs[i];
    const auto* cd0 = static_cast<const PhysxCollisionObjectWrapper*>(cp.shapes[0]->getActor()->userData);
    const auto* cd1 = static_cast<const PhysxCollisionObjectWrapper*>(cp.shapes[1]->getActor()->userData);

    ObjectPairKey pc = getObjectPairKey(cd0->getName(), cd1->getName());

    const auto& it = physx_scene_->getContactTestData().res->find(pc);
    bool found = (it != physx_scene_->getContactTestData().res->end());

    //    size_t l = 0;
    //    if (found)
    //    {
    //      l = it->second.size();
    //      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
    //          return 0;

    //    }

    Eigen::Isometry3d tf0 = cd0->getWorldTransform();
    Eigen::Isometry3d tf1 = cd1->getWorldTransform();
    Eigen::Isometry3d tf0_inv = tf0.inverse();
    Eigen::Isometry3d tf1_inv = tf1.inverse();

    // These contacts are what make up the contact manifold. We will average the contact information for the manifold
    ContactResult contact;
    contact.link_names[0] = cd0->getName();
    contact.link_names[1] = cd1->getName();
//    contact.shape_id[0] = colObj0Wrap->getCollisionShape()->getUserIndex();
//    contact.shape_id[1] = colObj1Wrap->getCollisionShape()->getUserIndex();
//    contact.subshape_id[0] = static_cast<int>(contacts[j].internalFaceIndex0);
//    contact.subshape_id[1] = static_cast<int>(contacts[j].internalFaceIndex1);
    contact.transform[0] = tf0;
    contact.transform[1] = tf1;
    contact.type_id[0] = cd0->getTypeID();
    contact.type_id[1] = cd1->getTypeID();
    physx::PxU32 nbContacts = pairs[i].extractContacts(contacts, bufferSize);
    for(physx::PxU32 j = 0; j < nbContacts; j++)
    {
      if (j == 0)
      {
        contact.distance = static_cast<double>(contacts[j].separation);
        contact.normal = convertPhysxToEigen(-1 * contacts[j].normal);
        contact.nearest_points[1] = convertPhysxToEigen(contacts[j].position);
      }
      else
      {
        if (static_cast<double>(contacts[j].separation) < contact.distance)
          contact.distance = static_cast<double>(contacts[j].separation);

        contact.normal += convertPhysxToEigen(-1 * contacts[j].normal);
        contact.nearest_points[1] += convertPhysxToEigen(contacts[j].position);
      }
    }
    contact.normal = contact.normal / nbContacts;
    contact.nearest_points[1] = contact.nearest_points[1] / nbContacts; ;
    contact.nearest_points[0] = contact.nearest_points[1] - (contact.distance * contact.normal);
    contact.nearest_points_local[0] = tf0_inv * contact.nearest_points[0];
    contact.nearest_points_local[1] = tf1_inv * contact.nearest_points[1];
    processResult(physx_scene_->getContactTestData(), contact, pc, found);

    if (cdata.done)
      return;
  }
}
}
