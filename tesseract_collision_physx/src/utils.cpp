/**
 * @file physx_utils.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/thread/mutex.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision_physx/tesseract_physx.h>
#include <tesseract_collision_physx/utils.h>

static physx::PxDefaultErrorCallback gDefaultErrorCallback;
static physx::PxDefaultAllocator gDefaultAllocatorCallback;

namespace tesseract_collision
{

physx::PxTransform convertEigenToPhysx(const Eigen::Isometry3d& tf)
{
  return physx::PxTransform(physx::PxMat44(tf.cast<float>().data()));
}

Eigen::Vector3d convertPhysxToEigen(const physx::PxVec3& p)
{
  return Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
}

Eigen::Quaterniond convertPhysxToEigen(const physx::PxQuat& q)
{
  return Eigen::Quaterniond(static_cast<double>(q.w),
                            static_cast<double>(q.x),
                            static_cast<double>(q.y),
                            static_cast<double>(q.z));
}

Eigen::Isometry3d convertPhysxToEigen(const physx::PxTransform& tf)
{
  Eigen::Isometry3d m(convertPhysxToEigen(tf.q));
  m.translation() = convertPhysxToEigen(tf.p);
  return m;
}

//physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& tesseract_physx, const tesseract_geometry::Plane::ConstPtr& geom)
//{
//  physx::PxPlane plane(float(geom->getA()), float(geom->getB()), float(geom->getC()), float(geom->getD()));
//  physx::PxPlaneGeometry pg;

//  physx::PxGeometryHolder gh(physx::PxPlaneGeometry(plane));

//  return gh;
//}

physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& /*tesseract_physx*/, const tesseract_geometry::Box::ConstPtr& geom)
{
  return physx::PxGeometryHolder(physx::PxBoxGeometry(static_cast<physx::PxReal>(geom->getX() / 2), static_cast<physx::PxReal>(geom->getY() / 2), static_cast<physx::PxReal>(geom->getZ() / 2)));
}

physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& /*tesseract_physx*/, const tesseract_geometry::Sphere::ConstPtr& geom)
{
  return physx::PxGeometryHolder(physx::PxSphereGeometry(static_cast<physx::PxReal>(geom->getRadius())));
}

physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& /*tesseract_physx*/, const tesseract_geometry::Cylinder::ConstPtr& /*geom*/)
{
//  return CollisionGeometryPtr(new fcl::Cylinderd(geom->getRadius(), geom->getLength()));
  return physx::PxGeometryHolder();
}

physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& /*tesseract_physx*/, const tesseract_geometry::Cone::ConstPtr& /*geom*/)
{
//  return CollisionGeometryPtr(new fcl::Coned(geom->getRadius(), geom->getLength()));
  return physx::PxGeometryHolder();
}

physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& /*tesseract_physx*/, const tesseract_geometry::Capsule::ConstPtr& geom)
{
  return physx::PxGeometryHolder(physx::PxCapsuleGeometry(static_cast<physx::PxReal>(geom->getRadius()), static_cast<physx::PxReal>(geom->getLength() / 2)));
}

physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& tesseract_physx, const tesseract_geometry::Mesh::ConstPtr& geom)
{
  int v_count = geom->getVerticeCount();
  int t_count = geom->getTriangleCount();
  const tesseract_common::VectorVector3d& v = *(geom->getVertices());
  const Eigen::VectorXi& t = *(geom->getTriangles());

  physx::PxVec3 vertices[v_count];
  physx::PxU32 triangles[t.size()];
  for (std::size_t i = 0; i < static_cast<std::size_t>(v_count); ++i)
  {
    vertices[i].x = static_cast<physx::PxReal>(v[i].x());
    vertices[i].y = static_cast<physx::PxReal>(v[i].y());
    vertices[i].z = static_cast<physx::PxReal>(v[i].z());
  }

  for (long i = 0; i < t.size(); ++i)
  {
    triangles[i] = static_cast<physx::PxU32>(t[i]);
  }

  physx::PxTriangleMeshDesc mesh_desc;
  mesh_desc.points.count  = static_cast<physx::PxU32>(v_count);
  mesh_desc.points.stride = sizeof(physx::PxVec3);
  mesh_desc.points.data   = vertices;

  mesh_desc.triangles.count = static_cast<physx::PxU32>(t_count);
  mesh_desc.triangles.stride = 3 * sizeof(physx::PxU32);
  mesh_desc.triangles.data = triangles;

  physx::PxDefaultMemoryOutputStream write_buffer;
  physx::PxTriangleMeshCookingResult::Enum write_result;
  bool status = tesseract_physx.getCooking()->cookTriangleMesh(mesh_desc, write_buffer, &write_result);
  if(!status)
    return physx::PxGeometryHolder();

  physx::PxDefaultMemoryInputData read_buffer(write_buffer.getData(), write_buffer.getSize());
  physx::PxTriangleMesh* triangle_mesh = tesseract_physx.getPhysics()->createTriangleMesh(read_buffer);
  return physx::PxGeometryHolder(physx::PxTriangleMeshGeometry(triangle_mesh)); // TODO: Who is responsible for releasing triangle_mesh;
}

physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& tesseract_physx, const tesseract_geometry::ConvexMesh::ConstPtr& geom)
{
  int v_count = geom->getVerticeCount();
  const tesseract_common::VectorVector3d& v = *(geom->getVertices());

  physx::PxVec3 vertices[v_count];
  for (std::size_t i = 0; i < static_cast<std::size_t>(v_count); ++i)
  {
    vertices[i].x = static_cast<physx::PxReal>(v[i].x());
    vertices[i].y = static_cast<physx::PxReal>(v[i].y());
    vertices[i].z = static_cast<physx::PxReal>(v[i].z());
  }

  physx::PxConvexMeshDesc convex_desc;
  convex_desc.points.count = static_cast<physx::PxU32>(v_count);
  convex_desc.points.stride = sizeof(physx::PxVec3);
  convex_desc.points.data = vertices;
  convex_desc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;

  physx::PxDefaultMemoryOutputStream buf;
  physx::PxConvexMeshCookingResult::Enum result;
  if(!tesseract_physx.getCooking()->cookConvexMesh(convex_desc, buf, &result))
      return physx::PxGeometryHolder();

  physx::PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  physx::PxConvexMesh* convex_mesh = tesseract_physx.getPhysics()->createConvexMesh(input);
  if (convex_mesh == nullptr)
    return physx::PxGeometryHolder();

  return physx::PxGeometryHolder(physx::PxConvexMeshGeometry(convex_mesh)); // TODO: Who is responsible for releasing triangle_mesh;
}

//CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Octree::ConstPtr& geom)
//{
//  switch (geom->getSubType())
//  {
//    case tesseract_geometry::Octree::SubType::BOX:
//    {
//      return CollisionGeometryPtr(new fcl::OcTreed(geom->getOctree()));
//    }
//    default:
//    {
//      CONSOLE_BRIDGE_logError("This fcl octree sub shape type (%d) is not supported for geometry octree",
//                              static_cast<int>(geom->getSubType()));
//      return nullptr;
//    }
//  }
//}

physx::PxGeometryHolder createShapePrimitive(TesseractPhysx& tesseract_physx, const CollisionShapeConstPtr& geom)
{
  switch (geom->getType())
  {
//    case tesseract_geometry::GeometryType::PLANE:
//    {
//      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::Plane>(geom));
//    }
    case tesseract_geometry::GeometryType::BOX:
    {
      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::Box>(geom));
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::Sphere>(geom));
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::Cylinder>(geom));
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::Cone>(geom));
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::Capsule>(geom));
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::Mesh>(geom));
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geom));
    }
//    case tesseract_geometry::GeometryType::OCTREE:
//    {
//      return createShapePrimitive(tesseract_physx, std::static_pointer_cast<const tesseract_geometry::Octree>(geom));
//    }
    default:
    {
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using fcl yet",
                              static_cast<int>(geom->getType()));
      return physx::PxGeometryHolder();
    }
  }
}

//// https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/AdvancedCollisionDetection.html#extracting-contact-information
//void TesseractSimulationEventCallback::onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs)
//{
//  UNUSED(pairHeader);
//  UNUSED(pairs);
//  UNUSED(nbPairs);
//  std::printf("TesseractSimulationEventCallback::onContact\n");
//  const physx::PxU32 bufferSize = 64;
//  physx::PxContactPairPoint contacts[bufferSize];
//  for(physx::PxU32 i=0; i < nbPairs; i++)
//  {
//    const physx::PxContactPair& cp = pairs[i];
//    physx::PxU32 nbContacts = pairs[i].extractContacts(contacts, bufferSize);
//    for(physx::PxU32 j=0; j < nbContacts; j++)
//    {
//        physx::PxVec3 point = contacts[j].position;
//        physx::PxVec3 impulse = contacts[j].impulse;
//        physx::PxU32 internalFaceIndex0 = contacts[j].internalFaceIndex0;
//        physx::PxU32 internalFaceIndex1 = contacts[j].internalFaceIndex1;
//        UNUSED(point);
//        UNUSED(impulse);
//        UNUSED(internalFaceIndex0);
//        UNUSED(internalFaceIndex1);
//        //...
//    }

//    if(cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_FOUND)
//    {
////            if((pairHeader.actors[0] == mSubmarineActor) ||
////                (pairHeader.actors[1] == mSubmarineActor))
////            {
////                PxActor* otherActor = (mSubmarineActor == pairHeader.actors[0]) ?
////                    pairHeader.actors[1] : pairHeader.actors[0];
////                Seamine* mine =  reinterpret_cast<Seamine*>(otherActor->userData);
////                // insert only once
////                if(std::find(mMinesToExplode.begin(), mMinesToExplode.end(), mine) ==
////                    mMinesToExplode.end())
////                    mMinesToExplode.push_back(mine);

////                break;
////            }
//    }
//  }
//}

//physx::PxFilterFlags contactReportFilterShader(physx::PxFilterObjectAttributes attributes0,
//                                               physx::PxFilterData filterData0,
//                                               physx::PxFilterObjectAttributes attributes1,
//                                               physx::PxFilterData filterData1,
//                                               physx::PxPairFlags& pairFlags,
//                                               const void* constantBlock,
//                                               physx::PxU32 constantBlockSize)
//{
//  UNUSED(attributes0);
//  UNUSED(attributes1);
//  UNUSED(filterData0);
//  UNUSED(filterData1);
//  UNUSED(constantBlockSize);
//  UNUSED(constantBlock);

//  // let triggers through
//  if(physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1))
//  {
//      pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
//      return physx::PxFilterFlag::eDEFAULT;
//  }

//  // generate contacts for all that were not filtered above
//  pairFlags = physx::PxPairFlag::eNOTIFY_CONTACT_POINTS | physx::PxPairFlag::eDETECT_DISCRETE_CONTACT;

//  // trigger the contact callback for pairs (A,B) where
//  // the filtermask of A contains the ID of B and vice versa.
//  if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
//      pairFlags |=physx::PxPairFlag::eNOTIFY_TOUCH_FOUND;

//  return physx::PxFilterFlag::eDEFAULT;

////  // all initial and persisting reports for everything, with per-point data
////  pairFlags = physx::PxPairFlag::eSOLVE_CONTACT | physx::PxPairFlag::eDETECT_DISCRETE_CONTACT
////        |	physx::PxPairFlag::eNOTIFY_TOUCH_FOUND
////        | physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS
////        | physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;
////  return physx::PxFilterFlag::eNOTIFY; //:eCALLBACK; //physx::PxFilterFlag::eDEFAULT;
//}

//physx::PxFilterFlags	TesseractSimulationFilterCallback::pairFound(physx::PxU32 pairID,
//                                                                   physx::PxFilterObjectAttributes attributes0,
//                                                                   physx::PxFilterData filterData0,
//                                                                   const physx::PxActor* a0,
//                                                                   const physx::PxShape* s0,
//                                                                   physx::PxFilterObjectAttributes attributes1,
//                                                                   physx::PxFilterData filterData1,
//                                                                   const physx::PxActor* a1,
//                                                                   const physx::PxShape* s1,
//                                                                   physx::PxPairFlags& pairFlags)
//{
//  UNUSED(pairID);
//  UNUSED(attributes0);
//  UNUSED(filterData0);
//  UNUSED(a0);
//  UNUSED(s0);
//  UNUSED(attributes1);
//  UNUSED(filterData1);
//  UNUSED(a1);
//  UNUSED(s1);

//  // all initial and persisting reports for everything, with per-point data
//  pairFlags = physx::PxPairFlag::eSOLVE_CONTACT | physx::PxPairFlag::eDETECT_DISCRETE_CONTACT
//        |	physx::PxPairFlag::eNOTIFY_TOUCH_FOUND
//        | physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS
//        | physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;

//  return physx::PxFilterFlag::eNOTIFY;
//}


//TesseractPhysx::TesseractPhysx()
//{
//  initialize();
//}

//TesseractPhysx::~TesseractPhysx()
//{
//  PX_RELEASE(scene_);
//  PX_RELEASE(dispatcher_);
//  PX_RELEASE(physics_);
//  PX_RELEASE(cooking_);

//  if(pvd_)
//  {
//    physx::PxPvdTransport* transport = pvd_->getTransport();
//    PX_RELEASE(pvd_);
//    PX_RELEASE(transport);
//  }

//  PX_RELEASE(foundation_);
//}

//physx::PxFoundation* TesseractPhysx::getFoundation() const
//{
//  return foundation_;
//}

//physx::PxPhysics* TesseractPhysx::getPhysics() const
//{
//  return physics_;
//}

//physx::PxCooking* TesseractPhysx::getCooking() const
//{
//  return cooking_;
//}

//physx::PxScene* TesseractPhysx::getScene() const
//{
//  return scene_;
//}

//physx::PxMaterial* TesseractPhysx::getMaterial() const
//{
//  return material_;
//}

//physx::PxDefaultAllocator& TesseractPhysx::getAllocator()
//{
//  return default_allocator_;
//}

//const physx::PxDefaultErrorCallback& TesseractPhysx::getErrorCallback()
//{
//  return error_callback_;
//}

//void TesseractPhysx::initialize()
//{
//  foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, default_allocator_, error_callback_);
//  if(!foundation_)
//      CONSOLE_BRIDGE_logError("PxCreateFoundation failed!");

//  pvd_ = physx::PxCreatePvd(*foundation_);
//  physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate(pvd_host_.c_str(), 5425, 10);
//  pvd_->connect(*transport,physx::PxPvdInstrumentationFlag::eALL);

//  physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, physx::PxTolerancesScale(), true, pvd_);
//  if(!physics_)
//    CONSOLE_BRIDGE_logError("PxCreatePhysics failed!");

//  // The PhysX cooking library provides utilities for creating, converting, and serializing bulk data. Depending on
//  // your application, you may wish to link to the cooking library in order to process such data at runtime.
//  // Alternatively you may be able to process all such data in advance and just load it into memory as required.
//  // Initialize the cooking library as follows:
//  cooking_ = PxCreateCooking(PX_PHYSICS_VERSION, *foundation_, physx::PxCookingParams(physx::PxTolerancesScale()));
//  if (!cooking_)
//      CONSOLE_BRIDGE_logError("PxCreateCooking failed!");

//  // The extensions library contains many functions that may be useful to a large class of users, but which some users
//  // may prefer to omit from their application either for code size reasons or to avoid use of certain subsystems,
//  // such as those pertaining to networking. Initializing the extensions library requires the PxPhysics object:
//  if (!PxInitExtensions(*physics_, pvd_))
//    CONSOLE_BRIDGE_logError("PxInitExtensions failed!");

//  physx::PxSceneDesc scene_desc(physics_->getTolerancesScale());
//  scene_desc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
//  dispatcher_ = physx::PxDefaultCpuDispatcherCreate(2);
//  scene_desc.cpuDispatcher	= dispatcher_;
//  scene_desc.filterShader	= physx::PxDefaultSimulationFilterShader;
//  scene_desc.kineKineFilteringMode = physx::PxPairFilteringMode::eKEEP; // So kin-kin contacts with be reported
//  scene_desc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP; // So static-kin constacts will be reported
//  scene_desc.simulationEventCallback = &event_cb_;
//  scene_desc.filterShader	= contactReportFilterShader;
////  scene_desc.filterCallback = &filter_cb_;
//  scene_ = physics_->createScene(scene_desc);

//  physx::PxPvdSceneClient* pvdClient = scene_->getScenePvdClient();
//  if(pvdClient)
//  {
//    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
//    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
//    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
//  }

//  material_ = physics_->createMaterial(0.5f, 0.5f, 0.6f);
//}


//bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
//{
//  auto* cdata = reinterpret_cast<ContactTestData*>(data);

//  if (cdata->done)
//    return true;

//  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
//  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

//  bool needs_collision =
//      cd1->m_enabled && cd2->m_enabled && (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&
//      (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&
//      !isContactAllowed(cd1->getName(), cd2->getName(), cdata->fn, false) &&
//      (std::find(cdata->active.begin(), cdata->active.end(), cd1->getName()) != cdata->active.end() ||
//       std::find(cdata->active.begin(), cdata->active.end(), cd2->getName()) != cdata->active.end());

//  if (!needs_collision)
//    return false;

//  size_t num_contacts = std::numeric_limits<size_t>::max();
//  if (cdata->type == ContactTestType::FIRST)
//    num_contacts = 1;

//  fcl::CollisionResultd col_result;
//  fcl::collide(o1, o2, fcl::CollisionRequestd(num_contacts, true, 1, false), col_result);

//  if (col_result.isCollision())
//  {
//    for (size_t i = 0; i < col_result.numContacts(); ++i)
//    {
//      const fcl::Contactd& fcl_contact = col_result.getContact(i);
//      ContactResult contact;
//      contact.link_names[0] = cd1->getName();
//      contact.link_names[1] = cd2->getName();
//      contact.shape_id[0] = cd1->getShapeIndex(o1);
//      contact.shape_id[1] = cd2->getShapeIndex(o2);
//      contact.subshape_id[0] = fcl_contact.b1;
//      contact.subshape_id[1] = fcl_contact.b2;
//      contact.nearest_points[0] = fcl_contact.pos;
//      contact.nearest_points[1] = fcl_contact.pos;
//      contact.type_id[0] = cd1->getTypeID();
//      contact.type_id[1] = cd2->getTypeID();
//      contact.distance = -1.0 * fcl_contact.penetration_depth;
//      contact.normal = fcl_contact.normal;

//      ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
//      const auto& it = cdata->res.find(pc);
//      bool found = (it != cdata->res.end());

//      processResult(*cdata, contact, pc, found);
//    }
//  }

//  return cdata->done;
//}

//bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist)
//{
//  auto* cdata = reinterpret_cast<ContactTestData*>(data);
//  min_dist = cdata->contact_distance;

//  if (cdata->done)
//    return true;

//  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
//  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

//  bool needs_collision =
//      cd1->m_enabled && cd2->m_enabled && (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&
//      (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&
//      !isContactAllowed(cd1->getName(), cd2->getName(), cdata->fn, false) &&
//      (std::find(cdata->active.begin(), cdata->active.end(), cd1->getName()) != cdata->active.end() ||
//       std::find(cdata->active.begin(), cdata->active.end(), cd2->getName()) != cdata->active.end());

//  if (!needs_collision)
//    return false;

//  fcl::DistanceResultd fcl_result;
//  fcl::DistanceRequestd fcl_request(true, true);
//  double d = fcl::distance(o1, o2, fcl_request, fcl_result);

//  if (d < cdata->contact_distance)
//  {
//    ContactResult contact;
//    contact.link_names[0] = cd1->getName();
//    contact.link_names[1] = cd2->getName();
//    contact.shape_id[0] = cd1->getShapeIndex(o1);
//    contact.shape_id[1] = cd2->getShapeIndex(o2);
//    contact.subshape_id[0] = fcl_result.b1;
//    contact.subshape_id[1] = fcl_result.b2;
//    contact.nearest_points[0] = fcl_result.nearest_points[0];
//    contact.nearest_points[1] = fcl_result.nearest_points[1];
//    contact.type_id[0] = cd1->getTypeID();
//    contact.type_id[1] = cd2->getTypeID();
//    contact.distance = fcl_result.min_distance;
//    contact.normal = (fcl_result.min_distance * (contact.nearest_points[1] - contact.nearest_points[0])).normalized();

//    // TODO: There is an issue with FCL need to track down
//    if (std::isnan(contact.nearest_points[0](0)))
//    {
//      CONSOLE_BRIDGE_logError("Nearest Points are NAN's");
//    }

//    ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
//    const auto& it = cdata->res.find(pc);
//    bool found = (it != cdata->res.end());

//    processResult(*cdata, contact, pc, found);
//  }

//  return cdata->done;
//}

//CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
//                                               const int& type_id,
//                                               CollisionShapesConst shapes,
//                                               tesseract_common::VectorIsometry3d shape_poses)
//  : name_(std::move(name)), type_id_(type_id), shapes_(std::move(shapes)), shape_poses_(std::move(shape_poses))
//{
//  assert(!shapes_.empty());
//  assert(!shape_poses_.empty());
//  assert(!name_.empty());
//  assert(shapes_.size() == shape_poses_.size());

//  collision_geometries_.reserve(shapes_.size());
//  collision_objects_.reserve(shapes_.size());
//  for (const auto& shape : shapes_)
//  {
//    CollisionGeometryPtr subshape = createShapePrimitive(shape);
//    if (subshape != nullptr)
//    {
//      collision_geometries_.push_back(subshape);
//      CollisionObjectPtr co(new fcl::CollisionObjectd(subshape));
//      co->setUserData(this);
//      collision_objects_.push_back(co);
//    }
//  }
//}

//CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
//                                               const int& type_id,
//                                               CollisionShapesConst shapes,
//                                               tesseract_common::VectorIsometry3d shape_poses,
//                                               std::vector<CollisionGeometryPtr> collision_geometries,
//                                               const std::vector<CollisionObjectPtr>& collision_objects)
//  : name_(std::move(name))
//  , type_id_(type_id)
//  , shapes_(std::move(shapes))
//  , shape_poses_(std::move(shape_poses))
//  , collision_geometries_(std::move(collision_geometries))
//{
//  collision_objects_.reserve(collision_objects.size());
//  for (const auto& co : collision_objects)
//  {
//    CollisionObjectPtr collObj(new fcl::CollisionObjectd(*co));
//    collObj->setUserData(this);
//    collision_objects_.push_back(collObj);
//  }
//}

//int CollisionObjectWrapper::getShapeIndex(const fcl::CollisionObjectd* co) const
//{
//  auto it = std::find_if(collision_objects_.begin(), collision_objects_.end(), [&co](const CollisionObjectPtr& c) {
//    return c.get() == co;
//  });

//  if (it != collision_objects_.end())
//    return static_cast<int>(std::distance(collision_objects_.begin(), it));

//  return -1;
//}

}  // namespace tesseract_collision
