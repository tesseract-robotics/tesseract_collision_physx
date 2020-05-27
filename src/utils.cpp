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
#include <cmath>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

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

/** @brief Create vertices that represent a cone */
tesseract_common::VectorVector3d createConeVertices(double radius, double height)
{
  double desired_error = 0.0005; // meters
  double dtheta = 2 * std::acos((radius-desired_error)/radius);
  std::size_t v_count = static_cast<std::size_t>(std::ceil(2 * M_PI / dtheta));
  dtheta = 2 * M_PI / static_cast<double>(v_count);

  tesseract_common::VectorVector3d vertices;
  vertices.reserve(v_count + 1);
  double theta = 0;
  while (theta < (2 * M_PI - (dtheta/2.)))
  {
    Eigen::Vector3d v;
    v(0) = radius * std::cos(theta);
    v(1) = radius * std::sin(theta);
    v(2) = -height / 2.;
    vertices.push_back(v);
    theta+=dtheta;
  }

  Eigen::Vector3d v;
  v(0) = 0;
  v(1) = 0;
  v(2) = height / 2.;
  vertices.push_back(v);

  return vertices;
}

/** @brief Create vertices that represent a cylinder */
tesseract_common::VectorVector3d createCylinderVertices(double radius, double height)
{
  double desired_error = 0.0005; // meters
  double dtheta = 2 * std::acos((radius-desired_error)/radius);
  std::size_t v_count = static_cast<std::size_t>(std::ceil(2 * M_PI / dtheta));
  dtheta = 2 * M_PI / static_cast<double>(v_count);

  tesseract_common::VectorVector3d vertices;
  vertices.reserve(v_count + 1);
  double theta = 0;
  while (theta < (2 * M_PI - (dtheta/2.)))
  {
    Eigen::Vector3d v;
    v(0) = radius * std::cos(theta);
    v(1) = radius * std::sin(theta);
    v(2) = -height / 2.;
    vertices.push_back(v);

    Eigen::Vector3d vt = v;
    vt(2) = height / 2.;
    vertices.push_back(vt);

    theta+=dtheta;
  }

  return vertices;
}

physx::PxGeometryHolder createConvexShapePrimitive(TesseractPhysxScene& physx_scene, const tesseract_common::VectorVector3d& v)
{
  std::size_t v_count = v.size();

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
  convex_desc.flags |= physx::PxConvexFlag::eSHIFT_VERTICES;

  if (physx_scene.getTesseractPhysx()->getDescription().enable_gpu)
  {
    // This will make the convex shape GPU compatible which means the geometry will change. The question is how does
    // it change the geometry. Is it conservative by make sure all of the orginal vertices are within the new convex
    // hull?
    convex_desc.flags |= physx::PxConvexFlag::eGPU_COMPATIBLE;
  }

  physx::PxDefaultMemoryOutputStream buf;
  physx::PxConvexMeshCookingResult::Enum result;
  if(!physx_scene.getTesseractPhysx()->getCooking()->cookConvexMesh(convex_desc, buf, &result))
      return physx::PxGeometryHolder();

  physx::PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  physx::PxConvexMesh* convex_mesh = physx_scene.getTesseractPhysx()->getPhysics()->createConvexMesh(input);
  if (convex_mesh == nullptr)
    return physx::PxGeometryHolder();

  auto g = physx::PxConvexMeshGeometry(convex_mesh);
  g.meshFlags |= physx::PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;
  return physx::PxGeometryHolder(g); // TODO: Who is responsible for releasing triangle_mesh;
}

//physx::PxGeometryHolder createShapePrimitive(TesseractPhysxScene& physx_scene, const tesseract_geometry::Plane::ConstPtr& geom)
//{
//  physx::PxPlane plane(float(geom->getA()), float(geom->getB()), float(geom->getC()), float(geom->getD()));
//  physx::PxPlaneGeometry pg;

//  physx::PxGeometryHolder gh(physx::PxPlaneGeometry(plane));

//  return gh;
//}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& /*physx_scene*/, const tesseract_geometry::Box::ConstPtr& geom)
{
  auto g = physx::PxGeometryHolder(physx::PxBoxGeometry(static_cast<physx::PxReal>(geom->getX() / 2), static_cast<physx::PxReal>(geom->getY() / 2), static_cast<physx::PxReal>(geom->getZ() / 2)));
  return {std::make_pair(g, physx::PxTransform(physx::PxIdentity))};
}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& /*physx_scene*/, const tesseract_geometry::Sphere::ConstPtr& geom)
{
  auto g = physx::PxGeometryHolder(physx::PxSphereGeometry(static_cast<physx::PxReal>(geom->getRadius())));
  return {std::make_pair(g, physx::PxTransform(physx::PxIdentity))};
}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene, const tesseract_geometry::Cylinder::ConstPtr& geom)
{
  tesseract_common::VectorVector3d vertices = createCylinderVertices(geom->getRadius(), geom->getLength());
  auto g = createConvexShapePrimitive(physx_scene, vertices);
  return {std::make_pair(g, physx::PxTransform(physx::PxIdentity))};
}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene, const tesseract_geometry::Cone::ConstPtr& geom)
{
  tesseract_common::VectorVector3d vertices = createConeVertices(geom->getRadius(), geom->getLength());
  auto g = createConvexShapePrimitive(physx_scene, vertices);
  return {std::make_pair(g, physx::PxTransform(physx::PxIdentity))};
}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& /*physx_scene*/, const tesseract_geometry::Capsule::ConstPtr& geom)
{
  auto g = physx::PxGeometryHolder(physx::PxCapsuleGeometry(static_cast<physx::PxReal>(geom->getRadius()), static_cast<physx::PxReal>(geom->getLength() / 2)));
  // The default orientation of a capsule for Physx is along the x-axs, so we must rotate around the y-axis so it
  // is along the z-axis.
  return {std::make_pair(g, physx::PxTransform(physx::PxQuat(physx::PxHalfPi, physx::PxVec3(0,1,0))))};
}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene, const tesseract_geometry::Mesh::ConstPtr& geom)
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
  bool status = physx_scene.getTesseractPhysx()->getCooking()->cookTriangleMesh(mesh_desc, write_buffer, &write_result);
  if(!status)
    return {std::make_pair(physx::PxGeometryHolder(), physx::PxTransform(physx::PxIdentity))};

  physx::PxDefaultMemoryInputData read_buffer(write_buffer.getData(), write_buffer.getSize());
  physx::PxTriangleMesh* triangle_mesh = physx_scene.getTesseractPhysx()->getPhysics()->createTriangleMesh(read_buffer);

  auto g = physx::PxGeometryHolder(physx::PxTriangleMeshGeometry(triangle_mesh)); // TODO: Who is responsible for releasing triangle_mesh;
  return {std::make_pair(g, physx::PxTransform(physx::PxIdentity))};
}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const tesseract_geometry::ConvexMesh::ConstPtr& geom)
{
  auto g = createConvexShapePrimitive(physx_scene, *(geom->getVertices()));
  return {std::make_pair(g, physx::PxTransform(physx::PxIdentity))};
}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& /*physx_scene*/,
                                                               const tesseract_geometry::Octree::ConstPtr& geom)
{
  const octomap::OcTree& octree = *(geom->getOctree());
  double occupancy_threshold = octree.getOccupancyThres();

  std::vector<TesseractPhysxGeometryHolder> geometries;
  geometries.reserve(octree.size());

  switch (geom->getSubType())
  {
    case tesseract_geometry::Octree::SubType::BOX:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
           ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          physx::PxTransform geom_tf(static_cast<physx::PxReal>(it.getX()), static_cast<physx::PxReal>(it.getY()), static_cast<physx::PxReal>(it.getZ()));
          auto l = static_cast<physx::PxReal>(size / 2.0);
          auto g = physx::PxGeometryHolder(physx::PxBoxGeometry(l, l, l));
          geometries.emplace_back(g, geom_tf);
        }
      }
      return geometries;
    }
    case tesseract_geometry::Octree::SubType::SPHERE_INSIDE:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
           ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          physx::PxTransform geom_tf(static_cast<physx::PxReal>(it.getX()), static_cast<physx::PxReal>(it.getY()), static_cast<physx::PxReal>(it.getZ()));
          auto g = physx::PxGeometryHolder(physx::PxSphereGeometry(static_cast<physx::PxReal>((size / 2))));
          geometries.emplace_back(g, geom_tf);
        }
      }
      return geometries;
    }
    case tesseract_geometry::Octree::SubType::SPHERE_OUTSIDE:
    {
      for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
           ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          physx::PxTransform geom_tf(static_cast<physx::PxReal>(it.getX()), static_cast<physx::PxReal>(it.getY()), static_cast<physx::PxReal>(it.getZ()));
          auto g = physx::PxGeometryHolder(physx::PxSphereGeometry(static_cast<physx::PxReal>(std::sqrt(2 * ((size / 2) * (size / 2))))));
          geometries.emplace_back(g, geom_tf);
        }
      }
      return geometries;
    }
  }

  CONSOLE_BRIDGE_logError("This Physx shape type (%d) is not supported for geometry octree",
                          static_cast<int>(geom->getSubType()));

  return std::vector<TesseractPhysxGeometryHolder>();
}

std::vector<TesseractPhysxGeometryHolder> createShapePrimitive(TesseractPhysxScene& physx_scene,
                                                               const CollisionShapeConstPtr& geom)
{
  switch (geom->getType())
  {
//    case tesseract_geometry::GeometryType::PLANE:
//    {
//      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::Plane>(geom));
//    }
    case tesseract_geometry::GeometryType::BOX:
    {
      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::Box>(geom));
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::Sphere>(geom));
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::Cylinder>(geom));
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::Cone>(geom));
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::Capsule>(geom));
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::Mesh>(geom));
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geom));
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      return createShapePrimitive(physx_scene, std::static_pointer_cast<const tesseract_geometry::Octree>(geom));
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using fcl yet",
                              static_cast<int>(geom->getType()));

      return std::vector<TesseractPhysxGeometryHolder>();
    }
  }
}
}  // namespace tesseract_collision
