
#include <tesseract_collision_physx/utils.h>
#include <geomutils/GuContactPoint.h>

static const std::size_t DIM = 10;

class TesseractOverlapCallback : public physx::PxOverlapCallback
{
public:
  TesseractOverlapCallback(physx::PxOverlapHit* touches, physx::PxU32 max_nb_touches)
    : physx::PxOverlapCallback(touches, max_nb_touches) {}

  physx::PxAgain processTouches(const physx::PxOverlapHit* /*buffer*/, physx::PxU32 /*nbHits*/) override
  {
    std::printf("ProcessTouches");
    return false;
  }
};

std::vector<physx::PxRigidStatic*> addCollisionObjects(tesseract_collision::TesseractPhysxScene& scene, bool /*use_single_link*/, bool /*use_convex_mesh*/)
{
  std::vector<physx::PxRigidStatic*> spheres;
  tesseract_common::VectorIsometry3d link_poses;
  for (std::size_t x = 0; x < DIM; ++x)
  {
    for (std::size_t y = 0; y < DIM; ++y)
    {
      for (std::size_t z = 0; z < DIM; ++z)
      {
        std::string link_name = "sphere_link_" + std::to_string(x) + std::to_string(y) + std::to_string(z);
        physx::PxTransform global_tf, shape_tf;
        global_tf.p = physx::PxVec3(static_cast<physx::PxReal>(x), static_cast<physx::PxReal>(y), static_cast<physx::PxReal>(z));
        global_tf.q = physx::PxQuat(0, 0, 1, 0);

        physx::PxRigidStatic* sphere = physx::PxCreateStatic(*scene.getTesseractPhysx()->getPhysics(), global_tf, physx::PxSphereGeometry(physx::PxReal(0.25)), *scene.getTesseractPhysx()->getMaterial());
        sphere->setName(link_name.c_str());
        std::printf("%s\n", sphere->getName());
        scene.getScene()->addActor(*sphere);
        spheres.push_back(sphere);
      }
    }
  }
  return spheres;
}

std::vector<physx::PxTransform> getTransforms(std::size_t num_poses)
{
  std::vector<physx::PxTransform> poses(num_poses);
  for (std::size_t i = 0; i < num_poses; ++i)
  {
    physx::PxReal x = static_cast<physx::PxReal>(rand() / RAND_MAX) * physx::PxReal(DIM);
    physx::PxReal y = static_cast<physx::PxReal>(rand() / RAND_MAX) * physx::PxReal(DIM);
    physx::PxReal z = static_cast<physx::PxReal>(rand() / RAND_MAX) * physx::PxReal(DIM);
    poses[i].p = physx::PxVec3(x, y, z);
    poses[i].q = physx::PxQuat(0, 0, 1, 0);
  }
  return poses;
}

void test1(tesseract_collision::TesseractPhysxScene& scene)
{
  std::vector<physx::PxRigidStatic*> spheres = addCollisionObjects(scene, false, false);

  std::vector<physx::PxTransform> tv = getTransforms(50);

  physx::PxGeometryHolder sphere(physx::PxSphereGeometry(physx::PxReal(0.5)));
  physx::PxTransform t;
  t.p = physx::PxVec3(0, 0, 0);
  t.q = physx::PxQuat(0, 0, 1, 0);

  physx::PxOverlapHit* touches = new physx::PxOverlapHit();
  TesseractOverlapCallback cb(touches, 2);

  for (const auto& trans : tv)
  {
    bool in_collision = scene.getScene()->overlap(sphere.sphere(), trans, cb);

    if (in_collision)
    {
      std::printf("Collision found with %s!\n", touches->actor->getName());

      physx::PxVec3 direction;
      physx::PxF32 depth;
      physx::PxGeometryQuery::computePenetration(direction, depth, sphere.sphere(), trans, touches->shape->getGeometry().sphere(), touches->actor->getGlobalPose() * touches->shape->getLocalPose());
      std::printf("Distance %f!\n", depth);
    }

  }

  delete touches;
}

void test2(tesseract_collision::TesseractPhysxScene& scene)
{
  std::string link_name = "static_link";
  physx::PxTransform global_tf;
  global_tf.p = physx::PxVec3(0, 0, 1.0);
  global_tf.q = physx::PxQuat(0, 0, 1, 0);

  physx::PxRigidStatic* sphere = physx::PxCreateStatic(*scene.getTesseractPhysx()->getPhysics(), global_tf, physx::PxSphereGeometry(physx::PxReal(0.25)), *scene.getTesseractPhysx()->getMaterial());
  sphere->setName(link_name.c_str());
  std::printf("%s\n", sphere->getName());
  scene.getScene()->addActor(*sphere);


  physx::PxGeometryHolder test_sphere(physx::PxSphereGeometry(physx::PxReal(0.5)));
  physx::PxTransform t;
  t.p = physx::PxVec3(0, 0, 0);
  t.q = physx::PxQuat(0, 0, 1, 0);

  physx::PxOverlapHit* touches = new physx::PxOverlapHit();
  TesseractOverlapCallback cb(touches, 2);

  bool in_collision = scene.getScene()->overlap(test_sphere.sphere(), t, cb);

  if (in_collision)
  {
    std::printf("Collision found with %s!\n", touches->actor->getName());

    physx::PxVec3 direction;
    physx::PxF32 depth;
    physx::PxGeometryQuery::computePenetration(direction, depth, test_sphere.sphere(), t, touches->shape->getGeometry().sphere(), touches->actor->getGlobalPose() * touches->shape->getLocalPose());
    std::printf("Distance %f!\n", depth);
  }
}

int main(int, const char*const*)
{
  auto phy = std::make_shared<tesseract_collision::TesseractPhysx>();
  tesseract_collision::TesseractPhysxScene scene(phy);
  test2(scene);

//  physx::PxRigidStatic* ground_plane = physx::PxCreatePlane(*phy.getPhysics(), physx::PxPlane(0,1,0,0), *phy.getMaterial());
//  ground_plane->setName("base_link");
//  phy.getScene()->addActor(*ground_plane);

//  kinematic_sphere->setName("kinematic_link");
//  phy.getScene()->addActor(*kinematic_sphere);

  return 0;
}
