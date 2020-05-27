#include <PxPhysicsAPI.h>
#include <istream>
#include <memory>

#define PX_RELEASE(x)	if(x)	{ x->release(); x = nullptr;}
#define UNUSED(x) (void)(x)
#define CONSOLE_BRIDGE_logError(x) { std::printf("%s\n", (x)); }

class TesseractSimulationEventCallback : public physx::PxSimulationEventCallback
{
public:
  void onConstraintBreak(physx::PxConstraintInfo* /*constraints*/, physx::PxU32 /*count*/) override {std::printf("onConstraintBreak\n");}
  void onWake(physx::PxActor** /*actors*/, physx::PxU32 /*count*/) override {std::printf("onWake\n");}
  void onSleep(physx::PxActor** /*actors*/, physx::PxU32 /*count*/) override {std::printf("onSleep\n");}
  void onTrigger(physx::PxTriggerPair* /*pairs*/, physx::PxU32 /*count*/) override {std::printf("onTrigger\n");}
  void onAdvance(const physx::PxRigidBody*const* /*bodyBuffer*/, const physx::PxTransform* /*poseBuffer*/, const physx::PxU32 /*count*/) override {std::printf("onAdvance\n");}

  // https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/AdvancedCollisionDetection.html#extracting-contact-information
  void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs) override
  {
    UNUSED(pairHeader);
    std::printf("TesseractSimulationEventCallback::onContact\n");
    const physx::PxU32 bufferSize = 64;
    physx::PxContactPairPoint contacts[bufferSize];
    for(physx::PxU32 i=0; i < nbPairs; i++)
    {
      const physx::PxContactPair& cp = pairs[i];
      physx::PxU32 nbContacts = pairs[i].extractContacts(contacts, bufferSize);
      for(physx::PxU32 j=0; j < nbContacts; j++)
      {
          physx::PxVec3 point = contacts[j].position;
          physx::PxVec3 impulse = contacts[j].impulse;
          physx::PxU32 internalFaceIndex0 = contacts[j].internalFaceIndex0;
          physx::PxU32 internalFaceIndex1 = contacts[j].internalFaceIndex1;
          UNUSED(point);
          UNUSED(impulse);
          UNUSED(internalFaceIndex0);
          UNUSED(internalFaceIndex1);
          //...
      }

      if(cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_FOUND)
      {
  //            if((pairHeader.actors[0] == mSubmarineActor) ||
  //                (pairHeader.actors[1] == mSubmarineActor))
  //            {
  //                PxActor* otherActor = (mSubmarineActor == pairHeader.actors[0]) ?
  //                    pairHeader.actors[1] : pairHeader.actors[0];
  //                Seamine* mine =  reinterpret_cast<Seamine*>(otherActor->userData);
  //                // insert only once
  //                if(std::find(mMinesToExplode.begin(), mMinesToExplode.end(), mine) ==
  //                    mMinesToExplode.end())
  //                    mMinesToExplode.push_back(mine);

  //                break;
  //            }
      }
    }
  }
};

enum class FilterGroup : physx::PxU32
{
    eSTATIC     = (1 << 0),
    eKINEMATIC  = (1 << 1)
};



physx::PxFilterFlags contactReportFilterShader(physx::PxFilterObjectAttributes attributes0,
                                               physx::PxFilterData filterData0,
                                               physx::PxFilterObjectAttributes attributes1,
                                               physx::PxFilterData filterData1,
                                               physx::PxPairFlags& pairFlags,
                                               const void* constantBlock,
                                               physx::PxU32 constantBlockSize)
{
  UNUSED(constantBlockSize);
  UNUSED(constantBlock);

  // let triggers through
  if(physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1))
  {
      pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
      return physx::PxFilterFlag::eDEFAULT;
  }

  // generate contacts for all that were not filtered above
  pairFlags = physx::PxPairFlag::eNOTIFY_CONTACT_POINTS | physx::PxPairFlag::eDETECT_DISCRETE_CONTACT;

  // trigger the contact callback for pairs (A,B) where
  // the filtermask of A contains the ID of B and vice versa.
  if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
      pairFlags |= (physx::PxPairFlag::eNOTIFY_TOUCH_FOUND | physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS);

  return physx::PxFilterFlag::eDEFAULT;
}

class TesseractPhysx
{
public:
  using Ptr = std::shared_ptr<TesseractPhysx>;
  using ConstPtr = std::shared_ptr<const TesseractPhysx>;

  TesseractPhysx()
  {
    initialize();
  }

  virtual ~TesseractPhysx()
  {
    PX_RELEASE(scene_);
    PX_RELEASE(dispatcher_);
    PX_RELEASE(physics_);
    PX_RELEASE(cooking_);

    if(pvd_)
    {
      physx::PxPvdTransport* transport = pvd_->getTransport();
      PX_RELEASE(pvd_);
      PX_RELEASE(transport);
    }

    PX_RELEASE(foundation_);
  }

  TesseractPhysx(const TesseractPhysx&) = delete;
  TesseractPhysx& operator=(const TesseractPhysx&) = delete;
  TesseractPhysx(TesseractPhysx&&) = delete;
  TesseractPhysx& operator=(TesseractPhysx&&) = delete;

  physx::PxFoundation* getFoundation() const { return foundation_; }
  physx::PxPhysics* getPhysics() const { return physics_; }
  physx::PxCooking* getCooking() const { return cooking_; }
  physx::PxScene* getScene() const { return scene_; }
  physx::PxMaterial* getMaterial() const { return material_; }

  physx::PxDefaultAllocator& getAllocator() { return default_allocator_; }
  const physx::PxDefaultErrorCallback& getErrorCallback() { return error_callback_; }

private:
  physx::PxDefaultAllocator		default_allocator_;
  physx::PxDefaultErrorCallback	error_callback_;

  physx::PxFoundation*           foundation_ {nullptr};
  physx::PxPhysics*              physics_ {nullptr};
  physx::PxCooking*              cooking_ {nullptr};
  physx::PxScene*                scene_ {nullptr};
  physx::PxDefaultCpuDispatcher* dispatcher_{nullptr};
  physx::PxMaterial*				     material_{nullptr};
  physx::PxPvd*                  pvd_{nullptr};
  std::string                    pvd_host_ {"127.0.0.1"};

  TesseractSimulationEventCallback event_cb_;

  void initialize()
  {
    foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, default_allocator_, error_callback_);
    if(!foundation_)
        CONSOLE_BRIDGE_logError("PxCreateFoundation failed!");

    pvd_ = physx::PxCreatePvd(*foundation_);
    physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate(pvd_host_.c_str(), 5425, 10);
    pvd_->connect(*transport,physx::PxPvdInstrumentationFlag::eALL);

    physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, physx::PxTolerancesScale(), true, pvd_);
    if(!physics_)
      CONSOLE_BRIDGE_logError("PxCreatePhysics failed!");

    // The PhysX cooking library provides utilities for creating, converting, and serializing bulk data. Depending on
    // your application, you may wish to link to the cooking library in order to process such data at runtime.
    // Alternatively you may be able to process all such data in advance and just load it into memory as required.
    // Initialize the cooking library as follows:
    cooking_ = PxCreateCooking(PX_PHYSICS_VERSION, *foundation_, physx::PxCookingParams(physx::PxTolerancesScale()));
    if (!cooking_)
        CONSOLE_BRIDGE_logError("PxCreateCooking failed!");

    // The extensions library contains many functions that may be useful to a large class of users, but which some users
    // may prefer to omit from their application either for code size reasons or to avoid use of certain subsystems,
    // such as those pertaining to networking. Initializing the extensions library requires the PxPhysics object:
    if (!PxInitExtensions(*physics_, pvd_))
      CONSOLE_BRIDGE_logError("PxInitExtensions failed!");

    physx::PxSceneDesc scene_desc(physics_->getTolerancesScale());
    scene_desc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
    dispatcher_ = physx::PxDefaultCpuDispatcherCreate(2);
    scene_desc.cpuDispatcher	= dispatcher_;
    scene_desc.filterShader	= physx::PxDefaultSimulationFilterShader;
    scene_desc.kineKineFilteringMode = physx::PxPairFilteringMode::eKEEP; // So kin-kin contacts with be reported
    scene_desc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP; // So static-kin constacts will be reported
    scene_desc.simulationEventCallback = &event_cb_;
    scene_desc.filterShader	= contactReportFilterShader;
    scene_ = physics_->createScene(scene_desc);

    physx::PxPvdSceneClient* pvdClient = scene_->getScenePvdClient();
    if(pvdClient)
    {
      pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
      pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
      pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    material_ = physics_->createMaterial(0.5f, 0.5f, 0.6f);
  }
};

void setupFiltering(TesseractPhysx& phy,
                    physx::PxRigidActor* actor,
                    physx::PxU32 filterGroup,
                    physx::PxU32 filterMask)
{
    physx::PxFilterData filterData;
    filterData.word0 = filterGroup; // word0 = own ID
    filterData.word1 = filterMask;  // word1 = ID mask to filter pairs that trigger a
                                    // contact callback;
    const physx::PxU32 numShapes = actor->getNbShapes();
    physx::PxShape** shapes = (physx::PxShape**)phy.getAllocator().allocate(sizeof(physx::PxShape*)*numShapes, nullptr, __FILE__, __LINE__);
    actor->getShapes(shapes, numShapes);
    for(physx::PxU32 i = 0; i < numShapes; i++)
    {
        physx::PxShape* shape = shapes[i];
        shape->setSimulationFilterData(filterData);
    }
    phy.getAllocator().deallocate(shapes);
}

int main(int, const char*const*)
{
  TesseractPhysx phy;

  std::string link_name = "static_link";
  physx::PxTransform global_tf;
  global_tf.p = physx::PxVec3(0, 0, 0);
  global_tf.q = physx::PxQuat(0, 0, 1, 0);

  physx::PxRigidStatic* geom = physx::PxCreateStatic(*phy.getPhysics(), global_tf, physx::PxBoxGeometry(physx::PxReal(0.5), physx::PxReal(0.5), physx::PxReal(0.5)), *phy.getMaterial());
  geom->setName(link_name.c_str());
  std::printf("%s\n", geom->getName());
  setupFiltering(phy, geom, static_cast<physx::PxU32>(FilterGroup::eSTATIC), static_cast<physx::PxU32>(FilterGroup::eKINEMATIC));
  phy.getScene()->addActor(*geom);

  physx::PxTransform kin_global_tf, kin_shape_tf;
  kin_global_tf.p = physx::PxVec3(0.2f, 0, 0);
  kin_global_tf.q = physx::PxQuat(0, 0, 1, 0);
  kin_shape_tf.p = physx::PxVec3(0, 0, 0);
  kin_shape_tf.q = physx::PxQuat(0, 0, 1, 0);
  physx::PxRigidDynamic* kinematic_sphere = physx::PxCreateKinematic(*phy.getPhysics(),
                                                                     kin_global_tf,
                                                                     physx::PxSphereGeometry(physx::PxReal(0.25)),
                                                                     *phy.getMaterial(),
                                                                     physx::PxReal(0.1),
                                                                     kin_shape_tf);
  kinematic_sphere->setName("kinematic_sphere");

  setupFiltering(phy, kinematic_sphere, static_cast<physx::PxU32>(FilterGroup::eKINEMATIC), static_cast<physx::PxU32>(FilterGroup::eSTATIC));
  phy.getScene()->addActor(*kinematic_sphere);

  for (int i = 0; i < 5; ++i)
  {
    physx::PxTransform update_tf;
    update_tf.p = physx::PxVec3(0, 0, 0.7f - ((physx::PxReal(i) + 1.f) * 0.1f));
    update_tf.q = physx::PxQuat(0, 0, 1, 0);
    kinematic_sphere->setKinematicTarget(update_tf);

    std::printf("%d!\n", i);
    phy.getScene()->simulate(0.0001f);
    phy.getScene()->fetchResults(true);
  }
  std::printf("Done!\n");
  return 0;
}
