#include "zjolt.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/RayCast.h>

#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include "Jolt/Core/JobSystemSingleThreaded.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#include "Jolt/Physics/Collision/Shape/CapsuleShape.h"
#include "Jolt/Physics/Collision/Shape/ConvexHullShape.h"
#include "Jolt/Physics/Collision/Shape/CylinderShape.h"
#include "Jolt/Physics/Collision/Shape/MeshShape.h"
#include "Jolt/Physics/Collision/Shape/SphereShape.h"
#include "Jolt/Physics/Collision/Shape/StaticCompoundShape.h"

#include "collision_collector.hpp"
#include "layer_filters.hpp"
#include "zjolt_math.hpp"

// Override new and free
void *operator new(size_t size) {
  if (void *ptr = JPH::Allocate(size))
    return ptr;
  throw std::bad_alloc();
}

void operator delete(void *ptr) noexcept { JPH::Free(ptr); }

// Types
struct Shape {
  JPH::Ref<JPH::Shape> ref;
  UserData user_data;
};

struct World {
  JPH::PhysicsSystem *physics_system;
  BroadPhaseLayerInterfaceImpl *broad_phase_layer_interface;
  ObjectVsBroadPhaseLayerFilterImpl *object_vs_broad_phase_layer_filter;
  AnyMatchObjectLayerPairFilter *object_vs_object_layer_filter;
};

// Global Variables

// TODO: Custom temp allocator?
JPH::TempAllocatorImplWithMallocFallback *temp_allocator;

// TODO: Implement custom pool that uses zig's thread pool
JPH::JobSystemThreadPool *mt_job_system;
JPH::JobSystemSingleThreaded *st_job_system;

void init(const AllocationFunctions *functions, uint32_t temp_allocation_size,
          uint16_t threads) {
  if (functions != nullptr) {
    JPH::Allocate = functions->alloc;
    JPH::Free = functions->free;
    JPH::AlignedAllocate = functions->aligned_alloc;
    JPH::AlignedFree = functions->aligned_free;
    JPH::Reallocate = functions->realloc;
  } else {
    JPH::RegisterDefaultAllocator();
  }

  auto factory = new JPH::Factory();
  JPH::Factory::sInstance = factory;
  JPH::RegisterTypes();

  temp_allocator =
      new JPH::TempAllocatorImplWithMallocFallback(temp_allocation_size);

  if (threads == 0 || threads == 1) {
    mt_job_system = new JPH::JobSystemThreadPool(1024, threads, threads);
    st_job_system = nullptr;
  } else {
    mt_job_system = nullptr;
    st_job_system = new JPH::JobSystemSingleThreaded();
  }
}

void deinit() {
  delete mt_job_system;
  delete st_job_system;
  delete temp_allocator;

  JPH::UnregisterTypes();
  JPH::Factory::sInstance->~Factory();
  JPH::Free((void *)JPH::Factory::sInstance);
  JPH::Factory::sInstance = nullptr;

  JPH::Allocate = nullptr;
  JPH::Free = nullptr;
  JPH::AlignedAllocate = nullptr;
  JPH::AlignedFree = nullptr;
}

// Shape functions
Shape *shapeCreateSphere(float radius, float density, UserData user_data) {
  Shape *shape_ptr = new Shape;
  shape_ptr->user_data = user_data;

  auto settings = JPH::SphereShapeSettings();
  settings.mRadius = radius;
  settings.mDensity = density;
  settings.mUserData = (uint64_t)shape_ptr;
  shape_ptr->ref = settings.Create().Get();

  return shape_ptr;
}

Shape *shapeCreateBox(const Vec3 half_extent, float density,
                      UserData user_data) {
  Shape *shape_ptr = new Shape;
  shape_ptr->user_data = user_data;

  auto settings = JPH::BoxShapeSettings();
  settings.mHalfExtent = loadVec3(half_extent);
  settings.mDensity = density;
  settings.mUserData = (uint64_t)shape_ptr;
  shape_ptr->ref = settings.Create().Get();

  return shape_ptr;
}

Shape *shapeCreateCylinder(float half_height, float radius, float density,
                           UserData user_data) {
  Shape *shape_ptr = new Shape;
  shape_ptr->user_data = user_data;

  auto settings = JPH::CylinderShapeSettings();
  settings.mHalfHeight = half_height;
  settings.mRadius = radius;
  settings.mDensity = density;
  settings.mUserData = (uint64_t)shape_ptr;
  shape_ptr->ref = settings.Create().Get();

  return shape_ptr;
}

Shape *shapeCreateCapsule(float half_height, float radius, float density,
                          UserData user_data) {
  Shape *shape_ptr = new Shape;
  shape_ptr->user_data = user_data;

  auto settings = JPH::CapsuleShapeSettings();
  settings.mHalfHeightOfCylinder = half_height;
  settings.mRadius = radius;
  settings.mDensity = density;
  settings.mUserData = (uint64_t)shape_ptr;
  shape_ptr->ref = settings.Create().Get();

  return shape_ptr;
}

Shape *shapeCreateConvexHull(const Vec3 *positions, size_t position_count,
                             float density, UserData user_data) {
  Shape *shape_ptr = new Shape;
  shape_ptr->user_data = user_data;

  JPH::Array<JPH::Vec3> point_list(position_count);
  for (size_t i = 0; i < position_count; i++) {
    point_list[i] = loadVec3(positions[i]);
  }
  auto settings = JPH::ConvexHullShapeSettings();
  settings.mPoints = point_list;
  settings.mDensity = density;
  settings.mUserData = (uint64_t)shape_ptr;
  shape_ptr->ref = settings.Create().Get();

  return shape_ptr;
}

Shape *shapeCreateMesh(const Vec3 *positions, size_t position_count,
                       const uint32_t *indices, size_t index_count,
                       UserData user_data) {
  Shape *shape_ptr = new Shape;
  shape_ptr->user_data = user_data;

  JPH::VertexList vertex_list;
  for (size_t i = 0; i < position_count; i++) {
    vertex_list.push_back(loadFloat3(positions[i]));
  }

  JPH::IndexedTriangleList triangle_list;
  if (index_count == 0) {
    for (int i = 0; i < position_count; i += 3) {
      triangle_list.push_back(JPH::IndexedTriangle(i + 0, i + 1, i + 2, 0));
    }
  } else {
    const size_t triangle_count = index_count / 3;
    for (int i = 0; i < triangle_count; i++) {
      const size_t offset = i * 3;
      triangle_list.push_back(JPH::IndexedTriangle(
          indices[offset + 0], indices[offset + 1], indices[offset + 2], 0));
    }
  }
  auto settings = JPH::MeshShapeSettings(vertex_list, triangle_list);
  settings.mUserData = (uint64_t)shape_ptr;
  shape_ptr->ref = settings.Create().Get();

  return shape_ptr;
}

Shape *shapeCreateCompound(const SubShapeSettings *sub_shapes,
                           size_t sub_shape_count, UserData user_data) {
  Shape *shape_ptr = new Shape;

  auto settings = JPH::StaticCompoundShapeSettings();
  settings.mUserData = (uint64_t)shape_ptr;

  for (size_t i = 0; i < sub_shape_count; i++) {
    auto &sub_shape = sub_shapes[i];
    settings.AddShape(loadRVec3(sub_shape.position),
                      loadQuat(sub_shape.rotation), sub_shape.shape->ref,
                      sub_shape.userdata);
  }

  shape_ptr->ref = settings.Create().Get();

  return nullptr;
}

void shapeDestroy(Shape *shape) {
  if (shape != nullptr) {
    delete shape;
  }
}

UserData shapeGetUserData(Shape *shape) {
  if (shape != nullptr) {
    return shape->user_data;
  }
  return 0;
}

MassProperties shapeGetMassProperties(Shape *shape) {
  auto shape_properties = shape->ref->GetMassProperties();
  MassProperties mass_properties;
  mass_properties.mass = shape_properties.mMass;
  shape_properties.mInertia.StoreFloat4x4(
      reinterpret_cast<JPH::Float4 *>(mass_properties.inertia_tensor));
  return mass_properties;
}

// World Functions
World *worldCreate(const WorldSettings *settings) {
  World *world = new World();
  world->broad_phase_layer_interface = new BroadPhaseLayerInterfaceImpl();
  world->object_vs_broad_phase_layer_filter =
      new ObjectVsBroadPhaseLayerFilterImpl();
  world->object_vs_object_layer_filter = new AnyMatchObjectLayerPairFilter();
  world->physics_system = new JPH::PhysicsSystem();
  world->physics_system->Init(settings->max_bodies, settings->num_body_mutexes,
                              settings->max_body_pairs,
                              settings->max_contact_constraints,
                              *world->broad_phase_layer_interface,
                              *world->object_vs_broad_phase_layer_filter,
                              *world->object_vs_object_layer_filter);
  world->physics_system->SetGravity(loadVec3(settings->gravity));

  return world;
}

void worldDestroy(World *world) {
  if (world == nullptr)
    return;
  delete world->physics_system;
  delete world->broad_phase_layer_interface;
  delete world->object_vs_broad_phase_layer_filter;
  delete world->object_vs_object_layer_filter;
  delete world;
}

WorldUpdateError worldUpdate(World *world, float delta_time,
                             int collision_steps) {
  JPH::JobSystem *job_system = mt_job_system;
  if (job_system != nullptr) {
    job_system = st_job_system;
  }

  auto error = world->physics_system->Update(delta_time, collision_steps,
                                             temp_allocator, job_system);
  return (WorldUpdateError)error;
}

JPH::BodyCreationSettings getJoltBodySettings(const BodySettings *settings) {
  auto result = JPH::BodyCreationSettings();

  if (settings->shape != nullptr) {
    result.SetShape(settings->shape->ref);
  }

  result.mPosition = loadRVec3(settings->position);
  result.mRotation = loadQuat(settings->rotation);
  result.mLinearVelocity = loadVec3(settings->linear_velocity);
  result.mAngularVelocity = loadVec3(settings->angular_velocity);

  result.mUserData = settings->user_data;
  result.mObjectLayer = settings->object_layer;
  result.mMotionType = (JPH::EMotionType)settings->motion_type;
  result.mIsSensor = false;
  result.mAllowSleeping = settings->allow_sleep;
  result.mFriction = settings->friction;
  result.mGravityFactor = settings->gravity_factor;
  result.mLinearDamping = settings->linear_damping;
  result.mAngularDamping = settings->angular_damping;
  result.mMaxLinearVelocity = settings->max_linear_velocity;
  result.mMaxAngularVelocity = settings->max_angular_velocity;

  return result;
}

// BodyInterface Functions
BodyID worldCreateBody(World *world, const BodySettings *settings) {
  world->physics_system->GetBodyInterface().CreateBody(
      getJoltBodySettings(settings));
  return INVALID_BODY_ID;
}

BodyID worldCreateAndAddBody(World *world, const BodySettings *settings,
                             Activation activation) {
  world->physics_system->GetBodyInterface().CreateAndAddBody(
      getJoltBodySettings(settings), (JPH::EActivation)activation);
  return INVALID_BODY_ID;
}

void worldRemoveBody(World *world, BodyID body_id) {
  world->physics_system->GetBodyInterface().RemoveBody(JPH::BodyID(body_id));
}

void worldAddBody(World *world, BodyID body_id, Activation activation) {
  world->physics_system->GetBodyInterface().AddBody(
      JPH::BodyID(body_id), (JPH::EActivation)activation);
}

void worldDestroyBody(World *world, BodyID body_id) {
  world->physics_system->GetBodyInterface().DestroyBody(JPH::BodyID(body_id));
}

bool worldIsBodyAdded(const World *world, BodyID body_id) {
  return world->physics_system->GetBodyInterface().IsAdded(
      JPH::BodyID(body_id));
}

bool worldIsBodyActive(const World *world, BodyID body_id) {
  return world->physics_system->GetBodyInterface().IsActive(
      JPH::BodyID(body_id));
}

void worldActivateBody(World *world, BodyID body_id) {
  world->physics_system->GetBodyInterface().ActivateBody(JPH::BodyID(body_id));
}

void worldDeactivateBody(World *world, BodyID body_id) {
  world->physics_system->GetBodyInterface().DeactivateBody(
      JPH::BodyID(body_id));
}

ReturnRVec3 worldGetBodyPosition(const World *world, BodyID body_id) {
  ReturnRVec3 result;
  storeRVec3(world->physics_system->GetBodyInterface().GetPosition(
                 JPH::BodyID(body_id)),
             result.data);
  return result;
}

void worldGetBodyPosition(World *world, BodyID body_id, const RVec3 position,
                          Activation activation) {
  world->physics_system->GetBodyInterface().SetPosition(
      JPH::BodyID(body_id), loadRVec3(position), (JPH::EActivation)activation);
}

ReturnRVec3 worldGetBodyCenterOfMassPosition(const World *world,
                                             BodyID body_id) {
  ReturnRVec3 result;
  storeRVec3(world->physics_system->GetBodyInterface().GetCenterOfMassPosition(
                 JPH::BodyID(body_id)),
             result.data);
  return result;
}

ReturnQuat worldGetBodyRotation(const World *world, BodyID body_id) {
  ReturnQuat result;
  storeQuat(world->physics_system->GetBodyInterface().GetRotation(
                JPH::BodyID(body_id)),
            result.data);
  return result;
}

void worldSetBodyRotation(World *world, BodyID body_id, const Quat rotation,
                          Activation activation) {
  world->physics_system->GetBodyInterface().SetRotation(
      JPH::BodyID(body_id), loadQuat(rotation), (JPH::EActivation)activation);
}

Transform worldGetBodyPositionAndRotation(World *world, BodyID body_id) {
  Transform result;
  JPH::RVec3 position;
  JPH::Quat rotation;
  world->physics_system->GetBodyInterface().GetPositionAndRotation(
      JPH::BodyID(body_id), position, rotation);
  storeRVec3(position, result.position);
  storeQuat(rotation, result.rotation);
  return result;
}

void worldSetBodyPositionAndRotation(World *world, BodyID body_id,
                                     const Transform *transform,
                                     Activation activation) {
  JPH::RVec3 position = loadRVec3(transform->position);
  JPH::Quat rotation = loadQuat(transform->rotation);
  world->physics_system->GetBodyInterface().SetPositionAndRotation(
      JPH::BodyID(body_id), position, rotation, (JPH::EActivation)activation);
}

void worldSetBodyPositionAndRotationWhenChanged(World *world, BodyID body_id,
                                                const Transform *transform,
                                                Activation activation) {
  JPH::RVec3 position = loadRVec3(transform->position);
  JPH::Quat rotation = loadQuat(transform->rotation);
  world->physics_system->GetBodyInterface().SetPositionAndRotationWhenChanged(
      JPH::BodyID(body_id), position, rotation, (JPH::EActivation)activation);
}

ReturnVec3 worldGetBodyLinearVelocity(const World *world, BodyID body_id) {
  ReturnVec3 result;
  storeVec3(world->physics_system->GetBodyInterface().GetLinearVelocity(
                JPH::BodyID(body_id)),
            result.data);
  return result;
}

void worldSetBodyLinearVelocity(World *world, BodyID body_id,
                                const Vec3 velocity) {
  world->physics_system->GetBodyInterface().SetLinearVelocity(
      JPH::BodyID(body_id), loadRVec3(velocity));
}

ReturnVec3 worldGetBodyAngularVelocity(const World *world, BodyID body_id) {
  ReturnVec3 result;
  storeVec3(world->physics_system->GetBodyInterface().GetAngularVelocity(
                JPH::BodyID(body_id)),
            result.data);
  return result;
}

void worldSetBodyAngularVelocity(World *world, BodyID body_id,
                                 const Vec3 velocity) {
  world->physics_system->GetBodyInterface().SetAngularVelocity(
      JPH::BodyID(body_id), loadVec3(velocity));
}

ReturnVec3 worldGetBodyPointVelocity(const World *world, BodyID body_id,
                                     const RVec3 point) {
  ReturnVec3 result;
  storeVec3(world->physics_system->GetBodyInterface().GetPointVelocity(
                JPH::BodyID(body_id), loadRVec3(point)),
            result.data);
  return result;
}

Velocity worldGetBodyLinearAndAngularVelocity(World *world, BodyID body_id) {
  Velocity result;
  JPH::Vec3 linear, angular;
  world->physics_system->GetBodyInterface().GetLinearAndAngularVelocity(
      JPH::BodyID(body_id), linear, angular);
  storeVec3(linear, result.linear);
  storeVec3(angular, result.angular);
  return result;
}

void worldSetBodyLinearAndAngularVelocity(World *world, BodyID body_id,
                                          const Velocity *velocity) {
  world->physics_system->GetBodyInterface().SetLinearAndAngularVelocity(
      JPH::BodyID(body_id), loadVec3(velocity->linear),
      loadVec3(velocity->angular));
}

void worldAddBodyForce(World *world, BodyID body_id, const Vec3 force) {
  world->physics_system->GetBodyInterface().AddForce(JPH::BodyID(body_id),
                                                     loadVec3(force));
}

void worldAddBodyForceAtPosition(World *world, BodyID body_id, const Vec3 force,
                                 const RVec3 position) {
  world->physics_system->GetBodyInterface().AddForce(
      JPH::BodyID(body_id), loadVec3(force), loadRVec3(position));
}

void worldAddBodyTorque(World *world, BodyID body_id, const Vec3 torque) {
  world->physics_system->GetBodyInterface().AddTorque(JPH::BodyID(body_id),
                                                      loadVec3(torque));
}

void worldAddBodyImpulse(World *world, BodyID body_id, const Vec3 impulse) {
  world->physics_system->GetBodyInterface().AddImpulse(JPH::BodyID(body_id),
                                                       loadVec3(impulse));
}

void worldAddBodyImpulseAtPosition(World *world, BodyID body_id,
                                   const Vec3 impulse, const RVec3 position) {
  world->physics_system->GetBodyInterface().AddImpulse(
      JPH::BodyID(body_id), loadVec3(impulse), loadRVec3(position));
}

void worldAddBodyAngularImpulse(World *world, BodyID body_id,
                                const Vec3 impulse) {
  world->physics_system->GetBodyInterface().AddAngularImpulse(
      JPH::BodyID(body_id), loadVec3(impulse));
}

float worldGetBodyFriction(const World *world, BodyID body_id) {
  return world->physics_system->GetBodyInterface().GetFriction(
      JPH::BodyID(body_id));
}

void worldSetBodyFriction(World *world, BodyID body_id, float friction) {
  world->physics_system->GetBodyInterface().SetFriction(JPH::BodyID(body_id),
                                                        friction);
}

float worldGetBodyRestitution(const World *world, BodyID body_id) {
  return world->physics_system->GetBodyInterface().GetRestitution(
      JPH::BodyID(body_id));
}

void worldSetBodyRestitution(World *world, BodyID body_id, float restitution) {
  world->physics_system->GetBodyInterface().SetRestitution(JPH::BodyID(body_id),
                                                           restitution);
}

float worldGetBodyGravityFactor(const World *world, BodyID body_id) {
  return world->physics_system->GetBodyInterface().GetGravityFactor(
      JPH::BodyID(body_id));
}

void worldSetBodyGravityFactor(World *world, BodyID body_id, float factor) {
  world->physics_system->GetBodyInterface().SetGravityFactor(
      JPH::BodyID(body_id), factor);
}

MotionType worldGetBodyMotionType(const World *world, BodyID body_id) {
  return (MotionType)world->physics_system->GetBodyInterface().GetMotionType(
      JPH::BodyID(body_id));
}

void worldSetBodyMotionType(World *world, BodyID body_id,
                            MotionType motion_type, Activation activation) {
  world->physics_system->GetBodyInterface().SetMotionType(
      JPH::BodyID(body_id), (JPH::EMotionType)motion_type,
      (JPH::EActivation)activation);
}

MotionQuality worldGetBodyMotionQuality(const World *world, BodyID body_id) {
  return (MotionQuality)world->physics_system->GetBodyInterface()
      .GetMotionQuality(JPH::BodyID(body_id));
}

void worldSetBodyMotionQuality(World *world, BodyID body_id,
                               MotionQuality quality) {
  world->physics_system->GetBodyInterface().SetMotionQuality(
      JPH::BodyID(body_id), (JPH::EMotionQuality)quality);
}

UserData worldGetBodyUserData(const World *world, BodyID body_id) {
  return world->physics_system->GetBodyInterface().GetUserData(
      JPH::BodyID(body_id));
}

void worldSetBodyUserData(World *world, BodyID body_id, UserData user_data) {
  world->physics_system->GetBodyInterface().SetUserData(JPH::BodyID(body_id),
                                                        user_data);
}

void worldSetBodyShape(World *world, BodyID body_id, Shape *shape,
                       bool update_mass_properties, Activation activation) {
  world->physics_system->GetBodyInterface().SetShape(
      JPH::BodyID(body_id), shape->ref, update_mass_properties,
      (JPH::EActivation)activation);
}

void convertRayHit(RayCastHit *hit_result, JPH::RRayCast &ray,
                   JPH::RayCastResult &hit,
                   const JPH::BodyLockInterfaceLocking &body_lock_interface) {
  {
    JPH::BodyLockRead lock(body_lock_interface, hit.mBodyID);
    if (lock.Succeeded()) {
      const JPH::Body &body = lock.GetBody();

      hit_result->body_id = hit.mBodyID.GetIndexAndSequenceNumber();
      hit_result->body_user_data = body.GetUserData();
      hit_result->sub_shape_id = hit.mSubShapeID2.GetValue();

      auto ray_distance = ray.mDirection * hit.mFraction;
      auto ws_position = ray.mOrigin + ray_distance;
      auto ws_normal =
          body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, ws_position);

      storeRVec3(ws_position, hit_result->ws_position);
      storeVec3(ws_normal, hit_result->ws_normal);
    }
  }
}

bool worldCastRayCloset(World *world, ObjectLayer object_layer_pattern,
                        RVec3 origin, Vec3 direction, RayCastHit *hit_result) {
  JPH::RRayCast ray(loadRVec3(origin), loadVec3(direction));
  JPH::RayCastResult hit;

  JPH::BroadPhaseLayerFilter broad_phase_filter{};
  AnyMatchObjectLayerFilter layer_filter(object_layer_pattern);
  JPH::BodyFilter body_filter{};

  bool has_hit = world->physics_system->GetNarrowPhaseQuery().CastRay(
      ray, hit, broad_phase_filter, layer_filter, body_filter);

  if (has_hit) {
    convertRayHit(hit_result, ray, hit,
                  world->physics_system->GetBodyLockInterface());
  }
  return has_hit;
}

bool worldCastRayClosetIgnoreBody(World *world,
                                  ObjectLayer object_layer_pattern,
                                  BodyID ignore_body, const RVec3 origin,
                                  const Vec3 direction,
                                  RayCastHit *hit_result) {
  JPH::RRayCast ray(loadRVec3(origin), loadVec3(direction));
  JPH::RayCastResult hit;

  JPH::BroadPhaseLayerFilter broad_phase_filter{};
  AnyMatchObjectLayerFilter layer_filter(object_layer_pattern);
  JPH::IgnoreSingleBodyFilter body_filter{JPH::BodyID(ignore_body)};

  bool has_hit = world->physics_system->GetNarrowPhaseQuery().CastRay(
      ray, hit, broad_phase_filter, layer_filter, body_filter);
  if (has_hit) {
    convertRayHit(hit_result, ray, hit,
                  world->physics_system->GetBodyLockInterface());
  }
  return has_hit;
}
void worldCastRayAll(World *world, ObjectLayer object_layer_pattern,
                     const RVec3 origin, const Vec3 direction,
                     RayCastCallback callback, void *callback_data) {}

void worldCastShape(World *world, ObjectLayer object_layer_pattern, Shape shape,
                    const Transform *c_transform, ShapeCastCallback callback,
                    void *callback_data) {
  auto position = loadRVec3(c_transform->position);
  auto center_of_mass_transform = JPH::RMat44::sRotationTranslation(
      loadQuat(c_transform->rotation), position);

  JPH::CollideShapeSettings settings = JPH::CollideShapeSettings();
  ShapeCastCallbackCollisionCollector collector(
      callback, callback_data, world->physics_system->GetBodyInterface());

  world->physics_system->GetNarrowPhaseQuery().CollideShape(
      shape.ref, JPH::Vec3::sReplicate(1.0f), center_of_mass_transform,
      settings, position, collector, {},
      AnyMatchObjectLayerFilter(object_layer_pattern), {}, {});
}
