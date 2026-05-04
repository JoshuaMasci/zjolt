#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ReturnStruct(NAME, TYPE)                                               \
  typedef struct NAME {                                                        \
    TYPE data;                                                                 \
  } NAME

// ─── Allocator ───────────────────────────────────────────────────────────────

typedef void *(*AllocateFunction)(size_t in_size);
typedef void (*FreeFunction)(void *in_block);
typedef void *(*AlignedAllocateFunction)(size_t in_size, size_t in_alignment);
typedef void (*AlignedFreeFunction)(void *in_block);
typedef void *(*ReallocateFunction)(void *in_block, size_t old_size,
                                    size_t new_size);

typedef struct AllocationFunctions {
  AllocateFunction alloc;
  FreeFunction free;
  AlignedAllocateFunction aligned_alloc;
  AlignedFreeFunction aligned_free;
  ReallocateFunction realloc;
} AllocationFunctions;

// ─── Global Fn ───────────────────────────────────────────────────────────────

void init(const AllocationFunctions *functions, uint32_t temp_allocation_size,
          uint16_t threads);
void deinit(void);

// ─── Primitive types ─────────────────────────────────────────────────────────

#ifdef JPH_DOUBLE_PRECISION
typedef double RVec3[3];
#else
typedef float RVec3[3];
#endif

typedef float Vec2[2];
typedef float Vec3[3];
typedef float Vec4[4];
typedef float Quat[4];
typedef float Mat44[16];

// Needed since the "c-abi" doesn't allow array results
ReturnStruct(ReturnVec3, Vec3);
ReturnStruct(ReturnRVec3, RVec3);
ReturnStruct(ReturnQuat, Quat);

typedef struct Transform {
  RVec3 position;
  Quat rotation;
} Transform;

typedef struct Velocity {
  Vec3 linear;
  Vec3 angular;
} Velocity;

// ─── IDs / Handles ───────────────────────────────────────────────────────────

typedef uint32_t BodyID;
typedef uint32_t SoftBodyID;
typedef uint32_t ShapeID;
typedef uint32_t SubShapeID;
typedef uint64_t UserData;
typedef uint16_t ObjectLayer;

#define INVALID_BODY_ID ((BodyID)0xFFFFFFFF)
#define INVALID_SOFT_BODY_ID ((SoftBodyID)0xFFFFFFFF)
#define INVALID_SHAPE ((ShapeID)0xFFFFFFFF)
#define INVALID_SUB_SHAPE ((SubShapeID)0xFFFFFFFF)

// ─── Enums ───────────────────────────────────────────────────────────────────

typedef enum MotionType {
  MOTION_TYPE_STATIC = 0,
  MOTION_TYPE_KINEMATIC = 1,
  MOTION_TYPE_DYNAMIC = 2,
} MotionType;

typedef enum Activation {
  ACTIVATION_ACTIVATE = 0,
  ACTIVATION_DONT_ACTIVATE = 1,
} Activation;

typedef enum MotionQuality {
  MOTION_QUALITY_DISCRETE = 0,
  MOTION_QUALITY_LINEAR_CAST = 1,
} MotionQuality;

typedef enum WorldUpdateError {
  MANIFOLD_CACHE_FULL = 1,
  BODYPAIR_CACHE_FULL = 2,
  CONTACT_CONSTRAINTS_FULL = 3,
} WorldUpdateError;

// ─── Structs ─────────────────────────────────────────────────────────────────

typedef struct World World;

typedef struct WorldSettings {
  uint32_t max_bodies;
  uint32_t num_body_mutexes;
  uint32_t max_body_pairs;
  uint32_t max_contact_constraints;
  Vec3 gravity;
} WorldSettings;

typedef struct MassProperties {
  float mass;
  float inertia_tensor[16];
} MassProperties;

typedef struct BodySettings {
  ShapeID shape;
  RVec3 position;
  Quat rotation;
  Vec3 linear_velocity;
  Vec3 angular_velocity;
  UserData user_data;
  ObjectLayer object_layer;
  MotionType motion_type;
  uint8_t allowed_dofs;
  MotionQuality motion_quality;
  bool is_sensor; // trigger volumes — no contact response
  bool allow_sleep;
  float friction;
  float restitution;
  float linear_damping;
  float angular_damping;
  float gravity_factor;
  float max_linear_velocity;
  float max_angular_velocity;
} BodySettings;

typedef struct SubShapeSettings {
  ShapeID shape;
  Vec3 position;
  Quat rotation;
  UserData user_data;
} SubShapeSettings;

typedef struct RayCastHit {
  BodyID body_id;
  SubShapeID sub_shape_id;
  float distance;
  RVec3 ws_position;
  Vec3 ws_normal;
  UserData body_user_data;
} RayCastHit;

typedef struct ShapeCastHit {
  BodyID body_id;
  SubShapeID sub_shape_id;
  UserData body_user_data;
} ShapeCastHit;

typedef void (*RayCastCallback)(void *user_data, RayCastHit hit);
typedef void (*ShapeCastCallback)(void *user_data, ShapeCastHit hit);

// ─── Shape functions ─────────────────────────────────────────────────────────

ShapeID shapeCreateSphere(float radius, float density, UserData user_data);

ShapeID shapeCreateBox(const Vec3 half_extent, float density,
                       UserData user_data);

ShapeID shapeCreateCylinder(float half_height, float radius, float density,
                            UserData user_data);

ShapeID shapeCreateCapsule(float half_height, float radius, float density,
                           UserData user_data);

ShapeID shapeCreateConvexHull(const Vec3 *positions, size_t position_count,
                              float density, UserData user_data);

ShapeID shapeCreateMesh(const Vec3 *positions, size_t position_count,
                        const uint32_t *indices, size_t index_count,
                        UserData user_data);

ShapeID shapeCreateMeshStride(const void *positions, size_t position_count,
                              size_t position_stride, const uint32_t *indices,
                              size_t index_count, UserData user_data);

ShapeID shapeCreateCompound(const SubShapeSettings *sub_shapes,
                            size_t sub_shape_count, UserData user_data);

void shapeDestroy(ShapeID shape);
UserData shapeGetUserData(ShapeID shape);
MassProperties shapeGetMassProperties(ShapeID shape);

// ─── World functions
// ──────────────────────────────────────────────────────────

World *worldCreate(const WorldSettings *settings);
void worldDestroy(World *world);
WorldUpdateError worldUpdate(World *world, float delta_time,
                             int collision_steps);

BodyID worldCreateBody(World *world, const BodySettings *settings);
BodyID worldCreateAndAddBody(World *world, const BodySettings *settings,
                             Activation activation);
void worldRemoveBody(World *world, BodyID body_id);
void worldAddBody(World *world, BodyID body_id, Activation activation);
void worldDestroyBody(World *world, BodyID body_id);

bool worldIsBodyAdded(const World *world, BodyID body_id);
bool worldIsBodyActive(const World *world, BodyID body_id);

void worldActivateBody(World *world, BodyID body_id);
void worldDeactivateBody(World *world, BodyID body_id);

ReturnRVec3 worldGetBodyPosition(const World *world, BodyID body_id);
ReturnRVec3 worldGetBodyCenterOfMassPosition(const World *world,
                                             BodyID body_id);
ReturnQuat worldGetBodyRotation(const World *world, BodyID body_id);

void worldSetBodyPosition(World *world, BodyID body_id, const RVec3 position,
                          Activation activation);
void worldSetBodyRotation(World *world, BodyID body_id, const Quat rotation,
                          Activation activation);

Transform worldGetBodyPositionAndRotation(World *world, BodyID body_id);
void worldSetBodyPositionAndRotation(World *world, BodyID body_id,
                                     const Transform *transform,
                                     Activation activation);
void worldSetBodyPositionAndRotationWhenChanged(World *world, BodyID body_id,
                                                const Transform *transform,
                                                Activation activation);

ReturnVec3 worldGetBodyLinearVelocity(const World *world, BodyID body_id);
void worldSetBodyLinearVelocity(World *world, BodyID body_id,
                                const Vec3 velocity);

ReturnVec3 worldGetBodyAngularVelocity(const World *world, BodyID body_id);
void worldSetBodyAngularVelocity(World *world, BodyID body_id,
                                 const Vec3 velocity);

ReturnVec3 worldGetBodyPointVelocity(const World *world, BodyID body_id,
                                     const RVec3 point);

Velocity worldGetBodyLinearAndAngularVelocity(World *world, BodyID body_id);
void worldSetBodyLinearAndAngularVelocity(World *world, BodyID body_id,
                                          const Velocity *velocity);

void worldAddBodyForce(World *world, BodyID body_id, const Vec3 force);
void worldAddBodyForceAtPosition(World *world, BodyID body_id, const Vec3 force,
                                 const RVec3 position);
void worldAddBodyTorque(World *world, BodyID body_id, const Vec3 torque);

void worldAddBodyImpulse(World *world, BodyID body_id, const Vec3 impulse);
void worldAddBodyImpulseAtPosition(World *world, BodyID body_id,
                                   const Vec3 impulse, const RVec3 position);
void worldAddBodyAngularImpulse(World *world, BodyID body_id,
                                const Vec3 impulse);

float worldGetBodyFriction(const World *world, BodyID body_id);
void worldSetBodyFriction(World *world, BodyID body_id, float friction);

float worldGetBodyRestitution(const World *world, BodyID body_id);
void worldSetBodyRestitution(World *world, BodyID body_id, float restitution);

float worldGetBodyGravityFactor(const World *world, BodyID body_id);
void worldSetBodyGravityFactor(World *world, BodyID body_id, float factor);

MotionType worldGetBodyMotionType(const World *world, BodyID body_id);
void worldSetBodyMotionType(World *world, BodyID body_id,
                            MotionType motion_type, Activation activation);

MotionQuality worldGetBodyMotionQuality(const World *world, BodyID body_id);
void worldSetBodyMotionQuality(World *world, BodyID body_id,
                               MotionQuality quality);

UserData worldGetBodyUserData(const World *world, BodyID body_id);
void worldSetBodyUserData(World *world, BodyID body_id, UserData user_data);

void worldSetBodyShape(World *world, BodyID body_id, ShapeID shape,
                       bool update_mass_properties, Activation activation);

bool worldCastRayCloset(World *world, ObjectLayer object_layer_pattern,
                        const RVec3 origin, const Vec3 direction,
                        RayCastHit *hit_result);
bool worldCastRayClosetIgnoreBody(World *world,
                                  ObjectLayer object_layer_pattern,
                                  BodyID ignore_body, const RVec3 origin,
                                  const Vec3 direction, RayCastHit *hit_result);
void worldCastRayAll(World *world, ObjectLayer object_layer_pattern,
                     const RVec3 origin, const Vec3 direction,
                     RayCastCallback callback, void *callback_data);
void worldCastShape(World *world, ObjectLayer object_layer_pattern,
                    ShapeID shape, const Transform *c_transform,
                    ShapeCastCallback callback, void *callback_data);

#ifdef __cplusplus
}
#endif
