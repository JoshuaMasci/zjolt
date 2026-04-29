const std = @import("std");

const options = @import("options");

const c = @cImport({
    if (options.use_double_precision) {
        @cDefine("JPH_DOUBLE_PRECISION", 1);
    }

    @cInclude("zjolt.h");
});

pub fn init(allocator: std.mem.Allocator, temp_allocation_size: u32, threads: ?u16) void {
    std.debug.assert(mem_allocator == null);

    mem_allocator = allocator;
    c.init(
        &c.AllocationFunctions{
            .alloc = alloc,
            .free = free,
            .aligned_alloc = alignedAlloc,
            .aligned_free = free,
            .realloc = reallocate,
        },
        temp_allocation_size,
        threads orelse 0,
    );
}
pub fn deinit() void {
    c.deinit();
    mem_allocator = null;
}

pub const DefaultGravity: Vec3 = .{ 0.0, -9.8, 0.0 };

// Types
pub const RVec3 = c.RVec3;
pub const Vec3 = c.Vec3;
pub const Quat = c.Quat;

pub const Transform = c.Transform;
pub const Velocity = c.Velocity;

pub const UserData = c.UserData;
pub const ObjectLayer = c.ObjectLayer;
pub const SubShapeIndex = c.SubShapeIndex;
pub const MassProperties = c.MassProperties;

pub const RayCastHit = c.RayCastHit;
pub const ShapeCastHit = c.ShapeCastHit;
pub const ShapeCastCallback = c.ShapeCastCallback;

pub const SubShapeSettings = struct {
    shape: Shape,
    position: Vec3,
    rotation: Quat,
    user_data: UserData,
};

//Shape namespace for organization
pub const Shape = struct {
    const Self = @This();

    id: c.ShapeID,

    pub fn initSphere(radius: f32, density: f32, user_data: UserData) Shape {
        return .{ .id = c.shapeCreateSphere(radius, density, user_data) };
    }

    pub fn initBox(half_extent: [3]f32, density: f32, user_data: UserData) Shape {
        return .{ .id = c.shapeCreateBox(&half_extent, density, user_data) };
    }

    pub fn initCylinder(half_height: f32, radius: f32, density: f32, user_data: UserData) Shape {
        return .{ .id = c.shapeCreateCylinder(half_height, radius, density, user_data) };
    }

    pub fn initCapsule(half_height: f32, radius: f32, density: f32, user_data: UserData) Shape {
        return .{ .id = c.shapeCreateCapsule(half_height, radius, density, user_data) };
    }

    pub fn initConvexHull(positions: [][3]f32, density: f32, user_data: UserData) Shape {
        return .{ .id = c.shapeCreateConvexHull(
            @ptrCast(@alignCast(positions.ptr)),
            positions.len,
            density,
            user_data,
        ) };
    }

    pub fn initMesh(positions: [][3]f32, indices: []u32, user_data: UserData) Shape {
        return .{ .id = c.shapeCreateMesh(
            @ptrCast(@alignCast(positions.ptr)),
            positions.len,
            @ptrCast(@alignCast(indices.ptr)),
            indices.len,
            user_data,
        ) };
    }

    pub fn initCompound(sub_shapes: []const SubShapeSettings, user_data: UserData) Shape {
        const sub_shapes_c = mem_allocator.?.alloc(c.SubShapeSettings, sub_shapes.len) catch @panic("Failed to allocate memory");
        defer mem_allocator.?.free(sub_shapes_c);

        for (sub_shapes_c, sub_shapes) |*sub_c, sub| {
            sub_c.* = .{
                .shape = sub.shape.id,
                .position = sub.position,
                .rotation = sub.rotation,
                .user_data = sub.user_data,
            };
        }
        return .{ .id = c.shapeCreateCompound(sub_shapes_c.ptr, sub_shapes_c.len, user_data) };
    }

    pub fn deinit(self: Self) void {
        c.shapeDestroy(self.id);
    }

    pub fn getUserData(self: Self) UserData {
        return c.shapeGetUserData(self.id);
    }

    pub fn getMassProperties(self: Self) MassProperties {
        return c.shapeGetMassProperties(self.id);
    }
};

pub const BodyID = enum(c.BodyID) { invalid = c.INVALID_BODY_ID, _ };

pub const MotionType = enum(u32) {
    static = 0,
    kinematic = 1,
    dynamic = 2,
};

pub const Activation = enum(u32) {
    activate = 0,
    dont_activate = 1,
};

pub const MotionQuality = enum(u32) {
    discrete = 0,
    linear_cast = 1,
};

pub const BodySettings = struct {
    shape: Shape,
    position: RVec3 = .{ 0, 0, 0 },
    rotation: Quat = .{ 0, 0, 0, 1 },
    linear_velocity: Vec3 = .{ 0, 0, 0 },
    angular_velocity: Vec3 = .{ 0, 0, 0 },
    user_data: UserData = 0,
    object_layer: ObjectLayer = 0,
    motion_type: MotionType = .static,
    motion_quality: MotionQuality = .discrete,
    is_sensor: bool = false,
    allow_sleep: bool = true,
    friction: f32 = 0.2,
    restitution: f32 = 0.0,
    linear_damping: f32 = 0.0,
    angular_damping: f32 = 0.0,
    gravity_factor: f32 = 1.0,
    max_linear_velocity: f32 = 500.0,
    max_angular_velocity: f32 = std.math.pi * 2.0,

    fn toC(settings: *const @This()) c.BodySettings {
        return .{
            .shape = settings.shape.id,
            .position = settings.position,
            .rotation = settings.rotation,
            .linear_velocity = settings.linear_velocity,
            .angular_velocity = settings.angular_velocity,
            .user_data = settings.user_data,
            .object_layer = settings.object_layer,
            .motion_type = @intFromEnum(settings.motion_type),
            .motion_quality = @intFromEnum(settings.motion_quality),
            .is_sensor = settings.is_sensor,
            .allow_sleep = settings.allow_sleep,
            .friction = settings.friction,
            .restitution = settings.restitution,
            .linear_damping = settings.linear_damping,
            .angular_damping = settings.angular_damping,
            .gravity_factor = settings.gravity_factor,
            .max_linear_velocity = settings.max_linear_velocity,
            .max_angular_velocity = settings.max_angular_velocity,
        };
    }
};

pub const WorldUpdateError = error{
    Manifold_Cache_Full,
    Bodypair_Cache_Full,
    Contact_Constraints_Full,
    Unknown,
};

pub const World = struct {
    pub const Settings = struct {
        max_bodies: u32,
        num_body_mutexes: u32,
        max_body_pairs: u32,
        max_contact_constraints: u32,
        gravity: Vec3,
    };

    const Self = @This();

    ptr: ?*c.World,

    pub fn init(settings: Settings) Self {
        return .{
            .ptr = c.worldCreate(&.{
                .max_bodies = settings.max_bodies,
                .num_body_mutexes = settings.num_body_mutexes,
                .max_body_pairs = settings.max_body_pairs,
                .max_contact_constraints = settings.max_contact_constraints,
                .gravity = settings.gravity,
            }),
        };
    }

    pub fn deinit(self: *Self) void {
        c.worldDestroy(self.ptr);
    }

    pub fn update(self: *Self, delta_time: f32, collisions_steps: i32) WorldUpdateError!void {
        const result = c.worldUpdate(self.ptr, delta_time, collisions_steps);
        return switch (result) {
            0 => {},
            c.MANIFOLD_CACHE_FULL => error.Manifold_Cache_Full,
            c.BODYPAIR_CACHE_FULL => error.Bodypair_Cache_Full,
            c.CONTACT_CONSTRAINTS_FULL => error.Contact_Constraints_Full,
            else => error.Unknown,
        };
    }

    // pub fn castRayClosestIgnoreBody(self: Self, object_layer_pattern: ObjectLayer, ignore_body: Body, origin: RVec3, direction: Vec3) ?c.RayCastHit {
    //     var hit: c.RayCastHit = .{};
    //     return if (c.worldCastRayClosetIgnoreBody(self.ptr, object_layer_pattern, ignore_body.ptr, @ptrCast(&origin[0]), @ptrCast(&direction[0]), &hit))
    //         hit
    //     else
    //         null;
    // }

    // pub fn castShape(self: Self, object_layer_pattern: ObjectLayer, shape: Shape, transform: *const Transform, callback: ShapeCastCallback, callback_data: *anyopaque) void {
    //     c.worldCastShape(self.ptr, object_layer_pattern, shape.handle, transform, callback, callback_data);
    // }

    pub fn createBody(self: *Self, settings: *const BodySettings) BodyID {
        return @enumFromInt(c.worldCreateBody(self.ptr, &settings.toC()));
    }

    pub fn createAndAddBody(self: *Self, settings: *const BodySettings, activation: Activation) BodyID {
        return @enumFromInt(c.worldCreateAndAddBody(self.ptr, &settings.toC(), @intFromEnum(activation)));
    }

    pub fn removeBody(self: *Self, body_id: BodyID) void {
        c.worldRemoveBody(self.ptr, @intFromEnum(body_id));
    }

    pub fn addBody(self: *Self, body_id: BodyID, activation: Activation) void {
        c.worldAddBody(self.ptr, @intFromEnum(body_id), @intFromEnum(activation));
    }

    pub fn destroyBody(self: *Self, body_id: BodyID) void {
        c.worldDestroyBody(self.ptr, @intFromEnum(body_id));
    }

    pub fn isBodyAdded(self: *const Self, body_id: BodyID) bool {
        return c.worldIsBodyAdded(self.ptr, @intFromEnum(body_id));
    }

    pub fn isBodyActive(self: *const Self, body_id: BodyID) bool {
        return c.worldIsBodyActive(self.ptr, @intFromEnum(body_id));
    }

    pub fn activateBody(self: *Self, body_id: BodyID) void {
        c.worldActivateBody(self.ptr, @intFromEnum(body_id));
    }

    pub fn deactivateBody(self: *Self, body_id: BodyID) void {
        c.worldDeactivateBody(self.ptr, @intFromEnum(body_id));
    }

    pub fn getBodyPosition(self: *const Self, body_id: BodyID) RVec3 {
        return c.worldGetBodyPosition(self.ptr, @intFromEnum(body_id)).data;
    }

    pub fn setBodyPosition(self: *Self, body_id: BodyID, position: RVec3, activation: Activation) void {
        c.worldSetBodyPosition(self.ptr, @intFromEnum(body_id), &position, @intFromEnum(activation));
    }

    pub fn getBodyCenterOfMassPosition(self: *const Self, body_id: BodyID) RVec3 {
        return c.worldGetBodyCenterOfMassPosition(self.ptr, @intFromEnum(body_id)).data;
    }

    pub fn getBodyRotation(self: *const Self, body_id: BodyID) Quat {
        return c.worldGetBodyRotation(self.ptr, @intFromEnum(body_id)).data;
    }

    pub fn setBodyRotation(self: *Self, body_id: BodyID, rotation: Quat, activation: Activation) void {
        c.worldSetBodyRotation(self.ptr, @intFromEnum(body_id), rotation, @intFromEnum(activation));
    }

    pub fn getBodyPositionAndRotation(self: *Self, body_id: BodyID) Transform {
        return c.worldGetBodyPositionAndRotation(self.ptr, @intFromEnum(body_id));
    }

    pub fn setBodyPositionAndRotation(self: *Self, body_id: BodyID, transform: *const Transform, activation: Activation) void {
        c.worldSetBodyPositionAndRotation(self.ptr, @intFromEnum(body_id), transform, @intFromEnum(activation));
    }

    pub fn setBodyPositionAndRotationWhenChanged(self: *Self, body_id: BodyID, transform: *const Transform, activation: Activation) void {
        c.worldSetBodyPositionAndRotationWhenChanged(self.ptr, @intFromEnum(body_id), transform, @intFromEnum(activation));
    }

    pub fn getBodyLinearVelocity(self: *const Self, body_id: BodyID) Vec3 {
        return c.worldGetBodyLinearVelocity(self.ptr, @intFromEnum(body_id)).data;
    }

    pub fn setBodyLinearVelocity(self: *Self, body_id: BodyID, velocity: Vec3) void {
        c.worldSetBodyLinearVelocity(self.ptr, @intFromEnum(body_id), &velocity);
    }

    pub fn getBodyAngularVelocity(self: *const Self, body_id: BodyID) Vec3 {
        return c.worldGetBodyAngularVelocity(self.ptr, @intFromEnum(body_id)).data;
    }

    pub fn setBodyAngularVelocity(self: *Self, body_id: BodyID, velocity: Vec3) void {
        c.worldSetBodyAngularVelocity(self.ptr, @intFromEnum(body_id), &velocity);
    }

    pub fn getBodyPointVelocity(self: *const Self, body_id: BodyID, point: RVec3) Vec3 {
        return c.worldGetBodyPointVelocity(self.ptr, @intFromEnum(body_id), &point).data;
    }

    pub fn getBodyLinearAndAngularVelocity(self: *Self, body_id: BodyID) Velocity {
        return c.worldGetBodyLinearAndAngularVelocity(self.ptr, @intFromEnum(body_id));
    }

    pub fn setBodyLinearAndAngularVelocity(self: *Self, body_id: BodyID, velocity: *const Velocity) void {
        c.worldSetBodyLinearAndAngularVelocity(self.ptr, @intFromEnum(body_id), velocity);
    }

    pub fn addBodyForce(self: *Self, body_id: BodyID, force: Vec3) void {
        c.worldAddBodyForce(self.ptr, @intFromEnum(body_id), &force);
    }

    pub fn addBodyForceAtPosition(self: *Self, body_id: BodyID, force: Vec3, position: RVec3) void {
        c.worldAddBodyForceAtPosition(self.ptr, @intFromEnum(body_id), force, position);
    }

    pub fn addBodyTorque(self: *Self, body_id: BodyID, torque: Vec3) void {
        c.worldAddBodyTorque(self.ptr, @intFromEnum(body_id), torque);
    }

    pub fn addBodyImpulse(self: *Self, body_id: BodyID, impulse: Vec3) void {
        c.worldAddBodyImpulse(self.ptr, @intFromEnum(body_id), &impulse);
    }

    pub fn addBodyImpulseAtPosition(self: *Self, body_id: BodyID, impulse: Vec3, position: RVec3) void {
        c.worldAddBodyImpulseAtPosition(self.ptr, @intFromEnum(body_id), impulse, position);
    }

    pub fn addBodyAngularImpulse(self: *Self, body_id: BodyID, impulse: Vec3) void {
        c.worldAddBodyAngularImpulse(self.ptr, @intFromEnum(body_id), &impulse);
    }

    pub fn getBodyFriction(self: *const Self, body_id: BodyID) f32 {
        return c.worldGetBodyFriction(self.ptr, @intFromEnum(body_id));
    }

    pub fn setBodyFriction(self: *Self, body_id: BodyID, friction: f32) void {
        c.worldSetBodyFriction(self.ptr, @intFromEnum(body_id), friction);
    }

    pub fn getBodyRestitution(self: *const Self, body_id: BodyID) f32 {
        return c.worldGetBodyRestitution(self.ptr, @intFromEnum(body_id));
    }

    pub fn setBodyRestitution(self: *Self, body_id: BodyID, restitution: f32) void {
        c.worldSetBodyRestitution(self.ptr, @intFromEnum(body_id), restitution);
    }

    pub fn getBodyGravityFactor(self: *const Self, body_id: BodyID) f32 {
        return c.worldGetBodyGravityFactor(self.ptr, @intFromEnum(body_id));
    }

    pub fn setBodyGravityFactor(self: *Self, body_id: BodyID, factor: f32) void {
        c.worldSetBodyGravityFactor(self.ptr, @intFromEnum(body_id), factor);
    }

    pub fn getBodyMotionType(self: *const Self, body_id: BodyID) MotionType {
        return @enumFromInt(c.worldGetBodyMotionType(self.ptr, @intFromEnum(body_id)));
    }

    pub fn setBodyMotionType(self: *Self, body_id: BodyID, motion_type: MotionType, activation: Activation) void {
        c.worldSetBodyMotionType(self.ptr, @intFromEnum(body_id), @intFromEnum(motion_type), @intFromEnum(activation));
    }

    pub fn getBodyMotionQuality(self: *const Self, body_id: BodyID) MotionQuality {
        return @enumFromInt(c.worldGetBodyMotionQuality(self.ptr, @intFromEnum(body_id)));
    }

    pub fn setBodyMotionQuality(self: *Self, body_id: BodyID, quality: MotionQuality) void {
        c.worldSetBodyMotionQuality(self.ptr, @intFromEnum(body_id), @intFromEnum(quality));
    }

    pub fn getBodyUserData(self: *const Self, body_id: BodyID) UserData {
        return c.worldGetBodyUserData(self.ptr, @intFromEnum(body_id));
    }

    pub fn setBodyUserData(self: *Self, body_id: BodyID, user_data: UserData) void {
        c.worldSetBodyUserData(self.ptr, @intFromEnum(body_id), user_data);
    }

    pub fn setBodyShape(self: *Self, body_id: BodyID, shape: Shape, update_mass_properties: bool, activation: Activation) void {
        c.worldSetBodyShape(self.ptr, @intFromEnum(body_id), shape.id, update_mass_properties, @intFromEnum(activation));
    }

    pub fn castRayClosest(self: Self, object_layer_pattern: ObjectLayer, origin: RVec3, direction: Vec3) ?RayCastHit {
        var hit: c.RayCastHit = .{};
        return if (c.worldCastRayCloset(self.ptr, object_layer_pattern, @ptrCast(&origin[0]), @ptrCast(&direction[0]), &hit))
            hit
        else
            null;
    }

    pub fn castRayClosestIgnoreBody(self: Self, object_layer_pattern: ObjectLayer, ignore_body: BodyID, origin: RVec3, direction: Vec3) ?c.RayCastHit {
        var hit: c.RayCastHit = .{};
        return if (c.worldCastRayClosetIgnoreBody(self.ptr, object_layer_pattern, @intFromEnum(ignore_body), @ptrCast(&origin[0]), @ptrCast(&direction[0]), &hit))
            hit
        else
            null;
    }
};

// Memory Allocation
var mem_allocator: ?std.mem.Allocator = null;
const Metadata = struct {
    size: usize,
    alignment: usize,
};

fn alloc(size: usize) callconv(.c) ?*anyopaque {
    return alignedAlloc(size, 16);
}

fn alignedAlloc(size: usize, alignment: usize) callconv(.c) ?*anyopaque {
    if (mem_allocator) |allocator| {
        const allocation_size = size + alignment;

        const allocation: []u8 = switch (alignment) {
            16 => allocator.alignedAlloc(u8, .@"16", allocation_size),
            32 => allocator.alignedAlloc(u8, .@"32", allocation_size),
            64 => allocator.alignedAlloc(u8, .@"64", allocation_size),
            else => |x| std.debug.panic("Unsupported memory alignment: {}", .{x}),
        } catch return null;

        const data_ptr_int: usize = @as(usize, @intFromPtr(allocation.ptr)) + alignment;
        const data_ptr: *u8 = @ptrFromInt(data_ptr_int);
        const metadata_ptr: *Metadata = @ptrFromInt(data_ptr_int - @sizeOf(Metadata));
        metadata_ptr.size = size;
        metadata_ptr.alignment = alignment;

        return data_ptr;
    }
    return null;
}

fn reallocate(maybe_ptr: ?*anyopaque, old_size: usize, new_size: usize) callconv(.c) ?*anyopaque {
    if (maybe_ptr) |data_ptr| {
        if (mem_allocator) |allocator| {
            const data_ptr_int: usize = @intFromPtr(data_ptr);
            const metadata_ptr: *Metadata = @ptrFromInt(data_ptr_int - @sizeOf(Metadata));
            const allocation_ptr: [*]u8 = @ptrFromInt(data_ptr_int - metadata_ptr.alignment);
            const allocation_size: usize = metadata_ptr.size + metadata_ptr.alignment;

            if (old_size != metadata_ptr.size) {
                std.log.warn("Jolt expected memory size({}) doesn't match memory's metadata({}), there may be a bug in the allocator", .{ old_size, metadata_ptr.size });
            }

            const new_allocation_size = new_size + metadata_ptr.alignment;

            // Try to resize current buffer
            const resized: bool = switch (metadata_ptr.alignment) {
                16 => allocator.resize(@as([*]align(16) u8, @alignCast(allocation_ptr))[0..allocation_size], new_allocation_size),
                32 => allocator.resize(@as([*]align(32) u8, @alignCast(allocation_ptr))[0..allocation_size], new_allocation_size),
                64 => allocator.resize(@as([*]align(64) u8, @alignCast(allocation_ptr))[0..allocation_size], new_allocation_size),
                else => |x| std.debug.panic("Unsupported memory alignment: {}", .{x}),
            };
            if (resized) {
                metadata_ptr.size = new_size;
                return data_ptr;
            }

            // If resize fails, try allocate and memcopy new buffer
            if (alignedAlloc(new_size, metadata_ptr.alignment)) |new_ptr| {
                defer free(maybe_ptr);

                if (new_size != 0) {
                    var old_data_ptr_bytes: [*]u8 = @ptrCast(data_ptr);
                    var new_data_ptr_bytes: [*]u8 = @ptrCast(new_ptr);
                    const copy_size = @min(metadata_ptr.size, new_size);
                    @memcpy(new_data_ptr_bytes[0..copy_size], old_data_ptr_bytes[0..copy_size]);
                    return new_ptr;
                } else {
                    return null;
                }
            }
        }
    }
    return alloc(new_size);
}

fn free(maybe_ptr: ?*anyopaque) callconv(.c) void {
    if (maybe_ptr) |data_ptr| {
        if (mem_allocator) |allocator| {
            const data_ptr_int: usize = @intFromPtr(data_ptr);
            const metadata_ptr: *Metadata = @ptrFromInt(data_ptr_int - @sizeOf(Metadata));
            const allocation_ptr: [*]u8 = @ptrFromInt(data_ptr_int - metadata_ptr.alignment);
            const allocation_size: usize = metadata_ptr.size + metadata_ptr.alignment;

            switch (metadata_ptr.alignment) {
                16 => allocator.free(@as([*]align(16) u8, @alignCast(allocation_ptr))[0..allocation_size]),
                32 => allocator.free(@as([*]align(32) u8, @alignCast(allocation_ptr))[0..allocation_size]),
                64 => allocator.free(@as([*]align(64) u8, @alignCast(allocation_ptr))[0..allocation_size]),
                else => |x| std.debug.panic("Unsupported memory alignment: {}", .{x}),
            }
        }
    }
}

// ─── Tests ─────────────────────────────────────────────────────────────────
const @"1MB" = 1024 * 1024;

test "Global Init No Thread Pool" {
    init(std.testing.allocator, @"1MB", null);
    defer deinit();
}

test "Global Init Thread Pool" {
    const thread_count: u16 = @intCast(try std.Thread.getCpuCount());
    init(std.testing.allocator, @"1MB", thread_count);
    defer deinit();
}
