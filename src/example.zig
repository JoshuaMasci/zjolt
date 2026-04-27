const std = @import("std");

const zjolt = @import("zjolt");

pub fn main() !void {
    var debug_allocator = std.heap.DebugAllocator(.{ .enable_memory_limit = true }).init;
    defer if (debug_allocator.deinit() == .leak) {
        std.log.err("DebugAllocator has a memory leak!", .{});
    };
    const allocator = debug_allocator.allocator();

    const @"10MB": usize = 1024 * 1024 * 10;
    const THREADS: u16 = @intCast(try std.Thread.getCpuCount());
    zjolt.init(allocator, @"10MB", THREADS);
    defer zjolt.deinit();

    const USER_DATA: zjolt.UserData = 13;
    const sphere = zjolt.Shape.initSphere(0.5, 1, USER_DATA);
    defer sphere.deinit();

    std.debug.assert(USER_DATA == sphere.getUserData());

    var world = zjolt.World.init(.{
        .max_bodies = 1024,
        .num_body_mutexes = THREADS,
        .max_body_pairs = 1024,
        .max_contact_constraints = 1024,
        .gravity = zjolt.DefaultGravity,
    });
    defer world.deinit();

    for (0..100) |_| {
        try world.update(1.0 / 60.0, 1);
    }
}
