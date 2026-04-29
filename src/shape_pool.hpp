#pragma once

#include "zjolt.h"
#include <Jolt/Core/UnorderedMap.h>

class ShapePool {
private:
  JPH::UnorderedMap<ShapeID, JPH::Ref<JPH::Shape>> pool;
  ShapeID next_id = 0;

public:
  ShapeID insert(const JPH::Ref<JPH::Shape> shape_ref) {
    ShapeID id = this->next_id++;
    this->pool[id] = shape_ref;
    return id;
  }

  JPH::Ref<JPH::Shape> get(ShapeID id) { return this->pool[id]; }

  void remove(ShapeID id) { this->pool.erase(id); }
};
