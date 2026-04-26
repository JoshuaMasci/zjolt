#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include "zjolt.h"

class ShapeCastCallbackCollisionCollector : public JPH::CollideShapeCollector {
public:
  ShapeCastCallbackCollisionCollector(ShapeCastCallback callback,
                                      void *callback_data,
                                      JPH::BodyInterface &body_interface)
      : body_interface(body_interface) {
    this->callback = callback;
    this->callback_data = callback_data;
  }

  // See: CollectorType::AddHit
  void AddHit(const ResultType &inResult) override {
    ShapeCastHit hit;

    hit.body_id = inResult.mBodyID2.GetIndexAndSequenceNumber();
    hit.body_user_data = this->body_interface.GetUserData(inResult.mBodyID2);
    hit.sub_shape_id = inResult.mSubShapeID2.GetValue();

    this->callback(this->callback_data, hit);
  }

private:
  ShapeCastCallback callback;
  void *callback_data;
  JPH::BodyInterface &body_interface;
};
