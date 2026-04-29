#pragma once

#include <Jolt/Jolt.h>

inline JPH::Float3 loadFloat3(const float float3[3]) {
  return {float3[0], float3[1], float3[2]};
}

inline void storeFloat3(const JPH::Float3 &inFloat3, float (&outFloat3)[3]) {
  outFloat3[0] = inFloat3.x;
  outFloat3[1] = inFloat3.y;
  outFloat3[2] = inFloat3.z;
}

inline JPH::Vec3 loadVec3(const float in[3]) {
  return JPH::Vec3(in[0], in[1], in[2]);
}

inline void storeVec3(const JPH::Vec3 &inVec, float (&outVec)[3]) {
  outVec[0] = inVec[0];
  outVec[1] = inVec[1];
  outVec[2] = inVec[2];
}

inline JPH::RVec3 loadRVec3(const JPH::Real in[3]) {
  return JPH::RVec3(in[0], in[1], in[2]);
}

inline void storeRVec3(const JPH::RVec3 &inVec, JPH::Real (&outVec)[3]) {
  outVec[0] = inVec[0];
  outVec[1] = inVec[1];
  outVec[2] = inVec[2];
}

inline JPH::Vec4 loadVec4(const float in[4]) {
  return JPH::Vec4(in[0], in[1], in[2], in[3]);
}

inline void storeVec4(const JPH::Vec4 &inVec, float (&outVec)[4]) {
  outVec[0] = inVec[0];
  outVec[1] = inVec[1];
  outVec[2] = inVec[2];
  outVec[3] = inVec[3];
}

inline JPH::Quat loadQuat(const float in[4]) {
  return JPH::Quat(loadVec4(in).Normalized());
}

inline void storeQuat(const JPH::Quat &inQuat, float (&outQuat)[4]) {
  storeVec4(inQuat.mValue, outQuat);
}

inline void storeMat44(const JPH::Mat44 &inMatrix, Mat44 outMatrix) {
  for (int column = 0; column < 4; ++column) {
    auto columnVec = inMatrix.GetColumn4(column);
    outMatrix[column * 4 + 0] = columnVec[0];
    outMatrix[column * 4 + 1] = columnVec[1];
    outMatrix[column * 4 + 2] = columnVec[2];
    outMatrix[column * 4 + 3] = columnVec[3];
  }
}
