#pragma once

#include <xmmintrin.h>
#include <emmintrin.h>
#include <cmath>
#include "vec3.hpp"
#include "mat4.hpp"

namespace math
{

struct quat
{
    union
    {
        __m128 m;
        struct { float x, y, z, w; };
    };

    // Identity: memory layout [x,y,z,w] = (0,0,0,1)
    quat() : m(_mm_set_ps(1.f, 0.f, 0.f, 0.f)) {}
    quat(__m128 m_) : m(m_) {}
    // Constructor order matches GLM: quat(w, x, y, z)
    quat(float w_, float x_, float y_, float z_) : m(_mm_set_ps(w_, z_, y_, x_)) {}
};

inline quat operator*(const quat& a, const quat& b)
{
    float rx = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    float ry = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    float rz = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    float rw = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    return quat(_mm_set_ps(rw, rz, ry, rx));
}

inline vec3 operator*(const quat& q, const vec3& v)
{
    // Rodrigues: t = 2 * cross(q.xyz, v); v' = v + q.w * t + cross(q.xyz, t)
    vec3 qxyz(q.x, q.y, q.z);
    vec3 t = cross(qxyz, v) * 2.0f;
    return v + q.w * t + cross(qxyz, t);
}

inline quat normalize(const quat& q)
{
    float len = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    float inv = 1.0f / len;
    return quat(_mm_mul_ps(q.m, _mm_set1_ps(inv)));
}

inline mat4 toMat4(const quat& q)
{
    float xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
    float xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
    float wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;

    mat4 result;
    // col[0] = (1-2(yy+zz), 2(xy+wz), 2(xz-wy), 0)
    result.col[0] = _mm_set_ps(0.f, 2.f*(xz-wy), 2.f*(xy+wz), 1.f-2.f*(yy+zz));
    // col[1] = (2(xy-wz), 1-2(xx+zz), 2(yz+wx), 0)
    result.col[1] = _mm_set_ps(0.f, 2.f*(yz+wx), 1.f-2.f*(xx+zz), 2.f*(xy-wz));
    // col[2] = (2(xz+wy), 2(yz-wx), 1-2(xx+yy), 0)
    result.col[2] = _mm_set_ps(0.f, 1.f-2.f*(xx+yy), 2.f*(yz-wx), 2.f*(xz+wy));
    // col[3] = (0, 0, 0, 1)
    result.col[3] = _mm_set_ps(1.f, 0.f, 0.f, 0.f);
    return result;
}

} // namespace math
