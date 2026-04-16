#pragma once

#include <xmmintrin.h>
#include <emmintrin.h>
#include "vec3.hpp"

namespace math
{

struct vec4
{
    union
    {
        __m128 m;
        struct { float x, y, z, w; };
        float  e[4];
    };

    vec4() : m(_mm_setzero_ps()) {}
    vec4(__m128 m_) : m(m_) {}
    vec4(float x_, float y_, float z_, float w_) : m(_mm_set_ps(w_, z_, y_, x_)) {}
    vec4(vec3 v, float w_) : m(_mm_set_ps(w_, v.z, v.y, v.x)) {}

    float& operator[](int i)       { return e[i]; }
    float  operator[](int i) const { return e[i]; }
};

inline vec4 operator+(vec4 a, vec4 b) { return vec4(_mm_add_ps(a.m, b.m)); }
inline vec4 operator-(vec4 a, vec4 b) { return vec4(_mm_sub_ps(a.m, b.m)); }
inline vec4 operator*(vec4 a, vec4 b) { return vec4(_mm_mul_ps(a.m, b.m)); }
inline vec4 operator/(vec4 a, vec4 b) { return vec4(_mm_div_ps(a.m, b.m)); }

} // namespace math
