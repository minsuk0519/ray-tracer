#pragma once

#include <xmmintrin.h>
#include <emmintrin.h>
#include <cmath>

namespace math
{

struct vec3
{
    union
    {
        __m128 m;
        struct { float x, y, z, _w; };
        float  e[4];
    };

    vec3() : m(_mm_setzero_ps()) {}
    vec3(__m128 m_) : m(m_) {}
    vec3(float x_, float y_, float z_) : m(_mm_set_ps(0.f, z_, y_, x_)) {}
    vec3(float s) : m(_mm_set_ps1(s)) {}

    float& operator[](int i)       { return e[i]; }
    float  operator[](int i) const { return e[i]; }

    vec3& operator+=(const vec3& o) { m = _mm_add_ps(m, o.m); return *this; }
    vec3& operator-=(const vec3& o) { m = _mm_sub_ps(m, o.m); return *this; }
    vec3& operator*=(float s)       { m = _mm_mul_ps(m, _mm_set1_ps(s)); return *this; }
    vec3& operator/=(float s)       { m = _mm_div_ps(m, _mm_set1_ps(s)); return *this; }
};

inline vec3 operator+(vec3 a, vec3 b) { return vec3(_mm_add_ps(a.m, b.m)); }
inline vec3 operator-(vec3 a, vec3 b) { return vec3(_mm_sub_ps(a.m, b.m)); }
inline vec3 operator*(vec3 a, vec3 b) { return vec3(_mm_mul_ps(a.m, b.m)); }
inline vec3 operator/(vec3 a, vec3 b) { return vec3(_mm_div_ps(a.m, b.m)); }
inline vec3 operator*(vec3 a, float s) { return vec3(_mm_mul_ps(a.m, _mm_set1_ps(s))); }
inline vec3 operator*(float s, vec3 a) { return vec3(_mm_mul_ps(_mm_set1_ps(s), a.m)); }
inline vec3 operator/(vec3 a, float s) { return vec3(_mm_div_ps(a.m, _mm_set1_ps(s))); }
inline vec3 operator-(vec3 a)          { return vec3(_mm_sub_ps(_mm_setzero_ps(), a.m)); }

inline float dot(vec3 a, vec3 b)
{
    __m128 mul  = _mm_mul_ps(a.m, b.m);
    __m128 shuf = _mm_shuffle_ps(mul, mul, _MM_SHUFFLE(2,3,0,1));
    __m128 sums = _mm_add_ps(mul, shuf);
    shuf = _mm_movehl_ps(shuf, sums);
    sums = _mm_add_ss(sums, shuf);
    return _mm_cvtss_f32(sums);
}

inline vec3 cross(vec3 a, vec3 b)
{
    __m128 tmp0 = _mm_shuffle_ps(a.m, a.m, _MM_SHUFFLE(3,0,2,1));
    __m128 tmp1 = _mm_shuffle_ps(b.m, b.m, _MM_SHUFFLE(3,1,0,2));
    __m128 tmp2 = _mm_shuffle_ps(a.m, a.m, _MM_SHUFFLE(3,1,0,2));
    __m128 tmp3 = _mm_shuffle_ps(b.m, b.m, _MM_SHUFFLE(3,0,2,1));
    return vec3(_mm_sub_ps(_mm_mul_ps(tmp0, tmp1), _mm_mul_ps(tmp2, tmp3)));
}

inline float length(vec3 v)
{
    return sqrtf(dot(v, v));
}

inline vec3 normalize(vec3 v)
{
    float invLen = 1.0f / sqrtf(dot(v, v));
    return vec3(_mm_mul_ps(v.m, _mm_set1_ps(invLen)));
}

inline vec3 min(vec3 a, vec3 b) { return vec3(_mm_min_ps(a.m, b.m)); }
inline vec3 max(vec3 a, vec3 b) { return vec3(_mm_max_ps(a.m, b.m)); }

} // namespace math
