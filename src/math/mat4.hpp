#pragma once

#include <xmmintrin.h>
#include <emmintrin.h>
#include "vec3.hpp"
#include "vec4.hpp"

namespace math
{

struct mat4
{
    union
    {
        __m128 col[4];
        float  e[4][4];  // e[c][r]: column c, row r — column-major
    };

    // Default: identity
    mat4()
    {
        col[0] = _mm_set_ps(0.f, 0.f, 0.f, 1.f);
        col[1] = _mm_set_ps(0.f, 0.f, 1.f, 0.f);
        col[2] = _mm_set_ps(0.f, 1.f, 0.f, 0.f);
        col[3] = _mm_set_ps(1.f, 0.f, 0.f, 0.f);
    }

    // Diagonal constructor: mat4(1.f) = identity
    explicit mat4(float diag)
    {
        col[0] = _mm_set_ps(0.f, 0.f, 0.f, diag);
        col[1] = _mm_set_ps(0.f, 0.f, diag, 0.f);
        col[2] = _mm_set_ps(0.f, diag, 0.f, 0.f);
        col[3] = _mm_set_ps(diag, 0.f, 0.f, 0.f);
    }
};

inline vec4 operator*(const mat4& m, const vec4& v)
{
    __m128 result = _mm_mul_ps(m.col[0], _mm_shuffle_ps(v.m, v.m, _MM_SHUFFLE(0,0,0,0)));
    result = _mm_add_ps(result, _mm_mul_ps(m.col[1], _mm_shuffle_ps(v.m, v.m, _MM_SHUFFLE(1,1,1,1))));
    result = _mm_add_ps(result, _mm_mul_ps(m.col[2], _mm_shuffle_ps(v.m, v.m, _MM_SHUFFLE(2,2,2,2))));
    result = _mm_add_ps(result, _mm_mul_ps(m.col[3], _mm_shuffle_ps(v.m, v.m, _MM_SHUFFLE(3,3,3,3))));
    return vec4(result);
}

inline mat4 operator*(const mat4& a, const mat4& b)
{
    mat4 result;
    for (int i = 0; i < 4; i++)
    {
        vec4 c = a * vec4(b.col[i]);
        result.col[i] = c.m;
    }
    return result;
}

inline mat4 transpose(const mat4& m)
{
    mat4 result;
    __m128 tmp0 = _mm_unpacklo_ps(m.col[0], m.col[1]);  // (m00,m10,m01,m11)
    __m128 tmp1 = _mm_unpacklo_ps(m.col[2], m.col[3]);  // (m20,m30,m21,m31)
    __m128 tmp2 = _mm_unpackhi_ps(m.col[0], m.col[1]);  // (m02,m12,m03,m13)
    __m128 tmp3 = _mm_unpackhi_ps(m.col[2], m.col[3]);  // (m22,m32,m23,m33)
    result.col[0] = _mm_movelh_ps(tmp0, tmp1);          // (m00,m10,m20,m30)
    result.col[1] = _mm_movehl_ps(tmp1, tmp0);          // (m01,m11,m21,m31)
    result.col[2] = _mm_movelh_ps(tmp2, tmp3);          // (m02,m12,m22,m32)
    result.col[3] = _mm_movehl_ps(tmp3, tmp2);          // (m03,m13,m23,m33)
    return result;
}

inline mat4 inverse(const mat4& m)
{
    // Scalar Cramer's rule implementation
    // Access elements as p[col*4 + row]
    const float* p = reinterpret_cast<const float*>(m.col);

    float m00 = p[0], m10 = p[1], m20 = p[2],  m30 = p[3];
    float m01 = p[4], m11 = p[5], m21 = p[6],  m31 = p[7];
    float m02 = p[8], m12 = p[9], m22 = p[10], m32 = p[11];
    float m03 = p[12],m13 = p[13],m23 = p[14], m33 = p[15];

    float c00 =  m11*(m22*m33 - m23*m32) - m12*(m21*m33 - m23*m31) + m13*(m21*m32 - m22*m31);
    float c10 = -(m10*(m22*m33 - m23*m32) - m12*(m20*m33 - m23*m30) + m13*(m20*m32 - m22*m30));
    float c20 =  m10*(m21*m33 - m23*m31) - m11*(m20*m33 - m23*m30) + m13*(m20*m31 - m21*m30);
    float c30 = -(m10*(m21*m32 - m22*m31) - m11*(m20*m32 - m22*m30) + m12*(m20*m31 - m21*m30));

    float det = m00*c00 + m01*c10 + m02*c20 + m03*c30;
    float invDet = 1.0f / det;

    float c01 = -(m01*(m22*m33 - m23*m32) - m02*(m21*m33 - m23*m31) + m03*(m21*m32 - m22*m31));
    float c11 =  m00*(m22*m33 - m23*m32) - m02*(m20*m33 - m23*m30) + m03*(m20*m32 - m22*m30);
    float c21 = -(m00*(m21*m33 - m23*m31) - m01*(m20*m33 - m23*m30) + m03*(m20*m31 - m21*m30));
    float c31 =  m00*(m21*m32 - m22*m31) - m01*(m20*m32 - m22*m30) + m02*(m20*m31 - m21*m30);

    float c02 =  m01*(m12*m33 - m13*m32) - m02*(m11*m33 - m13*m31) + m03*(m11*m32 - m12*m31);
    float c12 = -(m00*(m12*m33 - m13*m32) - m02*(m10*m33 - m13*m30) + m03*(m10*m32 - m12*m30));
    float c22 =  m00*(m11*m33 - m13*m31) - m01*(m10*m33 - m13*m30) + m03*(m10*m31 - m11*m30);
    float c32 = -(m00*(m11*m32 - m12*m31) - m01*(m10*m32 - m12*m30) + m02*(m10*m31 - m11*m30));

    float c03 = -(m01*(m12*m23 - m13*m22) - m02*(m11*m23 - m13*m21) + m03*(m11*m22 - m12*m21));
    float c13 =  m00*(m12*m23 - m13*m22) - m02*(m10*m23 - m13*m20) + m03*(m10*m22 - m12*m20);
    float c23 = -(m00*(m11*m23 - m13*m21) - m01*(m10*m23 - m13*m20) + m03*(m10*m21 - m11*m20));
    float c33 =  m00*(m11*m22 - m12*m21) - m01*(m10*m22 - m12*m20) + m02*(m10*m21 - m11*m20);

    // Inverse = cofactor matrix transposed (adjugate) * invDet
    // Result col[c][r] = cofactor[r][c] * invDet
    mat4 result;
    // col[0] = (c00, c10, c20, c30) * invDet
    result.col[0] = _mm_mul_ps(_mm_set_ps(c30, c20, c10, c00), _mm_set1_ps(invDet));
    // col[1] = (c01, c11, c21, c31) * invDet
    result.col[1] = _mm_mul_ps(_mm_set_ps(c31, c21, c11, c01), _mm_set1_ps(invDet));
    // col[2] = (c02, c12, c22, c32) * invDet
    result.col[2] = _mm_mul_ps(_mm_set_ps(c32, c22, c12, c02), _mm_set1_ps(invDet));
    // col[3] = (c03, c13, c23, c33) * invDet
    result.col[3] = _mm_mul_ps(_mm_set_ps(c33, c23, c13, c03), _mm_set1_ps(invDet));
    return result;
}

inline mat4 translate(const mat4& m, vec3 t)
{
    mat4 result = m;
    __m128 tx = _mm_mul_ps(m.col[0], _mm_set1_ps(t.x));
    __m128 ty = _mm_mul_ps(m.col[1], _mm_set1_ps(t.y));
    __m128 tz = _mm_mul_ps(m.col[2], _mm_set1_ps(t.z));
    result.col[3] = _mm_add_ps(_mm_add_ps(_mm_add_ps(tx, ty), tz), m.col[3]);
    return result;
}

inline mat4 scale(const mat4& m, vec3 s)
{
    mat4 result;
    result.col[0] = _mm_mul_ps(m.col[0], _mm_set1_ps(s.x));
    result.col[1] = _mm_mul_ps(m.col[1], _mm_set1_ps(s.y));
    result.col[2] = _mm_mul_ps(m.col[2], _mm_set1_ps(s.z));
    result.col[3] = m.col[3];
    return result;
}

} // namespace math
