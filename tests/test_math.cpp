// test_math.cpp -- unit tests for src/math/ SIMD math library
//
// Build (from tests/):
//   g++ -std=c++2b -msse2 -I../src -o test_math test_math.cpp
// Run:
//   ./test_math

#include "math/math.hpp"

#include <cstdio>
#include <cmath>

using namespace math;

// ---------------------------------------------------------------------------
// Minimal test framework
// ---------------------------------------------------------------------------

static int g_total  = 0;
static int g_passed = 0;
static int g_failed = 0;

static void suite(const char* name)
{
    printf("\n[%s]\n", name);
}

static void check(bool cond, const char* expr, const char* file, int line)
{
    ++g_total;
    if (cond)
    {
        ++g_passed;
        printf("  PASS  %s\n", expr);
    }
    else
    {
        ++g_failed;
        printf("  FAIL  %s  (%s:%d)\n", expr, file, line);
    }
}

#define CHECK(expr)              check((expr), #expr, __FILE__, __LINE__)
#define CHECK_EQ(a, b)           check((a) == (b), #a " == " #b, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, eps)    check(fabsf((a)-(b)) <= (eps), #a " ~= " #b, __FILE__, __LINE__)

static bool vec3_near(vec3 a, vec3 b, float eps = 1e-5f)
{
    return fabsf(a.x-b.x) <= eps && fabsf(a.y-b.y) <= eps && fabsf(a.z-b.z) <= eps;
}
#define CHECK_VEC3(a, b)      check(vec3_near((a),(b)), #a " ~= " #b, __FILE__, __LINE__)
#define CHECK_VEC3_EPS(a,b,e) check(vec3_near((a),(b),(e)), #a " ~= " #b, __FILE__, __LINE__)

static bool vec4_near(vec4 a, vec4 b, float eps = 1e-5f)
{
    return fabsf(a.x-b.x) <= eps && fabsf(a.y-b.y) <= eps
        && fabsf(a.z-b.z) <= eps && fabsf(a.w-b.w) <= eps;
}
#define CHECK_VEC4(a, b) check(vec4_near((a),(b)), #a " ~= " #b, __FILE__, __LINE__)

static bool mat4_near(const mat4& a, const mat4& b, float eps = 1e-4f)
{
    for (int c = 0; c < 4; c++)
    {
        for (int r = 0; r < 4; r++)
        {
            if (fabsf(a.e[c][r] - b.e[c][r]) > eps)
            {
                return false;
            }
        }
    }
    return true;
}
#define CHECK_MAT4(a, b) check(mat4_near((a),(b)), #a " ~= " #b, __FILE__, __LINE__)

// ---------------------------------------------------------------------------
// vec3 -- construction
// ---------------------------------------------------------------------------

static void test_vec3_construction()
{
    suite("vec3 construction");

    {
        vec3 v(1.f, 2.f, 3.f);
        CHECK_EQ(v.x, 1.f);
        CHECK_EQ(v.y, 2.f);
        CHECK_EQ(v.z, 3.f);
    }

    // uniform constructor
    {
        vec3 v(5.f);
        CHECK_EQ(v.x, 5.f);
        CHECK_EQ(v.y, 5.f);
        CHECK_EQ(v.z, 5.f);
    }

    // default: zero
    {
        vec3 v;
        CHECK_EQ(v.x, 0.f);
        CHECK_EQ(v.y, 0.f);
        CHECK_EQ(v.z, 0.f);
    }

    // array indexing matches named fields
    {
        vec3 v(7.f, 8.f, 9.f);
        CHECK_EQ(v[0], v.x);
        CHECK_EQ(v[1], v.y);
        CHECK_EQ(v[2], v.z);
    }
}

// ---------------------------------------------------------------------------
// vec3 -- arithmetic
// ---------------------------------------------------------------------------

static void test_vec3_arithmetic()
{
    suite("vec3 arithmetic");

    CHECK_VEC3(vec3(1,2,3) + vec3(4,5,6), vec3(5,7,9));
    CHECK_VEC3(vec3(4,5,6) - vec3(1,2,3), vec3(3,3,3));
    CHECK_VEC3(vec3(1,2,3) * vec3(2,3,4), vec3(2,6,12));
    CHECK_VEC3(vec3(2,4,6) * 0.5f,        vec3(1,2,3));
    CHECK_VEC3(0.5f * vec3(2,4,6),        vec3(1,2,3));
    CHECK_VEC3(vec3(2,4,6) / 2.f,         vec3(1,2,3));
    CHECK_VEC3(-vec3(1,2,3),              vec3(-1,-2,-3));

    // compound assign
    {
        vec3 v(1,2,3);
        v += vec3(1,1,1);
        CHECK_VEC3(v, vec3(2,3,4));
    }
    {
        vec3 v(4,5,6);
        v -= vec3(1,2,3);
        CHECK_VEC3(v, vec3(3,3,3));
    }
    {
        vec3 v(1,2,3);
        v *= 2.f;
        CHECK_VEC3(v, vec3(2,4,6));
    }
}

// ---------------------------------------------------------------------------
// vec3 -- min / max
// ---------------------------------------------------------------------------

static void test_vec3_minmax()
{
    suite("vec3 min/max");

    CHECK_VEC3(min(vec3(1,5,2), vec3(3,2,4)), vec3(1,2,2));
    CHECK_VEC3(max(vec3(1,5,2), vec3(3,2,4)), vec3(3,5,4));

    // min/max with negative values
    CHECK_VEC3(min(vec3(-1,0,3), vec3(2,-1,1)), vec3(-1,-1,1));
    CHECK_VEC3(max(vec3(-1,0,3), vec3(2,-1,1)), vec3(2,0,3));
}

// ---------------------------------------------------------------------------
// vec3 -- geometric operations
// ---------------------------------------------------------------------------

static void test_vec3_geometric()
{
    suite("vec3 geometric");

    // dot product
    CHECK_NEAR(dot(vec3(1,0,0), vec3(0,1,0)),    0.f,  1e-6f);
    CHECK_NEAR(dot(vec3(1,0,0), vec3(1,0,0)),    1.f,  1e-6f);
    CHECK_NEAR(dot(vec3(2,3,4), vec3(1,2,3)),    20.f, 1e-5f);
    CHECK_NEAR(dot(vec3(1,2,3), vec3(-1,-2,-3)), -14.f,1e-5f);

    // cross product
    CHECK_VEC3(cross(vec3(1,0,0), vec3(0,1,0)), vec3(0,0,1));
    CHECK_VEC3(cross(vec3(0,1,0), vec3(0,0,1)), vec3(1,0,0));
    CHECK_VEC3(cross(vec3(0,0,1), vec3(1,0,0)), vec3(0,1,0));

    // cross anti-commutativity: cross(a,b) == -cross(b,a)
    {
        vec3 a(1,2,3), b(4,5,6);
        CHECK_VEC3(cross(a,b), -cross(b,a));
    }

    // cross perpendicular to both inputs
    {
        vec3 a(1,2,3), b(4,5,6);
        vec3 c = cross(a,b);
        CHECK_NEAR(dot(c,a), 0.f, 1e-4f);
        CHECK_NEAR(dot(c,b), 0.f, 1e-4f);
    }

    // length
    CHECK_NEAR(length(vec3(3,4,0)), 5.f, 1e-5f);
    CHECK_NEAR(length(vec3(1,0,0)), 1.f, 1e-6f);

    // normalize produces unit vector
    CHECK_NEAR(length(normalize(vec3(1,2,3))), 1.f, 1e-5f);
    CHECK_NEAR(length(normalize(vec3(3,4,0))), 1.f, 1e-5f);
    CHECK_NEAR(length(normalize(vec3(7,0,0))), 1.f, 1e-5f);

    // normalize direction preserved
    {
        vec3 v = normalize(vec3(0,5,0));
        CHECK_VEC3(v, vec3(0,1,0));
    }
}

// ---------------------------------------------------------------------------
// vec4 -- basic
// ---------------------------------------------------------------------------

static void test_vec4_basic()
{
    suite("vec4 basic");

    {
        vec4 v(1,2,3,4);
        CHECK_EQ(v.x, 1.f);
        CHECK_EQ(v.y, 2.f);
        CHECK_EQ(v.z, 3.f);
        CHECK_EQ(v.w, 4.f);
    }

    // from vec3 + w
    {
        vec4 v(vec3(1,2,3), 0.f);
        CHECK_EQ(v.x, 1.f);
        CHECK_EQ(v.y, 2.f);
        CHECK_EQ(v.z, 3.f);
        CHECK_EQ(v.w, 0.f);
    }
    {
        vec4 v(vec3(1,2,3), 1.f);
        CHECK_EQ(v.w, 1.f);
    }

    // array indexing
    {
        vec4 v(1,2,3,4);
        CHECK_EQ(v[0], 1.f);
        CHECK_EQ(v[1], 2.f);
        CHECK_EQ(v[2], 3.f);
        CHECK_EQ(v[3], 4.f);
    }

    // arithmetic
    CHECK_VEC4(vec4(1,2,3,4) + vec4(1,1,1,1), vec4(2,3,4,5));
    CHECK_VEC4(vec4(4,3,2,1) - vec4(1,1,1,1), vec4(3,2,1,0));
}

// ---------------------------------------------------------------------------
// mat4 -- construction and identity
// ---------------------------------------------------------------------------

static void test_mat4_identity()
{
    suite("mat4 identity");

    mat4 I(1.f);

    // identity * point = point
    CHECK_VEC4(I * vec4(1,2,3,1), vec4(1,2,3,1));
    // identity * direction = direction
    CHECK_VEC4(I * vec4(1,2,3,0), vec4(1,2,3,0));
    // identity * identity = identity
    CHECK_MAT4(I * I, I);
}

// ---------------------------------------------------------------------------
// mat4 -- translate and scale
// ---------------------------------------------------------------------------

static void test_mat4_transforms()
{
    suite("mat4 translate / scale");

    // translate: moves points, not directions
    {
        mat4 T = translate(mat4(1.f), vec3(1,2,3));
        CHECK_VEC4(T * vec4(0,0,0,1), vec4(1,2,3,1));
        CHECK_VEC4(T * vec4(1,1,1,1), vec4(2,3,4,1));
        CHECK_VEC4(T * vec4(1,0,0,0), vec4(1,0,0,0)); // direction unchanged
    }

    // scale
    {
        mat4 S = scale(mat4(1.f), vec3(2,3,4));
        CHECK_VEC4(S * vec4(1,1,1,1), vec4(2,3,4,1));
        CHECK_VEC4(S * vec4(1,1,1,0), vec4(2,3,4,0));
    }

    // S * T: scale applied after translate scales the translation column too
    {
        mat4 M = scale(mat4(1.f), vec3(2,2,2)) * translate(mat4(1.f), vec3(1,0,0));
        CHECK_VEC4(M * vec4(0,0,0,1), vec4(2,0,0,1));
    }
}

// ---------------------------------------------------------------------------
// mat4 -- transpose
// ---------------------------------------------------------------------------

static void test_mat4_transpose()
{
    suite("mat4 transpose");

    // transpose of transpose = original
    {
        mat4 M = translate(scale(mat4(1.f), vec3(2,3,4)), vec3(5,6,7));
        CHECK_MAT4(transpose(transpose(M)), M);
    }

    // transpose of identity = identity
    {
        CHECK_MAT4(transpose(mat4(1.f)), mat4(1.f));
    }

    // (AB)^T = B^T * A^T
    {
        mat4 A = translate(mat4(1.f), vec3(1,2,3));
        mat4 B = scale(mat4(1.f), vec3(2,1,3));
        CHECK_MAT4(transpose(A * B), transpose(B) * transpose(A));
    }
}

// ---------------------------------------------------------------------------
// mat4 -- inverse
// ---------------------------------------------------------------------------

static void test_mat4_inverse()
{
    suite("mat4 inverse");

    // M * inverse(M) ~= I
    {
        mat4 M = translate(scale(mat4(1.f), vec3(2,3,4)), vec3(5,6,7));
        CHECK_MAT4(M * inverse(M), mat4(1.f));
    }

    // inverse(I) = I
    {
        CHECK_MAT4(inverse(mat4(1.f)), mat4(1.f));
    }

    // inverse of translate
    {
        mat4 T    = translate(mat4(1.f), vec3(3,4,5));
        mat4 Tinv = inverse(T);
        CHECK_VEC4(Tinv * vec4(3,4,5,1), vec4(0,0,0,1));
    }
}

// ---------------------------------------------------------------------------
// mat4 -- multiply associativity
// ---------------------------------------------------------------------------

static void test_mat4_multiply()
{
    suite("mat4 multiply");

    mat4 A = translate(mat4(1.f), vec3(1,0,0));
    mat4 B = scale(mat4(1.f), vec3(2,2,2));
    mat4 C = translate(mat4(1.f), vec3(0,1,0));

    // (A*B)*C == A*(B*C)
    CHECK_MAT4((A * B) * C, A * (B * C));
}

// ---------------------------------------------------------------------------
// quat -- identity and rotation
// ---------------------------------------------------------------------------

static void test_quat_rotation()
{
    suite("quat rotation");

    const float half = sqrtf(0.5f);

    // identity: no rotation
    {
        quat q(1.f, 0.f, 0.f, 0.f);
        CHECK_VEC3_EPS(q * vec3(1,2,3), vec3(1,2,3), 1e-5f);
    }

    // 90° around Y: (1,0,0) → (0,0,-1)
    {
        quat q(half, 0.f, half, 0.f);
        CHECK_VEC3_EPS(q * vec3(1,0,0), vec3(0,0,-1), 1e-5f);
    }

    // 90° around X: (0,1,0) → (0,0,1)
    {
        quat q(half, half, 0.f, 0.f);
        CHECK_VEC3_EPS(q * vec3(0,1,0), vec3(0,0,1), 1e-5f);
    }

    // 90° around Z: (1,0,0) → (0,1,0)
    {
        quat q(half, 0.f, 0.f, half);
        CHECK_VEC3_EPS(q * vec3(1,0,0), vec3(0,1,0), 1e-5f);
    }

    // 180° around Y: (1,0,0) → (-1,0,0)
    {
        quat q(0.f, 0.f, 1.f, 0.f);
        CHECK_VEC3_EPS(q * vec3(1,0,0), vec3(-1,0,0), 1e-5f);
    }
}

// ---------------------------------------------------------------------------
// quat -- toMat4
// ---------------------------------------------------------------------------

static void test_quat_toMat4()
{
    suite("quat toMat4");

    const float half = sqrtf(0.5f);

    // identity quat → identity matrix
    {
        CHECK_MAT4(toMat4(quat(1,0,0,0)), mat4(1.f));
    }

    // toMat4 applied to vec4 matches quat*vec3
    {
        quat  q(half, 0.f, half, 0.f);  // 90° around Y
        vec3  v(1,0,0);
        vec3  byQuat  = q * v;
        vec4  byMat   = toMat4(q) * vec4(v, 0.f);
        CHECK_VEC3_EPS(byQuat, vec3(byMat.x, byMat.y, byMat.z), 1e-5f);
    }

    // same check for 90° around X
    {
        quat  q(half, half, 0.f, 0.f);
        vec3  v(0,1,0);
        vec3  byQuat  = q * v;
        vec4  byMat   = toMat4(q) * vec4(v, 0.f);
        CHECK_VEC3_EPS(byQuat, vec3(byMat.x, byMat.y, byMat.z), 1e-5f);
    }
}

// ---------------------------------------------------------------------------
// quat -- normalize
// ---------------------------------------------------------------------------

static void test_quat_normalize()
{
    suite("quat normalize");

    quat q(2.f, 0.f, 0.f, 0.f);
    quat n = normalize(q);
    float len = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z + n.w*n.w);
    CHECK_NEAR(len, 1.f, 1e-5f);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    printf("=== math library unit tests ===\n");

    test_vec3_construction();
    test_vec3_arithmetic();
    test_vec3_minmax();
    test_vec3_geometric();
    test_vec4_basic();
    test_mat4_identity();
    test_mat4_transforms();
    test_mat4_transpose();
    test_mat4_inverse();
    test_mat4_multiply();
    test_quat_rotation();
    test_quat_toMat4();
    test_quat_normalize();

    printf("\n==============================\n");
    printf("Results: %d/%d passed", g_passed, g_total);
    if (g_failed > 0)
    {
        printf(", %d FAILED", g_failed);
    }
    printf("\n");

    return (g_failed == 0) ? 0 : 1;
}
