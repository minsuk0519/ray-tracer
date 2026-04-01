// test_aabb.cpp -- hand-rolled unit tests for AABB.hpp
//
// Build (from repo root):
//   g++ -std=c++2b -I. -Ilibs/glm -o tests/test_aabb tests/test_aabb.cpp
// Run:
//   ./tests/test_aabb

#include "../AABB.hpp"

#include <cstdio>
#include <cmath>

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

#define CHECK(expr)              check((expr),                    #expr,          __FILE__, __LINE__)
#define CHECK_EQ(a, b)           check((a) == (b),               #a " == " #b,   __FILE__, __LINE__)
#define CHECK_NEAR(a, b, eps)    check(fabsf((a)-(b)) <= (eps),  #a " ~= " #b,   __FILE__, __LINE__)

static bool vec3_near(vec3 a, vec3 b, float eps = 1e-5f)
{
    return fabsf(a.x - b.x) <= eps
        && fabsf(a.y - b.y) <= eps
        && fabsf(a.z - b.z) <= eps;
}
#define CHECK_VEC3(a, b)  check(vec3_near((a), (b)), #a " ~= " #b, __FILE__, __LINE__)

// ---------------------------------------------------------------------------
// Tests: default constructor
// ---------------------------------------------------------------------------

static void test_default_constructor()
{
    suite("default constructor (empty AABB)");

    AABB box;

    // An empty AABB must have min > max on every axis (min=+inf, max=-inf).
    CHECK(box.min.x > box.max.x);
    CHECK(box.min.y > box.max.y);
    CHECK(box.min.z > box.max.z);

    // valid() must return false for an empty box.
    CHECK(!box.valid());
}

// ---------------------------------------------------------------------------
// Tests: two-point constructor
// ---------------------------------------------------------------------------

static void test_two_point_constructor()
{
    suite("two-point constructor");

    // Normal case: p1 < p2 on every axis.
    {
        AABB box(vec3(1, 2, 3), vec3(4, 5, 6));
        CHECK_VEC3(box.min, vec3(1, 2, 3));
        CHECK_VEC3(box.max, vec3(4, 5, 6));
        CHECK(box.valid());
    }

    // Inverted input: constructor must normalise regardless of argument order.
    {
        AABB box(vec3(4, 5, 6), vec3(1, 2, 3));
        CHECK_VEC3(box.min, vec3(1, 2, 3));
        CHECK_VEC3(box.max, vec3(4, 5, 6));
        CHECK(box.valid());
    }

    // Mixed axes (some inverted, some not).
    {
        AABB box(vec3(4, 2, 6), vec3(1, 5, 3));
        CHECK_VEC3(box.min, vec3(1, 2, 3));
        CHECK_VEC3(box.max, vec3(4, 5, 6));
        CHECK(box.valid());
    }

    // Degenerate: identical points -> zero-size box, still valid.
    {
        AABB box(vec3(3, 3, 3), vec3(3, 3, 3));
        CHECK_VEC3(box.min, vec3(3, 3, 3));
        CHECK_VEC3(box.max, vec3(3, 3, 3));
        CHECK(box.valid());
    }
}

// ---------------------------------------------------------------------------
// Tests: extend(vec3)
// ---------------------------------------------------------------------------

static void test_extend_point()
{
    suite("extend(vec3)");

    // Extending an empty AABB with one point -> zero-size valid box.
    {
        AABB box;
        box.extend(vec3(1, 2, 3));
        CHECK(box.valid());
        CHECK_VEC3(box.min, vec3(1, 2, 3));
        CHECK_VEC3(box.max, vec3(1, 2, 3));
    }

    // Extending with a second point outside on every axis.
    {
        AABB box;
        box.extend(vec3(1, 2, 3));
        box.extend(vec3(4, 5, 6));
        CHECK_VEC3(box.min, vec3(1, 2, 3));
        CHECK_VEC3(box.max, vec3(4, 5, 6));
    }

    // Extending with a point strictly inside -> box unchanged.
    {
        AABB box(vec3(0, 0, 0), vec3(10, 10, 10));
        box.extend(vec3(5, 5, 5));
        CHECK_VEC3(box.min, vec3( 0,  0,  0));
        CHECK_VEC3(box.max, vec3(10, 10, 10));
    }

    // Extending past one boundary on two axes simultaneously.
    {
        AABB box(vec3(0, 0, 0), vec3(10, 10, 10));
        box.extend(vec3(-5, 5, 15));
        CHECK_VEC3(box.min, vec3(-5,  0,  0));
        CHECK_VEC3(box.max, vec3(10, 10, 15));
    }
}

// ---------------------------------------------------------------------------
// Tests: extend(AABB)
// ---------------------------------------------------------------------------

static void test_extend_aabb()
{
    suite("extend(AABB)");

    // Extending with an identical box -> unchanged.
    {
        AABB box(vec3(1, 1, 1), vec3(3, 3, 3));
        AABB other(vec3(1, 1, 1), vec3(3, 3, 3));
        box.extend(other);
        CHECK_VEC3(box.min, vec3(1, 1, 1));
        CHECK_VEC3(box.max, vec3(3, 3, 3));
    }

    // Extending with a disjoint box -> union of both.
    {
        AABB box(vec3(0, 0, 0), vec3(1, 1, 1));
        AABB other(vec3(2, 2, 2), vec3(3, 3, 3));
        box.extend(other);
        CHECK_VEC3(box.min, vec3(0, 0, 0));
        CHECK_VEC3(box.max, vec3(3, 3, 3));
    }

    // Extending an empty AABB with a valid one -> becomes the valid one.
    {
        AABB empty;
        AABB other(vec3(-1, -2, -3), vec3(4, 5, 6));
        empty.extend(other);
        CHECK(empty.valid());
        CHECK_VEC3(empty.min, vec3(-1, -2, -3));
        CHECK_VEC3(empty.max, vec3( 4,  5,  6));
    }

    // Extending a valid AABB with an empty one -> unchanged.
    {
        AABB box(vec3(1, 1, 1), vec3(5, 5, 5));
        AABB empty;
        box.extend(empty);
        CHECK_VEC3(box.min, vec3(1, 1, 1));
        CHECK_VEC3(box.max, vec3(5, 5, 5));
    }

    // Partially overlapping boxes.
    {
        AABB box(vec3(0, 0, 0), vec3(4, 4, 4));
        AABB other(vec3(2, 2, 2), vec3(6, 6, 6));
        box.extend(other);
        CHECK_VEC3(box.min, vec3(0, 0, 0));
        CHECK_VEC3(box.max, vec3(6, 6, 6));
    }
}

// ---------------------------------------------------------------------------
// Tests: center()
// ---------------------------------------------------------------------------

static void test_center()
{
    suite("center()");

    // Unit cube centred at origin.
    {
        AABB box(vec3(-1, -1, -1), vec3(1, 1, 1));
        CHECK_VEC3(box.center(), vec3(0, 0, 0));
    }

    // Asymmetric box.
    {
        AABB box(vec3(0, 0, 0), vec3(4, 6, 10));
        CHECK_VEC3(box.center(), vec3(2, 3, 5));
    }

    // Single point -> center equals that point.
    {
        AABB box(vec3(7, 7, 7), vec3(7, 7, 7));
        CHECK_VEC3(box.center(), vec3(7, 7, 7));
    }

    // Negative coordinates.
    {
        AABB box(vec3(-10, -6, -4), vec3(-2, -2, 0));
        CHECK_VEC3(box.center(), vec3(-6, -4, -2));
    }
}

// ---------------------------------------------------------------------------
// Tests: half_area()  [= dx*dy + dy*dz + dz*dx]
// ---------------------------------------------------------------------------

static void test_half_area()
{
    suite("half_area()");

    // Unit cube: dx=dy=dz=1  ->  1+1+1 = 3.
    {
        AABB box(vec3(0, 0, 0), vec3(1, 1, 1));
        CHECK_NEAR(box.half_area(), 3.0f, 1e-5f);
    }

    // Non-uniform box: dx=2, dy=3, dz=4  ->  6+12+8 = 26.
    {
        AABB box(vec3(0, 0, 0), vec3(2, 3, 4));
        CHECK_NEAR(box.half_area(), 26.0f, 1e-5f);
    }

    // Flat slab (dz=0): dx=4, dy=2, dz=0  ->  8+0+0 = 8.
    {
        AABB box(vec3(0, 0, 0), vec3(4, 2, 0));
        CHECK_NEAR(box.half_area(), 8.0f, 1e-5f);
    }

    // Single point -> half_area == 0.
    {
        AABB box(vec3(1, 1, 1), vec3(1, 1, 1));
        CHECK_NEAR(box.half_area(), 0.0f, 1e-5f);
    }

    // Inverted constructor input must not change the area.
    {
        AABB box_normal(vec3(0, 0, 0), vec3(2, 3, 4));
        AABB box_inv   (vec3(2, 3, 4), vec3(0, 0, 0));
        CHECK_NEAR(box_normal.half_area(), box_inv.half_area(), 1e-5f);
    }
}

// ---------------------------------------------------------------------------
// Tests: largest_axis()
// ---------------------------------------------------------------------------

static void test_largest_axis()
{
    suite("largest_axis()");

    // X is longest.
    {
        AABB box(vec3(0, 0, 0), vec3(10, 2, 3));
        CHECK_EQ(box.largest_axis(), Axis::AXIS_X);
    }

    // Y is longest.
    {
        AABB box(vec3(0, 0, 0), vec3(1, 9, 2));
        CHECK_EQ(box.largest_axis(), Axis::AXIS_Y);
    }

    // Z is longest.
    {
        AABB box(vec3(0, 0, 0), vec3(3, 4, 8));
        CHECK_EQ(box.largest_axis(), Axis::AXIS_Z);
    }

    // All equal -> any valid axis is acceptable.
    {
        AABB box(vec3(0, 0, 0), vec3(5, 5, 5));
        Axis a = box.largest_axis();
        CHECK(a == Axis::AXIS_X || a == Axis::AXIS_Y || a == Axis::AXIS_Z);
    }

    // Non-uniform box with negative coordinates.
    {
        // dx=10, dy=2, dz=2
        AABB box(vec3(-5, -1, -1), vec3(5, 1, 1));
        CHECK_EQ(box.largest_axis(), Axis::AXIS_X);
    }

    // Y just barely beats X and Z.
    {
        AABB box(vec3(0, 0, 0), vec3(3.0f, 3.1f, 3.0f));
        CHECK_EQ(box.largest_axis(), Axis::AXIS_Y);
    }
}

// ---------------------------------------------------------------------------
// Tests: valid()
// ---------------------------------------------------------------------------

static void test_valid()
{
    suite("valid()");

    // Default-constructed box is invalid.
    {
        AABB box;
        CHECK(!box.valid());
    }

    // Single-point box is valid.
    {
        AABB box(vec3(0, 0, 0), vec3(0, 0, 0));
        CHECK(box.valid());
    }

    // Normal box is valid.
    {
        AABB box(vec3(-1, -2, -3), vec3(1, 2, 3));
        CHECK(box.valid());
    }

    // Empty box becomes valid after first extend(vec3).
    {
        AABB box;
        CHECK(!box.valid());
        box.extend(vec3(1, 2, 3));
        CHECK(box.valid());
    }
}

// ---------------------------------------------------------------------------
// Tests: merge()
// ---------------------------------------------------------------------------

static void test_merge()
{
    suite("merge()");

    // Two disjoint boxes.
    {
        AABB a(vec3(0, 0, 0), vec3(1, 1, 1));
        AABB b(vec3(2, 2, 2), vec3(3, 3, 3));
        AABB m = merge(a, b);
        CHECK_VEC3(m.min, vec3(0, 0, 0));
        CHECK_VEC3(m.max, vec3(3, 3, 3));
        CHECK(m.valid());
    }

    // One box contains the other.
    {
        AABB outer(vec3(-5, -5, -5), vec3(5, 5, 5));
        AABB inner(vec3(-1, -1, -1), vec3(1, 1, 1));
        AABB m = merge(outer, inner);
        CHECK_VEC3(m.min, vec3(-5, -5, -5));
        CHECK_VEC3(m.max, vec3( 5,  5,  5));
    }

    // Merging with an empty AABB -> returns the non-empty one.
    {
        AABB a(vec3(1, 2, 3), vec3(4, 5, 6));
        AABB e;
        AABB m = merge(a, e);
        CHECK(m.valid());
        CHECK_VEC3(m.min, vec3(1, 2, 3));
        CHECK_VEC3(m.max, vec3(4, 5, 6));
    }

    // Merging two empty AABBs -> result is also empty.
    {
        AABB e1, e2;
        AABB m = merge(e1, e2);
        CHECK(!m.valid());
    }

    // Overlapping boxes.
    {
        AABB a(vec3(0, 0, 0), vec3(3, 3, 3));
        AABB b(vec3(1, 1, 1), vec3(5, 5, 5));
        AABB m = merge(a, b);
        CHECK_VEC3(m.min, vec3(0, 0, 0));
        CHECK_VEC3(m.max, vec3(5, 5, 5));
    }

    // Commutativity: merge(a,b) == merge(b,a).
    {
        AABB a(vec3(0, 1, 2), vec3(4, 5, 6));
        AABB b(vec3(-1, 0, 3), vec3(3, 7, 8));
        AABB mab = merge(a, b);
        AABB mba = merge(b, a);
        CHECK_VEC3(mab.min, mba.min);
        CHECK_VEC3(mab.max, mba.max);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    printf("=== AABB unit tests ===\n");

    test_default_constructor();
    test_two_point_constructor();
    test_extend_point();
    test_extend_aabb();
    test_center();
    test_half_area();
    test_largest_axis();
    test_valid();
    test_merge();

    printf("\n==============================\n");
    printf("Results: %d/%d passed", g_passed, g_total);
    if (g_failed > 0)
        printf(", %d FAILED", g_failed);
    printf("\n");

    return (g_failed == 0) ? 0 : 1;
}
