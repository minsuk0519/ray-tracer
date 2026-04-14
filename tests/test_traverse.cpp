// test_traverse.cpp -- full pipeline integration tests for BVH traversal
//
// Build (from tests/):
//   g++ -std=c++2b -I../src/bvh -I../libs/glm -I../libs/assimp/include \
//       -o test_traverse test_traverse.cpp \
//       ../src/bvh/bvh_traverse.cpp \
//       ../src/bvh/bvh_bake.cpp \
//       ../src/bvh/bvh_bfs.cpp \
//       ../src/bvh/bvh_IO.cpp \
//       ../src/bvh/bvh_morton.cpp \
//       ../src/bvh/bvh_sah.cpp \
//       ../src/bvh/bvh_geo.cpp \
//       ../src/bvh/AABB.cpp \
//       -lassimp
// Run:
//   ./test_traverse

#define BAKING
#include "bvh.hpp"
#include "bvh_traverse.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>

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

#define CHECK(expr)           check((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, eps) check(fabsf((a)-(b)) <= (eps), #a " ~= " #b, __FILE__, __LINE__)

// ---------------------------------------------------------------------------
// Setup: write scene file, bake, load
// ---------------------------------------------------------------------------

static const char* SCENE_PATH = "test_scene.scn";
static const char* BAKED_PATH = "test_scene.bvh";

static bool setup()
{
    // Write scene file
    std::ofstream f(SCENE_PATH);
    if (!f.is_open())
    {
        fprintf(stderr, "SETUP FAILED: could not write %s\n", SCENE_PATH);
        return false;
    }
    f << "box 0 0 5 1 1 1 q 0 0 0 1\n";       // Box A
    f << "box 0 0 12 1 1 1 q 0 0 0 1\n";      // Box B
    f << "sphere 0 5 0 2\n";                   // Sphere (default rings/sectors)
    f << "box 0 -3 0 20 0.1 20 q 0 0 0 1\n";  // Large ground plane (triggers spatial splits)
    f.close();

    if (!bvh::bakeBVH(SCENE_PATH))
    {
        fprintf(stderr, "SETUP FAILED: bakeBVH failed\n");
        return false;
    }

    if (!bvh::loadBVH(BAKED_PATH))
    {
        fprintf(stderr, "SETUP FAILED: loadBVH failed\n");
        return false;
    }

    return true;
}

static void cleanup()
{
    std::remove(SCENE_PATH);
    std::remove(BAKED_PATH);
}

// ---------------------------------------------------------------------------
// Helper: build a Ray
// ---------------------------------------------------------------------------

static bvh::Ray makeRay(float ox, float oy, float oz,
                         float dx, float dy, float dz)
{
    bvh::Ray r;
    r.origin = glm::vec3(ox, oy, oz);
    r.dir    = glm::normalize(glm::vec3(dx, dy, dz));
    return r;
}

// ---------------------------------------------------------------------------
// Tests: box hits
// ---------------------------------------------------------------------------

static void test_box_hits()
{
    suite("box hits");

    // Ray 1: (0,0,0) -> (0,0,1) hits Box A front face (z=4), NOT Box B (z=11)
    {
        bvh::Hit h = bvh::trace(makeRay(0,0,0, 0,0,1));
        CHECK(h.hit);
        CHECK(h.t >= 3.9f && h.t <= 4.1f);
        CHECK(h.normal.z < 0.f);
        CHECK(h.t < 11.f);  // Box A occludes Box B
    }

    // Ray 2: (0,0,8) -> (0,0,1) hits Box B front face (z=11), t=3
    {
        bvh::Hit h = bvh::trace(makeRay(0,0,8, 0,0,1));
        CHECK(h.hit);
        CHECK(h.t >= 2.9f && h.t <= 3.1f);
        CHECK(h.normal.z < 0.f);
    }

    // Ray 3: (0,0,15) -> (0,0,-1) hits Box B back face (z=13), t=2
    {
        bvh::Hit h = bvh::trace(makeRay(0,0,15, 0,0,-1));
        CHECK(h.hit);
        CHECK(h.t >= 1.9f && h.t <= 2.1f);
        CHECK(h.normal.z > 0.f);
    }

    // Ray 4: (-5,0,5) -> (1,0,0) hits Box A -X face (x=-1), t=4
    {
        bvh::Hit h = bvh::trace(makeRay(-5,0,5, 1,0,0));
        CHECK(h.hit);
        CHECK(h.t >= 3.9f && h.t <= 4.1f);
        CHECK(h.normal.x < 0.f);
    }

    // Ray 5: (0,0,8) -> (0,0,-1) hits Box A back face (z=6), t=2
    {
        bvh::Hit h = bvh::trace(makeRay(0,0,8, 0,0,-1));
        CHECK(h.hit);
        CHECK(h.t >= 1.9f && h.t <= 2.1f);
        CHECK(h.normal.z > 0.f);
    }
}

// ---------------------------------------------------------------------------
// Tests: misses
// ---------------------------------------------------------------------------

static void test_misses()
{
    suite("misses");

    // Ray 6: x=3 is outside all boxes and sphere (sphere radius=2, center x=0)
    {
        bvh::Hit h = bvh::trace(makeRay(3,0,0, 0,0,1));
        CHECK(!h.hit);
    }

    // Ray 7: y=10 is above everything
    {
        bvh::Hit h = bvh::trace(makeRay(0,10,0, 0,0,1));
        CHECK(!h.hit);
    }

    // Ray 8: (0,0,0) shooting backward (-Z), all geometry is at z>0 or y=5
    {
        bvh::Hit h = bvh::trace(makeRay(0,0,0, 0,0,-1));
        CHECK(!h.hit);
    }
}

// ---------------------------------------------------------------------------
// Tests: sphere
// ---------------------------------------------------------------------------

static void test_sphere()
{
    suite("sphere");

    // Ray 9: (0,0,0) -> (0,1,0) hits sphere bottom (analytic t=3), tessellated so range check
    {
        bvh::Hit h = bvh::trace(makeRay(0,0,0, 0,1,0));
        CHECK(h.hit);
        CHECK(h.t >= 2.5f && h.t <= 3.5f);
        CHECK(h.normal.y < 0.f);
    }

    // Ray 10: (3,0,0) -> (0,1,0) — distance from ray to sphere center = 3 > radius 2, miss
    {
        bvh::Hit h = bvh::trace(makeRay(3,0,0, 0,1,0));
        CHECK(!h.hit);
    }
}

// ---------------------------------------------------------------------------
// Tests: ground plane (spatial splits)
// ---------------------------------------------------------------------------

static void test_ground_spatial_splits()
{
    suite("ground plane / spatial splits");

    // Ray 11: (0,-5,0) -> (0,1,0) hits ground top face (y=-2.9), t=2.1
    // The large triangles (20x20) are virtually guaranteed to trigger spatial splits.
    // If s_triIndex indirection is broken, this ray will miss.
    {
        bvh::Hit h = bvh::trace(makeRay(0,-5,0, 0,1,0));
        CHECK(h.hit);
        CHECK(h.t >= 1.9f && h.t <= 2.2f);
        CHECK(h.normal.y > 0.f);
    }

    // Ray 12: (25,0,0) -> (0,-1,0) — x=25 is outside ground half-extent of 20, miss
    {
        bvh::Hit h = bvh::trace(makeRay(25,0,0, 0,-1,0));
        CHECK(!h.hit);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    printf("=== BVH traversal integration tests ===\n");
    printf("Setting up: writing scene, baking, loading...\n");

    if (!setup())
    {
        cleanup();
        return 1;
    }

    printf("Setup complete.\n");

    test_box_hits();
    test_misses();
    test_sphere();
    test_ground_spatial_splits();

    printf("\n==============================\n");
    printf("Results: %d/%d passed", g_passed, g_total);
    if (g_failed > 0)
    {
        printf(", %d FAILED", g_failed);
    }
    printf("\n");

    cleanup();

    return (g_failed == 0) ? 0 : 1;
}
