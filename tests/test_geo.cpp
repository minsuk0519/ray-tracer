// test_geo.cpp -- unit tests for bvh_geo.cpp (addSphere / addBox)
//
// Build (from tests/):
//   g++ -std=c++2b -I../src/bvh -I../libs/glm -o test_geo test_geo.cpp ../src/bvh/bvh_geo.cpp ../src/bvh/AABB.cpp
// Run:
//   ./test_geo

#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_bake_types.hpp"
#include "bvh_geo.hpp"

#include <cstdio>
#include <cmath>
#include <deque>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// Define all extern state vars required by bvh_bake_state.hpp
// (bvh_geo.cpp only uses s_vertices and s_triangles; the rest are stubs)
// ---------------------------------------------------------------------------

namespace bvh
{
    std::string                s_scenePath;
    std::vector<Vertex>        s_vertices;
    std::vector<Triangle>      s_triangles;
    std::vector<BVHNode>       s_nodes;
    std::vector<MortonData>    s_mortonData;
    std::deque<NodeBakingJob>  s_queue;
    std::vector<uint>          s_sortedTris;
    std::vector<uint>          s_triIndex;
    uint                       s_triIndexSize   = 0;
    uint                       s_sortedTrisSize = 0;
    uint                       s_totalNodeCount = 0;
    AABB                       s_sceneAABB;
}

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
#define CHECK_EQ(a, b)        check((a) == (b), #a " == " #b, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, eps) check(fabsf((a) - (b)) <= (eps), #a " ~= " #b, __FILE__, __LINE__)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void clearState()
{
    bvh::s_vertices.clear();
    bvh::s_triangles.clear();
}

static float vec3len(glm::vec3 v)
{
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

static bool allIndicesValid()
{
    uint vcount = (uint)bvh::s_vertices.size();
    for (uint i = 0; i < (uint)bvh::s_triangles.size(); i++)
    {
        for (int k = 0; k < 3; k++)
        {
            if (bvh::s_triangles[i].v[k] >= vcount)
            {
                return false;
            }
        }
    }
    return true;
}

static bool noDegenTris()
{
    for (uint i = 0; i < (uint)bvh::s_triangles.size(); i++)
    {
        uint i0 = bvh::s_triangles[i].v[0];
        uint i1 = bvh::s_triangles[i].v[1];
        uint i2 = bvh::s_triangles[i].v[2];
        glm::vec3 p0(bvh::s_vertices[i0].x, bvh::s_vertices[i0].y, bvh::s_vertices[i0].z);
        glm::vec3 p1(bvh::s_vertices[i1].x, bvh::s_vertices[i1].y, bvh::s_vertices[i1].z);
        glm::vec3 p2(bvh::s_vertices[i2].x, bvh::s_vertices[i2].y, bvh::s_vertices[i2].z);
        glm::vec3 cross = glm::cross(p1 - p0, p2 - p0);
        if (vec3len(cross) <= 1e-6f)
        {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Sphere: count tests
// ---------------------------------------------------------------------------

static void test_sphere_counts()
{
    suite("sphere counts");

    // rings=2, sectors=8 -> verts = 2+1*8=10, tris = 2*8*1=16
    {
        clearState();
        bvh::addSphere({0,0,0}, 1.f, 2, 8);
        CHECK_EQ((int)bvh::s_vertices.size(),  10);
        CHECK_EQ((int)bvh::s_triangles.size(), 16);
    }

    // rings=4, sectors=6 -> verts = 2+3*6=20, tris = 2*6*3=36
    {
        clearState();
        bvh::addSphere({0,0,0}, 1.f, 4, 6);
        CHECK_EQ((int)bvh::s_vertices.size(),  20);
        CHECK_EQ((int)bvh::s_triangles.size(), 36);
    }

    // rings=8, sectors=12 -> verts = 2+7*12=86, tris = 2*12*7=168
    {
        clearState();
        bvh::addSphere({0,0,0}, 1.f, 8, 12);
        CHECK_EQ((int)bvh::s_vertices.size(),  86);
        CHECK_EQ((int)bvh::s_triangles.size(), 168);
    }

    // default (rings=16, sectors=32) -> verts = 2+15*32=482, tris = 2*32*15=960
    {
        clearState();
        bvh::addSphere({0,0,0}, 1.f);
        CHECK_EQ((int)bvh::s_vertices.size(),  482);
        CHECK_EQ((int)bvh::s_triangles.size(), 960);
    }
}

// ---------------------------------------------------------------------------
// Sphere: geometry tests
// ---------------------------------------------------------------------------

static void test_sphere_geometry()
{
    suite("sphere geometry (rings=4, sectors=6, radius=3, center=(1,2,3))");

    const float    radius = 3.f;
    const glm::vec3 center(1.f, 2.f, 3.f);
    const float eps_surf = 1e-4f;
    const float eps_unit = 1e-5f;

    clearState();
    bvh::addSphere(center, radius, 4, 6);

    bool allOnSurface      = true;
    bool allNormUnit       = true;
    bool allNormOutward    = true;
    bool normEqualsDir     = true;

    for (uint i = 0; i < (uint)bvh::s_vertices.size(); i++)
    {
        glm::vec3 pos(bvh::s_vertices[i].x, bvh::s_vertices[i].y, bvh::s_vertices[i].z);
        glm::vec3 nor(bvh::s_vertices[i].nx, bvh::s_vertices[i].ny, bvh::s_vertices[i].nz);
        glm::vec3 dir = pos - center;
        float dist = vec3len(dir);

        if (fabsf(dist - radius) > eps_surf)
        {
            allOnSurface = false;
        }
        if (fabsf(vec3len(nor) - 1.f) > eps_unit)
        {
            allNormUnit = false;
        }

        glm::vec3 outward = (dist > 1e-6f) ? (dir / dist) : glm::vec3(0,1,0);
        float d = nor.x*outward.x + nor.y*outward.y + nor.z*outward.z;
        if (d <= 0.f)
        {
            allNormOutward = false;
        }
        if (fabsf(nor.x - outward.x) > eps_surf ||
            fabsf(nor.y - outward.y) > eps_surf ||
            fabsf(nor.z - outward.z) > eps_surf)
        {
            normEqualsDir = false;
        }
    }

    CHECK(allOnSurface);
    CHECK(allNormUnit);
    CHECK(allNormOutward);
    CHECK(normEqualsDir);
    CHECK(allIndicesValid());
    CHECK(noDegenTris());
}

// ---------------------------------------------------------------------------
// Sphere: appends, doesn't clobber existing state
// ---------------------------------------------------------------------------

static void test_sphere_appends()
{
    suite("sphere appends to existing state");

    clearState();
    bvh::addSphere({0,0,0}, 1.f, 2, 4);   // verts=6, tris=8
    int v1 = (int)bvh::s_vertices.size();
    int t1 = (int)bvh::s_triangles.size();

    bvh::addSphere({5,0,0}, 1.f, 2, 4);   // adds another 6 verts, 8 tris
    int v2 = (int)bvh::s_vertices.size();
    int t2 = (int)bvh::s_triangles.size();

    CHECK_EQ(v2, v1 * 2);
    CHECK_EQ(t2, t1 * 2);
    CHECK(allIndicesValid());
}

// ---------------------------------------------------------------------------
// Box: count tests
// ---------------------------------------------------------------------------

static void test_box_counts()
{
    suite("box counts");

    clearState();
    bvh::addBox({0,0,0}, {1,1,1}, glm::quat(1,0,0,0));
    CHECK_EQ((int)bvh::s_vertices.size(),  24);
    CHECK_EQ((int)bvh::s_triangles.size(), 12);

    // Non-uniform extents -- same counts
    clearState();
    bvh::addBox({0,0,0}, {2,5,3}, glm::quat(1,0,0,0));
    CHECK_EQ((int)bvh::s_vertices.size(),  24);
    CHECK_EQ((int)bvh::s_triangles.size(), 12);
}

// ---------------------------------------------------------------------------
// Box: geometry tests (identity rotation)
// ---------------------------------------------------------------------------

static void test_box_geometry_identity()
{
    suite("box geometry (he=(1,2,3), center=(5,0,-3), identity rotation)");

    const glm::vec3 center(5.f, 0.f, -3.f);
    const glm::vec3 he(1.f, 2.f, 3.f);
    const float eps = 1e-4f;

    clearState();
    bvh::addBox(center, he, glm::quat(1,0,0,0));

    bool allNormUnit    = true;
    bool allOnFacePlane = true;
    bool allNormOutward = true;

    for (uint i = 0; i < (uint)bvh::s_vertices.size(); i++)
    {
        glm::vec3 pos(bvh::s_vertices[i].x, bvh::s_vertices[i].y, bvh::s_vertices[i].z);
        glm::vec3 nor(bvh::s_vertices[i].nx, bvh::s_vertices[i].ny, bvh::s_vertices[i].nz);

        if (fabsf(vec3len(nor) - 1.f) > 1e-5f)
        {
            allNormUnit = false;
        }

        glm::vec3 local = pos - center;

        // Each vertex must lie exactly on one face plane
        bool onX = fabsf(fabsf(local.x) - he.x) <= eps;
        bool onY = fabsf(fabsf(local.y) - he.y) <= eps;
        bool onZ = fabsf(fabsf(local.z) - he.z) <= eps;
        if (!onX && !onY && !onZ)
        {
            allOnFacePlane = false;
        }

        float outDot = nor.x*local.x + nor.y*local.y + nor.z*local.z;
        if (outDot <= 0.f)
        {
            allNormOutward = false;
        }
    }

    CHECK(allNormUnit);
    CHECK(allOnFacePlane);
    CHECK(allNormOutward);
    CHECK(allIndicesValid());
    CHECK(noDegenTris());
}

// ---------------------------------------------------------------------------
// Box: rotation test
// ---------------------------------------------------------------------------

static void test_box_geometry_rotated()
{
    suite("box geometry (90 deg rotation around Y)");

    // 90 degree rotation around Y: quat = (cos45, 0, sin45, 0)
    const float half = sqrtf(0.5f);
    glm::quat orient(half, 0.f, half, 0.f);
    const glm::vec3 center(0.f, 0.f, 0.f);
    const glm::vec3 he(1.f, 1.f, 1.f);

    clearState();
    bvh::addBox(center, he, orient);

    bool allNormUnit    = true;
    bool allNormOutward = true;

    for (uint i = 0; i < (uint)bvh::s_vertices.size(); i++)
    {
        glm::vec3 pos(bvh::s_vertices[i].x, bvh::s_vertices[i].y, bvh::s_vertices[i].z);
        glm::vec3 nor(bvh::s_vertices[i].nx, bvh::s_vertices[i].ny, bvh::s_vertices[i].nz);

        if (fabsf(vec3len(nor) - 1.f) > 1e-5f)
        {
            allNormUnit = false;
        }

        float outDot = nor.x*pos.x + nor.y*pos.y + nor.z*pos.z; // center is origin
        if (outDot <= 0.f)
        {
            allNormOutward = false;
        }
    }

    CHECK(allNormUnit);
    CHECK(allNormOutward);
    CHECK_EQ((int)bvh::s_vertices.size(),  24);
    CHECK_EQ((int)bvh::s_triangles.size(), 12);
    CHECK(allIndicesValid());
    CHECK(noDegenTris());
}

// ---------------------------------------------------------------------------
// Box: appends
// ---------------------------------------------------------------------------

static void test_box_appends()
{
    suite("box appends to existing state");

    clearState();
    bvh::addBox({0,0,0}, {1,1,1}, glm::quat(1,0,0,0));
    bvh::addBox({5,0,0}, {1,1,1}, glm::quat(1,0,0,0));

    CHECK_EQ((int)bvh::s_vertices.size(),  48);
    CHECK_EQ((int)bvh::s_triangles.size(), 24);
    CHECK(allIndicesValid());
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    printf("=== bvh_geo unit tests ===\n");

    test_sphere_counts();
    test_sphere_geometry();
    test_sphere_appends();
    test_box_counts();
    test_box_geometry_identity();
    test_box_geometry_rotated();
    test_box_appends();

    printf("\n==============================\n");
    printf("Results: %d/%d passed", g_passed, g_total);
    if (g_failed > 0)
    {
        printf(", %d FAILED", g_failed);
    }
    printf("\n");

    return (g_failed == 0) ? 0 : 1;
}
