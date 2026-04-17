#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_geo.hpp"

#include <cmath>


namespace bvh
{

// ── Internal helpers ──────────────────────────────────────────────────────────

// Layout: [north pole][ring 1 sectors...][ring 2 sectors...]...[south pole]. r in [1, rings-1].
static uint ringVert(uint vertexOffset, int r, int s, int sectors)
{
    return vertexOffset + 1u + (uint)((r - 1) * sectors + s);
}

// ── Public API ────────────────────────────────────────────────────────────────

void addSphere(math::vec3 center, float radius, int rings, int sectors)
{
    const uint vertexOffset = (uint)s_vertices.size();
    const float pi  = math::pi;

    {
        Vertex v;
        v.x  = center.x;
        v.y  = center.y + radius;
        v.z  = center.z;
        v.nx = 0.f;
        v.ny = 1.f;
        v.nz = 0.f;
        s_vertices.push_back(v);
    }

    for (int r = 1; r < rings; r++)
    {
        float theta = pi * (float)r / (float)rings;
        for (int s = 0; s < sectors; s++)
        {
            float phi = 2.f * pi * (float)s / (float)sectors;
            float nx  = sinf(theta) * cosf(phi);
            float ny  = cosf(theta);
            float nz  = sinf(theta) * sinf(phi);
            Vertex v;
            v.x  = center.x + radius * nx;
            v.y  = center.y + radius * ny;
            v.z  = center.z + radius * nz;
            v.nx = nx;
            v.ny = ny;
            v.nz = nz;
            s_vertices.push_back(v);
        }
    }

    const uint southPole = (uint)s_vertices.size();
    {
        Vertex v;
        v.x  = center.x;
        v.y  = center.y - radius;
        v.z  = center.z;
        v.nx = 0.f;
        v.ny = -1.f;
        v.nz = 0.f;
        s_vertices.push_back(v);
    }

    const uint northPole = vertexOffset;

    for (int s = 0; s < sectors; s++)
    {
        int sNext = (s + 1) % sectors;
        Triangle tri;
        tri.v[0] = northPole;
        tri.v[1] = ringVert(vertexOffset, 1, sNext, sectors);
        tri.v[2] = ringVert(vertexOffset, 1, s, sectors);
        s_triangles.push_back(tri);
    }

    for (int r = 1; r < rings - 1; r++)
    {
        for (int s = 0; s < sectors; s++)
        {
            int sNext = (s + 1) % sectors;
            uint a = ringVert(vertexOffset, r,     s,     sectors);
            uint b = ringVert(vertexOffset, r,     sNext, sectors);
            uint c = ringVert(vertexOffset, r + 1, sNext, sectors);
            uint d = ringVert(vertexOffset, r + 1, s,     sectors);

            Triangle t0;
            t0.v[0] = a; t0.v[1] = b; t0.v[2] = c;
            s_triangles.push_back(t0);

            Triangle t1;
            t1.v[0] = a; t1.v[1] = c; t1.v[2] = d;
            s_triangles.push_back(t1);
        }
    }

    // winding reversed vs. top cap so back-face culling (reject a>=0) accepts external hits from below
    for (int s = 0; s < sectors; s++)
    {
        int sNext = (s + 1) % sectors;
        Triangle tri;
        tri.v[0] = southPole;
        tri.v[1] = ringVert(vertexOffset, rings - 1, sNext, sectors);
        tri.v[2] = ringVert(vertexOffset, rings - 1, s,     sectors);
        s_triangles.push_back(tri);
    }
}

void addBox(math::vec3 center, math::vec3 he, math::quat orient)
{
    // corners are CCW-wound seen from outside so (0,1,2)+(0,2,3) triangles face outward
    struct FaceDef
    {
        math::vec3 normal;
        math::vec3 corners[4];
    };

    FaceDef faces[6];

    faces[0].normal     = math::vec3( 1, 0, 0);
    faces[0].corners[0] = math::vec3( he.x,  he.y, -he.z);
    faces[0].corners[1] = math::vec3( he.x, -he.y, -he.z);
    faces[0].corners[2] = math::vec3( he.x, -he.y,  he.z);
    faces[0].corners[3] = math::vec3( he.x,  he.y,  he.z);

    faces[1].normal     = math::vec3(-1, 0, 0);
    faces[1].corners[0] = math::vec3(-he.x,  he.y,  he.z);
    faces[1].corners[1] = math::vec3(-he.x, -he.y,  he.z);
    faces[1].corners[2] = math::vec3(-he.x, -he.y, -he.z);
    faces[1].corners[3] = math::vec3(-he.x,  he.y, -he.z);

    faces[2].normal     = math::vec3(0,  1, 0);
    faces[2].corners[0] = math::vec3(-he.x, he.y, -he.z);
    faces[2].corners[1] = math::vec3(-he.x, he.y,  he.z);
    faces[2].corners[2] = math::vec3( he.x, he.y,  he.z);
    faces[2].corners[3] = math::vec3( he.x, he.y, -he.z);

    faces[3].normal     = math::vec3(0, -1, 0);
    faces[3].corners[0] = math::vec3(-he.x, -he.y,  he.z);
    faces[3].corners[1] = math::vec3(-he.x, -he.y, -he.z);
    faces[3].corners[2] = math::vec3( he.x, -he.y, -he.z);
    faces[3].corners[3] = math::vec3( he.x, -he.y,  he.z);

    faces[4].normal     = math::vec3(0, 0,  1);
    faces[4].corners[0] = math::vec3( he.x,  he.y, he.z);
    faces[4].corners[1] = math::vec3( he.x, -he.y, he.z);
    faces[4].corners[2] = math::vec3(-he.x, -he.y, he.z);
    faces[4].corners[3] = math::vec3(-he.x,  he.y, he.z);

    faces[5].normal     = math::vec3(0, 0, -1);
    faces[5].corners[0] = math::vec3(-he.x,  he.y, -he.z);
    faces[5].corners[1] = math::vec3(-he.x, -he.y, -he.z);
    faces[5].corners[2] = math::vec3( he.x, -he.y, -he.z);
    faces[5].corners[3] = math::vec3( he.x,  he.y, -he.z);

    for (int f = 0; f < 6; f++)
    {
        uint vertexOffset = (uint)s_vertices.size();

        math::vec3 rotatedNormal = orient * faces[f].normal;

        for (int c = 0; c < 4; c++)
        {
            math::vec3 rotatedCorner = orient * faces[f].corners[c];
            Vertex v;
            v.x  = center.x + rotatedCorner.x;
            v.y  = center.y + rotatedCorner.y;
            v.z  = center.z + rotatedCorner.z;
            v.nx = rotatedNormal.x;
            v.ny = rotatedNormal.y;
            v.nz = rotatedNormal.z;
            s_vertices.push_back(v);
        }

        Triangle t0;
        t0.v[0] = vertexOffset + 0;
        t0.v[1] = vertexOffset + 1;
        t0.v[2] = vertexOffset + 2;
        s_triangles.push_back(t0);

        Triangle t1;
        t1.v[0] = vertexOffset + 0;
        t1.v[1] = vertexOffset + 2;
        t1.v[2] = vertexOffset + 3;
        s_triangles.push_back(t1);
    }
}

} // namespace bvh
