#include "bvh_traverse.hpp"
#include "bvh.hpp"
#include "AABB.hpp"

#include <vector>
#include <fstream>
#include <limits>
#include <cmath>

#include "glm/glm.hpp"

namespace bvh
{

// ── Traversal state ───────────────────────────────────────────────────────────

static std::vector<BVHNode>  t_nodes;
static std::vector<Triangle> t_tris;
static std::vector<Vertex>   t_verts;

// ── Internal: slab method AABB test ──────────────────────────────────────────

static float rayAABB(const Ray& ray, const AABB& box, float tmax)
{
    const float inf = std::numeric_limits<float>::infinity();

    float invDirX = 1.f / ray.dir.x;
    float invDirY = 1.f / ray.dir.y;
    float invDirZ = 1.f / ray.dir.z;

    float t0x = (box.min.x - ray.origin.x) * invDirX;
    float t1x = (box.max.x - ray.origin.x) * invDirX;
    float t0y = (box.min.y - ray.origin.y) * invDirY;
    float t1y = (box.max.y - ray.origin.y) * invDirY;
    float t0z = (box.min.z - ray.origin.z) * invDirZ;
    float t1z = (box.max.z - ray.origin.z) * invDirZ;

    float tNear = std::max(std::max(std::min(t0x, t1x), std::min(t0y, t1y)), std::max(std::min(t0z, t1z), ray.tmin));
    float tFar  = std::min(std::min(std::max(t0x, t1x), std::max(t0y, t1y)), std::min(std::max(t0z, t1z), tmax));

    return (tNear <= tFar) ? tNear : inf;
}

// ── Internal: Möller–Trumbore triangle intersection ───────────────────────────

struct TriHit { float t, u, v; };

static TriHit rayTriangle(const Ray& ray, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2)
{
    const float inf = std::numeric_limits<float>::infinity();
    const TriHit miss = { inf, 0.f, 0.f };

    glm::vec3 e1 = v1 - v0;
    glm::vec3 e2 = v2 - v0;
    glm::vec3 h  = glm::cross(ray.dir, e2);
    float     a  = glm::dot(e1, h);

    if (a >= 0)
    {
        return miss;
    }

    float     f = 1.f / a;
    glm::vec3 s = ray.origin - v0;
    float     u = f * glm::dot(s, h);

    if (u < 0.f || u > 1.f)
    {
        return miss;
    }

    glm::vec3 q = glm::cross(s, e1);
    float     v = f * glm::dot(ray.dir, q);

    if (v < 0.f || u + v > 1.f)
    {
        return miss;
    }

    float t = f * glm::dot(e2, q);

    if (t < ray.tmin || t > ray.tmax)
    {
        return miss;
    }

    return { t, u, v };
}

// ── Internal: iterative stack traversal ──────────────────────────────────────

static Hit traverse(const Ray& ray)
{
    const float inf = std::numeric_limits<float>::infinity();

    Hit   best;
    float bestT = ray.tmax;

    uint stack[64];
    int  top = 0;
    stack[top++] = 0;  // push root

    while (top > 0)
    {
        uint nodeIdx = stack[--top];

        float tbox = rayAABB(ray, t_nodes[nodeIdx].aabb, bestT);
        if (tbox >= bestT)
        {
            continue;
        }

        if (t_nodes[nodeIdx].isLeaf)
        {
            for (uint k = 0; k < t_nodes[nodeIdx].triSize; k++)
            {
                uint ti  = t_nodes[nodeIdx].beginTriIndex + k;
                uint vi0 = t_tris[ti].v[0];
                uint vi1 = t_tris[ti].v[1];
                uint vi2 = t_tris[ti].v[2];

                glm::vec3 p0 = glm::vec3(t_verts[vi0].x, t_verts[vi0].y, t_verts[vi0].z);
                glm::vec3 p1 = glm::vec3(t_verts[vi1].x, t_verts[vi1].y, t_verts[vi1].z);
                glm::vec3 p2 = glm::vec3(t_verts[vi2].x, t_verts[vi2].y, t_verts[vi2].z);

                TriHit th = rayTriangle(ray, p0, p1, p2);
                if (th.t < bestT)
                {
                    bestT         = th.t;
                    best.hit      = true;
                    best.t        = th.t;
                    best.u        = th.u;
                    best.v        = th.v;
                    best.triIndex = ti;

                    float  w  = 1.f - th.u - th.v;
                    glm::vec3 n0 = glm::vec3(t_verts[vi0].nx, t_verts[vi0].ny, t_verts[vi0].nz);
                    glm::vec3 n1 = glm::vec3(t_verts[vi1].nx, t_verts[vi1].ny, t_verts[vi1].nz);
                    glm::vec3 n2 = glm::vec3(t_verts[vi2].nx, t_verts[vi2].ny, t_verts[vi2].nz);
                    best.normal   = glm::normalize(w * n0 + th.u * n1 + th.v * n2);
                }
            }
        }
        else
        {
            uint left  = t_nodes[nodeIdx].childID[0];
            uint right = t_nodes[nodeIdx].childID[1];

            float tLeft  = rayAABB(ray, t_nodes[left].aabb,  bestT);
            float tRight = rayAABB(ray, t_nodes[right].aabb, bestT);

            // push farther child first so nearer is processed first
            if (tLeft <= tRight)
            {
                if (tRight < bestT)
                {
                    stack[top++] = right;
                }
                if (tLeft < bestT)
                {
                    stack[top++] = left;
                }
            }
            else
            {
                if (tLeft < bestT)
                {
                    stack[top++] = left;
                }
                if (tRight < bestT)
                {
                    stack[top++] = right;
                }
            }
        }
    }

    return best;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool loadBVH(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open())
    {
        fprintf(stderr, "Error : failed to open %s on loadBVH()\n", path.c_str());
        return false;
    }

    uint version   = 0;
    uint nodeCount = 0;
    uint triCount  = 0;
    uint vertCount = 0;

    in.read(reinterpret_cast<char*>(&version),   sizeof(uint));
    in.read(reinterpret_cast<char*>(&nodeCount),  sizeof(uint));
    in.read(reinterpret_cast<char*>(&triCount),   sizeof(uint));
    in.read(reinterpret_cast<char*>(&vertCount),  sizeof(uint));

    if (!in.good())
    {
        fprintf(stderr, "Error : failed to read header from %s on loadBVH()\n", path.c_str());
        return false;
    }

    if (version != BVH_FILE_VERSION)
    {
        fprintf(stderr, "Error : version mismatch in %s (got %u, expected %u) on loadBVH()\n",
                path.c_str(), version, BVH_FILE_VERSION);
        return false;
    }

    t_nodes.clear();
    t_tris.clear();
    t_verts.clear();

    t_nodes.resize(nodeCount);
    t_tris.resize(triCount);
    t_verts.resize(vertCount);

    in.read(reinterpret_cast<char*>(t_nodes.data()), nodeCount * sizeof(BVHNode));
    in.read(reinterpret_cast<char*>(t_tris.data()),  triCount  * sizeof(Triangle));
    in.read(reinterpret_cast<char*>(t_verts.data()), vertCount * sizeof(Vertex));

    if (!in.good())
    {
        fprintf(stderr, "Error : failed to read payload from %s on loadBVH()\n", path.c_str());
        return false;
    }

    fprintf(stdout, "BVH loaded: %s  (%u nodes, %u tris, %u verts)\n",
            path.c_str(), nodeCount, triCount, vertCount);
    return true;
}

Hit trace(const Ray& ray)
{
    if (t_nodes.empty())
    {
        fprintf(stderr, "Error : BVH not loaded on trace()\n");
        return Hit{};
    }

    return traverse(ray);
}

} // namespace bvh
