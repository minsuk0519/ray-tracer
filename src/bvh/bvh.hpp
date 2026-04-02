#pragma once

#include <string>

#include "AABB.hpp"
#include "bvh_defines.hpp"

// ── Shared types (Baker + Tracer) ─────────────────────────────────────────────

struct Triangle
{
    uint v[3];   // indices into Vertex[]
};

struct BVHNode
{
    AABB aabb;
    bool isLeaf;

    union
    {
        uint childID[2];        // internal: left = childID[0], right = childID[1]
        struct
        {
            uint beginTriIndex; // leaf: first index into Triangle[]
            uint triSize;       // leaf: number of triangles
        };
    };
};

struct Vertex { float x, y, z, nx, ny, nz; };

// ── Baking entry point (#ifdef BAKING) ────────────────────────────────────────

#ifdef BAKING

namespace bvh
{
    bool bakeBVH(const std::string& scenePath);
}

#endif // BAKING
