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

struct Vertex
{
    float x, y, z, nx, ny, nz;

    float operator[](int axis) const
    {
        if (axis == 0) { return x; }
        if (axis == 1) { return y; }
        return z;
    }
};

// ── Baking entry point (#ifdef BAKING) ────────────────────────────────────────

#ifdef BAKING

namespace bvh
{
    bool bakeBVH(const std::string& scenePath);
}

#endif // BAKING
