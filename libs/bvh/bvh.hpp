#pragma once

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

// ── Baking types (#ifdef BAKING) ──────────────────────────────────────────────

#ifdef BAKING

namespace bvh
{
    void setScenePath(const std::string& path);

    bool bakeBVH();

    bool preBake();
    bool sortTrisByMorton();
    bool initRoot();
    bool bfsLoop();
    bool reorderNodes();
    bool reorderTriangles();
    bool writeBakedData();
    bool finishBake();
}

struct MortonData
{
    uint64_t mortonCode;
};

struct SAHData
{
    Axis  axis;
    float splitValue;
};

struct NodeBakingJob
{
    uint nodeIndex;
    uint depth;
    bool isSAH = false;
};

#endif // BAKING
