#pragma once

#include "bvh_defines.hpp"
#include "AABB.hpp"

// ── Baking-only types (no #ifdef guard — include only from baking modules) ────

struct MortonData
{
    uint64_t mortonCode;
};

struct NodeBakingJob
{
    uint nodeIndex;
    uint depth;
    bool isSAH = false;
};
