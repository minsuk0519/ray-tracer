#pragma once

#include <cstdint>

using uint = uint32_t;

constexpr uint  INVALID_NODE_INDEX    = 0xFFFFFFFF;
constexpr int   BVH_LBVH_THRESHOLD   = 64;
constexpr int   BVH_SAH_BINS_MIN     = 8;
constexpr int   BVH_SAH_BINS_MAX     = 32;
constexpr int   BVH_SAH_TRIS_PER_BIN = 4;  // target triangle count per bin when scaling bin count
constexpr float BVH_C_TRAV           = 1.0f;
constexpr float BVH_C_ISECT          = 1.2f;
constexpr float BVH_CENTROID_EPS     = 1e-6f;  // minimum centroid range to attempt an object split
constexpr float BVH_SPATIAL_EPS      = 1e-6f;  // minimum spatial extent to attempt a spatial split
constexpr float BVH_AREA_EPS         = 1e-6f;  // minimum half-area floor for SAH parent cost
constexpr int   BVH_MAX_LEAF_SIZE    = 16;
static_assert(BVH_MAX_LEAF_SIZE <= BVH_LBVH_THRESHOLD,
    "BVH_MAX_LEAF_SIZE must not exceed BVH_LBVH_THRESHOLD or SAH splits silently never fire");
constexpr int   MORTON_UNIT_BIT_SIZE = 21;
constexpr uint  MORTON_UNIT_MAX      = (1 << MORTON_UNIT_BIT_SIZE) - 1;

constexpr uint  BVH_FILE_VERSION = 1;

constexpr int   BVH_TRAVERSE_MAX_DEPTH         = 64;     // max stack depth for tree traversal; safe for ~2^64 node trees
constexpr float BVH_SPATIAL_TRICOUNT_MULTIPLIER = 1.5f;  // conservative space multiplier for triangles added during spatial splits
