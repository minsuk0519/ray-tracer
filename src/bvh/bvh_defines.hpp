#pragma once

#include <cstdint>

using uint = uint32_t;

constexpr uint  INVALID_NODE_INDEX    = 0xFFFFFFFF;
constexpr int   BVH_LBVH_THRESHOLD   = 64;
constexpr int   BVH_SAH_BINS_MIN     = 8;
constexpr int   BVH_SAH_BINS_MAX     = 32;
constexpr float BVH_C_TRAV           = 1.0f;
constexpr float BVH_C_ISECT          = 1.2f;
constexpr float BVH_OVERLAP_THRESHOLD = 1e-4f;
constexpr int   BVH_MAX_LEAF_SIZE    = 16;
constexpr uint  BVH_MORTON_START_DEPTH  = 62;
constexpr int   MORTON_UNIT_BIT_SIZE    = 21;
constexpr uint  MORTON_UNIT_MAX         = (1 << MORTON_UNIT_BIT_SIZE) - 1;

constexpr uint  BVH_FILE_VERSION = 1;
