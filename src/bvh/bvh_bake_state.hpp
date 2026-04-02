#pragma once

#include <vector>
#include <string>

#include "bvh.hpp"
#include "bvh_bake_types.hpp"
#include "AABB.hpp"
#include "bvh_defines.hpp"

// ── Shared baking state (extern declarations) ─────────────────────────────────
// NOTE: s_sahData is intentionally absent — it is SAH-private (lives in bvh_sah.cpp).

namespace bvh
{
    extern std::string                s_scenePath;

    extern std::vector<Vertex>        s_vertices;
    extern std::vector<Triangle>      s_triangles;
    extern std::vector<BVHNode>       s_nodes;
    extern std::vector<MortonData>    s_mortonData;
    extern std::vector<NodeBakingJob> s_queue;
    extern std::vector<uint>          s_sortedTris;
    extern std::vector<uint>          s_triIndex;       // reserved for spatial split indirection; mirrors s_sortedTris
    extern uint                       s_totalNodeCount;
    extern AABB                       s_sceneAABB;
}
