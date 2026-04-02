#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_bake_types.hpp"
#include "bvh_IO.hpp"
#include "bvh_morton.hpp"
#include "bvh_bfs.hpp"

#include <vector>
#include <string>

namespace bvh
{

// ── Shared baking state definitions ───────────────────────────────────────────
// These match the extern declarations in bvh_bake_state.hpp.
// NOTE: s_sahData is NOT defined here — it lives in bvh_sah.cpp.

std::string                s_scenePath;

std::vector<Vertex>        s_vertices;
std::vector<Triangle>      s_triangles;
std::vector<BVHNode>       s_nodes;
std::vector<MortonData>    s_mortonData;
std::vector<NodeBakingJob> s_queue;
std::vector<uint>          s_sortedTris;
std::vector<uint>          s_triIndex;
uint                       s_totalNodeCount = 0;
AABB                       s_sceneAABB;

// ── Module-internal pipeline steps ────────────────────────────────────────────

static bool preBake()
{
    if (!readScene())
        return false;

    if (s_vertices.empty() || s_triangles.empty())
        return false;

    if (!computeMortonCodes())
        return false;

    return true;
}

static bool reorderNodes()
{
    // DFS pre-order traversal: parent always precedes its children in memory.
    // Build a remapping table old index → new index, then copy into a fresh array.

    std::vector<BVHNode> ordered;
    ordered.reserve(s_totalNodeCount);

    std::vector<uint> remap(s_totalNodeCount, INVALID_NODE_INDEX);
    std::vector<uint> stack;
    stack.reserve(64);
    stack.push_back(0);  // root

    while (!stack.empty())
    {
        uint oldIdx  = stack.back();
        stack.pop_back();

        uint newIdx  = (uint)ordered.size();
        remap[oldIdx] = newIdx;
        ordered.push_back(s_nodes[oldIdx]);

        // push right then left so left is processed first (LIFO)
        const BVHNode& n = ordered.back();
        if (!n.isLeaf)
        {
            stack.push_back(n.childID[1]);
            stack.push_back(n.childID[0]);
        }
    }

    // fix up child indices using the remap table
    for (uint i = 0; i < (uint)ordered.size(); i++)
    {
        BVHNode& n = ordered[i];
        if (!n.isLeaf)
        {
            n.childID[0] = remap[n.childID[0]];
            n.childID[1] = remap[n.childID[1]];
        }
    }

    s_nodes          = std::move(ordered);
    s_totalNodeCount = (uint)s_nodes.size();
    return true;
}

static bool reorderTriangles()
{
    // Walk nodes in their post-reorderNodes() DFS order.
    // For each leaf, copy its triangles into a new contiguous array
    // and update beginTriIndex to the new position.

    std::vector<Triangle> ordered;
    ordered.reserve(s_triangles.size());

    for (uint i = 0; i < s_totalNodeCount; i++)
    {
        BVHNode& node = s_nodes[i];
        if (!node.isLeaf) continue;

        uint newBegin = (uint)ordered.size();

        for (uint k = 0; k < node.triSize; k++)
            ordered.push_back(s_triangles[s_sortedTris[node.beginTriIndex + k]]);

        node.beginTriIndex = newBegin;
    }

    s_triangles = std::move(ordered);
    return true;
}

static bool finishBake()
{
    s_scenePath.clear();
    s_vertices.clear();
    s_vertices.shrink_to_fit();
    s_triangles.clear();
    s_triangles.shrink_to_fit();
    s_nodes.clear();
    s_nodes.shrink_to_fit();
    s_mortonData.clear();
    s_mortonData.shrink_to_fit();
    s_queue.clear();
    s_queue.shrink_to_fit();
    s_sortedTris.clear();
    s_sortedTris.shrink_to_fit();
    s_triIndex.clear();
    s_triIndex.shrink_to_fit();
    s_totalNodeCount = 0;
    s_sceneAABB      = AABB();
    return true;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool bakeBVH(const std::string& scenePath)
{
    s_scenePath = scenePath;

    if (!preBake())           return false;
    if (!sortTrisByMorton())  return false;
    if (!initRoot())          return false;
    if (!bfsLoop())           return false;
    if (!reorderNodes())      return false;
    if (!reorderTriangles())  return false;
    if (!writeBakedData())    return false;
    if (!finishBake())        return false;
    return true;
}

} // namespace bvh
