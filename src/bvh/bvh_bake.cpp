#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_bake_types.hpp"
#include "bvh_IO.hpp"
#include "bvh_morton.hpp"
#include "bvh_bfs.hpp"

#include <deque>
#include <vector>
#include <string>

namespace bvh
{

// ── Shared baking state definitions ───────────────────────────────────────────
// These match the extern declarations in bvh_bake_state.hpp.

std::string                s_scenePath;

std::vector<Vertex>        s_vertices;
std::vector<Triangle>      s_triangles;
std::vector<BVHNode>       s_nodes;
std::vector<MortonData>    s_mortonData;
std::deque<NodeBakingJob>  s_queue;
std::vector<uint>          s_sortedTris;
std::vector<uint>          s_triIndex;
uint                       s_triIndexSize   = 0;
uint                       s_sortedTrisSize = 0;
uint                       s_totalNodeCount = 0;
AABB                       s_sceneAABB;

// ── Module-internal pipeline steps ────────────────────────────────────────────

static bool preBake()
{
    if (!readScene())
    {
        return false;
    }

    if (!initGeos())
    {
        return false;
    }

    return true;
}

static bool reorderNodes()
{
    // DFS pre-order traversal: parent always precedes its children in memory.
    // Build a remapping table old index → new index, then copy into a fresh array.

    std::vector<BVHNode> ordered;
    ordered.reserve(s_totalNodeCount);
    uint orderedSize = 0;

    std::vector<uint> remap(s_totalNodeCount, INVALID_NODE_INDEX);
    std::vector<uint> stack;
    stack.reserve(64);
    stack.push_back(0);  // root

    while (!stack.empty())
    {
        uint oldIdx  = stack.back();
        stack.pop_back();

        remap[oldIdx] = orderedSize;
        ordered.push_back(s_nodes[oldIdx]);
        orderedSize++;

        // push right then left so left is processed first (LIFO)
        if (!s_nodes[oldIdx].isLeaf)
        {
            stack.push_back(s_nodes[oldIdx].childID[1]);
            stack.push_back(s_nodes[oldIdx].childID[0]);
        }
    }

    // fix up child indices using the remap table
    for (uint i = 0; i < orderedSize; i++)
    {
        if (!ordered[i].isLeaf)
        {
            ordered[i].childID[0] = remap[ordered[i].childID[0]];
            ordered[i].childID[1] = remap[ordered[i].childID[1]];
        }
    }

    s_nodes = std::move(ordered);
    return true;
}

static bool reorderTriangles()
{
    // Walk nodes in their post-reorderNodes() DFS order.
    // For each leaf, copy its triangles into a new contiguous array
    // and update beginTriIndex to the new position.

    std::vector<Triangle> ordered;
    ordered.reserve(s_triIndexSize);
    uint orderedSize = 0;

    for (uint i = 0; i < s_totalNodeCount; i++)
    {
        if (!s_nodes[i].isLeaf)
        {
            continue;
        }

        uint oldBegin = s_nodes[i].beginTriIndex;
        s_nodes[i].beginTriIndex = orderedSize;

        for (uint k = 0; k < s_nodes[i].triSize; k++)
        {
            ordered.push_back(s_triangles[s_sortedTris[s_triIndex[oldBegin + k]]]);
            orderedSize++;
        }
    }

    s_triangles = std::move(ordered);
    return true;
}

static bool finishBake()
{
    s_scenePath.clear();
    s_scenePath.shrink_to_fit();
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
    s_triIndexSize   = 0;
    s_sortedTrisSize = 0;
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
