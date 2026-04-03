#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_bake_types.hpp"
#include "bvh_bfs.hpp"
#include "bvh_sah.hpp"

#include "glm/glm.hpp"

namespace bvh
{

AABB computeRangeAABB(uint begin, uint end)
{
    AABB aabb;
    for (uint i = begin; i < end; i++)
    {
        const Triangle& tri = s_triangles[s_sortedTris[i]];
        const Vertex&   v0  = s_vertices[tri.v[0]];
        const Vertex&   v1  = s_vertices[tri.v[1]];
        const Vertex&   v2  = s_vertices[tri.v[2]];
        aabb.extend(glm::vec3(v0.x, v0.y, v0.z));
        aabb.extend(glm::vec3(v1.x, v1.y, v1.z));
        aabb.extend(glm::vec3(v2.x, v2.y, v2.z));
    }
    return aabb;
}

bool initRoot()
{
    const uint triCount = (uint)s_triangles.size();
    if (triCount == 0) return false;

    s_nodes.resize(triCount * 3 + 1);
    s_totalNodeCount = 1;

    BVHNode& root       = s_nodes[0];
    root.aabb           = s_sceneAABB;
    root.isLeaf         = true;
    root.beginTriIndex  = 0;
    root.triSize        = triCount;

    s_queue.push_back({ 0, BVH_MORTON_START_DEPTH, false });

    return true;
}

bool bfsLoop()
{
    while (!s_queue.empty())
    {
        NodeBakingJob job = s_queue.back();
        s_queue.pop_back();

        BVHNode& node  = s_nodes[job.nodeIndex];
        uint begin     = node.beginTriIndex;
        uint end       = begin + node.triSize;
        uint triCount  = node.triSize;

        // make leaf if small enough
        if (triCount <= (uint)BVH_MAX_LEAF_SIZE)
        {
            node.isLeaf = true;
            continue;
        }

        if (job.isSAH)
        {
            // delegate to SAH module — trySAHSplit marks the node as leaf on failure
            trySAHSplit(job.nodeIndex);
            continue;
        }

        // ── Morton path ──────────────────────────────────────────────────────

        uint splitIndex = INVALID_NODE_INDEX;
        uint64_t mask   = 1ULL << job.depth;

        for (uint i = begin; i < end - 1; i++)
        {
            uint64_t codeA = s_mortonData[s_sortedTris[i]].mortonCode;
            uint64_t codeB = s_mortonData[s_sortedTris[i + 1]].mortonCode;
            if ((codeA & mask) != (codeB & mask))
            {
                splitIndex = i + 1;
                break;
            }
        }

        if (splitIndex == INVALID_NODE_INDEX)
        {
            // no flip at this bit — go deeper or switch to SAH
            NodeBakingJob retry  = job;
            retry.depth          = (job.depth > 0) ? job.depth - 1 : 0;
            retry.isSAH          = (job.depth == 0 || triCount <= (uint)BVH_LBVH_THRESHOLD);
            s_queue.push_back(retry);
            continue;
        }

        // allocate two child nodes
        uint leftIndex   = s_totalNodeCount++;
        uint rightIndex  = s_totalNodeCount++;

        node.isLeaf      = false;
        node.childID[0]  = leftIndex;
        node.childID[1]  = rightIndex;

        uint childDepth  = (job.depth > 0) ? job.depth - 1 : 0;

        BVHNode& left    = s_nodes[leftIndex];
        left.aabb        = computeRangeAABB(begin, splitIndex);
        left.isLeaf      = true;
        left.beginTriIndex = begin;
        left.triSize     = splitIndex - begin;

        BVHNode& right   = s_nodes[rightIndex];
        right.aabb       = computeRangeAABB(splitIndex, end);
        right.isLeaf     = true;
        right.beginTriIndex = splitIndex;
        right.triSize    = end - splitIndex;

        s_queue.push_back({ leftIndex,  childDepth, left.triSize  <= (uint)BVH_LBVH_THRESHOLD });
        s_queue.push_back({ rightIndex, childDepth, right.triSize <= (uint)BVH_LBVH_THRESHOLD });
    }

    return true;
}

} // namespace bvh
