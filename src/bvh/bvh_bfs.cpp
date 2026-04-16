#define BAKING
#include "bvh.hpp"
#include <bit>
#include "bvh_bake_state.hpp"
#include "bvh_bake_types.hpp"
#include "bvh_bfs.hpp"
#include "bvh_sah.hpp"

#include "../math/math.hpp"

namespace bvh
{

AABB computeRangeAABB(uint begin, uint end)
{
    AABB aabb;
    for (uint i = begin; i < end; i++)
    {
        uint sortedIndex   = s_triIndex[i];
        uint triangleIndex = s_sortedTris[sortedIndex];
        for (int j = 0; j < 3; j++)
        {
            uint vertexIndex = s_triangles[triangleIndex].v[j];
            aabb.extend(math::vec3(s_vertices[vertexIndex].x, s_vertices[vertexIndex].y, s_vertices[vertexIndex].z));
        }
    }
    return aabb;
}

static bool shouldSAH(const BVHNode& node)
{
    return node.triSize <= (uint)BVH_LBVH_THRESHOLD;
}

static uint findMortonSplitIndex(uint begin, uint triCount, uint64_t splitMask)
{
    for (uint i = 0; i < triCount - 1; i++)
    {
        uint beginCodeIndex = begin + i;
        uint64_t codeA      = s_mortonData[s_sortedTris[beginCodeIndex]].mortonCode;
        uint64_t codeB      = s_mortonData[s_sortedTris[beginCodeIndex + 1]].mortonCode;
        if ((codeA & splitMask) != (codeB & splitMask))
        {
            return beginCodeIndex + 1;
        }
    }
    return INVALID_NODE_INDEX;  // unreachable if xorCodes != 0
}

bool initRoot()
{
    const uint triCount = (uint)s_triangles.size();
    if (triCount == 0)
    {
        return false;
    }

    // worst case: spatial splits produce triCount * SPATIAL_TRICOUNT_MULTIPLIER leaves,
    // a full binary tree over N leaves has 2N - 1 nodes total
    s_nodes.resize((uint)(triCount * SPATIAL_TRICOUNT_MULTIPLIER) * 2 - 1);
    s_totalNodeCount = 1;

    s_nodes[0].aabb          = s_sceneAABB;
    s_nodes[0].isLeaf        = true;
    s_nodes[0].beginTriIndex = 0;
    s_nodes[0].triSize       = triCount;

    s_queue.push_back({ 0, shouldSAH(s_nodes[0]) });

    return true;
}

bool bfsLoop()
{
    while (!s_queue.empty())
    {
        NodeBakingJob job = s_queue.front();
        s_queue.pop_front();

        // make leaf if small enough
        if (s_nodes[job.nodeIndex].triSize <= (uint)BVH_MAX_LEAF_SIZE)
        {
            s_nodes[job.nodeIndex].isLeaf = true;
            continue;
        }

        if (job.isSAH)
        {
            // delegate to SAH module — trySAHSplit marks the node as leaf on failure
            trySAHSplit(job.nodeIndex);
            continue;
        }

        // ── Morton path ──────────────────────────────────────────────────────

        uint begin    = s_nodes[job.nodeIndex].beginTriIndex;
        uint triCount = s_nodes[job.nodeIndex].triSize;
        uint end      = begin + triCount;

        uint64_t firstCode = s_mortonData[s_sortedTris[begin]].mortonCode;
        uint64_t lastCode  = s_mortonData[s_sortedTris[end - 1]].mortonCode;
        uint64_t xorCodes  = firstCode ^ lastCode;

        if (xorCodes == 0)
        {
            // all codes identical — no Morton split possible
            trySAHSplit(job.nodeIndex);
            continue;
        }

        uint64_t splitMask  = std::bit_floor(xorCodes);
        uint     splitIndex = findMortonSplitIndex(begin, triCount, splitMask);

        // allocate two child nodes
        uint leftIndex  = s_totalNodeCount++;
        uint rightIndex = s_totalNodeCount++;

        s_nodes[job.nodeIndex].isLeaf     = false;
        s_nodes[job.nodeIndex].childID[0] = leftIndex;
        s_nodes[job.nodeIndex].childID[1] = rightIndex;

        s_nodes[leftIndex].aabb          = computeRangeAABB(begin, splitIndex);
        s_nodes[leftIndex].isLeaf        = true;
        s_nodes[leftIndex].beginTriIndex = begin;
        s_nodes[leftIndex].triSize       = splitIndex - begin;

        s_nodes[rightIndex].aabb          = computeRangeAABB(splitIndex, end);
        s_nodes[rightIndex].isLeaf        = true;
        s_nodes[rightIndex].beginTriIndex = splitIndex;
        s_nodes[rightIndex].triSize       = end - splitIndex;

        s_queue.push_back({ leftIndex,  shouldSAH(s_nodes[leftIndex])  });
        s_queue.push_back({ rightIndex, shouldSAH(s_nodes[rightIndex]) });
    }

    return true;
}

} // namespace bvh
