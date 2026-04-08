#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_bake_types.hpp"
#include "bvh_sah.hpp"
#include "bvh_bfs.hpp"

#include "glm/glm.hpp"

#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

namespace bvh
{

bool trySAHSplit(uint nodeIndex)
{
    uint begin    = s_nodes[nodeIndex].beginTriIndex;
    uint end      = begin + s_nodes[nodeIndex].triSize;
    uint triCount = s_nodes[nodeIndex].triSize;

    // Binned SAH split: try all 3 axes, pick minimum cost
    int numBins = std::clamp((int)triCount / 4, BVH_SAH_BINS_MIN, BVH_SAH_BINS_MAX);

    float bestCost  = BVH_C_ISECT * (float)triCount;  // leaf cost — only split if cheaper
    int   bestAxis  = -1;
    int   bestBin   = -1;
    float bestCMin  = 0.f;
    float bestScale = 0.f;

    for (int axis = 0; axis < 3; axis++)
    {
        // centroid bounds along this axis
        float cmin =  std::numeric_limits<float>::max();
        float cmax = -std::numeric_limits<float>::max();
        for (uint i = begin; i < end; i++)
        {
            uint sortedIndex   = s_triIndex[i];
            uint triangleIndex = s_sortedTris[sortedIndex];
            float c = 0.f;
            for (int j = 0; j < 3; j++) { c += s_vertices[s_triangles[triangleIndex].v[j]][axis]; }
            c /= 3.0f;
            cmin = std::min(cmin, c);
            cmax = std::max(cmax, c);
        }
        if (cmax - cmin < 1e-6f) continue;

        float binScale = (float)numBins / (cmax - cmin);

        struct Bin { AABB aabb; uint count = 0; };
        std::vector<Bin> bins(numBins);

        for (uint i = begin; i < end; i++)
        {
            uint sortedIndex   = s_triIndex[i];
            uint triangleIndex = s_sortedTris[sortedIndex];
            float c = 0.f;
            for (int j = 0; j < 3; j++) { c += s_vertices[s_triangles[triangleIndex].v[j]][axis]; }
            c /= 3.0f;
            int b = std::min((int)((c - cmin) * binScale), numBins - 1);
            bins[b].count++;
            for (int j = 0; j < 3; j++)
            {
                uint vertexIndex = s_triangles[triangleIndex].v[j];
                bins[b].aabb.extend(glm::vec3(s_vertices[vertexIndex].x, s_vertices[vertexIndex].y, s_vertices[vertexIndex].z));
            }
        }

        // prefix (left) sweep
        std::vector<AABB> lAABB(numBins - 1);
        std::vector<uint> lCount(numBins - 1);
        AABB la; uint lc = 0;
        for (int b = 0; b < numBins - 1; b++)
        {
            la.extend(bins[b].aabb);
            lc += bins[b].count;
            lAABB[b]  = la;
            lCount[b] = lc;
        }

        // suffix (right) sweep — evaluate each split plane
        float parentInvArea = 1.0f / s_nodes[nodeIndex].aabb.half_area();
        AABB ra; uint rc = 0;
        for (int b = numBins - 1; b >= 1; b--)
        {
            ra.extend(bins[b].aabb);
            rc += bins[b].count;
            uint lc2 = lCount[b - 1];
            if (lc2 == 0 || rc == 0) continue;

            float cost = BVH_C_TRAV + BVH_C_ISECT *
                (lAABB[b - 1].half_area() * lc2 + ra.half_area() * rc) * parentInvArea;

            if (cost < bestCost)
            {
                bestCost  = cost;
                bestAxis  = axis;
                bestBin   = b;
                bestCMin  = cmin;
                bestScale = binScale;
            }
        }
    }

    if (bestAxis == -1)
    {
        // no split improves on the leaf cost
        s_nodes[nodeIndex].isLeaf = true;
        return false;
    }

    // partition s_triIndex[begin..end) around the winning split plane
    auto mid = std::partition(
        s_triIndex.begin() + begin,
        s_triIndex.begin() + end,
        [&](uint sortedIndex) {
            uint triangleIndex = s_sortedTris[sortedIndex];
            float c = 0.f;
            for (int j = 0; j < 3; j++) { c += s_vertices[s_triangles[triangleIndex].v[j]][bestAxis]; }
            c /= 3.0f;
            return std::min((int)((c - bestCMin) * bestScale), numBins - 1) < bestBin;
        });

    uint splitPos = (uint)(mid - s_triIndex.begin());

    // guard against a degenerate partition (all on one side)
    if (splitPos == begin || splitPos == end)
    {
        s_nodes[nodeIndex].isLeaf = true;
        return false;
    }

    uint leftIndex  = s_totalNodeCount++;
    uint rightIndex = s_totalNodeCount++;

    s_nodes[nodeIndex].isLeaf     = false;
    s_nodes[nodeIndex].childID[0] = leftIndex;
    s_nodes[nodeIndex].childID[1] = rightIndex;

    s_nodes[leftIndex].aabb           = computeRangeAABB(begin, splitPos);
    s_nodes[leftIndex].isLeaf         = true;
    s_nodes[leftIndex].beginTriIndex  = begin;
    s_nodes[leftIndex].triSize        = splitPos - begin;

    s_nodes[rightIndex].aabb          = computeRangeAABB(splitPos, end);
    s_nodes[rightIndex].isLeaf        = true;
    s_nodes[rightIndex].beginTriIndex = splitPos;
    s_nodes[rightIndex].triSize       = end - splitPos;

    s_queue.push_back({ leftIndex,  true });
    s_queue.push_back({ rightIndex, true });
    return true;
}

} // namespace bvh
