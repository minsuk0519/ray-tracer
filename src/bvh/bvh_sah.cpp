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
    BVHNode& node  = s_nodes[nodeIndex];
    uint begin     = node.beginTriIndex;
    uint end       = begin + node.triSize;
    uint triCount  = node.triSize;

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
            const Triangle& tri = s_triangles[s_sortedTris[i]];
            float c = ((&s_vertices[tri.v[0]].x)[axis] +
                       (&s_vertices[tri.v[1]].x)[axis] +
                       (&s_vertices[tri.v[2]].x)[axis]) / 3.0f;
            cmin = std::min(cmin, c);
            cmax = std::max(cmax, c);
        }
        if (cmax - cmin < 1e-6f) continue;

        float binScale = (float)numBins / (cmax - cmin);

        struct Bin { AABB aabb; uint count = 0; };
        std::vector<Bin> bins(numBins);

        for (uint i = begin; i < end; i++)
        {
            const Triangle& tri = s_triangles[s_sortedTris[i]];
            const Vertex&   v0  = s_vertices[tri.v[0]];
            const Vertex&   v1  = s_vertices[tri.v[1]];
            const Vertex&   v2  = s_vertices[tri.v[2]];
            float c = ((&v0.x)[axis] + (&v1.x)[axis] + (&v2.x)[axis]) / 3.0f;
            int b   = std::min((int)((c - cmin) * binScale), numBins - 1);
            bins[b].count++;
            bins[b].aabb.extend(glm::vec3(v0.x, v0.y, v0.z));
            bins[b].aabb.extend(glm::vec3(v1.x, v1.y, v1.z));
            bins[b].aabb.extend(glm::vec3(v2.x, v2.y, v2.z));
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
        float parentInvArea = 1.0f / node.aabb.half_area();
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
        node.isLeaf = true;
        return false;
    }

    // partition s_sortedTris[begin..end) around the winning split plane
    auto mid = std::partition(
        s_sortedTris.begin() + begin,
        s_sortedTris.begin() + end,
        [&](uint triIdx) {
            const Triangle& tri = s_triangles[triIdx];
            float c = ((&s_vertices[tri.v[0]].x)[bestAxis] +
                       (&s_vertices[tri.v[1]].x)[bestAxis] +
                       (&s_vertices[tri.v[2]].x)[bestAxis]) / 3.0f;
            return std::min((int)((c - bestCMin) * bestScale), numBins - 1) < bestBin;
        });

    uint splitPos = (uint)(mid - s_sortedTris.begin());

    // guard against a degenerate partition (all on one side)
    if (splitPos == begin || splitPos == end)
    {
        node.isLeaf = true;
        return false;
    }

    uint leftIndex  = s_totalNodeCount++;
    uint rightIndex = s_totalNodeCount++;

    node.isLeaf     = false;
    node.childID[0] = leftIndex;
    node.childID[1] = rightIndex;

    BVHNode& left        = s_nodes[leftIndex];
    left.aabb            = computeRangeAABB(begin, splitPos);
    left.isLeaf          = true;
    left.beginTriIndex   = begin;
    left.triSize         = splitPos - begin;

    BVHNode& right       = s_nodes[rightIndex];
    right.aabb           = computeRangeAABB(splitPos, end);
    right.isLeaf         = true;
    right.beginTriIndex  = splitPos;
    right.triSize        = end - splitPos;

    s_queue.push_back({ leftIndex,  0, true });
    s_queue.push_back({ rightIndex, 0, true });
    return true;
}

} // namespace bvh
