#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_bake_types.hpp"
#include "bvh_sah.hpp"
#include "bvh_bfs.hpp"

#include "../math/math.hpp"

#include <algorithm>
#include <limits>
#include <cmath>

namespace bvh
{

static void triVerts(uint triangleIndex, math::vec3& v0, math::vec3& v1, math::vec3& v2)
{
    v0 = math::vec3(s_vertices[s_triangles[triangleIndex].v[0]][0], s_vertices[s_triangles[triangleIndex].v[0]][1], s_vertices[s_triangles[triangleIndex].v[0]][2]);
    v1 = math::vec3(s_vertices[s_triangles[triangleIndex].v[1]][0], s_vertices[s_triangles[triangleIndex].v[1]][1], s_vertices[s_triangles[triangleIndex].v[1]][2]);
    v2 = math::vec3(s_vertices[s_triangles[triangleIndex].v[2]][0], s_vertices[s_triangles[triangleIndex].v[2]][1], s_vertices[s_triangles[triangleIndex].v[2]][2]);
}

bool trySAHSplit(uint nodeIndex)
{
    uint begin    = s_nodes[nodeIndex].beginTriIndex;
    uint end      = begin + s_nodes[nodeIndex].triSize;
    uint triCount = s_nodes[nodeIndex].triSize;

    if (triCount <= (uint)BVH_MAX_LEAF_SIZE)
    {
        s_nodes[nodeIndex].isLeaf = true;
        return false;
    }

    if (s_totalNodeCount + 2 > (uint)s_nodes.size())
    {
        return false;
    }

    int numBins = std::clamp((int)triCount / BVH_SAH_TRIS_PER_BIN, BVH_SAH_BINS_MIN, BVH_SAH_BINS_MAX);

    float bestCost  = BVH_C_ISECT * (float)triCount;  // leaf cost — only split if cheaper
    int   bestAxis  = -1;
    int   bestBin   = -1;
    float bestCMin  = 0.f;
    float bestScale = 0.f;

    // Save object-split winner separately for spatial-split fallback
    int   bestObjectAxis  = -1;
    int   bestObjectBin   = -1;
    float bestObjectCMin  = 0.f;
    float bestObjectScale = 0.f;

    float parentInvArea = 1.0f / std::max(s_nodes[nodeIndex].aabb.half_area(), BVH_AREA_EPS);

    struct Bin { AABB aabb; uint count = 0; };
    Bin   bins[BVH_SAH_BINS_MAX];
    AABB  lAABB[BVH_SAH_BINS_MAX];
    uint  lCount[BVH_SAH_BINS_MAX];

    // ── Phase 1: Object split — bin by triangle centroid ──────────────────────
    // Centroid bounds give the range for bin mapping. We need centroid bounds
    // (not full AABB bounds) because the bin formula maps centroid position to a
    // bin index: bin = (centroid - cmin) / (cmax - cmin) * numBins.
    // Using full AABB bounds would over-widen the range and waste bin resolution.

    for (int axis = 0; axis < 3; axis++)
    {
        float cmin =  std::numeric_limits<float>::max();
        float cmax = -std::numeric_limits<float>::max();
        for (uint i = begin; i < end; i++)
        {
            float c = s_centroids[s_sortedTris[s_triIndex[i]]][axis];
            cmin = std::min(cmin, c);
            cmax = std::max(cmax, c);
        }
        if (cmax - cmin < BVH_CENTROID_EPS)
        {
            continue;
        }

        float binScale = (float)numBins / (cmax - cmin);

        for (int b = 0; b < numBins; b++)
        {
            bins[b] = Bin{};
        }

        for (uint i = begin; i < end; i++)
        {
            uint sortedIndex   = s_triIndex[i];
            uint triangleIndex = s_sortedTris[sortedIndex];
            float c = s_centroids[triangleIndex][axis];
            int b = std::clamp((int)((c - cmin) * binScale), 0, numBins - 1);
            bins[b].count++;
            math::vec3 v0, v1, v2;
            triVerts(triangleIndex, v0, v1, v2);
            bins[b].aabb.extend(v0);
            bins[b].aabb.extend(v1);
            bins[b].aabb.extend(v2);
        }

        AABB la; uint lc = 0;
        for (int b = 0; b < numBins - 1; b++)
        {
            la.extend(bins[b].aabb);
            lc += bins[b].count;
            lAABB[b]  = la;
            lCount[b] = lc;
        }

        AABB ra; uint rc = 0;
        for (int b = numBins - 1; b >= 1; b--)
        {
            ra.extend(bins[b].aabb);
            rc += bins[b].count;
            uint lc2 = lCount[b - 1];
            if (lc2 == 0 || rc == 0)
            {
                continue;
            }

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

    // Save the object-split winner before attempting spatial split
    bestObjectAxis  = bestAxis;
    bestObjectBin   = bestBin;
    bestObjectCMin  = bestCMin;
    bestObjectScale = bestScale;

    // ── Phase 2: Spatial binning ───────────────────────────────────────────────
    // Bins are spaced over the node's full spatial AABB extent (not centroid bounds).
    // Triangles are clipped per-bin via Sutherland-Hodgman, giving tighter child AABBs
    // at the cost of potential triangle duplication (straddlers appear in both children).
    // TODO: add SBVH lambda overlap threshold to restrict spatial splits to nodes where
    // the overlap between child AABBs is large enough to justify the duplication cost.

    struct SpatialBin
    {
        AABB aabb;
        uint countIn  = 0;   // triangles whose AABB min falls in this bin
        uint countOut = 0;   // triangles whose AABB max falls in this bin
    };

    SpatialBin spatialBins[BVH_SAH_BINS_MAX];
    AABB       spLAABB[BVH_SAH_BINS_MAX];
    uint       spLCount[BVH_SAH_BINS_MAX];

    bool  isSpatialSplit      = false;
    float bestNodeMin         = 0.f;
    float bestSpatialBinScale = 0.f;

    for (int axis = 0; axis < 3; axis++)
    {
        float nodeMin = s_nodes[nodeIndex].aabb.min[axis];
        float nodeMax = s_nodes[nodeIndex].aabb.max[axis];

        if (nodeMax - nodeMin < BVH_SPATIAL_EPS)
        {
            continue;
        }

        float spatialBinScale = (float)numBins / (nodeMax - nodeMin);

        for (int b = 0; b < numBins; b++)
        {
            spatialBins[b] = SpatialBin{};
        }

        for (uint i = begin; i < end; i++)
        {
            uint sortedIndex   = s_triIndex[i];
            uint triangleIndex = s_sortedTris[sortedIndex];
            math::vec3 v0, v1, v2;
            triVerts(triangleIndex, v0, v1, v2);

            AABB triAABB;
            triAABB.extend(v0);
            triAABB.extend(v1);
            triAABB.extend(v2);

            int binMin = std::clamp((int)((triAABB.min[axis] - nodeMin) * spatialBinScale), 0, numBins - 1);
            int binMax = std::clamp((int)((triAABB.max[axis] - nodeMin) * spatialBinScale), 0, numBins - 1);

            spatialBins[binMin].countIn++;
            spatialBins[binMax].countOut++;

            AABB binRegion;
            binRegion.min = s_nodes[nodeIndex].aabb.min;
            binRegion.max = s_nodes[nodeIndex].aabb.max;
            for (int b = binMin; b <= binMax; b++)
            {
                binRegion.min[axis] = nodeMin + (float)b       / spatialBinScale;
                binRegion.max[axis] = nodeMin + (float)(b + 1) / spatialBinScale;

                AABB clipped = clipTriangleToAABB(v0, v1, v2, binRegion);
                if (clipped.valid())
                {
                    spatialBins[b].aabb.extend(clipped);
                }
            }
        }

        AABB spa; uint spLc = 0;
        for (int b = 0; b < numBins - 1; b++)
        {
            spa.extend(spatialBins[b].aabb);
            spLc += spatialBins[b].countIn;
            spLAABB[b]  = spa;
            spLCount[b] = spLc;
        }

        // Left count = accumulated entry; right count = accumulated exit (duplicates allowed)
        AABB spra; uint spRc = 0;
        for (int b = numBins - 1; b >= 1; b--)
        {
            spra.extend(spatialBins[b].aabb);
            spRc += spatialBins[b].countOut;
            uint spLc2 = spLCount[b - 1];
            if (spLc2 == 0 || spRc == 0)
            {
                continue;
            }

            float cost = BVH_C_TRAV + BVH_C_ISECT *
                (spLAABB[b - 1].half_area() * (float)spLc2 + spra.half_area() * (float)spRc) * parentInvArea;

            // Strict < so equal-cost ties prefer object split (no ref-count increase)
            if (cost < bestCost)
            {
                bestCost              = cost;
                bestAxis              = axis;
                bestBin               = b;
                isSpatialSplit        = true;
                bestNodeMin           = nodeMin;
                bestSpatialBinScale   = spatialBinScale;
            }
        }
    }

    // ── Phase 3: Apply the winning split ──────────────────────────────────────

    if (bestAxis == -1)
    {
        s_nodes[nodeIndex].isLeaf = true;
        return false;
    }

    // Shared helper: partition s_triIndex[begin..end) by object-split bin and commit child nodes
    auto commitObjectSplit = [&](int axis, int bin, float cmin, float scale) -> bool
    {
        auto mid = std::partition(
            s_triIndex.begin() + begin,
            s_triIndex.begin() + end,
            [&](uint sortedIndex) {
                float c = s_centroids[s_sortedTris[sortedIndex]][axis];
                return std::clamp((int)((c - cmin) * scale), 0, numBins - 1) < bin;
            });

        uint splitPos = (uint)(mid - s_triIndex.begin());
        if (splitPos == begin || splitPos == end)
        {
            s_nodes[nodeIndex].isLeaf = true;
            return false;
        }

        if (s_totalNodeCount + 2 > (uint)s_nodes.size())
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
    };

    // ── Object split path ─────────────────────────────────────────────────────
    if (!isSpatialSplit)
    {
        return commitObjectSplit(bestAxis, bestBin, bestCMin, bestScale);
    }

    // ── Spatial split path ────────────────────────────────────────────────────
    float splitPlane = bestNodeMin + (float)bestBin / bestSpatialBinScale;

    // 3-way partition of s_triIndex[begin..end):
    //   [left-only | straddling | right-only]
    // left-only:   triAABB.max[bestAxis] <= splitPlane  (triangles touching the plane exactly go left)
    // right-only:  triAABB.min[bestAxis] >= splitPlane
    // straddling:  everything else
    // Note: a zero-extent triangle lying exactly on the plane goes left by the <= predicate;
    // the SAH estimate may be off by at most one triangle for such degenerate cases.

    // bestAxis is finalized by Phase 2 before this lambda is defined
    auto triAxisBounds = [&](uint sortedIndex) -> std::pair<float, float>
    {
        uint triangleIndex = s_sortedTris[sortedIndex];
        math::vec3 v0, v1, v2;
        triVerts(triangleIndex, v0, v1, v2);
        AABB triAABB;
        triAABB.extend(v0);
        triAABB.extend(v1);
        triAABB.extend(v2);
        return { triAABB.min[bestAxis], triAABB.max[bestAxis] };
    };

    auto leftEndIt = std::partition(
        s_triIndex.begin() + begin,
        s_triIndex.begin() + end,
        [&](uint sortedIndex) {
            return triAxisBounds(sortedIndex).second <= splitPlane;
        });

    uint leftEnd = (uint)(leftEndIt - s_triIndex.begin());

    auto straddleEndIt = std::partition(
        s_triIndex.begin() + leftEnd,
        s_triIndex.begin() + end,
        [&](uint sortedIndex) {
            return triAxisBounds(sortedIndex).first < splitPlane;
        });

    uint straddleEnd   = (uint)(straddleEndIt - s_triIndex.begin());
    uint leftCount     = leftEnd - begin;
    uint straddleCount = straddleEnd - leftEnd;
    uint rightCount    = end - straddleEnd;

    auto applyObjectSplitFallback = [&]() -> bool
    {
        if (bestObjectAxis == -1)
        {
            s_nodes[nodeIndex].isLeaf = true;
            return false;
        }
        return commitObjectSplit(bestObjectAxis, bestObjectBin, bestObjectCMin, bestObjectScale);
    };

    if ((leftCount + straddleCount == 0) || (straddleCount + rightCount == 0))
    {
        return applyObjectSplitFallback();
    }

    uint kNeededTriIndex   = rightCount + straddleCount;
    uint kNeededSortedTris = straddleCount;
    if (s_triIndexSize + kNeededTriIndex > (uint)s_triIndex.size() ||
        s_sortedTrisSize + kNeededSortedTris > (uint)s_sortedTris.size() ||
        s_totalNodeCount + 2 > (uint)s_nodes.size())
    {
        return applyObjectSplitFallback();
    }

    uint rightBegin = s_triIndexSize;

    // Right-only: copy s_triIndex reference (no new s_sortedTris slot)
    for (uint k = 0; k < rightCount; k++)
    {
        s_triIndex[s_triIndexSize] = s_triIndex[straddleEnd + k];
        s_triIndexSize++;
    }

    // Straddling: duplicate into s_sortedTris (centroid lookup is by triangleIndex,
    // so the same s_centroids entry is shared — no centroid duplication needed)
    for (uint k = 0; k < straddleCount; k++)
    {
        uint oldSortedIndex            = s_triIndex[leftEnd + k];
        s_sortedTris[s_sortedTrisSize] = s_sortedTris[oldSortedIndex];
        s_triIndex  [s_triIndexSize]   = s_sortedTrisSize;
        s_sortedTrisSize++;
        s_triIndexSize++;
    }

    uint rightSize = rightCount + straddleCount;
    uint leftBegin = begin;
    uint leftSize  = leftCount + straddleCount;

    uint leftIndex  = s_totalNodeCount++;
    uint rightIndex = s_totalNodeCount++;

    s_nodes[nodeIndex].isLeaf     = false;
    s_nodes[nodeIndex].childID[0] = leftIndex;
    s_nodes[nodeIndex].childID[1] = rightIndex;

    // Child AABBs are computed by clipping all triangles in each child's range against
    // the split half-space. This uses the full parent AABB as the clip box (not per-bin
    // clip boxes), so child SA may be slightly looser than the per-bin SAH estimate —
    // the difference is bounded and acceptable for traversal quality.
    AABB leftClipBox  = s_nodes[nodeIndex].aabb;
    AABB rightClipBox = s_nodes[nodeIndex].aabb;
    leftClipBox.max[bestAxis]  = splitPlane;
    rightClipBox.min[bestAxis] = splitPlane;

    auto clipExtend = [&](AABB& out, uint k, const AABB& clipBox)
    {
        uint triIdx = s_sortedTris[s_triIndex[k]];
        math::vec3 v0, v1, v2;
        triVerts(triIdx, v0, v1, v2);
        AABB clipped = clipTriangleToAABB(v0, v1, v2, clipBox);
        if (clipped.valid())
        {
            out.extend(clipped);
        }
    };

    AABB leftChildAABB;
    for (uint k = leftBegin; k < leftBegin + leftSize; k++)
    {
        clipExtend(leftChildAABB, k, leftClipBox);
    }

    AABB rightChildAABB;
    for (uint k = rightBegin; k < rightBegin + rightSize; k++)
    {
        clipExtend(rightChildAABB, k, rightClipBox);
    }

    s_nodes[leftIndex].aabb           = leftChildAABB;
    s_nodes[leftIndex].isLeaf         = true;
    s_nodes[leftIndex].beginTriIndex  = leftBegin;
    s_nodes[leftIndex].triSize        = leftSize;

    s_nodes[rightIndex].aabb          = rightChildAABB;
    s_nodes[rightIndex].isLeaf        = true;
    s_nodes[rightIndex].beginTriIndex = rightBegin;
    s_nodes[rightIndex].triSize       = rightSize;

    s_queue.push_back({ leftIndex,  true });
    s_queue.push_back({ rightIndex, true });
    return true;
}

} // namespace bvh
