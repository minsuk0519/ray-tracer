#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_bake_types.hpp"
#include "bvh_morton.hpp"

#include "../math/math.hpp"

#include <vector>
#include <cassert>
#include <cstring>
#include <algorithm>

namespace bvh
{

// ── Morton helpers (internal) ─────────────────────────────────────────────────

static uint64_t expandBits(uint v)
{
    uint64_t x = v;
    x = (x | (x << 32)) & 0x1f00000000ffff;
    x = (x | (x << 16)) & 0x1f0000ff0000ff;
    x = (x | (x <<  8)) & 0x100f00f00f00f00f;
    x = (x | (x <<  4)) & 0x10c30c30c30c30c3;
    x = (x | (x <<  2)) & 0x1249249249249249;
    return x;
}

static uint64_t packMortonBit(uint x, uint y, uint z)
{
    assert(x <= MORTON_UNIT_MAX && "x exceeds 21 bits");
    assert(y <= MORTON_UNIT_MAX && "y exceeds 21 bits");
    assert(z <= MORTON_UNIT_MAX && "z exceeds 21 bits");
    return (expandBits(x) << 2) | (expandBits(y) << 1) | expandBits(z);
}

// ── Morton code computation ───────────────────────────────────────────────────

static bool computeMortonCodes()
{
    const uint triCount = (uint)s_triangles.size();

    s_mortonData.resize(triCount);
    s_sortedTris.resize((uint)(triCount * SPATIAL_TRICOUNT_MULTIPLIER));  // extra capacity for spatial splits
    s_triIndex.resize((uint)(triCount * SPATIAL_TRICOUNT_MULTIPLIER));    // mirrors s_sortedTris; s_triIndex[k] → s_sortedTris[k]

    // pass 1: compute scene AABB
    s_sceneAABB = AABB();
    for (uint i = 0; i < triCount; i++)
    {
        const Triangle& tri = s_triangles[i];
        const Vertex&   v0  = s_vertices[tri.v[0]];
        const Vertex&   v1  = s_vertices[tri.v[1]];
        const Vertex&   v2  = s_vertices[tri.v[2]];

        s_sceneAABB.extend(math::vec3(v0.x, v0.y, v0.z));
        s_sceneAABB.extend(math::vec3(v1.x, v1.y, v1.z));
        s_sceneAABB.extend(math::vec3(v2.x, v2.y, v2.z));
    }

    // pass 2: compute centroids, quantize directly, pack Morton codes
    math::vec3 sceneMin  = s_sceneAABB.min;
    math::vec3 sceneSize = s_sceneAABB.max - s_sceneAABB.min;
    if (sceneSize.x == 0.f)
    {
        sceneSize.x = 1.f;
    }
    if (sceneSize.y == 0.f)
    {
        sceneSize.y = 1.f;
    }
    if (sceneSize.z == 0.f)
    {
        sceneSize.z = 1.f;
    }

    // quantize directly to 21-bit integers without intermediate [0,1] normalization
    auto quantize = [](float c, float min, float size) -> uint {
        uint q = (uint)((c - min) * (float)MORTON_UNIT_MAX / size);
        return q > MORTON_UNIT_MAX ? MORTON_UNIT_MAX : q;
    };

    for (uint i = 0; i < triCount; i++)
    {
        const Triangle& tri = s_triangles[i];
        const Vertex&   v0  = s_vertices[tri.v[0]];
        const Vertex&   v1  = s_vertices[tri.v[1]];
        const Vertex&   v2  = s_vertices[tri.v[2]];

        float cx = (v0.x + v1.x + v2.x) / 3.0f;
        float cy = (v0.y + v1.y + v2.y) / 3.0f;
        float cz = (v0.z + v1.z + v2.z) / 3.0f;

        uint qx = quantize(cx, sceneMin.x, sceneSize.x);
        uint qy = quantize(cy, sceneMin.y, sceneSize.y);
        uint qz = quantize(cz, sceneMin.z, sceneSize.z);

        s_mortonData[i].mortonCode = packMortonBit(qx, qy, qz);
        s_sortedTris[i]            = i;
        s_triIndex[i]              = i;
    }

    s_triIndexSize   = triCount;
    s_sortedTrisSize = triCount;

    return true;
}

// ── Radix sort ────────────────────────────────────────────────────────────────

bool sortTrisByMorton()
{
    const uint triCount = (uint)s_triangles.size();
    if (triCount == 0)
    {
        return false;
    }
    if (!computeMortonCodes())
    {
        return false;
    }

    constexpr int RADIX_BITS  = 8;
    constexpr int RADIX_SIZE  = 1 << RADIX_BITS;  // 256
    constexpr int RADIX_MASK  = RADIX_SIZE - 1;
    constexpr int PASSES      = (sizeof(uint64_t) * 8) / RADIX_BITS;  // 8 passes for 64-bit key

    std::vector<uint> temp(triCount);
    uint count[RADIX_SIZE];

    for (int pass = 0; pass < PASSES; pass++)
    {
        const int shift = pass * RADIX_BITS;

        memset(count, 0, sizeof(count));

        // count
        for (uint i = 0; i < triCount; i++)
        {
            uint bucket = (s_mortonData[s_sortedTris[i]].mortonCode >> shift) & RADIX_MASK;
            count[bucket]++;
        }

        // prefix sum
        uint total = 0;
        for (int b = 0; b < RADIX_SIZE; b++)
        {
            uint c    = count[b];
            count[b]  = total;
            total    += c;
        }

        // scatter
        for (uint i = 0; i < triCount; i++)
        {
            uint bucket    = (s_mortonData[s_sortedTris[i]].mortonCode >> shift) & RADIX_MASK;
            temp[count[bucket]++] = s_sortedTris[i];
        }

        std::swap(s_sortedTris, temp);
    }

    return true;
}

} // namespace bvh
