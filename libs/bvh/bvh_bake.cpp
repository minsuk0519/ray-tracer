#define BAKING
#include "bvh.hpp"

#include <vector>
#include <cassert>
#include <algorithm>
#include <limits>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "libs/glm/glm/glm.hpp"
#include "libs/glm/glm/gtc/matrix_transform.hpp"
#include "libs/glm/glm/gtx/quaternion.hpp"

namespace bvh
{

// ── TODO: DELETE THIS — placeholder until math library is ready ───────────────
struct Vertex { float x, y, z, nx, ny, nz; };
// ─────────────────────────────────────────────────────────────────────────────

// ── Baking state ──────────────────────────────────────────────────────────────

static std::string                s_scenePath;

static std::vector<Vertex>        s_vertices;
static std::vector<Triangle>      s_triangles;
static std::vector<BVHNode>       s_nodes;
static std::vector<MortonData>    s_mortonData;       // one per triangle, indexed by triIndex
static std::vector<SAHData>       s_sahData;
static std::vector<NodeBakingJob> s_queue;
static std::vector<uint>          s_sortedTris;       // sortedTriangleIndex: original tri index at each Morton-sorted position
static std::vector<uint>          s_triIndex;         // indirection: s_triIndex[k] → s_sortedTris[k]; leaf beginTriIndex indexes here
static uint                       s_totalNodeCount = 0;
static AABB                       s_sceneAABB;

// ── Morton helpers ────────────────────────────────────────────────────────────

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

// ── Scene loading ─────────────────────────────────────────────────────────────

static bool loadMesh(const std::string& path, const glm::mat4& transform)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path,
        aiProcess_Triangulate | aiProcess_GenSmoothNormals);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        fprintf(stderr, "Assimp error loading %s: %s\n", path.c_str(), importer.GetErrorString());
        return false;
    }

    glm::mat4 normalTransform = glm::transpose(glm::inverse(transform));

    for (unsigned int mi = 0; mi < scene->mNumMeshes; mi++)
    {
        aiMesh* aimesh = scene->mMeshes[mi];
        uint vertexOffset = (uint)s_vertices.size();

        for (unsigned int vi = 0; vi < aimesh->mNumVertices; vi++)
        {
            aiVector3D p = aimesh->mVertices[vi];
            aiVector3D n = aimesh->mNormals[vi];

            glm::vec4 tp = transform       * glm::vec4(p.x, p.y, p.z, 1.0f);
            glm::vec4 tn = normalTransform * glm::vec4(n.x, n.y, n.z, 0.0f);

            Vertex v;
            v.x  = tp.x; v.y  = tp.y; v.z  = tp.z;
            v.nx = tn.x; v.ny = tn.y; v.nz = tn.z;
            s_vertices.push_back(v);
        }

        for (unsigned int fi = 0; fi < aimesh->mNumFaces; fi++)
        {
            aiFace& face = aimesh->mFaces[fi];
            if (face.mNumIndices == 3)
            {
                Triangle tri;
                tri.v[0] = vertexOffset + face.mIndices[0];
                tri.v[1] = vertexOffset + face.mIndices[1];
                tri.v[2] = vertexOffset + face.mIndices[2];
                s_triangles.push_back(tri);
            }
        }
    }

    return true;
}

static bool readFile()
{
    if (s_scenePath.empty())
        return false;

    std::ifstream file(s_scenePath);
    if (!file.is_open())
    {
        fprintf(stderr, "Failed to open scene file: %s\n", s_scenePath.c_str());
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        if (!(iss >> token) || token != "mesh")
            continue;

        // syntax: mesh <filename> <tx> <ty> <tz> <scale> q <qx> <qy> <qz> <qw>
        std::string meshPath;
        float tx, ty, tz, scale;
        std::string orientType;
        float qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;

        if (!(iss >> meshPath >> tx >> ty >> tz >> scale >> orientType))
            continue;

        if (orientType == "q")
            iss >> qx >> qy >> qz >> qw;

        glm::mat4 transform =
            glm::translate(glm::mat4(1.f), glm::vec3(tx, ty, tz)) *
            glm::scale(glm::mat4(1.f), glm::vec3(scale)) *
            glm::toMat4(glm::quat(qw, qx, qy, qz));

        if (!loadMesh(meshPath, transform))
            return false;
    }

    return true;
}

static bool initGeos()
{
    if (s_vertices.empty())
    {
        fprintf(stderr, "initGeos: no vertices loaded\n");
        return false;
    }
    if (s_triangles.empty())
    {
        fprintf(stderr, "initGeos: no triangles loaded\n");
        return false;
    }

    const uint vertCount = (uint)s_vertices.size();
    const uint triCount  = (uint)s_triangles.size();

    // validate all triangle vertex indices are in bounds
    for (uint i = 0; i < triCount; i++)
    {
        const Triangle& tri = s_triangles[i];
        for (int k = 0; k < 3; k++)
        {
            if (tri.v[k] >= vertCount)
            {
                fprintf(stderr, "initGeos: triangle %u has out-of-bounds vertex index %u (vertCount=%u)\n",
                        i, tri.v[k], vertCount);
                return false;
            }
        }
    }

    // normalize vertex normals — the inverse-transpose normal transform in loadMesh
    // does not preserve length, so we re-normalize here
    for (uint i = 0; i < vertCount; i++)
    {
        Vertex& v   = s_vertices[i];
        float   len = std::sqrt(v.nx * v.nx + v.ny * v.ny + v.nz * v.nz);
        if (len > 1e-6f)
        {
            v.nx /= len;
            v.ny /= len;
            v.nz /= len;
        }
        else
        {
            v.nx = 0.f; v.ny = 1.f; v.nz = 0.f;  // degenerate normal — fallback to up
        }
    }

    return true;
}

static bool readScene()
{
    if (!readFile())  return false;
    if (!initGeos())  return false;
    return true;
}

// ── Pipeline steps ────────────────────────────────────────────────────────────

bool preBake()
{
    if (!readScene())
        return false;

    if (s_vertices.empty() || s_triangles.empty())
        return false;

    const uint triCount = (uint)s_triangles.size();

    s_mortonData.resize(triCount);
    s_sortedTris.resize((uint)(triCount * 1.5f));  // extra capacity for spatial splits
    s_triIndex.resize((uint)(triCount * 1.5f));    // mirrors s_sortedTris; s_triIndex[k] → s_sortedTris[k]

    // pass 1: compute scene AABB
    AABB sceneAABB;
    s_sceneAABB = AABB();
    for (uint i = 0; i < triCount; i++)
    {
        const Triangle& tri = s_triangles[i];
        const Vertex&   v0  = s_vertices[tri.v[0]];
        const Vertex&   v1  = s_vertices[tri.v[1]];
        const Vertex&   v2  = s_vertices[tri.v[2]];

        sceneAABB.extend(glm::vec3(v0.x, v0.y, v0.z));
        sceneAABB.extend(glm::vec3(v1.x, v1.y, v1.z));
        sceneAABB.extend(glm::vec3(v2.x, v2.y, v2.z));
    }

    // pass 2: compute centroids, quantize directly, pack Morton codes
    s_sceneAABB = sceneAABB;
    glm::vec3 sceneMin  = sceneAABB.min;
    glm::vec3 sceneSize = sceneAABB.max - sceneAABB.min;
    if (sceneSize.x == 0.f) sceneSize.x = 1.f;
    if (sceneSize.y == 0.f) sceneSize.y = 1.f;
    if (sceneSize.z == 0.f) sceneSize.z = 1.f;

    for (uint i = 0; i < triCount; i++)
    {
        const Triangle& tri = s_triangles[i];
        const Vertex&   v0  = s_vertices[tri.v[0]];
        const Vertex&   v1  = s_vertices[tri.v[1]];
        const Vertex&   v2  = s_vertices[tri.v[2]];

        float cx = (v0.x + v1.x + v2.x) / 3.0f;
        float cy = (v0.y + v1.y + v2.y) / 3.0f;
        float cz = (v0.z + v1.z + v2.z) / 3.0f;

        // quantize directly to 21-bit integers without intermediate [0,1] normalization
        auto quantize = [&](float c, float min, float size) -> uint {
            uint q = (uint)((c - min) * (float)MORTON_UNIT_MAX / size);
            return q > MORTON_UNIT_MAX ? MORTON_UNIT_MAX : q;
        };

        uint qx = quantize(cx, sceneMin.x, sceneSize.x);
        uint qy = quantize(cy, sceneMin.y, sceneSize.y);
        uint qz = quantize(cz, sceneMin.z, sceneSize.z);

        s_mortonData[i].mortonCode = packMortonBit(qx, qy, qz);
        s_sortedTris[i]            = i;
        s_triIndex[i]              = i;
    }

    return true;
}

bool sortTrisByMorton()
{
    const uint triCount = (uint)s_triangles.size();
    if (triCount == 0) return false;

    constexpr int RADIX_BITS  = 8;
    constexpr int RADIX_SIZE  = 1 << RADIX_BITS;  // 256
    constexpr int RADIX_MASK  = RADIX_SIZE - 1;
    constexpr int PASSES      = sizeof(uint64_t);  // 8 passes for 64-bit key

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
static AABB computeRangeAABB(uint begin, uint end)
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
                continue;
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
                continue;
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
bool reorderNodes()
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
bool reorderTriangles() { return false; }
bool writeBakedData()   { return false; }
bool finishBake()       { return false; }

bool bakeBVH()
{
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
