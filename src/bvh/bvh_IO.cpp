#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_IO.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/quaternion.hpp"

namespace bvh
{

// ── Internal helpers ──────────────────────────────────────────────────────────

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

// ── Public API ────────────────────────────────────────────────────────────────

bool readScene()
{
    if (!readFile())  return false;
    if (!initGeos())  return false;
    return true;
}

bool writeBakedData()
{
    // derive output path: replace extension with .bvh
    std::string outPath = s_scenePath;
    auto dot = outPath.rfind('.');
    if (dot != std::string::npos)
        outPath = outPath.substr(0, dot);
    outPath += ".bvh";

    std::ofstream out(outPath, std::ios::binary);
    if (!out.is_open())
    {
        fprintf(stderr, "writeBakedData: failed to open %s\n", outPath.c_str());
        return false;
    }

    const uint nodeCount = s_totalNodeCount;
    const uint triCount  = (uint)s_triangles.size();
    const uint vertCount = (uint)s_vertices.size();

    // header: version, counts
    out.write(reinterpret_cast<const char*>(&BVH_FILE_VERSION), sizeof(uint));
    out.write(reinterpret_cast<const char*>(&nodeCount),        sizeof(uint));
    out.write(reinterpret_cast<const char*>(&triCount),         sizeof(uint));
    out.write(reinterpret_cast<const char*>(&vertCount),        sizeof(uint));

    // payload
    out.write(reinterpret_cast<const char*>(s_nodes.data()),     nodeCount * sizeof(BVHNode));
    out.write(reinterpret_cast<const char*>(s_triangles.data()), triCount  * sizeof(Triangle));
    out.write(reinterpret_cast<const char*>(s_vertices.data()),  vertCount * sizeof(Vertex));

    if (!out.good())
    {
        fprintf(stderr, "writeBakedData: write error on %s\n", outPath.c_str());
        return false;
    }

    fprintf(stdout, "BVH baked: %s  (%u nodes, %u tris, %u verts)\n",
            outPath.c_str(), nodeCount, triCount, vertCount);
    return true;
}

} // namespace bvh
