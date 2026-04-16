#define BAKING
#include "bvh.hpp"
#include "bvh_bake_state.hpp"
#include "bvh_IO.hpp"
#include "bvh_geo.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "../math/math.hpp"

namespace bvh
{

// ── Internal helpers ──────────────────────────────────────────────────────────

static bool loadMesh(const std::string& path, const math::mat4& transform)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path,
        aiProcess_Triangulate | aiProcess_GenSmoothNormals);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        fprintf(stderr, "Error : assimp failed to load %s : %s on loadMesh()\n", path.c_str(), importer.GetErrorString());
        return false;
    }

    math::mat4 normalTransform = math::transpose(math::inverse(transform));

    for (unsigned int mi = 0; mi < scene->mNumMeshes; mi++)
    {
        aiMesh* aimesh = scene->mMeshes[mi];
        uint vertexOffset = (uint)s_vertices.size();

        for (unsigned int vi = 0; vi < aimesh->mNumVertices; vi++)
        {
            aiVector3D p = aimesh->mVertices[vi];
            aiVector3D n = aimesh->mNormals[vi];

            math::vec4 tp = transform       * math::vec4(p.x, p.y, p.z, 1.0f);
            math::vec4 tn = normalTransform * math::vec4(n.x, n.y, n.z, 0.0f);

            Vertex v;
            v.x  = tp.x; v.y  = tp.y; v.z  = tp.z;
            v.nx = tn.x; v.ny = tn.y; v.nz = tn.z;
            s_vertices.push_back(v);
        }

        for (unsigned int fi = 0; fi < aimesh->mNumFaces; fi++)
        {
            if (aimesh->mFaces[fi].mNumIndices == 3)
            {
                Triangle tri;
                tri.v[0] = vertexOffset + aimesh->mFaces[fi].mIndices[0];
                tri.v[1] = vertexOffset + aimesh->mFaces[fi].mIndices[1];
                tri.v[2] = vertexOffset + aimesh->mFaces[fi].mIndices[2];
                s_triangles.push_back(tri);
            }
        }
    }

    return true;
}

bool initGeos()
{
    if (s_vertices.empty())
    {
        fprintf(stderr, "Error : no vertices loaded on initGeos()\n");
        return false;
    }
    if (s_triangles.empty())
    {
        fprintf(stderr, "Error : no triangles loaded on initGeos()\n");
        return false;
    }

    const uint vertCount = (uint)s_vertices.size();
    const uint triCount  = (uint)s_triangles.size();

    // validate all triangle vertex indices are in bounds
    for (uint i = 0; i < triCount; i++)
    {
        for (uint k = 0; k < 3; k++)
        {
            if (s_triangles[i].v[k] >= vertCount)
            {
                fprintf(stderr, "Error : triangle %u has out-of-bounds vertex index %u (vertCount=%u) on initGeos()\n",
                        i, s_triangles[i].v[k], vertCount);
                return false;
            }
        }
    }

    // normalize vertex normals — the inverse-transpose normal transform in loadMesh
    // does not preserve length, so we re-normalize here
    for (uint i = 0; i < vertCount; i++)
    {
        float len = std::hypot(s_vertices[i].nx, s_vertices[i].ny, s_vertices[i].nz);
        if (len > 1e-6f)
        {
            s_vertices[i].nx /= len;
            s_vertices[i].ny /= len;
            s_vertices[i].nz /= len;
        }
        else
        {
            s_vertices[i].nx = 0.f;  // degenerate normal — fallback to up
            s_vertices[i].ny = 1.f;
            s_vertices[i].nz = 0.f;
        }
    }

    return true;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool readScene()
{
    if (s_scenePath.empty())
    {
        fprintf(stderr, "Error : empty scene path on readScene()\n");
        return false;
    }

    std::ifstream file(s_scenePath);
    if (!file.is_open())
    {
        fprintf(stderr, "Error : failed to open scene file %s on readScene()\n", s_scenePath.c_str());
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        if (!(iss >> token))
        {
            continue;
        }

        if (token == "sphere")
        {
            // syntax: sphere <cx> <cy> <cz> <radius> [<rings> <sectors>]
            float cx, cy, cz, radius;
            if (!(iss >> cx >> cy >> cz >> radius))
            {
                fprintf(stderr, "Error : malformed sphere line on readScene()\n");
                continue;
            }

            int rings   = 16;
            int sectors = 32;
            int r, s;
            if (iss >> r >> s)
            {
                rings   = r;
                sectors = s;
            }

            addSphere(math::vec3(cx, cy, cz), radius, rings, sectors);
            fprintf(stdout, "Info : Added Sphere\n");
            continue;
        }

        if (token == "box")
        {
            // syntax: box <cx> <cy> <cz> <hx> <hy> <hz> q <qx> <qy> <qz> <qw>
            float cx, cy, cz, hx, hy, hz;
            std::string orientType;
            float qx, qy, qz, qw;
            if (!(iss >> cx >> cy >> cz >> hx >> hy >> hz >> orientType >> qx >> qy >> qz >> qw))
            {
                fprintf(stderr, "Error : malformed box line on readScene()\n");
                continue;
            }

            if (orientType != "q")
            {
                fprintf(stderr, "Warning : unknown orientation type '%s', defaulting to identity on readScene()\n", orientType.c_str());
                qx = 0.f; qy = 0.f; qz = 0.f; qw = 1.f;
            }

            addBox(math::vec3(cx, cy, cz), math::vec3(hx, hy, hz), math::quat(qw, qx, qy, qz));
            fprintf(stdout, "Info : Added Box\n");
            continue;
        }

        if (token != "mesh")
        {
            continue;
        }

        // syntax: mesh <filename> <tx> <ty> <tz> <scale> q <qx> <qy> <qz> <qw>
        std::string meshPath;
        float tx, ty, tz, scale;
        std::string orientType;
        float qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;

        if (!(iss >> meshPath >> tx >> ty >> tz >> scale >> orientType))
        {
            continue;
        }

        if (orientType == "q")
        {
            iss >> qx >> qy >> qz >> qw;
        }
        else
        {
            fprintf(stderr, "Warning : unknown orientation type '%s', defaulting to identity on readScene()\n", orientType.c_str());
        }

        fprintf(stdout, "Info : Found Mesh : %s\n", meshPath.c_str());

        math::mat4 transform =
            math::translate(math::mat4(1.f), math::vec3(tx, ty, tz)) *
            math::scale(math::mat4(1.f), math::vec3(scale)) *
            math::toMat4(math::quat(qw, qx, qy, qz));

        if (!loadMesh(meshPath, transform))
        {
            fprintf(stderr, "Error : failed to load mesh %s on readScene()\n", meshPath.c_str());
            return false;
        }
    }

    return true;
}

bool writeBakedData()
{
    // derive output path: replace extension with .bvh
    std::string outPath = s_scenePath;
    auto dot = outPath.rfind('.');
    if (dot != std::string::npos)
    {
        outPath = outPath.substr(0, dot);
    }
    outPath += ".bvh";

    std::ofstream out(outPath, std::ios::binary);
    if (!out.is_open())
    {
        fprintf(stderr, "Error : failed to open output file %s on writeBakedData()\n", outPath.c_str());
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
        fprintf(stderr, "Error : write error for %s on writeBakedData()\n", outPath.c_str());
        return false;
    }

    fprintf(stdout, "BVH baked: %s  (%u nodes, %u tris, %u verts)\n",
            outPath.c_str(), nodeCount, triCount, vertCount);
    return true;
}

} // namespace bvh
