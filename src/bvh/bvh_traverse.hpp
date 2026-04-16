#pragma once

#include <string>
#include <limits>
#include "../math/math.hpp"
#include "bvh_defines.hpp"

namespace bvh
{
    struct Ray
    {
        math::vec3 origin;
        math::vec3 dir;
        float tmin = 0.f;
        float tmax = std::numeric_limits<float>::infinity();
    };

    struct Hit
    {
        bool       hit      = false;
        float      t        = 0.f;
        float      u        = 0.f;   // barycentric
        float      v        = 0.f;   // barycentric
        uint       triIndex = 0;
        math::vec3 normal;            // interpolated from vertex normals
    };

    bool loadBVH(const std::string& path);
    Hit  trace  (const Ray& ray);
}
