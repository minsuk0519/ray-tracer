#pragma once

#include "libs/glm/glm/glm.hpp"
#include "libs/glm/glm/gtc/constants.hpp"

using glm::vec3;

enum class Axis { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };

struct AABB 
{
    vec3 min, max;

    AABB();
    AABB(vec3 p1, vec3 p2);

    void extend(vec3 p);
    void extend(const AABB& b);

    vec3  center()       const;
    float half_area()    const;
    Axis  largest_axis() const;
    bool  valid()        const;
};

AABB merge(const AABB& a, const AABB& b);
