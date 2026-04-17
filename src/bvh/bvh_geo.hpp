#pragma once

#include "../math/math.hpp"

namespace bvh
{
    // rings must be >= 2, sectors must be >= 3
    void addSphere(math::vec3 center, float radius, int rings = 16, int sectors = 32);

    void addBox(math::vec3 center, math::vec3 he, math::quat orient);
}
