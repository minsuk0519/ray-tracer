#pragma once

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace bvh
{
    // UV sphere centred at `center` with given `radius`.
    // `rings`   >= 2 : number of latitude bands (poles count as rings), default 16
    // `sectors` >= 3 : number of longitude divisions, default 32
    void addSphere(glm::vec3 center, float radius, int rings = 16, int sectors = 32);

    // Box centred at `center` with per-axis half-extents `he` and orientation `orient`.
    void addBox(glm::vec3 center, glm::vec3 he, glm::quat orient);
}
