#pragma once

#include "AABB.hpp"
#include "bvh_defines.hpp"

namespace bvh
{
    AABB computeRangeAABB(uint begin, uint end);
    bool initRoot();
    bool bfsLoop();
}
