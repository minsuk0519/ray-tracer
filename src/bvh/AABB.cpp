#include "AABB.hpp"

#include <limits>

AABB::AABB() : 
    min(vec3( std::numeric_limits<float>::infinity())),
    max(vec3(-std::numeric_limits<float>::infinity())) {}

AABB::AABB(vec3 p1, vec3 p2) : 
    min(glm::min(p1, p2)), max(glm::max(p1, p2)) {}

void AABB::extend(vec3 p)
{
    min = glm::min(min, p);
    max = glm::max(max, p);
}

void AABB::extend(const AABB& b)
{
    min = glm::min(min, b.min);
    max = glm::max(max, b.max);
}

vec3 AABB::center() const
{
    return (min + max) * 0.5f;
}

float AABB::half_area() const
{
    vec3 d = max - min;
    return d.x * d.y + d.y * d.z + d.z * d.x;
}

Axis AABB::largest_axis() const
{
    vec3 d = max - min;
    if (d.x >= d.y && d.x >= d.z)
    {
        return Axis::AXIS_X;
    }
    if (d.y >= d.z)
    {
        return Axis::AXIS_Y;
    }
    return Axis::AXIS_Z;
}

bool AABB::valid() const
{
    return min.x <= max.x && min.y <= max.y && min.z <= max.z;
}

AABB merge(const AABB& a, const AABB& b)
{
    AABB result;
    result.min = glm::min(a.min, b.min);
    result.max = glm::max(a.max, b.max);
    return result;
}
