#include "AABB.hpp"

#include <limits>

AABB::AABB() : 
    min(vec3( std::numeric_limits<float>::infinity())),
    max(vec3(-std::numeric_limits<float>::infinity())) {}

AABB::AABB(vec3 p1, vec3 p2) : 
    min(math::min(p1, p2)), max(math::max(p1, p2)) {}

void AABB::extend(vec3 p)
{
    min = math::min(min, p);
    max = math::max(max, p);
}

void AABB::extend(const AABB& b)
{
    min = math::min(min, b.min);
    max = math::max(max, b.max);
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
    result.min = math::min(a.min, b.min);
    result.max = math::max(a.max, b.max);
    return result;
}

AABB clipAABB(const AABB& box, int axis, float splitPos, bool leftSide)
{
    if (leftSide)
    {
        // Keep everything at axis <= splitPos.
        // If box is entirely on the wrong side, return invalid.
        if (box.min[axis] >= splitPos)
        {
            return AABB();
        }
        AABB result = box;
        result.max[axis] = std::min(box.max[axis], splitPos);
        return result;
    }
    else
    {
        // Keep everything at axis >= splitPos.
        // If box is entirely on the wrong side, return invalid.
        if (box.max[axis] <= splitPos)
        {
            return AABB();
        }
        AABB result = box;
        result.min[axis] = std::max(box.min[axis], splitPos);
        return result;
    }
}

AABB clipTriangleToAABB(vec3 v0, vec3 v1, vec3 v2, const AABB& clipRegion)
{
    // Sutherland-Hodgman clip of a triangle polygon against all 6 AABB planes.
    // Maximum vertices after clipping a triangle against 6 planes: 9 (each plane
    // adds at most 1 vertex). Use a fixed-size buffer with capacity 12.
    const int kMaxVerts = 12;
    vec3 poly[kMaxVerts];
    vec3 tmp[kMaxVerts];
    int polySize = 0;
    int tmpSize  = 0;

    poly[0] = v0;
    poly[1] = v1;
    poly[2] = v2;
    polySize = 3;

    // Clip against 6 planes: -x, +x, -y, +y, -z, +z.
    // For each axis, clip against the min plane then the max plane.
    for (int axis = 0; axis < 3; ++axis)
    {
        // --- min plane: inside if point[axis] >= clipRegion.min[axis] ---
        {
            tmpSize = 0;
            float planePos = clipRegion.min[axis];
            for (int i = 0; i < polySize; ++i)
            {
                int next = (i + 1) % polySize;
                bool curInside  = poly[i][axis]    >= planePos;
                bool nextInside = poly[next][axis] >= planePos;

                if (curInside)
                {
                    tmp[tmpSize++] = poly[i];
                }

                if (curInside != nextInside)
                {
                    // Compute intersection with the plane.
                    float t = (planePos - poly[i][axis]) / (poly[next][axis] - poly[i][axis]);
                    tmp[tmpSize++] = poly[i] + t * (poly[next] - poly[i]);
                }
            }
            for (int i = 0; i < tmpSize; ++i)
            {
                poly[i] = tmp[i];
            }
            polySize = tmpSize;
            if (polySize == 0)
            {
                return AABB();
            }
        }

        // --- max plane: inside if point[axis] <= clipRegion.max[axis] ---
        {
            tmpSize = 0;
            float planePos = clipRegion.max[axis];
            for (int i = 0; i < polySize; ++i)
            {
                int next = (i + 1) % polySize;
                bool curInside  = poly[i][axis]    <= planePos;
                bool nextInside = poly[next][axis] <= planePos;

                if (curInside)
                {
                    tmp[tmpSize++] = poly[i];
                }

                if (curInside != nextInside)
                {
                    float t = (planePos - poly[i][axis]) / (poly[next][axis] - poly[i][axis]);
                    tmp[tmpSize++] = poly[i] + t * (poly[next] - poly[i]);
                }
            }
            for (int i = 0; i < tmpSize; ++i)
            {
                poly[i] = tmp[i];
            }
            polySize = tmpSize;
            if (polySize == 0)
            {
                return AABB();
            }
        }
    }

    // Compute AABB of the clipped polygon.
    AABB result;
    for (int i = 0; i < polySize; ++i)
    {
        result.extend(poly[i]);
    }
    return result;
}
