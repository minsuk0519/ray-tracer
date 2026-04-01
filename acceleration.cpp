
#include "geom.h"
#include "raytrace.h"
#include "acceleration.h"

#include <bvh/sweep_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>

/////////////////////////////
// Vector and ray conversions
Ray RayFromBvh(const bvh::Ray<float>& r)
{
    return Ray(vec3FromBvh(r.origin), vec3FromBvh(r.direction));
}
bvh::Ray<float> RayToBvh(const Ray& r)
{
    return bvh::Ray<float>(vec3ToBvh(r.o), vec3ToBvh(r.d));
}


/////////////////////////////
// SimpleBox
bvh::Vector3<float> vec3ToBvh(const vec3& v)
{
    return bvh::Vector3<float>(v[0], v[1], v[2]);
}

vec3 vec3FromBvh(const bvh::Vector3<float>& v)
{
    return vec3(v[0], v[1], v[2]);
}

SimpleBox::SimpleBox() : bvh::BoundingBox<float>() {}
SimpleBox::SimpleBox(const vec3 v) : bvh::BoundingBox<float>(vec3ToBvh(v)) {}

SimpleBox& SimpleBox::extend(const vec3 v)
{
    bvh::BoundingBox<float>::extend(vec3ToBvh(v));
    return *this;
}


/////////////////////////////
// BvhShape

SimpleBox BvhShape::bounding_box() const
{
    return shape->boundingbox();
}

bvh::Vector3<float> BvhShape::center() const
{
    return bounding_box().center();
}

std::optional<Intersection> BvhShape::intersect(const bvh::Ray<float>& bvhray) const
{
    Intersection intersection;
    intersection.t = 0;

    if (shape->type == SHAPE_SPHERE)
    {
        Sphere* sphere = dynamic_cast<Sphere*>(shape);

        float r = sphere->r;
        bvh::Vector3<float> C = vec3ToBvh(sphere->C);

        float b = dot(bvhray.origin - C, bvhray.direction);
        float c = dot(bvhray.origin - C, bvhray.origin - C) - r * r;

        float discriminant = b * b - c;

        if (discriminant < 0) return std::nullopt;

        discriminant = sqrt(discriminant);

        float tplus = (-b + discriminant);
        float tminus = (-b - discriminant);

        if (tminus > epsilon) intersection.t = tminus;
        else if (tplus > epsilon) intersection.t = tplus;

        if (intersection.t != 0)
        {
            intersection.P = vec3FromBvh(intersection.t * bvhray.direction + bvhray.origin);
            intersection.N = normalize(intersection.P - vec3FromBvh(C));
            float theta = atan2(intersection.N.y, intersection.N.x);
            float phi = acos(intersection.N.z);
            intersection.UV = vec2(theta / (2 * PI) + 0.5, phi / PI);
            intersection.object = shape;
        }
    }
    else if (shape->type == SHAPE_BOX)
    {
        bvh::Vector3<float> C = vec3ToBvh(dynamic_cast<Box*>(shape)->C);
        bvh::Vector3<float> d = vec3ToBvh(dynamic_cast<Box*>(shape)->d);

        Slab slabs[3];
        slabs[0].N = vec3(1, 0, 0);
        slabs[0].d1 = -C[0];
        slabs[0].d2 = -C[0] - d[0];
        slabs[1].N = vec3(0, 1, 0);
        slabs[1].d1 = -C[1];
        slabs[1].d2 = -C[1] - d[1];
        slabs[2].N = vec3(0, 0, 1);
        slabs[2].d1 = -C[2];
        slabs[2].d2 = -C[2] - d[2];

        float t0 = 0;
        float t1 = INFINITY;
        vec3 N0;
        vec3 N1;
        Interval interval;
        for (int i = 0; i < 3; ++i)
        {
            interval.intersect(bvhray, slabs[i]);
            if (interval.t0 > interval.t1) continue;
            if (t0 < interval.t0)
            {
                t0 = interval.t0;
                N0 = interval.N0;
            }
            if (t1 > interval.t1)
            {
                t1 = interval.t1;
                N1 = interval.N1;
            }
        }

        if (t0 >= t1) return std::nullopt;

        if (t1 < t0)
        {
            if (t1 > epsilon)
            {
                intersection.t = t1;
                intersection.N = N1;
            }
            else if (t0 > epsilon)
            {
                intersection.t = t0;
                intersection.N = N0;
            }
        }
        else
        {
            if (t0 > epsilon)
            {
                intersection.t = t0;
                intersection.N = N0;
            }
            else if (t1 > epsilon)
            {
                intersection.t = t1;
                intersection.N = N1;
            }
        }

        if (intersection.t != 0)
        {
            intersection.P = vec3FromBvh(intersection.t * bvhray.direction + bvhray.origin);
            vec2 planeDir;
            vec3 pointTocenter = intersection.P - vec3FromBvh(C) - vec3(d[0] * 0.5, d[1] * 0.5, d[2] * 0.5);
            if (intersection.N == vec3(1, 0, 0) || intersection.N == vec3(-1, 0, 0))
            {
                planeDir = vec2(pointTocenter.y * 2 / d[1], pointTocenter.z * 2 / d[2]);
            }
            else if (intersection.N == vec3(0, 1, 0) || intersection.N == vec3(0, -1, 0))
            {
                planeDir = vec2(pointTocenter.x * 2 / d[0], pointTocenter.z * 2 / d[2]);
            }
            else if (intersection.N == vec3(0, 0, 1) || intersection.N == vec3(0, 0, -1))
            {
                planeDir = vec2(pointTocenter.x * 2 / d[0], pointTocenter.y * 2 / d[1]);
            }
            intersection.UV = vec2(planeDir.x * 0.5 + 0.5, planeDir.y * 0.5 + 0.5);
            intersection.object = shape;
        }
    }
    else if (shape->type == SHAPE_CYLINDER)
    {
        Cylinder* cylinder = dynamic_cast<Cylinder*>(shape);
        bvh::Vector3<float> B = vec3ToBvh(cylinder->B);
        bvh::Vector3<float> A = vec3ToBvh(cylinder->A);
        float r = cylinder->r;

        vec3 rot_A = normalize(cylinder->A);
        vec3 rot_B;
        if (dot(rot_A, vec3(0, 0, 1)) != 1) rot_B = normalize(cross(vec3(0, 0, 1), rot_A));
        else rot_B = normalize(cross(vec3(1, 0, 0), rot_A));
        vec3 rot_C = cross(rot_A, rot_B);

        mat3 rotation = glm::mat3(rot_B, rot_C, rot_A);
        rotation = glm::transpose(rotation);

        vec3 origin = rotation * (vec3FromBvh(bvhray.origin - B));
        vec3 direction = rotation * (vec3FromBvh(bvhray.direction));
        bvh::Ray<float> transformedray(vec3ToBvh(origin), vec3ToBvh(direction));

        Slab slab;
        slab.N = vec3(0, 0, 1);
        slab.d1 = 0;
        slab.d2 = -length(A);

        Interval interval;
        interval.intersect(transformedray, slab);
        float a0 = interval.t0;
        float a1 = interval.t1;

        float a = transformedray.direction[0] * transformedray.direction[0] + transformedray.direction[1] * transformedray.direction[1];
        float b = 2 * (transformedray.direction[0] * transformedray.origin[0] + transformedray.direction[1] * transformedray.origin[1]);
        float c = transformedray.origin[0] * transformedray.origin[0] + transformedray.origin[1] * transformedray.origin[1] - r * r;

        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return std::nullopt;

        discriminant = sqrt(discriminant);
        float b0 = (-b - discriminant) / (2 * a);
        float b1 = (-b + discriminant) / (2 * a);

        float t0;
        vec3 N0;
        if (a0 > b0)
        {
            t0 = a0;
            N0 = vec3(0, 0, 1);
        }
        else
        {
            t0 = b0;
            N0 = vec3(transformedray.origin[0] + b0 * transformedray.direction[0], transformedray.origin[1] + b0 * transformedray.direction[1], 0);
        }
        float t1;
        vec3 N1;
        if (a1 < b1)
        {
            t1 = a1;
            N1 = vec3(0, 0, -1);
        }
        else
        {
            t1 = b1;
            N1 = vec3(transformedray.origin[0] + b1 * transformedray.direction[0], transformedray.origin[1] + b1 * transformedray.direction[1], 0);
        }

        if (t0 > t1) return std::nullopt;

        if (t1 < t0)
        {
            if (t1 > epsilon)
            {
                intersection.t = t1;
                intersection.N = N1;
            }
            else if (t0 > epsilon)
            {
                intersection.t = t0;
                intersection.N = N0;
            }
        }
        else
        {
            if (t0 > epsilon)
            {
                intersection.t = t0;
                intersection.N = N0;
            }
            else if (t1 > epsilon)
            {
                intersection.t = t1;
                intersection.N = N1;
            }
        }

        if (intersection.t != 0)
        {
            intersection.P = vec3FromBvh(bvhray.origin + bvhray.direction * intersection.t);

            float theta = atan2(intersection.N.y, intersection.N.x);
            intersection.UV = vec2(theta / (2.0f * PI) + 0.5, dot((intersection.P - cylinder->B), normalize(cylinder->A)) / (length(A)));
            intersection.object = shape;

            rotation = glm::transpose(rotation);
            intersection.N = normalize(rotation * intersection.N);
        }
    }
    else if (shape->type == SHAPE_TRIANGLE)
    {
        Triangle* triangle = dynamic_cast<Triangle*>(shape);

        bvh::Vector3<float> V0 = vec3ToBvh(triangle->V0);
        bvh::Vector3<float> V1 = vec3ToBvh(triangle->V1);
        bvh::Vector3<float> V2 = vec3ToBvh(triangle->V2);

        bvh::Vector3<float> E1 = V1 - V0;
        bvh::Vector3<float> E2 = V2 - V0;
        bvh::Vector3<float> S = bvhray.origin - V0;

        bvh::Vector3<float> p = cross(bvhray.direction, E2);
        float d = dot(p, E1);

        if (d == 0) return std::nullopt;

        float u = dot(p, S) / d;

        if (u < 0 || u > 1) return std::nullopt;

        bvh::Vector3<float> q = cross(S, E1);
        float v = dot(q, bvhray.direction) / d;

        if (v < 0 || (u + v) > 1) return std::nullopt;

        intersection.t = dot(q, E2) / d;

        if (intersection.t < 0) return std::nullopt;

        intersection.P = vec3FromBvh(bvhray.origin + bvhray.direction * intersection.t);
        
        if (triangle->N0.has_value()) intersection.N = ((1.0f - u - v) * triangle->N0.value() + u * triangle->N1.value() + v * triangle->N2.value());
        else intersection.N = vec3FromBvh(cross(E2, E1));
        intersection.N = normalize(intersection.N);

        if (triangle->T0.has_value()) intersection.UV = (1.0f - u - v) * triangle->T0.value() + u * triangle->T1.value() + v * triangle->T2.value();
        else intersection.UV = vec2(0, 0);
        intersection.object = shape;
    }
    else
    {
        float t = 0.001f;

        vec3 D = vec3FromBvh(bvhray.direction);
        vec3 O = vec3FromBvh(bvhray.origin);

        Shape* selectedmat;

        int step = 0;
        while (true)
        {
            vec3 P = t * D + O;

            //P += vec3(sin(20.0 * P.x) * sin(20.0 * P.y) * sin(20.0 * P.z)) * 0.05f;
            

            float dt = DistanceObject(P, shape, selectedmat);
            t += abs(dt);

            ++step;

            if (dt < 0.000001) break;
            if (step > 2500) return std::nullopt;
            if (t >= 10000) return std::nullopt;
        }

        if (t <= 0.001f) return std::nullopt;

        vec3 P = t * D + O;
        float h = 0.001;

        Shape* a;

        float nx = DistanceObject(vec3(P.x + h, P.y, P.z), shape, a) - DistanceObject(vec3(P.x - h, P.y, P.z), shape, a);
        float ny = DistanceObject(vec3(P.x, P.y + h, P.z), shape, a) - DistanceObject(vec3(P.x, P.y - h, P.z), shape, a);
        float nz = DistanceObject(vec3(P.x, P.y, P.z + h), shape, a) - DistanceObject(vec3(P.x, P.y, P.z - h), shape, a);
        vec3 N = normalize(vec3(nx, ny, nz));
        intersection.P = P;
        intersection.N = N;
        intersection.object = selectedmat;
        intersection.t = t;
        intersection.UV = vec2(0, 0);

        return intersection;
    }

    if (intersection.t == 0) return std::nullopt;
    if (intersection.t < bvhray.tmin || intersection.t > bvhray.tmax) return std::nullopt;

    return intersection;
}

AccelerationBvh::AccelerationBvh(std::vector<Shape*>& objs)
{
    // Wrap all Shape*'s with a bvh specific instance
    for (Shape* shape : objs) {
        shapeVector.emplace_back(shape);
    }

    // Magic found in the bvh examples:
    auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(shapeVector.data(),
        shapeVector.size());
    auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), shapeVector.size());

    bvh::SweepSahBuilder<bvh::Bvh<float>> builder(bvh);
    builder.build(global_bbox, bboxes.get(), centers.get(), shapeVector.size());
}

Intersection AccelerationBvh::intersect(const Ray& ray)
{
    bvh::Ray<float> bvhRay = RayToBvh(ray);

    // Magic found in the bvh examples:
    bvh::ClosestPrimitiveIntersector<bvh::Bvh<float>, BvhShape> intersector(bvh, shapeVector.data());
    bvh::SingleRayTraverser<bvh::Bvh<float>> traverser(bvh);

    auto hit = traverser.traverse(bvhRay, intersector);
    if (hit) {
        return hit->intersection;
    }
    else
        return  Intersection();  // Return an IntersectionRecord which indicates NO-INTERSECTION
}

Interval::Interval() : t0(0), t1(INFINITY) {}

Interval::Interval(float startpoint, float endpoint, vec3 normal1, vec3 normal2)
{
    if (t0 < t1)
    {
        t0 = startpoint;
        t1 = endpoint;
        N0 = normal1;
        N1 = normal2;
    }
    else
    {
        t0 = endpoint;
        t1 = startpoint;
        N0 = normal2;
        N1 = normal1;
    }
}

void Interval::empty()
{
    t0 = 0;
    t1 = -1;
}

void Interval::intersect(const Interval& other)
{
    ////
}

void Interval::intersect(const bvh::Ray<float>& ray, const Slab& slab)
{
    float NO = dot(vec3ToBvh(slab.N), ray.origin);
    float ND = dot(vec3ToBvh(slab.N), ray.direction);

    if (ND != 0)
    {
        float t_0 = -(slab.d1 + NO) / ND;
        float t_1 = -(slab.d2 + NO) / ND;

        if (t_0 < t_1)
        {
            t0 = t_0;
            t1 = t_1;
            N0 = -slab.N;
            N1 = slab.N;
        }
        else
        {
            t0 = t_1;
            t1 = t_0;
            N0 = slab.N;
            N1 = -slab.N;
        }
    }
    else
    {
        float s_0 = NO + slab.d1;
        float s_1 = NO + slab.d2;

        if ((s_0 * s_1) < 0)
        {
            t0 = 0;
            t1 = INFINITY;
        }
        else
        {
            t0 = 1;
            t1 = 0;
        }
    }
}
