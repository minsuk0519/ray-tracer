#ifndef _ACC_H
#define _ACC_H

// This uses a library called bvh for a bounding volume hierarchy acceleration structure.
// See https://github.com/madmann91/bvh

// The bvh library uses its own version of vectors, bounding boxes,
// rays, and list of shapes, so there will be some conversions to/from
// the raytracer's versions of these structures.  Alternatively, the
// raytracer could be written to directly use bvh's version of these
// structures.  This seems feasible, but has not been tested.

#include <optional>
#include <bvh/bvh.hpp>
#include <bvh/vector.hpp>
#include <bvh/ray.hpp>

// FIX THIS:  Dummy ray to allow for compilation
class Ray { 
public:
    vec3 o, d;
    Ray(const vec3 _o, const vec3 _d) : o(_o), d(_d) {}

    vec3 eval(float t) const { return o + t * d; }
};

// Vectors:
// Expectation: The raytracer uses glm::vec3 throughout 
// These convert between glm::vec3 and bvh::Vector3<float>
bvh::Vector3<float> vec3ToBvh(const vec3& v);
vec3 vec3FromBvh(const bvh::Vector3<float>& v);

// Bounding boxes
// Expectation: The raytracer uses SimpleBox throughout.
// This class derives from the bvh bounding box, overloading two methods to take glm::vec3.
// Use: Construct a BB with zero or one points (vec3), extend it with further points.
//      Access its bounds as two bvh vectors box.min and box.max.
bvh::Vector3<float> vec3ToBvh(const vec3& v);
vec3 vec3FromBvh(const bvh::Vector3<float>& v);

class SimpleBox: public bvh::BoundingBox<float> {
public:
    SimpleBox();
    SimpleBox(const vec3 v);
    SimpleBox& extend(const vec3 v);
};

// FIX THIS: 
// Rays:  Rays have an origin and a direction.  bvh::Ray also has a tmin and a tmax.
// Supply your own ray class and convert.
bvh::Ray<float> RayToBvh(const Ray &r);
Ray RayFromBvh(const bvh::Ray<float> &r);

class Shape;
class Slab;

// FIX THIS:  This dummy Intersection record is defficient -- just barely enough to compile.
class Intersection {
public:
    float t;
    Shape* object;
    vec3 P;
    vec3 N;
    vec2 UV;
    float distance() const { return t; }  // A function the BVH traversal needs to be supplied.
};

class Interval
{
public:
    Interval();
    Interval(float startpoint, float endpoint, vec3 normal1, vec3 normal2);

    void empty();
    void intersect(const Interval& other);
    void intersect(const bvh::Ray<float>& ray, const Slab& slab);

    float t0, t1;
    vec3 N0;
    vec3 N1;
};

// Wrapper of a single Shape*:

// The ray tracer stores the scene objects as a list of Shape*.  BVH
// expects a list of NON-POINTERS with various methods and type
// declarations.

class BvhShape {
    Shape* shape;
public:
    using ScalarType = float;   // Float or double for rays and vectors.
    using IntersectionType = Intersection; // Specify the intersection record type.
    
    BvhShape(Shape* s) : shape(s) { }; // Constructor given the Shape to wrap
    
    SimpleBox bounding_box() const; // Returns the bounding box of the shape
    bvh::Vector3<float> center() const; // Returns the bounding_box().center()

    // The intersection routine.
    // Given a bvh::Ray, intersect it with the shape and return either of:
    //     an Intersection
    //         ( it exists, and is between the ray's tmin and tmax.
    //     std::nullopt ((otherwise)
    std::optional<Intersection> intersect(const bvh::Ray<float>& bvhray) const;
};

// Encapsulates the BVH structure, the list of shapes it's built from,
// and method to intersect a ray with the full scene and return the
// front most intersection point.
class AccelerationBvh  {
    bvh::Bvh<float> bvh;
    std::vector<BvhShape> shapeVector;
 public:
    AccelerationBvh(std::vector<Shape*> &objs);
    Intersection intersect(const Ray& ray);
};

#endif
