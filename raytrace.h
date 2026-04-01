///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#include "acceleration.h"

const float PI = glm::pi<float>();
const float Radians = PI/180.0f;    // Convert degrees to radians

constexpr float epsilon = 0.01f;

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    vec3 Kd, Ks, Kt;
    float ior;
    float alpha;
    unsigned int texid;

    virtual bool isLight() { return false; }

    Material()  : Kd(vec3(1.0, 0.5, 0.0)), Ks(vec3(1,1,1)), alpha(1.0), texid(0) {}
    Material(const vec3 d, const vec3 s, const float a, const vec3 t, float n) 
        : Kd(d), Ks(s), alpha(a), texid(0), Kt(t), ior(n) {}
    Material(const vec3 d, const vec3 s, const float a)
        : Kd(d), Ks(s), alpha(a), texid(0) {}
    Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha;  texid=o.texid; }

    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (ivec3: consisting of three
// indices into the vertex array).
    
class VertexData
{
 public:
    vec3 pnt;
    vec3 nrm;
    vec2 tex;
    vec3 tan;
    VertexData(const vec3& p, const vec3& n, const vec2& t, const vec3& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<ivec3> triangles;
    Material *mat;
};

class Shape;

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:
    Light(const vec3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }

    Shape* shape = nullptr;
};

class IBL : public Light
{
public:
    IBL(float* c, int w, int h) : Light(vec3(0, 0, 0)), pixel(c), width(w), height(h) {}
    void preprocess();
    virtual bool isLight() { return true; }

    float* pixel;
    int width;
    int height;

    float* pBuffer;
    float* pUDist;

    Shape* shape = nullptr;
};

////////////////////////////////////////////////////////////////////////
// Shape
////////////////////////////////////////////////////////////////////////
enum SHAPE_TYPE
{
    SHAPE_SPHERE,
    SHAPE_CYLINDER,
    SHAPE_BOX,
    SHAPE_TRIANGLE,
    SHAPE_CSG,
    SHAPE_RayMarching,
};

class Slab
{
public:
    vec3 N;
    float d1, d2;
};

class Shape
{
public:
    Shape(Material* material, SHAPE_TYPE shapetype) : mat(material), type(shapetype) {}
    Material* mat;
    SHAPE_TYPE type;

    virtual float distance(vec3 P) = 0;

    virtual SimpleBox boundingbox() = 0;
};

class Sphere : public Shape
{
public:
    Sphere(Material* material, const vec3& center, const float radius) : Shape(material, SHAPE_SPHERE), C(center), r(radius) {}
    vec3 C;
    float r;

    virtual float distance(vec3 P) override;

    virtual SimpleBox boundingbox() override;
};

class Cylinder : public Shape
{
public:
    Cylinder(Material* material, const vec3& center, const vec3& axis, float radius) : Shape(material, SHAPE_CYLINDER), B(center), A(axis), r(radius) {}
    vec3 B;
    vec3 A;
    float r;

    virtual float distance(vec3 P) override;

    virtual SimpleBox boundingbox() override;
};

class Box : public Shape
{
public:
    Box(Material* material, const vec3& corner, const vec3& diagonal) : Shape(material, SHAPE_BOX), C(corner), d(diagonal) {}
    vec3 C;
    vec3 d;

    virtual float distance(vec3 P) override;

    virtual SimpleBox boundingbox() override;
};

class Triangle : public Shape
{
public:
    Triangle(Material* material, vec3 v0, vec3 v1, vec3 v2) : Shape(material, SHAPE_TRIANGLE), V0(v0), V1(v1), V2(v2) {}
    Triangle(Material* material, vec3 v0, vec3 v1, vec3 v2, vec3 n0, vec3 n1, vec3 n2) : 
        Shape(material, SHAPE_TRIANGLE), V0(v0), V1(v1), V2(v2), N0(n0), N1(n1), N2(n2) {}
    Triangle(Material* material, vec3 v0, vec3 v1, vec3 v2, vec3 n0, vec3 n1, vec3 n2, vec2 t0, vec2 t1, vec2 t2) : 
        Shape(material, SHAPE_TRIANGLE), V0(v0), V1(v1), V2(v2), N0(n0), N1(n1), N2(n2), T0(t0), T1(t1), T2(t2) {}

    vec3 V0;
    vec3 V1;
    vec3 V2;

    std::optional<vec3> N0;
    std::optional<vec3> N1;
    std::optional<vec3> N2;

    std::optional<vec2> T0;
    std::optional<vec2> T1;
    std::optional<vec2> T2;

    virtual float distance(vec3 P) override;

    virtual SimpleBox boundingbox() override;
};

enum CSG_INDEX
{
    CSG_PLANE,
    CSG_CONE,
    CSG_TORUS,
};

enum CSG_COMB
{
    CSG_UNION,
    CSG_SMOOTH_UNION,
    CSG_SUBTRACTION,
    CSG_SMOOTH_SUBTRACTION,
    CSG_INTERSECTION,
    CSG_SMOOTH_INTERSECTION,
    CSG_UNDEFINED,
    CSG_ROOT,
};

class RayMarchingObject : public Shape
{
public:
    RayMarchingObject(Material* material) : Shape(material, SHAPE_RayMarching) {}

    Shape* root;
    std::vector<std::pair<CSG_COMB, Shape*>> childs;

    virtual float distance(vec3 P);
    virtual SimpleBox boundingbox() override;
};

class CSG : public Shape
{
public:
    CSG(Material* material, vec3 p, vec3 r) : Shape(material, SHAPE_CSG), pos(p)
    {
        rot = glm::rotation(vec3(0, 0, 1), r);
    }

    quat rot;
    vec3 pos;

    vec3 transform(vec3 P);

    virtual float distance(vec3 P) = 0;
};

class CSG_Plane : public CSG
{
public:
    CSG_Plane(Material* material, vec3 n, float c, vec3 r, vec3 p) : CSG(material, r, p), N(n), d(c) {}

    vec3 N;
    float d;

    virtual float distance(vec3 P) override;
    virtual SimpleBox boundingbox() override;
};

class CSG_Cone : public CSG
{
public:
    CSG_Cone(Material* material, float angle, vec3 r, vec3 p) : CSG(material, r, p), theta(angle) {}

    float theta;

    virtual float distance(vec3 P) override;
    virtual SimpleBox boundingbox() override;
};

class CSG_Torus : public CSG
{
public:
    CSG_Torus(Material* material, float radius1, float radius2, vec3 r, vec3 p) : CSG(material, r, p), R(radius1), r(radius2) {}

    float R;
    float r;

    virtual float distance(vec3 P) override;
    virtual SimpleBox boundingbox() override;
};

class CSG_Curve : public CSG
{
public:
    CSG_Curve(Material* material, vec3 r, vec3 p) : CSG(material, r, p) 
    {
        std::vector<vec3> control;
        std::vector<float> knots;
        control.push_back(vec3(0, 30, -372));
        control.push_back(vec3(0, 49, -121));
        control.push_back(vec3(0, 71, -283));
        control.push_back(vec3(0, 99, -133));
        control.push_back(vec3(0, 107, -251));
        control.push_back(vec3(0, 141, -242));
        control.push_back(vec3(0, 147, -151));
        control.push_back(vec3(0, 146, -253));
        control.push_back(vec3(0, 179, -243));
        control.push_back(vec3(0, 178, -181));
        control.push_back(vec3(0, 229, -182));
        control.push_back(vec3(0, 227, -250));
        control.push_back(vec3(0, 325, -237));
        control.push_back(vec3(0, 281, -202));
        control.push_back(vec3(0, 257, -179));
        control.push_back(vec3(0, 293, -168));
        control.push_back(vec3(0, 332, -170));
        control.push_back(vec3(0, 333, -240));
        control.push_back(vec3(0, 379, -234));
        control.push_back(vec3(0, 376, -166));
        control.push_back(vec3(0, 386, -226));
        control.push_back(vec3(0, 429, -263));
        control.push_back(vec3(0, 418, -159));
        control.push_back(vec3(0, 430, -219));
        control.push_back(vec3(0, 462, -180));
        control.push_back(vec3(0, 455, -206));
        control.push_back(vec3(0, 419, -218));
        control.push_back(vec3(0, 508, -286));

        for (int i = 0; i < 30; ++i)
        {
            knots.push_back(i);
        }

        const int N = 29;
        const int s = 27;
        const int d = 2;
        vec3 Q[s + 1];

        vec3 scale = vec3(1.0f / 75.0f);

        vec3 prev;

        for (float t = 2; t < 28;)
        {
            for (int i = 0; i < s + 1; ++i)
            {
                Q[i] = control[i];
            }

            int J;
            for (J = d; J < N - d; ++J)
            {
                if (knots[J] <= t && knots[J + 1] > t)
                {
                    break;
                }
            }

            for (int p = 1; p <= s; ++p)
            {
                for (int i = J; i >= J - d + p; --i)
                {
                    Q[i] = Q[i] * ((t - knots[i]) /
                        (knots[i + d - p + 1] - knots[i])) +
                        Q[i - 1] * ((knots[i + d - p + 1] - t) /
                            (knots[i + d - p + 1] - knots[i]));
                }
            }

            if (t == 2)
            {
                pts.push_back(scale * Q[J]);
            }
            else
            {
                auto c = length(prev - Q[J]);
                if (c > 10.0f)
                {
                    pts.push_back(scale * Q[J]);
                    prev = Q[J];
                }
            }

            t += 0.01f;
        }

        rot = glm::rotation(vec3(0, 1, 0), normalize(vec3(1.8, 2, 0)));
    }

    std::vector<vec3> pts;

    virtual float distance(vec3 P) override;
    virtual SimpleBox boundingbox() override;
};

////////////////////////////////////////////////////////////////////////////////
// Scene
class Scene {
public:
    int width, height;
    vec3 eye;
    quat orient;
    float ry;

    vec3 ambient;

    std::vector<Light*> lights;
    std::vector<Shape*> vectorOfShapes;

    Material* currentMat;
    RayMarchingObject* raymarching;
    CSG_COMB comb = CSG_UNDEFINED;

    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const mat4& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);

    Color TracePath(const Ray& ray, AccelerationBvh& bvh);

    Intersection SampleLight();
    float PdfLight(Intersection Q);
};

float GeometryFactor(Intersection A, Intersection B);
vec3 SampleLobe(vec3 A, float c, float phi);
Intersection SampleSphere(vec3 C, float R, Shape* obj);

vec3 EvalRadiance(Intersection Q);

vec3 SampleBrdf(vec3 w0, vec3 N, Material* mat);
float PdfBrdf(vec3 w0, vec3 N, vec3 wi, Material* mat);
vec3 EvalScattering(vec3 w0, vec3 N, vec3 wi, float t, Material* mat);

vec3 fresenl(float d, vec3 ks);
float fresnel(float g, float c);
float distribution(vec3 m, vec3 N, float alpha);
float geometry_smith(vec3 wi, vec3 w0, vec3 m, vec3 N, float alpha);
float geometry(float vDotm, float vDotN, float alpha);

void ReadHdrImage(const std::string readName, int& width, int& height, float*& image);

float DistanceObject(vec3 P, Shape* input, Shape*& output);