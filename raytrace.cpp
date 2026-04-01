//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
std::uniform_int_distribution<> myrandom_INT(0, 1);

// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

#define TIME_MEASURE
#ifdef TIME_MEASURE
    #include <chrono>
#endif

constexpr float RussianRoulette = 0.8f;

constexpr int PATH_PASS = 8;

enum BRDF_TYPE
{
    BRDF_PHONG,
    BRDF_BECKMAN,
    BRDF_GGX,
};

BRDF_TYPE BRDFTYPE = BRDF_GGX;

Scene::Scene() {}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    for (auto& triangle : mesh->triangles)
    {
        vec3 V0 = mesh->vertices[triangle.x].pnt;
        vec3 V1 = mesh->vertices[triangle.y].pnt;
        vec3 V2 = mesh->vertices[triangle.z].pnt;

        vec3 N0 = mesh->vertices[triangle.x].nrm;
        vec3 N1 = mesh->vertices[triangle.y].nrm;
        vec3 N2 = mesh->vertices[triangle.z].nrm;

        vec2 T0 = mesh->vertices[triangle.x].tex;
        vec2 T1 = mesh->vertices[triangle.y].tex;
        vec2 T2 = mesh->vertices[triangle.z].tex;
        
        vectorOfShapes.push_back(new Triangle(mesh->mat, V0, V1, V2, N0, N1, N2, T0, T1, T2));
    }
}

void Scene::ReadAssimpFile(const std::string& path, const mat4& M)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path,
        aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        fprintf(stderr, "Assimp error loading %s: %s\n", path.c_str(), importer.GetErrorString());
        return;
    }

    mat4 Minv = glm::transpose(glm::inverse(M));

    for (unsigned int mi = 0; mi < scene->mNumMeshes; mi++) {
        aiMesh* aimesh = scene->mMeshes[mi];
        MeshData* mesh = new MeshData();
        mesh->mat = currentMat;

        for (unsigned int vi = 0; vi < aimesh->mNumVertices; vi++) {
            aiVector3D p = aimesh->mVertices[vi];
            aiVector3D n = aimesh->mNormals[vi];
            aiVector3D t = aimesh->HasTextureCoords(0) ? aimesh->mTextureCoords[0][vi] : aiVector3D(0,0,0);
            aiVector3D a = aimesh->HasTangentsAndBitangents() ? aimesh->mTangents[vi] : aiVector3D(0,0,0);

            vec4 tp = M * vec4(p.x, p.y, p.z, 1.0f);
            vec4 tn = Minv * vec4(n.x, n.y, n.z, 0.0f);

            mesh->vertices.emplace_back(
                vec3(tp),
                normalize(vec3(tn)),
                vec2(t.x, t.y),
                vec3(a.x, a.y, a.z)
            );
        }

        for (unsigned int fi = 0; fi < aimesh->mNumFaces; fi++) {
            aiFace& face = aimesh->mFaces[fi];
            if (face.mNumIndices == 3)
                mesh->triangles.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }

        triangleMesh(mesh);
    }
}

quat Orientation(int i,
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    quat q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Xaxis());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Yaxis());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Zaxis());
        else if (c == "q")  {
            q *= quat(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, normalize(vec3(f[i+1], f[i+2], f[i+3])));
            i+=4; } }
    return q;
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") 
    {
        // syntax: screen width height
        width = int(f[1]);
        height = int(f[2]); 
    }
    else if (c == "camera") 
    {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        eye = vec3(f[1], f[2], f[3]);
        orient = Orientation(5, strings, f);
        ry = f[4];
    }
    else if (c == "ambient") 
    {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        ambient = vec3(f[1], f[2], f[3]); 
    }
    else if (c == "brdf")  
    {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], vec3(f[8], f[9], f[10]), f[11]); 
    }
    else if (c == "light") 
    {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(vec3(f[1], f[2], f[3])); 
    }
    else if (c == "sphere") 
    {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        //realtime->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
        Shape* shape = new Sphere(currentMat, vec3(f[1], f[2], f[3]), f[4]);

        if (comb == CSG_ROOT)
        {
            raymarching->root = shape;
            comb = CSG_UNDEFINED;
        }
        else if (comb != CSG_UNDEFINED)
        {
            raymarching->childs.push_back(std::make_pair(comb, shape));
            comb = CSG_UNDEFINED;
        }
        else
        {
            vectorOfShapes.push_back(shape);
        }

        if (currentMat->isLight())
        {
            Light* light = dynamic_cast<Light*>(currentMat);
            light->shape = shape;
            lights.push_back(light);
        }
    }
    else if (c == "box") 
    {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        Shape* shape = new Box(currentMat, vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]));

        if (comb == CSG_ROOT)
        {
            raymarching->root = shape;
            comb = CSG_UNDEFINED;
        }
        else if (comb != CSG_UNDEFINED)
        {
            raymarching->childs.push_back(std::make_pair(comb, shape));
            comb = CSG_UNDEFINED;
        }
        else
        {
            vectorOfShapes.push_back(shape);
        }

        if (currentMat->isLight())
        {
            Light* light = dynamic_cast<Light*>(currentMat);
            light->shape = shape;
            lights.push_back(light);
        }
    }
    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        Shape* shape = new Cylinder(currentMat, vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]);
        
        if (comb == CSG_ROOT)
        {
            raymarching->root = shape;
            comb = CSG_UNDEFINED;
        }
        else if (comb != CSG_UNDEFINED)
        {
            raymarching->childs.push_back(std::make_pair(comb, shape));
            comb = CSG_UNDEFINED;
        }
        else
        {
            vectorOfShapes.push_back(shape);
        }

        if (currentMat->isLight())
        {
            Light* light = dynamic_cast<Light*>(currentMat);
            light->shape = shape;
            lights.push_back(light);
        }
    }
    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        mat4 modelTr = translate(vec3(f[2],f[3],f[4]))
                          *scale(vec3(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  }
    else if (c == "ibl")
    {
        float* pixels;
        int width;
        int height;
        ReadHdrImage(strings[1], width, height, pixels);
        IBL* ibl = new IBL(pixels, width, height);
        ibl->preprocess();
        currentMat = ibl;
    }
    else if (c == "raymarching")
    {
        raymarching = new RayMarchingObject(currentMat);
        vectorOfShapes.push_back(raymarching);

        comb = CSG_ROOT;
    }
    else if(c == "csg")
    {
        std::string c = strings[1];
        Shape* shape;

        if (c == "plane")
        {
            shape = new CSG_Plane(currentMat, vec3(f[2], f[3], f[4]), f[5], vec3(f[6], f[7], f[8]), vec3(f[9], f[10], f[11]));
        }
        else if (c == "torus")
        {
            shape = new CSG_Torus(currentMat, f[2], f[3], vec3(f[4], f[5], f[6]), vec3(f[7], f[8], f[9]));
        }
        else if (c == "cone")
        {
            shape = new CSG_Cone(currentMat, f[2], vec3(f[3], f[4], f[5]), vec3(f[6], f[7], f[8]));
        }
        else if (c == "curve")
        {
            shape = new CSG_Curve(currentMat, vec3(f[2], f[3], f[4]), vec3(f[5], f[6], f[7]));
        }
        else if (c == "raymarching")
        {
            RayMarchingObject* newraymarching = new RayMarchingObject(currentMat);
            raymarching->childs.push_back(std::make_pair(comb, newraymarching));
            raymarching = newraymarching;

            comb = CSG_ROOT;

            return;
        }

        if (comb == CSG_ROOT)
        {
            raymarching->root = shape;
            comb = CSG_UNDEFINED;
        }
        else if (comb != CSG_UNDEFINED)
        {
            raymarching->childs.push_back(std::make_pair(comb, shape));
            comb = CSG_UNDEFINED;
        }

        if (currentMat->isLight())
        {
            Light* light = dynamic_cast<Light*>(currentMat);
            light->shape = shape;
            lights.push_back(light);
        }
    }
    else if (c == "union") comb = CSG_UNION;
    else if (c == "unionsmooth") comb = CSG_SMOOTH_UNION;
    else if (c == "subtraction") comb = CSG_SUBTRACTION;
    else if (c == "subtractionsmooth") comb = CSG_SMOOTH_SUBTRACTION;
    else if (c == "intersection") comb = CSG_INTERSECTION;
    else if (c == "intersectionsmooth") comb = CSG_SMOOTH_INTERSECTION;
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(Color* image, const int pass)
{
    float rx = ry * width / (float)(height);
    vec3 X = rx * transformVector(orient, Xaxis());
    vec3 Y = ry * transformVector(orient, Yaxis());
    vec3 Z = transformVector(orient, Zaxis());

    AccelerationBvh bvh(vectorOfShapes);

#ifdef TIME_MEASURE
    auto before = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "Trace Image start!\n");
#endif // TIME_MEASURE

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            image[y * width + x] = Color(0, 0, 0);
        }
    }

    for (int pass = 0; pass < PATH_PASS; ++pass)
    {
#ifndef _DEBUG
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
#endif
        for (int y = 0; y < height; y++) {
            fprintf(stderr, "%d, Rendering %4d\r", pass, y);
            for (int x = 0; x < width; x++) {
                float dx = 2 * (x + myrandom(RNGen)) / width - 1;
                float dy = 2 * (y + myrandom(RNGen)) / height - 1;
                Ray ray(eye, normalize(dx * X + dy * Y - Z));

                Color c = TracePath(ray, bvh);

                if (!isnan(c.x) && !isnan(c.y) && !isnan(c.z) &&
                    !isinf(c.x) && !isinf(c.y) && !isinf(c.z))
                {
                    if (c.x < 0.0f) c.x = 0.0f;
                    if (c.y < 0.0f) c.y = 0.0f;
                    if (c.z < 0.0f) c.z = 0.0f;

                    if (c.x > 1.0f) c.x = 1.0f;
                    if (c.y > 1.0f) c.y = 1.0f;
                    if (c.z > 1.0f) c.z = 1.0f;

                    image[y * width + x] += c;
                }
            }
        }

    }

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            image[y * width + x] /= (float)PATH_PASS;
        }
    }
    fprintf(stderr, "\n");

#ifdef TIME_MEASURE
    auto after = std::chrono::high_resolution_clock::now();
    float result = (float)(std::chrono::duration<double, std::ratio<1, 1>>(after -before).count());

    fprintf(stderr, "Trace Image time passed : %4f\n", result);
#endif // TIME_MEASURE
}

Color Scene::TracePath(const Ray& ray, AccelerationBvh& bvh)
{
    vec3 C = vec3(0, 0, 0);
    vec3 W = vec3(1, 1, 1);

    Intersection P = bvh.intersect(ray);
    vec3 N = P.N;

    if (P.t == INFINITY || P.t <= 0) return C;
    if (P.object->mat->isLight())
    {
        return EvalRadiance(P);
    }

    vec3 wo = -ray.d;
    vec3 wi;
    float wmis;
    while (myrandom(RNGen) <= RussianRoulette)
    {
        {
            Intersection L = SampleLight();
            float p = PdfLight(L) / GeometryFactor(P, L);
            wi = normalize(L.P - P.P);
            float q = PdfBrdf(wo, N, wi, P.object->mat) * RussianRoulette;
            float psquare = p * p;
            wmis = psquare / (psquare + q * q);
            Ray shadowray = Ray(P.P, wi);
            Intersection I = bvh.intersect(shadowray);
            if (p > 0 && I.t != INFINITY && I.t > epsilon)
            {
                //if (I.P == L.P && I.object == L.object)
                if (glm::length(I.P - L.P) < 0.0001f && I.object == L.object)
                {
                    vec3 f = EvalScattering(wo, N, wi, P.t, P.object->mat);

                    C += W * wmis * (f / p) * EvalRadiance(L);
                }
            }
        }

        {
            wi = SampleBrdf(wo, N, P.object->mat);

            float a = dot(wi, N);

            Ray newray(P.P, wi);

            Intersection Q = bvh.intersect(newray);

            if (Q.t == INFINITY || Q.t <= 0) break;

            vec3 f = EvalScattering(wo, N, wi, P.t, P.object->mat);
            float p = PdfBrdf(wo, N, wi, P.object->mat) * RussianRoulette;

            if (p < epsilon) break;

            W *= f / p;

            if (Q.object->mat->isLight())
            {
                float q = PdfLight(Q) / GeometryFactor(P, Q);
                float psquare = p * p;
                wmis = psquare / (psquare + q * q);
                C += W * wmis * EvalRadiance(Q);
                break;
            }

            P = Q;
            N = P.N;
            wo = -wi;
        }
    }

    return C;
}

float Sphere::distance(vec3 P)
{
    return length(P - C) - r;
}

SimpleBox Sphere::boundingbox()
{
    vec3 diag = vec3(r, r, r);

    SimpleBox boundingbox(C - diag);
    boundingbox.extend(C + diag);

    return boundingbox;
}

float Cylinder::distance(vec3 P)
{
    vec3 diff = P - B;
    float h = dot(A, A);
    float theta = dot(diff, A);

    float x = length(diff * h - A * theta) - r * h;
    float y = abs(theta - h * 0.5) - h * 0.5;

    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;

    float x2 = x * x;
    float y2 = y * y * h;

    float d = (std::max(x, y) < 0.0) ? -std::min(x2, y2) : x2 + y2;

    return sqrt(abs(d)) / h;
}

SimpleBox Cylinder::boundingbox()
{
    vec3 rrr = vec3(r, r, r);

    SimpleBox boundingbox(B + rrr);
    boundingbox.extend(B - rrr);
    boundingbox.extend(B + A + rrr);
    boundingbox.extend(B + A - rrr);

    return boundingbox;
}

float Box::distance(vec3 P)
{
    vec3 max = C + d;
    vec3 min = C;

    float xmax = std::max(P.x - max.x, min.x - P.x);
    float ymax = std::max(P.y - max.y, min.y - P.y);
    float zmax = std::max(P.z - max.z, min.z - P.z);

    return std::max(std::max(xmax, ymax), zmax);
}

SimpleBox Box::boundingbox()
{
    SimpleBox boundingbox(C);
    boundingbox.extend(C + d);

    return boundingbox;
}

float Triangle::distance(vec3 P)
{
    return FLT_MAX;
}

SimpleBox Triangle::boundingbox()
{
    SimpleBox bounding_box(V0);
    bounding_box.extend(V1);
    bounding_box.extend(V2);

    return bounding_box;
}

float GeometryFactor(Intersection A, Intersection B)
{
    vec3 D = A.P - B.P;
    float DdotD = dot(D, D);
    float ANdotD = dot(A.N, D);
    float BNdotD = dot(B.N, D);

    return abs(ANdotD * BNdotD / (DdotD * DdotD));
}

vec3 SampleLobe(vec3 A, float c, float phi)
{
    float s = sqrt(1 - c * c);

    vec3 K = vec3(s * cos(phi), s * sin(phi), c);

    if (abs(A.z - 1) < 0.001) return K;
    if (abs(A.z + 1) < 0.001) return vec3(K.x, -K.y, -K.z);

    vec3 B = normalize(vec3(-A.y, A.x, 0));
    vec3 C = cross(A, B);

    return K.x * B + K.y * C + K.z * A;
}

Intersection SampleSphere(vec3 C, float R, Shape* obj)
{
    float x1 = myrandom(RNGen);
    float x2 = myrandom(RNGen);

    float z = 2 * x1 - 1;
    float r = sqrt(1 - z * z);

    float a = 2 * PI * x2;

    Intersection result;

    result.N = vec3(r * cos(a), r * sin(a), z);
    result.P = C + R * result.N;
    result.object = obj;

    return result;
}

Intersection Scene::SampleLight()
{
    Intersection B;

    IBL* ibl = dynamic_cast<IBL*>(lights.at(0));

    double u = myrandom(RNGen);
    double v = myrandom(RNGen);

    float maxUVal = ibl->pUDist[ibl->width - 1];
    
    float* pUPos = std::lower_bound(ibl->pUDist, ibl->pUDist + ibl->width, u * maxUVal);
    
    int iu = pUPos - ibl->pUDist;

    float* pVDist = &ibl->pBuffer[ibl->height * iu];
    float* pVPos = std::lower_bound(pVDist, pVDist + ibl->height, v * pVDist[ibl->height - 1]);
    
    int iv = pVPos - pVDist;
    
    double phi = 0 - 2 * PI * iu / ibl->width;
    double theta = PI * iv / ibl->height;
    
    B.N = vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
    B.P = B.N * 1000.0f;
    B.object = ibl->shape;
    return B;


    //only for sphere light
    unsigned int numberoflights = lights.size();
    int N = myrandom_INT(RNGen) * (numberoflights - 1);
    Light* light = lights[N];
    if (light->shape->type != SHAPE_SPHERE) fprintf(stderr, "Error! light is not sphere!\n");
    Sphere* lightsphere = dynamic_cast<Sphere*>(light->shape);
    return SampleSphere(lightsphere->C, lightsphere->r, lightsphere);
}

float Scene::PdfLight(Intersection Q)
{
    IBL* ibl = dynamic_cast<IBL*>(lights.at(0));

    vec3 P = normalize(Q.P);
    
    double fu = (0 - atan2(P[1], P[0])) / (PI * 2);
    fu = fu - floor(fu);
    
    int u = floor(ibl->width * fu);
    int v = floor(ibl->height * acos(P[2]) / PI);
    
    float angleFrac = PI / float(ibl->height);

    float* pVDist = &ibl->pBuffer[ibl->height * u];

    float pdfU = (u == 0) ? (ibl->pUDist[0]) : (ibl->pUDist[u] - ibl->pUDist[u - 1]);
    pdfU /= ibl->pUDist[ibl->width - 1];
    pdfU *= ibl->width / (PI * 2);
    
    float pdfV = (v == 0) ? (pVDist[0]) : (pVDist[v] - pVDist[v - 1]);
    pdfV /= pVDist[ibl->height - 1];
    pdfV *= ibl->height / PI;

    float theta = angleFrac * 0.5 + angleFrac * v;

    float r = 1000.0f;
    float areaoflight = r * r * 4.0 * PI;
    float pdf = pdfU * pdfV * sin(theta) / areaoflight;

    return pdf;


    //float r = dynamic_cast<Sphere*>(Q.object)->r;
    //float areaoflight = r * r * 4.0 * PI;
    //return 1.0f / (lights.size() * areaoflight);
}

vec3 SampleBrdf(vec3 wo, vec3 N, Material* mat)
{
    float random = myrandom(RNGen);

    float kd = glm::length(mat->Kd);
    float ks = glm::length(mat->Ks);
    float kt = glm::length(mat->Kt);
    float alpha = mat->alpha;

    float pd = kd / (kd + ks + kt);
    float pr = ks / (kd + ks + kt);

    float xi1 = myrandom(RNGen);
    float xi2 = myrandom(RNGen);

    //diffuse
    if (random < pd)
    {
        return SampleLobe(N, sqrt(xi1), 2 * PI * xi2);
    }
    //reflective
    else if(random < pd + pr)
    {
        float costheta;
        if (BRDFTYPE == BRDF_PHONG)
        {
            costheta = pow(xi1, 1 / (alpha + 1));
        }
        else if (BRDFTYPE == BRDF_GGX)
        {
            float alphag = sqrt(2.0f / (alpha + 2));
            costheta = cos(atan(alphag * sqrt(xi1) / (sqrt(1 - xi1))));
        }
        else
        {
            float alphab = sqrt(2.0f / (alpha + 2));
            costheta = cos(atan(sqrt(-alphab * alphab * log10(1 - xi1))));
        }

        vec3 m = SampleLobe(N, costheta, 2 * PI * xi2);

        return 2 * abs(dot(wo, m)) * m - wo;
    }
    else
    {
        float costheta;
        if (BRDFTYPE == BRDF_PHONG)
        {
            costheta = pow(xi1, 1 / (alpha + 1));
        }
        else if (BRDFTYPE == BRDF_GGX)
        {
            float alphag = sqrt(2.0f / (alpha + 2));
            costheta = cos(atan(alphag * sqrt(xi1) / (sqrt(1 - xi1))));
        }
        else
        {
            float alphab = sqrt(2.0f / (alpha + 2));
            costheta = cos(atan(sqrt(-alphab * alphab * log10(1 - xi1))));
        }

        vec3 m = SampleLobe(N, costheta, 2 * PI * xi2);

        float n;

        float wodotN = dot(wo, N);

        if (wodotN > 0) n = 1.0 / mat->ior;
        else
        {
            n = mat->ior;
        }

        float wodotm = dot(wo, m);

        float r = 1 - n * n * (1 - (wodotm * wodotm));

        if (r < 0)
        {
            return 2 * abs(wodotm) * m - wo;
        }
        else
        {
            float sign = (wodotN >= 0) ? 1 : -1;

            return (n * wodotm - sign * sqrt(r)) * m - n * wo;
        }
    }
}

vec3 EvalRadiance(Intersection Q)
{
    if (IBL* tex = dynamic_cast<IBL*>(Q.object->mat); tex != nullptr)
    {
        vec3 P = glm::normalize(Q.P);
        double u = (0 - atan2(P[1], P[0])) / (PI * 2);
        u = u - floor(u);
        double v = acos(P[2]) / PI;

        int i0 = floor(u * tex->width);
        int j0 = floor(v * tex->height);

        double uw[2], vw[2];
        uw[1] = u * tex->width - i0; 
        uw[0] = 1.0 - uw[1];
        vw[1] = v * tex->height - j0; 
        vw[0] = 1.0 - vw[1];

        vec3 r(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 2; i++) 
        {
            for (int j = 0; j < 2; j++) 
            {
                int k = 3 * (((j0 + j) % tex->height) * tex->width + ((i0 + i) % tex->width));
                
                for (int c = 0; c < 3; c++) 
                {
                    r[c] += uw[i] * vw[j] * tex->pixel[k + c];
                }
            }
        }
        return r;
    }

    return Q.object->mat->Kd;
}

float PdfBrdf(vec3 wo, vec3 N, vec3 wi, Material* mat)
{
    float kd = glm::length(mat->Kd);
    float ks = glm::length(mat->Ks);
    float kt = glm::length(mat->Kt);
    float alpha = mat->alpha;

    float s = (kd + ks + kt);

    float pd = kd / s;
    float pr = ks / s;
    float pt = kt / s;

    float Pd = std::abs(dot(wi, N)) / PI;
    vec3 m = glm::normalize(wo + wi);
    float Pr = distribution(m, N, alpha) * std::abs(dot(m, N)) / (4 * std::abs(dot(wi, m)));

    float ni;
    float no;

    if (dot(wo, N) > 0)
    {
        ni = 1.0;
        no = mat->ior;
    }
    else
    {
        ni = mat->ior;
        no = 1.0;
    }

    float n = ni / no;

    vec3 mt = -glm::normalize(no * wi + ni * wo);
    float wodotmt = dot(wo, mt);
    float r = 1 - n * n * (1 - wodotmt * wodotmt);

    float Pt = Pr;

    if (r < 0) Pt = Pr;
    else
    {
        float widotmt = dot(wi, mt);
        float denom = no * widotmt + ni * dot(wo, mt);
        Pt = distribution(mt, N, alpha) * std::abs(dot(mt, N)) * no * no * std::abs(widotmt) / (denom * denom);
    }

    return pd * Pd + pr * Pr + pt * Pt;
}

vec3 EvalScattering(vec3 wo, vec3 N, vec3 wi, float t, Material* mat)
{
    vec3 Ed = mat->Kd / PI;
    vec3 m = glm::normalize(wo + wi);

    vec3 ks = mat->Ks;

    float D = distribution(m, N, mat->alpha);
    vec3 F = fresenl(std::abs(dot(wi, m)), ks);

    float ni;
    float no;

    vec3 A;

    if (dot(wo, N) > 0)
    {
        A = vec3(1, 1, 1);

        ni = 1.0;
        no = mat->ior;
    }
    else
    {
        A.x = exp(t * log(mat->Kt.x));
        A.y = exp(t * log(mat->Kt.y));
        A.z = exp(t * log(mat->Kt.z));

        ni = mat->ior;
        no = 1.0;
    }

    float c = std::abs(dot(wi, m));
    float g = sqrt((no * no) / (ni * ni) - 1 + c * c);

    //vec3 F = vec3(fresnel(g, c));
    float G = geometry_smith(wi, wo, m, N, mat->alpha);

    float widotN = std::abs(dot(wi, N));
    float wodotN = std::abs(dot(wo, N));

    vec3 Er;

    if (widotN * wodotN < epsilon)
    {
        Er = vec3(0.0f);
    }
    else
    {
        Er = D * G * F / (4 * widotN * wodotN);
    }

    float n = ni / no;

    vec3 mt = -glm::normalize(no * wi + ni * wo);
    float wodotmt = dot(wo, mt);
    float r = 1 - n * n * (1 - wodotmt * wodotmt);

    vec3 Et;

    if (r < 0) Et = Er * A;
    else
    {
        D = distribution(mt, N, mat->alpha);
        F = fresenl(std::abs(dot(wi, mt)), mat->Ks);

        c = std::abs(dot(wi, mt));
        g = sqrt((no * no) / (ni * ni) - 1 + c * c);

        //F = vec3(fresnel(g, c));
        G = geometry_smith(wi, wo, mt, N, mat->alpha);

        float widotmt = dot(wi, mt);
        float denom = (no * widotmt + ni * wodotmt);
        denom = denom * denom;
        
        float widotN = std::abs(dot(wi, N));
        float wodotN = std::abs(dot(wo, N));

        if (widotN * wodotN < epsilon || denom < epsilon)
        {
            Et = vec3(0.0f);
        }
        else
        {
            F = vec3(1, 1, 1) - F;
            F.x = std::abs(F.x);
            F.y = std::abs(F.y);
            F.z = std::abs(F.z);

            Et = D * G * F / (widotN * wodotN);
            Et *= A * (std::abs(widotmt) * std::abs(wodotmt) * no * no) / (denom);
        }

    }

    return abs(dot(N, wi)) * (Ed + Er + Et);
}

vec3 fresenl(float d, vec3 ks)
{
    return ks + (vec3(1, 1, 1) - ks) * static_cast<float>(std::pow(1 - d, 5));
}

float fresnel(float g, float c)
{
    float gplusc = g + c;
    float gminusc = g - c;

    float numerator = c * gplusc - 1.0f;
    float denominator = c * gminusc + 1.0f;

    return 0.5f * ((gminusc * gminusc) / (gplusc * gplusc)) * (1.0f + ((numerator * numerator) / (denominator * denominator)));
}

float distribution(vec3 m, vec3 N, float alpha)
{
    float mDotN = dot(m, N);
    if (mDotN <= 0) return 0.0f;

    if (BRDFTYPE == BRDF_PHONG)
    {
        return std::pow(mDotN, alpha) * (alpha + 2) / (2 * PI);
    }
    else if (BRDFTYPE == BRDF_GGX)
    {
        float alphag = sqrt(2.0f / (alpha + 2));
        float alphagsqure = alphag * alphag;
        float tantheta = sqrt(1.0 - mDotN * mDotN) / mDotN;

        return alphagsqure / (PI * pow((alphagsqure + tantheta * tantheta), 2) * pow(mDotN, 4));
    }
    else
    {
        float tantheta = sqrt(1.0 - mDotN * mDotN) / mDotN;
        float alphab = sqrt(2.0f / (alpha + 2));
        float alphabsqure = alphab * alphab;

        return (1 / (PI * alphabsqure * pow(mDotN, 4)))* exp(-(tantheta * tantheta) / alphabsqure);
    }

    return 0.0f;
}

float geometry_smith(vec3 wi, vec3 wo, vec3 m, vec3 N, float alpha)
{
    float wiDotm = dot(wi, m);
    float wiDotN = dot(wi, N);
    float woDotm = dot(wo, m);
    float woDotN = dot(wo, N);

    return geometry(wiDotm, wiDotN, alpha) * geometry(woDotm, woDotN, alpha);
}

float geometry(float vDotm, float vDotN, float alpha)
{
    if (vDotN > 1.0) return 1.0;

    float tantheta = sqrt(1.0 - vDotN * vDotN) / vDotN;

    if (tantheta == 0.0f) return 1.0f;

    if (BRDFTYPE == BRDF_PHONG)
    {
        float a = std::sqrt((alpha / 2) + 1) / tantheta;

        if (a >= 1.6f) return 1;

        float asquare = a * a;

        return (3.535f * a + 2.181 * asquare) / (1.0 + 2.276 * a + 2.577 * asquare);
    }
    else if (BRDFTYPE == BRDF_GGX)
    {
        float alphag = sqrt(2.0f / (alpha + 2));

        return 2 / (1 + sqrt(1 + alphag * alphag * tantheta * tantheta));
    }
    else
    {
        float alphab = sqrt(2.0f / (alpha + 2));
        float a = 1 / (alphab * tantheta);

        if (a >= 1.6f) return 1;

        float asquare = a * a;

        return (3.535f * a + 2.181 * asquare) / (1.0 + 2.276 * a + 2.577 * asquare);
    }
}

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
void ReadHdrImage(const std::string readName, int& width, int& height, float*& image)
{
    int channels;
    image = stbi_loadf(readName.c_str(), &width, &height, &channels, 3);
    if (!image)
        printf("error reading HDR image: %s\n", readName.c_str());
}

float DistanceObject(vec3 P, Shape* input, Shape*& output)
{
    RayMarchingObject* raymarching = dynamic_cast<RayMarchingObject*>(input);

    float result;
    
    if (CSG* csg = dynamic_cast<CSG*>(raymarching->root); csg != nullptr)
    {
        vec3 Pprime = csg->transform(P);

        result = raymarching->root->distance(Pprime);
    }
    else
    {
        result = raymarching->root->distance(P);
    }

    output = raymarching->root;
    for (std::pair<CSG_COMB, Shape*> child : raymarching->childs)
    {
        float target;
        if (RayMarchingObject* raymarchingchild = dynamic_cast<RayMarchingObject*>(child.second))
        {
            target = DistanceObject(P, raymarchingchild, output);
        }
        else
        {
            if (CSG* csg = dynamic_cast<CSG*>(child.second); csg != nullptr)
            {
                vec3 Pprime = csg->transform(P);

                target = child.second->distance(Pprime);
            }
            else
            {
                target = child.second->distance(P);
            }
        }

        if (child.first == CSG_UNION)
        {
            result = std::min(target, result);
            if (target == result) output = child.second;
        }
        else if (child.first == CSG_SMOOTH_UNION)
        {
            float k = 0.25f;
            float h = 0.5 + 0.5 * (result - target) / k;
            if (h > 1.0) h = 1.0;
            if (h < 0.0) h = 0.0;

            result = result * (1 - h) + target * h;
            result -= k * h * (1.0 - h);
            if (target == result) output = child.second;
        }
        else if (child.first == CSG_SUBTRACTION)
        {
            result = std::max(result, -target);
        }
        else if (child.first == CSG_SMOOTH_SUBTRACTION)
        {
            float k = 0.25f;

            float h = 0.5 - 0.5 * (result + target) / k;
            if (h > 1.0) h = 1.0;
            if (h < 0.0) h = 0.0;

            result = result * (1 - h) - target * h;
            result += k * h * (1.0 - h);
        }
        else if (child.first == CSG_INTERSECTION)
        {
            result = std::max(result, target);
        }
        else if (child.first == CSG_SMOOTH_INTERSECTION)
        {
            float k = 0.25f;

            float h = 0.5 - 0.5 * (result - target) / k;
            if (h > 1.0) h = 1.0;
            if (h < 0.0) h = 0.0;

            result = result * (1 - h) + target * h;
            result += k * h * (1.0 - h);
        }
    }
    return result;
}

void IBL::preprocess()
{
    float* pSinTheta = new float[height];
    float angleFrac = PI / float(height);
    float theta = angleFrac * 0.5f;

    for (unsigned int i = 0; i < height; ++i)
    {
        pSinTheta[i] = sin(theta);

        theta += angleFrac;
    }

    pBuffer = new float[width * (height + 1)];
    pUDist = &pBuffer[width * height];

    for (unsigned int i = 0; i < width; ++i)
    {
        float* pVDist = &pBuffer[i * height];
 
        pVDist[0] = 0.2126f * pixel[i * 3 + 0] + 0.7152f * pixel[i * 3 + 1] + 0.0722f * pixel[i * 3 + 2];
        pVDist[0] *= pSinTheta[0];

        for (unsigned int j = 1; j < height; j++)
        {
            float lum = 0.2126f * pixel[(width + i + j) * 3 + 0] + 0.7152f * pixel[(width + i + j) * 3 + 1] + 0.0722f * pixel[(width + i + j) * 3 + 2];
            pVDist[j] = pVDist[j - 1] + lum * pSinTheta[j];
        }

        if (i == 0)
        {
            pUDist[i] = pVDist[height - 1];
        }
        else
        {
            pUDist[i] = pUDist[i - 1] + pVDist[height - 1];
        }
    }
}

float CSG_Plane::distance(vec3 P)
{
    return dot(N, P) + d;
}

SimpleBox CSG_Plane::boundingbox()
{
    return SimpleBox();
}

float CSG_Torus::distance(vec3 P)
{
    vec3 Pxy = vec3(P.x, P.y, 0.0);
    vec3 v = vec3(length(Pxy) - R, P.z, 0.0);
    return length(v) - r;
}

SimpleBox CSG_Torus::boundingbox()
{
    SimpleBox result;
    result.extend(glm::vec3(-10, -10, -10));
    result.extend(glm::vec3(10, 10, 10));
    return result;

    return SimpleBox();
}

// not exact calculation
float CSG_Cone::distance(vec3 P)
{
    return length(vec3(P.x, P.y, 1.0)) * cos(theta) - abs(P.z) * sin(theta);
}

SimpleBox CSG_Cone::boundingbox()
{
    return SimpleBox();
}

//this function should not be called!
float RayMarchingObject::distance(vec3 P)
{
    printf("cannot be called!");
    return 0.0f;
}

SimpleBox RayMarchingObject::boundingbox()
{
    SimpleBox result = root->boundingbox();
    for (std::pair<CSG_COMB, Shape*> child : childs)
    {
        SimpleBox bb = child.second->boundingbox();
        result.extend(vec3FromBvh(bb.min));
        result.extend(vec3FromBvh(bb.max));
    }
    return result;
}

vec3 CSG::transform(vec3 P)
{
    vec3 result = P - pos;

    result = rot * result;


    //result += vec3(sin(20.0 * result.x) * sin(20.0 * result.y) * sin(20.0 * result.z)) * 0.05f;


    return result;
}

float capsuledistance(vec3 P, vec3 offset)
{
    //P -= offset;

    //const float height = 0.2;
    //const float r = 0.2;
    //float zoffset = P.z;
    //if (P.z < 0.0) zoffset = 0.0;
    //else if (P.z > height) zoffset = height;
    //P.z -= zoffset;
    //return length(P) - r;

    const float r = 0.1f;

    return length(P - offset) - r;
}

float CSG_Curve::distance(vec3 P)
{
    float result = FLT_MAX;

    for (vec3 pt : pts)
    {
        float target = capsuledistance(P, pt);

        float k = 0.1f;
        float h = 0.5 + 0.5 * (result - target) / k;
        if (h > 1.0) h = 1.0;
        if (h < 0.0) h = 0.0;

        result = result * (1 - h) + target * h;
        result -= k * h * (1.0 - h);
    }
    
    return result;
}

SimpleBox CSG_Curve::boundingbox()
{
    SimpleBox result;
    result.extend(glm::vec3(-10, -10, -10));
    result.extend(glm::vec3(10, 10, 10));
    return result;
}
