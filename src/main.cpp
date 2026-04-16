// main.cpp — Monte Carlo path tracer
//
// Usage: ./raytrace [scene.scn] [spp]
//
// Reads screen / camera / ambient / ibl from scene.scn.
// Bakes BVH from the scene geometry, loads it, then path-traces.
// Output: <scene>.hdr

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>

#define BAKING
#include "bvh/bvh.hpp"
#include "bvh/bvh_traverse.hpp"
#include "math/math.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// ── Camera ────────────────────────────────────────────────────────────────────

struct Camera
{
    math::vec3 pos;
    math::vec3 fwd;          // world-space forward  (normalized)
    math::vec3 right;        // world-space right    (normalized)
    math::vec3 up;           // world-space up       (normalized)
    float      tanHalfFovY;  // tan(fov_y / 2)
};

// Build camera from position, quaternion orientation, and focal length.
// focal: distance to image plane; image plane height = 1.0 unit.
// tanHalfFovY = 0.5 / focal.
static Camera makeCamera(math::vec3 pos, math::quat orient, float focal)
{
    Camera cam;
    cam.pos         = pos;
    cam.fwd         = math::normalize(orient * math::vec3( 0.f,  0.f, -1.f));
    cam.right       = math::normalize(orient * math::vec3( 1.f,  0.f,  0.f));
    cam.up          = math::normalize(orient * math::vec3( 0.f,  1.f,  0.f));
    cam.tanHalfFovY = 0.5f / focal;
    return cam;
}

// Generate a primary ray for pixel (px, py).
static bvh::Ray primaryRay(const Camera& cam, int px, int py, int width, int height,
                            float jitterX, float jitterY)
{
    float aspect = (float)width / (float)height;
    float u = ((px + jitterX) / width  * 2.f - 1.f) * aspect;
    float v =  (py + jitterY) / height * 2.f - 1.f;

    bvh::Ray ray;
    ray.origin = cam.pos;
    ray.dir    = math::normalize(cam.fwd
                               + cam.right * (u * cam.tanHalfFovY)
                               + cam.up    * (v * cam.tanHalfFovY));
    ray.tmin   = 1e-3f;
    return ray;
}

// ── IBL ───────────────────────────────────────────────────────────────────────

struct IBL
{
    float* data = nullptr;
    int    w    = 0;
    int    h    = 0;

    ~IBL() { if (data) { stbi_image_free(data); } }

    bool load(const std::string& path)
    {
        int ch;
        data = stbi_loadf(path.c_str(), &w, &h, &ch, 3);
        if (!data)
        {
            fprintf(stderr, "Warning: failed to load IBL %s\n", path.c_str());
        }
        return data != nullptr;
    }

    // Sample equirectangular HDR map.
    math::vec3 sample(math::vec3 dir) const
    {
        float phi   =  atan2f(dir.z, dir.x);               // [-pi, pi]
        float theta =  acosf(std::clamp(dir.y, -1.f, 1.f)); // [0, pi]
        float u     = (phi + math::pi) / (2.f * math::pi);
        float v     =  theta / math::pi;
        int   px    =  std::min((int)(u * w), w - 1);
        int   py    =  std::min((int)(v * h), h - 1);
        const float* p = data + (py * w + px) * 3;
        return math::vec3(p[0], p[1], p[2]);
    }
};

// ── Sampling helpers ──────────────────────────────────────────────────────────

// Cosine-weighted hemisphere sample in local space (y = up).
static math::vec3 sampleCosineHemisphere(float u1, float u2)
{
    float r     = sqrtf(u1);
    float theta = 2.f * math::pi * u2;
    float x     = r * cosf(theta);
    float z     = r * sinf(theta);
    float y     = sqrtf(std::max(0.f, 1.f - u1));
    return math::vec3(x, y, z);
}

// Build orthonormal basis from n, then transform localDir into world space.
static math::vec3 toWorld(math::vec3 localDir, math::vec3 n)
{
    math::vec3 up = fabsf(n.x) < 0.9f ? math::vec3(1.f, 0.f, 0.f)
                                       : math::vec3(0.f, 1.f, 0.f);
    math::vec3 t  = math::normalize(math::cross(n, up));
    math::vec3 b  = math::cross(n, t);
    return localDir.x * t + localDir.y * n + localDir.z * b;
}

// ── Path tracing ──────────────────────────────────────────────────────────────

static math::vec3 pathTrace(bvh::Ray ray,
                             const IBL&        ibl,
                             math::vec3        ambient,
                             std::mt19937&     rng,
                             int               maxBounce)
{
    std::uniform_real_distribution<float> ud(0.f, 1.f);

    math::vec3 throughput(1.f, 1.f, 1.f);
    math::vec3 radiance (0.f, 0.f, 0.f);

    for (int bounce = 0; bounce <= maxBounce; ++bounce)
    {
        bvh::Hit hit = bvh::trace(ray);

        if (!hit.hit)
        {
            // Environment: sample IBL or use ambient colour
            math::vec3 env = ibl.data ? ibl.sample(ray.dir) : ambient;
            radiance += throughput * env;
            break;
        }

        // Lambertian albedo (uniform white for now — no per-triangle material)
        const math::vec3 albedo(0.8f, 0.8f, 0.8f);

        // Cosine-weighted hemisphere sample
        math::vec3 localDir = sampleCosineHemisphere(ud(rng), ud(rng));
        math::vec3 worldDir = toWorld(localDir, hit.normal);

        // Lambertian throughput: albedo * cos / (cos/pi) / pi = albedo
        throughput = throughput * albedo;

        // Russian roulette after the 2nd bounce
        if (bounce > 2)
        {
            float p = std::max({ throughput.x, throughput.y, throughput.z });
            if (ud(rng) > p) { break; }
            throughput /= p;
        }

        // Offset origin to avoid self-intersection
        math::vec3 hitPos = ray.origin + ray.dir * hit.t;
        ray.origin        = hitPos;
        ray.dir           = worldDir;
        ray.tmin          = 1e-4f;
        ray.tmax          = std::numeric_limits<float>::infinity();
    }

    return radiance;
}

// ── Scene config parsing ──────────────────────────────────────────────────────

struct SceneConfig
{
    int        width    = 800;
    int        height   = 600;
    math::vec3 camPos   = math::vec3(0.f, 1.f, -5.f);
    math::quat camOrient;                              // identity
    float      focal    = 1.f;
    math::vec3 ambient  = math::vec3(0.1f, 0.1f, 0.1f);
    std::string iblPath;
};

static SceneConfig parseSceneConfig(const std::string& path)
{
    SceneConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) { return cfg; }

    std::string line;
    while (std::getline(f, line))
    {
        std::istringstream iss(line);
        std::string token;
        if (!(iss >> token) || token[0] == '#') { continue; }

        if (token == "screen")
        {
            iss >> cfg.width >> cfg.height;
        }
        else if (token == "camera")
        {
            // camera <px> <py> <pz> <focal> q <qx> <qy> <qz> <qw>
            float px, py, pz, focal;
            std::string orientType;
            float qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;
            if (iss >> px >> py >> pz >> focal >> orientType)
            {
                cfg.camPos  = math::vec3(px, py, pz);
                cfg.focal   = focal;
                if (orientType == "q") { iss >> qx >> qy >> qz >> qw; }
                cfg.camOrient = math::normalize(math::quat(qw, qx, qy, qz));
            }
        }
        else if (token == "ambient")
        {
            float r, g, b;
            if (iss >> r >> g >> b) { cfg.ambient = math::vec3(r, g, b); }
        }
        else if (token == "ibl")
        {
            iss >> cfg.iblPath;
        }
    }

    return cfg;
}

// ── Utilities ─────────────────────────────────────────────────────────────────

static std::string replaceExtension(const std::string& path, const std::string& ext)
{
    auto dot = path.rfind('.');
    return (dot != std::string::npos ? path.substr(0, dot) : path) + ext;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    std::string scenePath = (argc > 1) ? argv[1] : "testscene.scn";
    int         spp       = (argc > 2) ? std::atoi(argv[2]) : 4;

    std::string bvhPath = replaceExtension(scenePath, ".bvh");
    std::string hdrPath = replaceExtension(scenePath, ".hdr");

    // Parse camera / screen / ambient / ibl from scene file
    SceneConfig cfg = parseSceneConfig(scenePath);

    // Bake BVH from scene geometry
    fprintf(stdout, "Baking BVH from %s...\n", scenePath.c_str());
    if (!bvh::bakeBVH(scenePath))
    {
        fprintf(stderr, "Error: bakeBVH failed\n");
        return 1;
    }

    // Load BVH
    if (!bvh::loadBVH(bvhPath))
    {
        fprintf(stderr, "Error: loadBVH failed\n");
        return 1;
    }

    // Load IBL (optional)
    IBL ibl;
    if (!cfg.iblPath.empty()) { ibl.load(cfg.iblPath); }

    const int W = cfg.width;
    const int H = cfg.height;

    Camera cam = makeCamera(cfg.camPos, cfg.camOrient, cfg.focal);

    fprintf(stdout, "Rendering %dx%d @ %d spp...\n", W, H, spp);

    // Accumulation buffer (linear light, top-down)
    std::vector<float> image(W * H * 3, 0.f);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> jitter(0.f, 1.f);

    const int maxBounce = 8;
    const float invSpp  = 1.f / (float)spp;

    for (int y = 0; y < H; ++y)
    {
        if (y % 64 == 0)
        {
            fprintf(stdout, "  row %d / %d\n", y, H);
            fflush(stdout);
        }

        for (int x = 0; x < W; ++x)
        {
            math::vec3 accum(0.f, 0.f, 0.f);

            for (int s = 0; s < spp; ++s)
            {
                bvh::Ray ray = primaryRay(cam, x, y, W, H,
                                          jitter(rng), jitter(rng));
                accum += pathTrace(ray, ibl, cfg.ambient, rng, maxBounce);
            }

            accum /= (float)spp;

            int idx = (y * W + x) * 3;
            image[idx + 0] = accum.x;
            image[idx + 1] = accum.y;
            image[idx + 2] = accum.z;
        }
    }

    // Flip y: stb writes top-down; our y=0 is bottom
    std::vector<float> flipped(W * H * 3);
    for (int y = 0; y < H; ++y)
    {
        int src = y;
        int dst = H - 1 - y;
        std::copy(image.begin() + src * W * 3,
                  image.begin() + src * W * 3 + W * 3,
                  flipped.begin() + dst * W * 3);
    }

    if (!stbi_write_hdr(hdrPath.c_str(), W, H, 3, flipped.data()))
    {
        fprintf(stderr, "Error: failed to write %s\n", hdrPath.c_str());
        return 1;
    }

    fprintf(stdout, "Written: %s\n", hdrPath.c_str());
    return 0;
}
