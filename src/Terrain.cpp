#include "Terrain.h"

#include <Arduino.h>
#include "fabgl.h"
#include <stdint.h>
#include <math.h>

struct Terrain::SkyShot {
    // Launch-time and interpolated endpoints for one streak in the sky.
    bool active;
    uint32_t launchMs;
    float farX;
    float farY;
    float nearX;
    float nearY;
};

class Terrain::Impl {
public:
    // Display
    fabgl::VGAController displayController;
    fabgl::Canvas *canvas = nullptr;
    bool useDoubleBuffer = false;

    static constexpr int SCREEN_W = 320;   // Output framebuffer width in pixels.
    static constexpr int SCREEN_H = 240;   // Output framebuffer height in pixels.

    // Terrain grid
    static constexpr int GRID_W_MAX = 30;  // Compile-time max terrain columns.
    static constexpr int GRID_D_MAX = 40;  // Compile-time max terrain depth rows.
    int gridW = 20;
    int gridD = 28;
    static constexpr float CELL_X = 100.0f; // World-space width per terrain column.
    static constexpr float NEAR_Z = 42.0f;  // Camera-space Z of nearest terrain row.
    static constexpr float CELL_Z = 48.0f;  // World/camera-space spacing between rows.
    static constexpr float AMP_LO = 90.0f;  // Low-frequency terrain height amplitude.
    static constexpr float AMP_HI = 30.0f;  // High-frequency terrain height amplitude.

    // Camera
    static constexpr float CAM_H = 300.0f;      // Camera height above terrain base.
    static constexpr float FOCAL = 200.0f;      // Perspective focal length in pixels.
    static constexpr int HORIZON_Y = 75;        // Screen Y location of horizon line.
    static constexpr float GRID_MIN_CZ = 10.0f; // Near clip for terrain rows (prevents overflow).

    // Flight
    static constexpr float FLY_SPEED = 4.5f; // Forward camera movement per frame.
    float cameraZ = 0.0f;

    // Sky shots
    static constexpr int MAX_SKY_SHOTS_CAP = 12; // Fixed pool size for sky-shot streaks.
    int activeSkyShots = 5;
    uint32_t shotIntervalMs = 220;
    static constexpr float SHOT_START_REL_Z = 1300.0f; // Spawn depth ahead of camera.
    static constexpr float SHOT_END_REL_Z = -160.0f;   // Despawn depth past camera.
    static constexpr float SHOT_SPEED = 1.9f;          // Sky-shot head speed in world-units/ms.
    static constexpr float SHOT_LENGTH = 240.0f;       // Sky-shot streak length in world units.

    uint32_t lastShotSpawnMs = 0;
    int nextShotSlot = 0;
    Terrain::SkyShot skyShots[MAX_SKY_SHOTS_CAP] = {};

    // Vertex cache
    int16_t vsx[GRID_D_MAX + 1][GRID_W_MAX + 1] = {};
    int16_t vsy[GRID_D_MAX + 1][GRID_W_MAX + 1] = {};
    bool vvalid[GRID_D_MAX + 1][GRID_W_MAX + 1] = {};
};

float Terrain::latticeNoise(int xi, int zi)
{
    // Deterministic integer hash mapped to [0, 1). Used as lattice noise.
    uint32_t n = (uint32_t)(xi * 1619 + zi * 31337);
    n = (n << 13u) ^ n;
    n = n * (n * n * 15731u + 789221u) + 1376312589u;
    return (float)(n & 0x7fffffffu) / (float)0x7fffffff;
}

float Terrain::smoothNoise(float wx, float wz)
{
    // Bilinear interpolation between neighboring lattice samples.
    int ix = (int)floorf(wx);
    int iz = (int)floorf(wz);
    float tx = wx - (float)ix;
    float tz = wz - (float)iz;
    tx = tx * tx * (3.0f - 2.0f * tx);
    tz = tz * tz * (3.0f - 2.0f * tz);

    float n00 = latticeNoise(ix, iz);
    float n10 = latticeNoise(ix + 1, iz);
    float n01 = latticeNoise(ix, iz + 1);
    float n11 = latticeNoise(ix + 1, iz + 1);

    return n00 * (1.0f - tx) * (1.0f - tz)
         + n10 * tx * (1.0f - tz)
         + n01 * (1.0f - tx) * tz
         + n11 * tx * tz;
}

float Terrain::terrainY(float wx, float wz) const
{
    // Two-octave value noise for broad hills plus fine detail.
    return smoothNoise(wx * 0.012f, wz * 0.012f) * Impl::AMP_LO
         + smoothNoise(wx * 0.040f, wz * 0.040f) * Impl::AMP_HI;
}

bool Terrain::project(float wx, float wy, float wz, int &sx, int &sy) const
{
    // Perspective project a world point; reject points too close/behind camera.
    float cz = wz - impl->cameraZ;
    if (cz < 0.5f) {
        return false;
    }

    float inv = Impl::FOCAL / cz;
    sx = (int)(wx * inv) + Impl::SCREEN_W / 2;
    sy = (int)(-(wy - Impl::CAM_H) * inv) + Impl::HORIZON_Y;
    return true;
}

void Terrain::projectRow(float wx, float wy, float invZ, int &sx, int &sy) const
{
    // Fast projection variant for rows that share the same 1/Z value.
    sx = (int)(wx * invZ) + Impl::SCREEN_W / 2;
    sy = (int)(-(wy - Impl::CAM_H) * invZ) + Impl::HORIZON_Y;
}

bool Terrain::lineOffScreen(int x0, int y0, int x1, int y1)
{
    // Quick reject if both endpoints are outside on the same side.
    if (x0 < 0 && x1 < 0) return true;
    if (x0 >= Impl::SCREEN_W && x1 >= Impl::SCREEN_W) return true;
    if (y0 < 0 && y1 < 0) return true;
    if (y0 >= Impl::SCREEN_H && y1 >= Impl::SCREEN_H) return true;
    return false;
}

float Terrain::randRange(float minValue, float maxValue)
{
    // Map Arduino integer RNG output to a floating-point range.
    long r = random(0, 10000);
    return minValue + ((float)r / 9999.0f) * (maxValue - minValue);
}

void Terrain::spawnSkyShot(uint32_t nowMs)
{
    // Ring-buffer allocation: recycle old slots instead of allocating/freeing.
    if (impl->activeSkyShots < 1) {
        return;
    }

    SkyShot &shot = impl->skyShots[impl->nextShotSlot];
    impl->nextShotSlot = (impl->nextShotSlot + 1) % impl->activeSkyShots;

    shot.active = true;
    shot.launchMs = nowMs;
    // Start far away in the sky, then fly by near the camera.
    shot.farX = randRange(-1200.0f, 1200.0f);
    shot.farY = Impl::CAM_H + randRange(70.0f, 180.0f);
    shot.nearX = randRange(-500.0f, 500.0f);
    shot.nearY = Impl::CAM_H + randRange(10.0f, 85.0f);
}

void Terrain::drawSkyShot(Terrain::SkyShot &shot, uint32_t nowMs)
{
    if (!shot.active) {
        return;
    }

    // Move the shot head toward the camera in camera-relative Z space.
    float ageMs = (float)(nowMs - shot.launchMs);
    float headRelZ = Impl::SHOT_START_REL_Z - ageMs * Impl::SHOT_SPEED;
    if (headRelZ < Impl::SHOT_END_REL_Z) {
        shot.active = false;
        return;
    }

    float denom = Impl::SHOT_START_REL_Z - Impl::SHOT_END_REL_Z;
    float headT = (Impl::SHOT_START_REL_Z - headRelZ) / denom;
    if (headT < 0.0f) headT = 0.0f;
    if (headT > 1.0f) headT = 1.0f;

    float tailRelZ = headRelZ + Impl::SHOT_LENGTH;
    float tailT = (Impl::SHOT_START_REL_Z - tailRelZ) / denom;
    if (tailT < 0.0f) tailT = 0.0f;
    if (tailT > 1.0f) tailT = 1.0f;

    static const int SEGMENTS = 4;
    int prevSX = 0;
    int prevSY = 0;
    bool prevValid = false;

    // Draw visible sub-segments so partially clipped streaks still render.
    impl->canvas->setPenColor(fabgl::RGB888(220, 70, 255));
    for (int i = 0; i <= SEGMENTS; ++i) {
        float t = (float)i / (float)SEGMENTS;
        float relZ = tailRelZ + (headRelZ - tailRelZ) * t;
        float pT = (Impl::SHOT_START_REL_Z - relZ) / denom;
        if (pT < 0.0f) pT = 0.0f;
        if (pT > 1.0f) pT = 1.0f;

        float px = shot.farX + (shot.nearX - shot.farX) * pT;
        float py = shot.farY + (shot.nearY - shot.farY) * pT;
        int sx;
        int sy;
        bool valid = project(px, py, impl->cameraZ + relZ, sx, sy);

        if (valid && prevValid) {
            impl->canvas->drawLine(prevSX, prevSY, sx, sy);
        }

        prevSX = sx;
        prevSY = sy;
        prevValid = valid;
    }
}

void Terrain::renderTerrain()
{
    // World-space origin of the visible grid patch.
    float ox = -(impl->gridW * 0.5f) * Impl::CELL_X;
    // Scroll rows through camera space so the grid itself appears to move.
    float rowPhase = fmodf(impl->cameraZ, Impl::CELL_Z);
    if (rowPhase < 0.0f) {
        rowPhase += Impl::CELL_Z;
    }

    // Project all vertices using one 1/Z per row for speed.
    for (int zi = 0; zi <= impl->gridD; zi++) {
        float cz = Impl::NEAR_Z + zi * Impl::CELL_Z - rowPhase;
        // Reject very near rows to avoid huge projections overflowing int16 cache.
        if (cz < Impl::GRID_MIN_CZ) {
            for (int xi = 0; xi <= impl->gridW; xi++) {
                impl->vvalid[zi][xi] = false;
            }
            continue;
        }

        // Height sampling stays in world space to keep terrain continuous.
        float wz = impl->cameraZ + cz;
        float invZ = Impl::FOCAL / cz;
        for (int xi = 0; xi <= impl->gridW; xi++) {
            float wx = ox + xi * Impl::CELL_X;
            float wy = terrainY(wx, wz);
            int px;
            int py;
            projectRow(wx, wy, invZ, px, py);
            impl->vsx[zi][xi] = (int16_t)px;
            impl->vsy[zi][xi] = (int16_t)py;
            impl->vvalid[zi][xi] = true;
        }
    }

    // Draw merged terrain pass: horizontal and depth lines per row.
    for (int zi = 0; zi <= impl->gridD; zi++) {
        // Fade distant rows darker to reinforce depth.
        uint8_t g = (uint8_t)(255 - (zi * 180) / (impl->gridD > 0 ? impl->gridD : 1));
        impl->canvas->setPenColor(fabgl::RGB888(0, g, 0));

        for (int xi = 0; xi < impl->gridW; xi++) {
            if (impl->vvalid[zi][xi] && impl->vvalid[zi][xi + 1]) {
                int x0 = impl->vsx[zi][xi];
                int y0 = impl->vsy[zi][xi];
                int x1 = impl->vsx[zi][xi + 1];
                int y1 = impl->vsy[zi][xi + 1];
                if (!lineOffScreen(x0, y0, x1, y1)) {
                    impl->canvas->drawLine(x0, y0, x1, y1);
                }
            }
        }

        if (zi < impl->gridD) {
            for (int xi = 0; xi <= impl->gridW; xi++) {
                if (impl->vvalid[zi][xi] && impl->vvalid[zi + 1][xi]) {
                    int x0 = impl->vsx[zi][xi];
                    int y0 = impl->vsy[zi][xi];
                    int x1 = impl->vsx[zi + 1][xi];
                    int y1 = impl->vsy[zi + 1][xi];
                    if (!lineOffScreen(x0, y0, x1, y1)) {
                        impl->canvas->drawLine(x0, y0, x1, y1);
                    }
                }
            }
        }
    }
}

void Terrain::renderSkyShots(uint32_t nowMs)
{
    // Update and draw each active sky streak this frame.
    for (int i = 0; i < impl->activeSkyShots; ++i) {
        drawSkyShot(impl->skyShots[i], nowMs);
    }
}

void Terrain::begin()
{
    // Allocate runtime state once, then reuse it across resets/restarts.
    if (impl == nullptr) {
        impl = new Impl();
    }
  
    uint32_t psramSize = ESP.getPsramSize();
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM size: %u bytes\n", psramSize);
    Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());

    // Choose render quality profile based on available PSRAM.
    if (psramSize >= 4UL * 1024UL * 1024UL) {
        impl->gridW = 30;
        impl->gridD = 40;
        impl->activeSkyShots = 10;
        impl->shotIntervalMs = 140;
    } else {
        impl->gridW = 20;
        impl->gridD = 28;
        impl->activeSkyShots = 5;
        impl->shotIntervalMs = 220;
    }

    impl->nextShotSlot = 0;
    Serial.printf("Quality profile: grid=%dx%d, shots=%d, interval=%ums\n", impl->gridW, impl->gridD, impl->activeSkyShots, impl->shotIntervalMs);

    // Seed RNG for sky-shot spawn variation and endpoint placement.
    randomSeed((uint32_t)micros());
    impl->displayController.begin();
    impl->displayController.setResolution(QVGA_320x240_60Hz, -1, -1, true);
    impl->useDoubleBuffer = impl->displayController.isDoubleBufferedEnabled();
    Serial.printf("Double buffering: %s\n", impl->useDoubleBuffer ? "enabled" : "disabled");

    if (impl->canvas == nullptr) {
        // Canvas wraps display controller drawing primitives.
        impl->canvas = new fabgl::Canvas(&impl->displayController);
    }
}

void Terrain::run()
{
    if (impl == nullptr || impl->canvas == nullptr) {
        return;
    }

    uint32_t nowMs = millis();

    // Clear frame before issuing terrain and sky draws.
    impl->canvas->setBrushColor(fabgl::Color::Black);
    impl->canvas->clear();

    // Spawn at fixed cadence independent from frame-rate jitter.
    if ((uint32_t)(nowMs - impl->lastShotSpawnMs) >= impl->shotIntervalMs) {
        impl->lastShotSpawnMs = nowMs;
        spawnSkyShot(nowMs);
    }

    // Batch rendering commands for higher throughput.
    impl->canvas->beginUpdate();

    renderTerrain();
    renderSkyShots(nowMs);

    impl->canvas->endUpdate();

    if (impl->useDoubleBuffer) {
        // Draw as fast as possible, then swap on VSync.
        impl->canvas->waitCompletion(false);
        impl->canvas->swapBuffers();
    } else {
        // Fallback when only a single frame buffer is available.
        impl->canvas->waitCompletion(true);
    }

    // Advance forward through the procedural terrain field.
    impl->cameraZ += Impl::FLY_SPEED;
}
