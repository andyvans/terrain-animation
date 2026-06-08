#include "fabgl.h"
#include <stdint.h>
#include <math.h>

// ── Display ───────────────────────────────────────────────────────────────────
fabgl::VGAController DisplayController;
fabgl::Canvas        *canvas;
bool                 useDoubleBuffer = false;

static const int SCREEN_W = 320;
static const int SCREEN_H = 240;

// ── Terrain grid ──────────────────────────────────────────────────────────────
// 20 columns x 28 depth rows of world-space quads rendered as wireframe.
// The grid is much wider than the screen so perspective convergence fills
// the full width at the far end.
static const int   GRID_W   = 20;       // number of columns
static const int   GRID_D   = 28;       // number of depth rows
static const float CELL_X   = 100.0f;  // world-units per column
static const float NEAR_Z   = 42.0f;   // world-Z of first row ahead of camera
static const float CELL_Z   = 48.0f;   // world-units per row
static const float AMP_LO   = 70.0f;   // low-freq height amplitude
static const float AMP_HI   = 30.0f;   // high-freq height amplitude

// ── Camera ────────────────────────────────────────────────────────────────────
static const float CAM_H     = 300.0f;  // camera height above terrain base
static const float FOCAL     = 200.0f; // perspective focal length (px)
static const int   HORIZON_Y = 75;     // screen-row of horizon

// ── Flight ────────────────────────────────────────────────────────────────────
static const float FLY_SPEED = 4.5f;   // world-units advanced per frame
static float       cameraZ   = 0.0f;

// ── Purple sky shot ─────────────────────────────────────────────────────────
static const int      MAX_SKY_SHOTS    = 5;
static const uint32_t SHOT_INTERVAL_MS = 220;
static const float    SHOT_START_REL_Z = 1300.0f;
static const float    SHOT_END_REL_Z   = -160.0f;
static const float    SHOT_SPEED       = 1.9f;     // world-units per ms
static const float    SHOT_LENGTH      = 240.0f;   // world-units

static uint32_t lastShotSpawnMs = 0;
static int      nextShotSlot    = 0;

struct SkyShot {
    bool active;
    uint32_t launchMs;
    float farX;
    float farY;
    float nearX;
    float nearY;
};

static SkyShot skyShots[MAX_SKY_SHOTS];

// ── Vertex cache (avoids re-projecting shared grid corners) ───────────────────
static int16_t vsx   [GRID_D + 1][GRID_W + 1];
static int16_t vsy   [GRID_D + 1][GRID_W + 1];
static bool    vvalid[GRID_D + 1][GRID_W + 1];

// ── Noise / terrain height ────────────────────────────────────────────────────
// Deterministic integer hash -> pseudo-random float in [0, 1).
static float latticeNoise(int xi, int zi)
{
    uint32_t n = (uint32_t)(xi * 1619 + zi * 31337);
    n = (n << 13u) ^ n;
    n = n * (n * n * 15731u + 789221u) + 1376312589u;
    return (float)(n & 0x7fffffffu) / (float)0x7fffffff;
}

// Smoothstep-interpolated 2-D value noise sampled at continuous position.
static float smoothNoise(float wx, float wz)
{
    int   ix = (int)floorf(wx);
    int   iz = (int)floorf(wz);
    float tx = wx - (float)ix;
    float tz = wz - (float)iz;
    // Smoothstep (3t^2 - 2t^3)
    tx = tx * tx * (3.0f - 2.0f * tx);
    tz = tz * tz * (3.0f - 2.0f * tz);
    float n00 = latticeNoise(ix,     iz    );
    float n10 = latticeNoise(ix + 1, iz    );
    float n01 = latticeNoise(ix,     iz + 1);
    float n11 = latticeNoise(ix + 1, iz + 1);
    return n00 * (1.0f - tx) * (1.0f - tz)
         + n10 *        tx   * (1.0f - tz)
         + n01 * (1.0f - tx) *        tz
         + n11 *        tx   *        tz;
}

// Two-octave fractal terrain height at world position (wx, wz).
static float terrainY(float wx, float wz)
{
    return smoothNoise(wx * 0.012f, wz * 0.012f) * AMP_LO
         + smoothNoise(wx * 0.040f, wz * 0.040f) * AMP_HI;
}

// ── Perspective projection ─────────────────────────────────────────────────────
// Projects world point (wx, wy, wz) into screen (sx, sy).
// Returns false if the point is at or behind the camera.
static inline bool project(float wx, float wy, float wz, int &sx, int &sy)
{
    float cz = wz - cameraZ;
    if (cz < 0.5f) return false;
    float inv = FOCAL / cz;
    sx = (int)( wx           * inv) + SCREEN_W / 2;
    sy = (int)(-(wy - CAM_H) * inv) + HORIZON_Y;
    return true;
}

// Fast project using precomputed 1/Z for an entire row.
static inline void projectRow(float wx, float wy, float invZ, int &sx, int &sy)
{
    sx = (int)(wx * invZ) + SCREEN_W / 2;
    sy = (int)(-(wy - CAM_H) * invZ) + HORIZON_Y;
}

// Returns true if a line between two screen points is entirely off-screen.
static inline bool lineOffScreen(int x0, int y0, int x1, int y1)
{
    if (x0 < 0 && x1 < 0) return true;
    if (x0 >= SCREEN_W && x1 >= SCREEN_W) return true;
    if (y0 < 0 && y1 < 0) return true;
    if (y0 >= SCREEN_H && y1 >= SCREEN_H) return true;
    return false;
}

static float randRange(float minValue, float maxValue)
{
    long r = random(0, 10000);
    return minValue + ((float)r / 9999.0f) * (maxValue - minValue);
}

static void spawnSkyShot(uint32_t nowMs)
{
    SkyShot &shot = skyShots[nextShotSlot];
    nextShotSlot = (nextShotSlot + 1) % MAX_SKY_SHOTS;

    shot.active = true;
    shot.launchMs = nowMs;

    // Start very far away in the sky, then pass near/past the camera at a random point.
    shot.farX  = randRange(-1200.0f, 1200.0f);
    shot.farY  = CAM_H + randRange(70.0f, 180.0f);
    shot.nearX = randRange(-500.0f, 500.0f);
    shot.nearY = CAM_H + randRange(10.0f, 85.0f);
}

static void drawSkyShot(SkyShot &shot, uint32_t nowMs)
{
    if (!shot.active)
        return;

    float ageMs = (float)(nowMs - shot.launchMs);
    float headRelZ = SHOT_START_REL_Z - ageMs * SHOT_SPEED;
    if (headRelZ < SHOT_END_REL_Z) {
        shot.active = false;
        return;
    }

    float denom = SHOT_START_REL_Z - SHOT_END_REL_Z;
    float headT = (SHOT_START_REL_Z - headRelZ) / denom;
    if (headT < 0.0f) headT = 0.0f;
    if (headT > 1.0f) headT = 1.0f;

    float tailRelZ = headRelZ + SHOT_LENGTH;
    float tailT = (SHOT_START_REL_Z - tailRelZ) / denom;
    if (tailT < 0.0f) tailT = 0.0f;
    if (tailT > 1.0f) tailT = 1.0f;

    float headX = shot.farX + (shot.nearX - shot.farX) * headT;
    float headY = shot.farY + (shot.nearY - shot.farY) * headT;
    float tailX = shot.farX + (shot.nearX - shot.farX) * tailT;
    float tailY = shot.farY + (shot.nearY - shot.farY) * tailT;

    // Draw only visible portions so the streak still appears when one endpoint
    // is out of view or behind the near plane.
    static const int SEGMENTS = 4;
    int prevSX = 0;
    int prevSY = 0;
    bool prevValid = false;

    canvas->setPenColor(fabgl::RGB888(220, 70, 255));
    for (int i = 0; i <= SEGMENTS; ++i) {
        float t = (float)i / (float)SEGMENTS;
        float relZ = tailRelZ + (headRelZ - tailRelZ) * t;
        float pT = (SHOT_START_REL_Z - relZ) / denom;
        if (pT < 0.0f) pT = 0.0f;
        if (pT > 1.0f) pT = 1.0f;

        float px = shot.farX + (shot.nearX - shot.farX) * pT;
        float py = shot.farY + (shot.nearY - shot.farY) * pT;
        int sx, sy;
        bool valid = project(px, py, cameraZ + relZ, sx, sy);

        if (valid && prevValid) {
            canvas->drawLine(prevSX, prevSY, sx, sy);
        }

        prevSX = sx;
        prevSY = sy;
        prevValid = valid;
    }
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    randomSeed((uint32_t)micros());
    DisplayController.begin();
    DisplayController.setResolution(QVGA_320x240_60Hz, -1, -1, true);
    useDoubleBuffer = DisplayController.isDoubleBufferedEnabled();
    Serial.printf("Double buffering: %s\n", useDoubleBuffer ? "enabled" : "disabled");
    canvas = new fabgl::Canvas(&DisplayController);
}

void loop()
{
    uint32_t nowMs = millis();

    // ── Clear to black ────────────────────────────────────────────────────────
    canvas->setBrushColor(fabgl::Color::Black);
    canvas->clear();

    if ((uint32_t)(nowMs - lastShotSpawnMs) >= SHOT_INTERVAL_MS) {
        lastShotSpawnMs = nowMs;
        spawnSkyShot(nowMs);
    }

    canvas->beginUpdate();

    // World-space origin of the visible grid patch
    float ox = -(GRID_W * 0.5f) * CELL_X;   // leftmost column X
    float oz =  cameraZ + NEAR_Z;            // nearest row Z

    // ── Project all vertices using precomputed 1/Z per row ────────────────────
    for (int zi = 0; zi <= GRID_D; zi++) {
        float wz = oz + zi * CELL_Z;
        float cz = wz - cameraZ;
        if (cz < 0.5f) {
            for (int xi = 0; xi <= GRID_W; xi++)
                vvalid[zi][xi] = false;
            continue;
        }
        float invZ = FOCAL / cz;
        for (int xi = 0; xi <= GRID_W; xi++) {
            float wx = ox + xi * CELL_X;
            float wy = terrainY(wx, wz);
            int px, py;
            projectRow(wx, wy, invZ, px, py);
            vsx   [zi][xi] = (int16_t)px;
            vsy   [zi][xi] = (int16_t)py;
            vvalid[zi][xi] = true;
        }
    }

    // ── Draw terrain in one merged pass (horizontal + depth lines per row) ────
    for (int zi = 0; zi <= GRID_D; zi++) {
        uint8_t g = (uint8_t)(255 - (zi * 180) / GRID_D);
        canvas->setPenColor(fabgl::RGB888(0, g, 0));

        // Horizontal lines for this row
        for (int xi = 0; xi < GRID_W; xi++) {
            if (vvalid[zi][xi] && vvalid[zi][xi + 1]) {
                int x0 = vsx[zi][xi], y0 = vsy[zi][xi];
                int x1 = vsx[zi][xi+1], y1 = vsy[zi][xi+1];
                if (!lineOffScreen(x0, y0, x1, y1))
                    canvas->drawLine(x0, y0, x1, y1);
            }
        }

        // Depth lines from this row to the next
        if (zi < GRID_D) {
            for (int xi = 0; xi <= GRID_W; xi++) {
                if (vvalid[zi][xi] && vvalid[zi + 1][xi]) {
                    int x0 = vsx[zi][xi], y0 = vsy[zi][xi];
                    int x1 = vsx[zi+1][xi], y1 = vsy[zi+1][xi];
                    if (!lineOffScreen(x0, y0, x1, y1))
                        canvas->drawLine(x0, y0, x1, y1);
                }
            }
        }
    }

    for (int i = 0; i < MAX_SKY_SHOTS; ++i) {
        drawSkyShot(skyShots[i], nowMs);
    }

    canvas->endUpdate();

    if (useDoubleBuffer) {
        // In double-buffer mode draw as fast as possible, then swap on VSync.
        canvas->waitCompletion(false);
        canvas->swapBuffers();
    } else {
        // Fallback path when there is not enough memory for two frame buffers.
        canvas->waitCompletion(true);
    }

    // Advance the camera forward through the infinite procedural terrain
    cameraZ += FLY_SPEED;
}
