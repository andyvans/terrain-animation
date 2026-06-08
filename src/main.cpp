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
static const float FLY_SPEED = 5.0f;   // world-units advanced per frame
static float       cameraZ   = 0.0f;

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
static bool project(float wx, float wy, float wz, int &sx, int &sy)
{
    float cz = wz - cameraZ;
    if (cz < 0.5f) return false;
    float inv = FOCAL / cz;
    sx = (int)( wx           * inv) + SCREEN_W / 2;
    sy = (int)(-(wy - CAM_H) * inv) + HORIZON_Y;
    return true;
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    DisplayController.begin();
    DisplayController.setResolution(QVGA_320x240_60Hz, -1, -1, true);
    useDoubleBuffer = DisplayController.isDoubleBufferedEnabled();
    Serial.printf("Double buffering: %s\n", useDoubleBuffer ? "enabled" : "disabled");
    canvas = new fabgl::Canvas(&DisplayController);
}

void loop()
{
    // ── Clear to black ────────────────────────────────────────────────────────
    canvas->setBrushColor(fabgl::Color::Black);
    canvas->clear();
    // World-space origin of the visible grid patch
    float ox = -(GRID_W * 0.5f) * CELL_X;   // leftmost column X
    float oz =  cameraZ + NEAR_Z;            // nearest row Z

    // ── Project all (GRID_W+1) x (GRID_D+1) vertices ─────────────────────────
    for (int zi = 0; zi <= GRID_D; zi++) {
        float wz = oz + zi * CELL_Z;
        for (int xi = 0; xi <= GRID_W; xi++) {
            float wx = ox + xi * CELL_X;
            float wy = terrainY(wx, wz);
            int px, py;
            bool ok    = project(wx, wy, wz, px, py);
            vsx   [zi][xi] = (int16_t)px;
            vsy   [zi][xi] = (int16_t)py;
            vvalid[zi][xi] = ok;
        }
    }

    // ── Draw horizontal (X-axis) grid lines ───────────────────────────────────
    for (int zi = 0; zi <= GRID_D; zi++) {
        // Near rows are brighter; far rows are darker.
        uint8_t g = (uint8_t)(255 - (zi * 180) / GRID_D);
        canvas->setPenColor(fabgl::RGB888(0, g, 0));
        for (int xi = 0; xi < GRID_W; xi++) {
            if (vvalid[zi][xi] && vvalid[zi][xi + 1]) {
                canvas->drawLine(vsx[zi][xi], vsy[zi][xi],
                                 vsx[zi][xi + 1], vsy[zi][xi + 1]);
            }
        }
    }

    // ── Draw depth (Z-axis) grid lines ────────────────────────────────────────
    for (int zi = 0; zi < GRID_D; zi++) {
        uint8_t g = (uint8_t)(255 - (zi * 180) / GRID_D);
        canvas->setPenColor(fabgl::RGB888(0, g, 0));
        for (int xi = 0; xi <= GRID_W; xi++) {
            if (vvalid[zi][xi] && vvalid[zi + 1][xi]) {
                canvas->drawLine(vsx[zi][xi],     vsy[zi][xi],
                                 vsx[zi + 1][xi], vsy[zi + 1][xi]);
            }
        }
    }

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
