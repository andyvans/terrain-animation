#pragma once

#include <stdint.h>

class Terrain {
public:
    void begin();
    void run();

private:
    struct SkyShot;

    static float latticeNoise(int xi, int zi);
    static float smoothNoise(float wx, float wz);
    float terrainY(float wx, float wz) const;

    bool project(float wx, float wy, float wz, int &sx, int &sy) const;
    void projectRow(float wx, float wy, float invZ, int &sx, int &sy) const;
    static bool lineOffScreen(int x0, int y0, int x1, int y1);

    static float randRange(float minValue, float maxValue);
    void spawnSkyShot(uint32_t nowMs);
    void drawSkyShot(SkyShot &shot, uint32_t nowMs);

    void renderTerrain();
    void renderSkyShots(uint32_t nowMs);

    class Impl;
    Impl *impl = nullptr;
};
