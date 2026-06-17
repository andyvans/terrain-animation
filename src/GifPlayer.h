#pragma once

#include <stddef.h>
#include <stdint.h>

// Forward declarations — AnimatedGIF.h is included only by GifPlayer.cpp
struct gif_draw_tag;
typedef struct gif_draw_tag GIFDRAW;
class AnimatedGIF;

namespace fabgl {
    class Canvas;
    struct Bitmap;
}

class GifPlayer {
public:
    void setCanvas(fabgl::Canvas *canvas);

    // Load and play a GIF from SD card, looping until durationMs elapses.
    // Returns false if the file could not be opened.
    bool begin(const char *filePath, uint32_t durationMs);

    // Call every loop iteration while isActive() is true.
    void tick();

    bool isActive() const;
    void stop();

private:
    static void drawCallback(GIFDRAW *pDraw);
    void openFromBuffer();

    fabgl::Canvas  *canvas      = nullptr;
    fabgl::Bitmap  *frameBitmap = nullptr;
    uint8_t        *frameBuffer = nullptr;  // RGBA8888 pixels in PSRAM
    size_t          frameBufferCapacity = 0;
    uint8_t        *fileBuffer  = nullptr;  // full GIF file in PSRAM
    size_t          fileBufferCapacity  = 0;
    size_t          fileSize    = 0;
    AnimatedGIF    *gif         = nullptr;  // heap-allocated decoder (reused)
    uint32_t        startTimeMs = 0;
    uint32_t        durationMs  = 0;
    uint32_t        nextFrameMs = 0;
    int             gifWidth    = 0;
    int             gifHeight   = 0;
    bool            active      = false;
};
