#include <AnimatedGIF.h>   // must come before GifPlayer.h so GIFDRAW/AnimatedGIF are defined
#include "GifPlayer.h"
#include <Arduino.h>
#include "fabgl.h"
#include <stdio.h>

#define SDCARD_MOUNT_PATH  "/sdcard"

void GifPlayer::setCanvas(fabgl::Canvas *c)
{
    canvas = c;
}

void GifPlayer::drawCallback(GIFDRAW *pDraw)
{
    GifPlayer *self = (GifPlayer *)pDraw->pUser;
    if (!self || !self->frameBuffer) return;

    int destY = pDraw->iY + pDraw->y;
    if (destY >= self->gifHeight) return;

    uint8_t *src = pDraw->pPixels;
    uint16_t *pal = pDraw->pPalette;  // RGB565 entries (native format — avoids RGB888 conversion bugs)
    uint8_t *dst = self->frameBuffer + (destY * self->gifWidth + pDraw->iX) * 4;

    // Disposal method 2: restore transparent pixels to the GIF background colour
    // before rendering — prevents old frame content bleeding through on palette change.
    if (pDraw->ucDisposalMethod == 2 && pDraw->ucHasTransparency) {
        for (int x = 0; x < pDraw->iWidth; x++) {
            if (src[x] == pDraw->ucTransparent)
                src[x] = pDraw->ucBackground;
        }
        pDraw->ucHasTransparency = 0;
    }

    for (int x = 0; x < pDraw->iWidth && (pDraw->iX + x) < self->gifWidth; x++) {
        uint8_t idx = src[x];
        if (pDraw->ucHasTransparency && idx == pDraw->ucTransparent) {
            dst += 4;
            continue;
        }
        // RGB565 → RGBA8888: expand 5/6/5 bits to 8 bits each
        uint16_t rgb = pal[idx];
        uint8_t r = (rgb >> 11) & 0x1F; r = (r << 3) | (r >> 2);
        uint8_t g = (rgb >> 5)  & 0x3F; g = (g << 2) | (g >> 4);
        uint8_t b =  rgb        & 0x1F; b = (b << 3) | (b >> 2);
        *dst++ = r;
        *dst++ = g;
        *dst++ = b;
        *dst++ = 255; // A
    }
}

void GifPlayer::openFromBuffer()
{
    // Clear to opaque black so frame 1 always composites against a known state
    if (frameBuffer && gifWidth > 0 && gifHeight > 0) {
        uint32_t *p32 = (uint32_t *)frameBuffer;
        for (int i = 0; i < gifWidth * gifHeight; i++) p32[i] = 0xFF000000;
    }
    gif->begin(GIF_PALETTE_RGB565_LE);
    gif->open(fileBuffer, (int)fileSize, drawCallback);
}

bool GifPlayer::begin(const char *filePath, uint32_t duration)
{
    stop();

    if (!canvas) {
        Serial.println("GifPlayer: canvas not set");
        return false;
    }

    char fullPath[128];
    snprintf(fullPath, sizeof(fullPath), "%s%s", SDCARD_MOUNT_PATH, filePath);
    FILE *f = fopen(fullPath, "rb");
    if (!f) {
        Serial.printf("GifPlayer: cannot open %s\n", fullPath);
        return false;
    }
    fseek(f, 0, SEEK_END);
    fileSize = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET);

    // Reuse file buffer if large enough, otherwise reallocate
    if (!fileBuffer || fileBufferCapacity < fileSize) {
        if (fileBuffer) free(fileBuffer);
        fileBuffer = (uint8_t *)ps_malloc(fileSize);
        fileBufferCapacity = fileBuffer ? fileSize : 0;
    }
    if (!fileBuffer) {
        Serial.printf("GifPlayer: not enough PSRAM for file (%u bytes)\n", fileSize);
        fclose(f);
        return false;
    }
    fread(fileBuffer, 1, fileSize, f);
    fclose(f);

    // Reuse AnimatedGIF object
    if (!gif) gif = new AnimatedGIF();
    openFromBuffer();

    gifWidth  = gif->getCanvasWidth();
    gifHeight = gif->getCanvasHeight();
    Serial.printf("GifPlayer: loaded %s  %dx%d  (%u bytes)\n", filePath, gifWidth, gifHeight, fileSize);

    // Reuse frame buffer if same dimensions
    int bufSize = gifWidth * gifHeight * 4;
    if (!frameBuffer || frameBufferCapacity < bufSize) {
        if (frameBuffer) free(frameBuffer);
        frameBuffer = (uint8_t *)ps_malloc(bufSize);
        frameBufferCapacity = frameBuffer ? bufSize : 0;
    }
    if (!frameBuffer) {
        Serial.printf("GifPlayer: not enough PSRAM for frame buffer (%d bytes)\n", bufSize);
        gif->close();
        return false;
    }
    // Initialise to opaque black
    uint32_t *p32 = (uint32_t *)frameBuffer;
    for (int i = 0; i < gifWidth * gifHeight; i++) p32[i] = 0xFF000000;

    // Reuse or recreate bitmap if dimensions changed
    if (frameBitmap) delete frameBitmap;
    frameBitmap = new fabgl::Bitmap(gifWidth, gifHeight, frameBuffer, fabgl::PixelFormat::RGBA8888, false);

    startTimeMs = millis();
    durationMs  = duration;
    nextFrameMs = 0;
    active      = true;
    return true;
}

void GifPlayer::tick()
{
    if (!active || !canvas || !frameBitmap || !gif) return;

    uint32_t now = millis();

    if (now - startTimeMs >= durationMs) {
        stop();
        return;
    }

    if (now < nextFrameMs) return;

    canvas->waitCompletion(false);

    int frameDelay = 0;
    int result = gif->playFrame(false, &frameDelay, this);

    canvas->drawBitmap(0, 0, frameBitmap);
    canvas->swapBuffers();

    nextFrameMs = now + max(frameDelay, 16);

    if (result == 0) {
        // Last frame — restart from in-RAM buffer
        gif->close();
        openFromBuffer();
    }
}

void GifPlayer::stop()
{
    active = false;
    if (gif) {
        gif->close();
        // Keep gif object for reuse — don't delete
    }
    if (frameBitmap) {
        delete frameBitmap;
        frameBitmap = nullptr;
    }
    // Keep frameBuffer and fileBuffer allocated for reuse
}

bool GifPlayer::isActive() const
{
    return active;
}
