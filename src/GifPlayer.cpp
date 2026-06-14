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
    uint8_t *pal = pDraw->pPalette24; // RGB888 entries when GIF_PALETTE_RGB888
    uint8_t *dst = self->frameBuffer + (destY * self->gifWidth + pDraw->iX) * 4;

    for (int x = 0; x < pDraw->iWidth && (pDraw->iX + x) < self->gifWidth; x++) {
        uint8_t idx = src[x];
        if (pDraw->ucHasTransparency && idx == pDraw->ucTransparent) {
            dst += 4;
            continue;
        }
        uint8_t *c = &pal[idx * 3];
        *dst++ = c[0]; // R
        *dst++ = c[1]; // G
        *dst++ = c[2]; // B
        *dst++ = 255;  // A
    }
}

void GifPlayer::openFromBuffer()
{
    gif->begin(GIF_PALETTE_RGB888);
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
    fileBuffer = (uint8_t *)ps_malloc(fileSize);
    if (!fileBuffer) {
        Serial.printf("GifPlayer: not enough PSRAM for file (%u bytes)\n", fileSize);
        fclose(f);
        return false;
    }
    fread(fileBuffer, 1, fileSize, f);
    fclose(f);

    gif = new AnimatedGIF();
    openFromBuffer();

    gifWidth  = gif->getCanvasWidth();
    gifHeight = gif->getCanvasHeight();
    Serial.printf("GifPlayer: loaded %s  %dx%d  (%u bytes)\n", filePath, gifWidth, gifHeight, fileSize);

    int bufSize = gifWidth * gifHeight * 4;
    frameBuffer = (uint8_t *)ps_malloc(bufSize);
    if (!frameBuffer) {
        Serial.printf("GifPlayer: not enough PSRAM for frame buffer (%d bytes)\n", bufSize);
        gif->close();
        delete gif; gif = nullptr;
        free(fileBuffer); fileBuffer = nullptr;
        return false;
    }
    memset(frameBuffer, 0, bufSize);

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
        // Last frame -- restart from in-RAM buffer
        gif->close();
        openFromBuffer();
    }
}

void GifPlayer::stop()
{
    active = false;
    if (gif) {
        gif->close();
        delete gif;
        gif = nullptr;
    }
    if (frameBitmap) {
        delete frameBitmap;
        frameBitmap = nullptr;
    }
    if (frameBuffer) {
        free(frameBuffer);
        frameBuffer = nullptr;
    }
    if (fileBuffer) {
        free(fileBuffer);
        fileBuffer = nullptr;
        fileSize = 0;
    }
}

bool GifPlayer::isActive() const
{
    return active;
}
