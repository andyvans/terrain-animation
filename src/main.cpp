#include <Arduino.h>
#include "fabgl.h"
#include "fabutils.h"
#include "Terrain.h"
#include "GifPlayer.h"
#include <vector>
#include <string>

#define SDCARD_MOUNT_PATH   "/sdcard"
#define GIF_DURATION_MS     15000
#define TERRAIN_DURATION_MS 15000

enum AppState { STATE_GIF, STATE_TERRAIN };

fabgl::VGAController vgaController;
Terrain    terrain;
GifPlayer  gifPlayer;
AppState   appState    = STATE_GIF;
uint32_t   terrainStartMs = 0;

std::vector<std::string> gifFiles;
int gifIndex = 0;

void scanGifs()
{
    gifFiles.clear();
    FileBrowser fb;
    if (!fb.setDirectory(SDCARD_MOUNT_PATH)) {
        Serial.println("scanGifs: cannot open SD root");
        return;
    }
    for (int i = 0; i < fb.count(); i++) {
        fabgl::DirItem const *item = fb.get(i);
        if (item->isDir) continue;
        std::string name(item->name);
        if (name.size() >= 4) {
            std::string ext = name.substr(name.size() - 4);
            for (char &c : ext) c = tolower(c);
            if (ext == ".gif") {
                gifFiles.push_back(std::string("/") + name);
                Serial.printf("Found GIF: %s\n", name.c_str());
            }
        }
    }
    Serial.printf("Total GIFs found: %d\n", (int)gifFiles.size());
}

void startNextGif()
{
    if (gifFiles.empty()) {
        Serial.println("No GIFs — going to terrain");
        terrainStartMs = millis();
        appState = STATE_TERRAIN;
        return;
    }
    const char *path = gifFiles[gifIndex].c_str();
    Serial.printf("Playing GIF %d/%d: %s\n", gifIndex + 1, (int)gifFiles.size(), path);
    gifPlayer.begin(path, GIF_DURATION_MS);
    appState = STATE_GIF;
}

void setup()
{
    Serial.begin(115200);

    vgaController.begin();
    vgaController.setResolution(QVGA_320x240_60Hz, -1, -1, true);
    bool doubleBuffered = vgaController.isDoubleBufferedEnabled();
    Serial.printf("Double buffering: %s\n", doubleBuffered ? "enabled" : "disabled");

    // Diagnose chip package — FabGL overrides SD pins based on this
    fabgl::ChipPackage pkg = fabgl::getChipPackage();
    const char *pkgName = "Unknown";
    switch (pkg) {
        case fabgl::ChipPackage::ESP32D0WDQ6: pkgName = "ESP32D0WDQ6 (WROOM-32)"; break;
        case fabgl::ChipPackage::ESP32D0WDQ5: pkgName = "ESP32D0WDQ5 (WROVER-B)"; break;
        case fabgl::ChipPackage::ESP32D2WDQ5: pkgName = "ESP32D2WDQ5"; break;
        case fabgl::ChipPackage::ESP32PICOD4: pkgName = "ESP32PICOD4 (TTGO-VGA32)"; break;
        default: break;
    }
    Serial.printf("Chip package: %s\n", pkgName);

    // Mount SD card — reduce speed to 400 kHz for reliable init
    FileBrowser::setSDCardMaxFreqKHz(400);
    if (!FileBrowser::mountSDCard(false, SDCARD_MOUNT_PATH)) {
        Serial.println("SD mount failed — check wiring");
    } else {
        Serial.println("SD mounted OK");
    }

    fabgl::Canvas *canvas = new fabgl::Canvas(&vgaController);
    terrain.begin();
    terrain.setCanvas(canvas);
    terrain.setDoubleBuffered(doubleBuffered);
    gifPlayer.setCanvas(canvas);

    scanGifs();
    gifIndex = 0;
    startNextGif();
}

void loop()
{
    switch (appState) {
        case STATE_GIF:
            gifPlayer.tick();
            if (!gifPlayer.isActive()) {
                gifIndex = (gifIndex + 1) % (int)gifFiles.size();
                if (gifIndex == 0) {
                    // completed one full cycle — switch to terrain
                    Serial.println("GIF cycle complete, playing terrain");
                    terrainStartMs = millis();
                    appState = STATE_TERRAIN;
                } else {
                    startNextGif();
                }
            }
            break;

        case STATE_TERRAIN:
            terrain.run();
            if (millis() - terrainStartMs >= TERRAIN_DURATION_MS) {
                Serial.println("Terrain done, restarting GIF cycle");
                gifIndex = 0;
                startNextGif();
            }
            break;
    }
}
