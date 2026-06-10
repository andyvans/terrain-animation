#include <Arduino.h>
#include "Terrain.h"

Terrain terrain;

void setup()
{
    Serial.begin(115200);

    terrain.begin();
}

void loop()
{
    terrain.run();
}
