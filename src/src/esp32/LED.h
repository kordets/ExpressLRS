#include "common.h"

/// LED SUPPORT ///////
#include <NeoPixelBus.h>

const uint16_t PixelCount = 4; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 27;   // make sure to set this to the correct pin, ignored for Esp8266
const uint8_t numberOfLEDs = 3;
uint16_t LEDGlowIndex = 0;
#define colorSaturation 50
NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

HslColor hslRed(red);
HslColor hslGreen(green);
HslColor hslBlue(blue);
HslColor hslWhite(white);
HslColor hslBlack(black);

void updateLEDs(uint8_t isRXconnected, uint8_t tlm)
{
    LEDGlowIndex = millis() % 5000;

    if (LEDGlowIndex < 2500)
    {
        LEDGlowIndex = LEDGlowIndex / 10;
    }
    else
    {
        LEDGlowIndex = 250 - (LEDGlowIndex - 2500) / 10;
    }

    for (int n = 0; n < numberOfLEDs; n++)
        strip.SetPixelColor(n, RgbColor(LEDGlowIndex, LEDGlowIndex, LEDGlowIndex));

    if (isRXconnected || tlm == 0)
    {
        uint8_t index = current_rate_config;
        if (index == 0)
        {
            for (int n = 0; n < numberOfLEDs; n++)
                strip.SetPixelColor(n, RgbColor(0, 0, LEDGlowIndex));
        }
        else if (index == 1)
        {
            for (int n = 0; n < numberOfLEDs; n++)
                strip.SetPixelColor(n, RgbColor(0, LEDGlowIndex, 0));
        }
        else if (index == 2)
        {
            for (int n = 0; n < numberOfLEDs; n++)
                strip.SetPixelColor(n, RgbColor(LEDGlowIndex, 0, 0));
        }
    }
    strip.Show();
}
