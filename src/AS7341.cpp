#include <Arduino.h>
#include <Wire.h>
#include <Debug.h>
#include "DEV_Config.h"
#include "Waveshare_AS7341.h"

#define TCA9548A_ADDR 0x70
#define TCA_CHANNEL 2

// Seleciona um canal do TCA9548A
void tcaSelect(uint8_t channel)
{
    if (channel > 7) return;

    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void setup()
{
    Serial.begin(115200);

    // Inicializa a biblioteca
    DEV_ModuleInit();

    // Seleciona o canal onde está o AS7341
    tcaSelect(TCA_CHANNEL);

    Serial.println("Inicializando AS7341...");

    AS7341_Init(eSpm);
    AS7341_ATIME_config(100);
    AS7341_ASTEP_config(999);
    AS7341_AGAIN_config(6);
    AS7341_EnableLED(true);

    Serial.println("AS7341 inicializado!");
}

void loop()
{
    // Garante que o canal correto está selecionado
    tcaSelect(TCA_CHANNEL);

    AS7341_ControlLed(false, 10);

    sModeOneData_t data1;
    sModeTwoData_t data2;

    AS7341_startMeasure(eF1F4ClearNIR);
    data1 = AS7341_ReadSpectralDataOne();

    Serial.print("F1: ");
    Serial.println(data1.channel1);
    Serial.print("F2: ");
    Serial.println(data1.channel2);
    Serial.print("F3: ");
    Serial.println(data1.channel3);
    Serial.print("F4: ");
    Serial.println(data1.channel4);

    AS7341_startMeasure(eF5F8ClearNIR);
    data2 = AS7341_ReadSpectralDataTwo();

    Serial.print("F5: ");
    Serial.println(data2.channel5);
    Serial.print("F6: ");
    Serial.println(data2.channel6);
    Serial.print("F7: ");
    Serial.println(data2.channel7);
    Serial.print("F8: ");
    Serial.println(data2.channel8);
    Serial.print("Clear: ");
    Serial.println(data2.CLEAR);
    Serial.print("NIR: ");
    Serial.println(data2.NIR);

    Serial.println("-------------------------");

    delay(500);
}