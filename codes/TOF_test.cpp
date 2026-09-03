#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "Hardware/TCA.h"

#define TCA_ENDERECO 0x70
#define TOF_CANAL 0

#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_VL53L0X tof;
TCA tca(TCA_ENDERECO);

unsigned long ultimoPrint = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" TESTE VL53L0X - DISTANCIA + STATUS");
    Serial.println("========================================");
    Serial.println();

    tca.begin(
        I2C_SDA,
        I2C_SCL,
        400000
    );

    if(!tca.selecionarCanal(TOF_CANAL))
    {
        Serial.println("ERRO AO SELECIONAR TCA");
        return;
    }

    Serial.println("TCA canal 0 selecionado.");

    if(!tof.begin(0x29))
    {
        Serial.println("ERRO: VL53L0X NAO ENCONTRADO");
        return;
    }

    Serial.println("VL53L0X OK.");

    tof.setMeasurementTimingBudgetMicroSeconds(33000);

    tof.startRangeContinuous(40);

    Serial.println("Medicao continua iniciada.");
    Serial.println();
}


void loop()
{
    if(!tof.isRangeComplete())
    {
        return;
    }

    uint16_t distancia = tof.readRange();

    uint8_t status = tof.readRangeStatus();

    if(millis() - ultimoPrint >= 100)
    {
        ultimoPrint = millis();

        Serial.print("DIST: ");
        Serial.print(distancia);

        Serial.print(" mm | STATUS: ");
        Serial.print(status);

        Serial.print(" | ");

        if(status == 0)
        {
            Serial.println("VALIDA");
        }
        else if(status == 1)
        {
            Serial.println("STATUS 1");
        }
        else if(status == 2)
        {
            Serial.println("STATUS 2");
        }
        else if(status == 3)
        {
            Serial.println("STATUS 3");
        }
        else if(status == 4)
        {
            Serial.println("OUT OF RANGE");
        }
        else
        {
            Serial.println("OUTRO STATUS");
        }
    }
}