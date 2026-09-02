#include <Arduino.h>
#include "hardware/ICM20948.h"

ICM20948 imu;

void setup()
{
    Serial.begin(115200);

    if (!imu.begin())
    {
        Serial.println("Erro ao iniciar ICM-20948!");

        while (true)
            delay(1000);
    }

    Serial.println("ICM-20948 iniciado!");
}

void loop()
{
    if (imu.update())
    {
        const ICM20948Data &dados =
            imu.getData();

        Serial.print("ACC: ");
        Serial.print(dados.accelX, 3);
        Serial.print(" | ");
        Serial.print(dados.accelY, 3);
        Serial.print(" | ");
        Serial.println(dados.accelZ, 3);

        Serial.print("GYRO: ");
        Serial.print(dados.gyroX, 3);
        Serial.print(" | ");
        Serial.print(dados.gyroY, 3);
        Serial.print(" | ");
        Serial.println(dados.gyroZ, 3);

        Serial.print("MAG: ");
        Serial.print(dados.magX, 2);
        Serial.print(" | ");
        Serial.print(dados.magY, 2);
        Serial.print(" | ");
        Serial.println(dados.magZ, 2);

        Serial.print("TEMP: ");
        Serial.print(dados.temperature, 2);
        Serial.println(" C");

        Serial.println();
    }

    delay(50);
}