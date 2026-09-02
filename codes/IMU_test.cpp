#include <Arduino.h>

#include "hardware/ICM20948.h"
#include "sensores/IMU.h"

ICM20948 icm;
IMU imu(icm);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" TESTE ICM20948 + IMU");
    Serial.println("=================================");

    // ICM20948:
    // SDA = GPIO 19
    // SCL = GPIO 23
    if (!icm.begin(19, 23, 400000))
    {
        Serial.println("ERRO: ICM20948 nao inicializou!");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("ICM20948 inicializado.");

    if (!imu.begin())
    {
        Serial.println("ERRO: IMU nao inicializou!");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("IMU inicializada.");
    Serial.println();

    Serial.println("Deixe o robo completamente parado.");
    Serial.println("Iniciando calibracao do giroscopio...");
    delay(2000);

    if (imu.calibrar())
    {
        Serial.println("Calibracao concluida!");
    }
    else
    {
        Serial.println("ERRO na calibracao!");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println("Teste iniciado.");
    Serial.println("Movimente o robo manualmente.");
    Serial.println();
}

void loop()
{
    if (!imu.update())
    {
        Serial.println("Falha na leitura da IMU!");
        delay(100);
        return;
    }

    const IMUData& dados = imu.getData();

    Serial.print("Roll: ");
    Serial.print(dados.roll, 2);

    Serial.print(" | Pitch: ");
    Serial.print(dados.pitch, 2);

    Serial.print(" | Yaw: ");
    Serial.print(dados.yaw, 2);

    Serial.print(" | Heading: ");
    Serial.print(dados.heading, 2);

    Serial.print(" | Gyro Z: ");
    Serial.print(dados.gyroZ, 2);

    Serial.print(" | Mag: ");
    Serial.print(dados.magX, 1);
    Serial.print(",");
    Serial.print(dados.magY, 1);
    Serial.print(",");
    Serial.print(dados.magZ, 1);

    Serial.println();

    delay(50);
}
