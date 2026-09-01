#include <Arduino.h>

HardwareSerial motorSerial(1);

void setup()
{
    Serial.begin(115200);

    motorSerial.begin(
        115200,
        SERIAL_8N1,
        17,
        16
    );

    delay(1000);

    Serial.println("Lendo bateria...");
}

void loop()
{
    // Solicita tensão da bateria
    motorSerial.print("$read_vol#");

    unsigned long inicio = millis();

    String resposta = "";

    while (millis() - inicio < 200)
    {
        while (motorSerial.available())
        {
            char c = motorSerial.read();

            resposta += c;

            if (c == '#')
                break;
        }

        if (resposta.endsWith("#"))
            break;
    }

    Serial.print("Resposta: ");
    Serial.println(resposta);

    delay(1000);
}