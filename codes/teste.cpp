#include <Arduino.h>

// UART do ArrayLinha
static constexpr int ARRAY_RX = 5;
static constexpr int ARRAY_TX = 18;
static constexpr int BAUD = 115200;

HardwareSerial arraySerial(2);

char buffer[120];
int indice = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" LEITURA BRUTA - ARRAY DE LINHA");
    Serial.println("=================================");
    Serial.println();

    arraySerial.begin(
        BAUD,
        SERIAL_8N1,
        ARRAY_RX,
        ARRAY_TX
    );

    // Solicita os valores analógicos ao módulo
    arraySerial.print("$0,1,0#");

    Serial.println("Array iniciado.");
    Serial.println("Coloque o robo sobre a superficie");
    Serial.println("que deseja medir.");
    Serial.println();
}

void loop()
{
    while(arraySerial.available())
    {
        char c = arraySerial.read();

        if(c == '#')
        {
            buffer[indice] = '\0';

            Serial.print("RAW: ");
            Serial.println(buffer);

            indice = 0;

            // Solicita uma nova leitura
            arraySerial.print("$0,1,0#");
        }
        else
        {
            if(indice < (int)sizeof(buffer) - 1)
            {
                buffer[indice++] = c;
            }
            else
            {
                // Evita overflow caso chegue um pacote inesperado
                indice = 0;
            }
        }
    }
}