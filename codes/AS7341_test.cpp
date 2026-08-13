#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7341.h>

#define TCA9548A_ADDR 0x70
#define TCA_CHANNEL   1

Adafruit_AS7341 as7341;

// ============================================================
// Configuração do AS7341
// ============================================================

// Aproximadamente 50 ms de integração:
// (29 + 1) * (599 + 1) * 2,78 us
#define AS7341_ATIME  29
#define AS7341_ASTEP  599

// Ganho de 32x na biblioteca Adafruit
#define AS7341_GAIN   AS7341_GAIN_32X


// ============================================================
// Seleciona um canal do TCA9548A
// ============================================================

void tcaSelect(uint8_t channel)
{
    if (channel > 7) return;

    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    // Inicializa I2C
    Wire.begin();

    // Seleciona o canal onde está o AS7341
    tcaSelect(TCA_CHANNEL);

    Serial.println("Inicializando AS7341...");

    // Inicializa o AS7341 através da biblioteca Adafruit
    if (!as7341.begin()) {
        Serial.println("ERRO: AS7341 nao encontrado!");
        while (1) {
            delay(10);
        }
    }

    // Configuração de integração
    as7341.setATIME(AS7341_ATIME);
    as7341.setASTEP(AS7341_ASTEP);

    // Ganho
    as7341.setGain(AS7341_GAIN);

    // LED interno do breakout desligado inicialmente
    as7341.enableLED(false);

    Serial.println("AS7341 inicializado!");
    Serial.println();

    Serial.print("ATIME: ");
    Serial.println(as7341.getATIME());

    Serial.print("ASTEP: ");
    Serial.println(as7341.getASTEP());

    Serial.println();
}


// ============================================================
// Loop
// ============================================================

void loop()
{
    // Garante que o canal correto do TCA9548A está selecionado
    tcaSelect(TCA_CHANNEL);

    uint16_t readings[12];

    // Faz uma leitura de todos os canais
    if (!as7341.readAllChannels(readings)) {
        Serial.println("ERRO na leitura do AS7341!");
        delay(100);
        return;
    }

    // --------------------------------------------------------
    // F1 - F4
    // --------------------------------------------------------

    Serial.print("F1: ");
    Serial.println(readings[0]);

    Serial.print("F2: ");
    Serial.println(readings[1]);

    Serial.print("F3: ");
    Serial.println(readings[2]);

    Serial.print("F4: ");
    Serial.println(readings[3]);


    // --------------------------------------------------------
    // F5 - F8
    // --------------------------------------------------------

    Serial.print("F5: ");
    Serial.println(readings[6]);

    Serial.print("F6: ");
    Serial.println(readings[7]);

    Serial.print("F7: ");
    Serial.println(readings[8]);

    Serial.print("F8: ");
    Serial.println(readings[9]);


    // --------------------------------------------------------
    // Clear / NIR
    // --------------------------------------------------------

    Serial.print("Clear: ");
    Serial.println(readings[10]);

    Serial.print("NIR: ");
    Serial.println(readings[11]);

    Serial.println("-------------------------");

    delay(500);
}