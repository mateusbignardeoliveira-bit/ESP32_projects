#include <Arduino.h>
#include <Wire.h>

#include "hardware/TCA.h"
#include "hardware/AS7341.h"


// ============================================================
// PINOS I2C
// ============================================================

#define I2C_SDA 21
#define I2C_SCL 22


// ============================================================
// TCA9548A
// ============================================================

#define TCA_ENDERECO 0x70

TCA tca(TCA_ENDERECO);


// ============================================================
// AS7341
// ============================================================

AS7341Sensores espectro(tca);


// ============================================================
// SETUP
// ============================================================

void setup()
{

    Serial.begin(115200);

    delay(1000);


    Serial.println();
    Serial.println("================================");
    Serial.println(" TESTE DOS SENSORES AS7341");
    Serial.println("================================");
    Serial.println();


    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.begin(
        I2C_SDA,
        I2C_SCL
    );


    // --------------------------------------------------------
    // TCA
    // --------------------------------------------------------

    tca.begin();


    // --------------------------------------------------------
    // AS7341
    // --------------------------------------------------------

    Serial.println("Inicializando AS7341...");


    if(!espectro.begin())
    {

        Serial.println(
            "ERRO: Falha ao inicializar os AS7341!"
        );

        while(true)
        {
            delay(1000);
        }

    }


    Serial.println(
        "AS7341 inicializados com sucesso!"
    );

    Serial.println();

}


// ============================================================
// LOOP
// ============================================================

void loop()
{

    // --------------------------------------------------------
    // Atualiza os dois sensores
    // --------------------------------------------------------

    if(!espectro.update())
    {

        Serial.println(
            "ERRO: Falha durante leitura dos AS7341!"
        );

        delay(500);

        return;

    }


    // --------------------------------------------------------
    // Obtém dados da direita
    // --------------------------------------------------------

    AS7341Data direita =
        espectro.getDireita();


    // --------------------------------------------------------
    // Obtém dados da esquerda
    // --------------------------------------------------------

    AS7341Data esquerda =
        espectro.getEsquerda();


    // ========================================================
    // SENSOR DIREITA
    // ========================================================

    Serial.println("----------- DIREITA -----------");

    Serial.print("F1: ");
    Serial.println(direita.F1);

    Serial.print("F2: ");
    Serial.println(direita.F2);

    Serial.print("F3: ");
    Serial.println(direita.F3);

    Serial.print("F4: ");
    Serial.println(direita.F4);

    Serial.print("F5: ");
    Serial.println(direita.F5);

    Serial.print("F6: ");
    Serial.println(direita.F6);

    Serial.print("F7: ");
    Serial.println(direita.F7);

    Serial.print("F8: ");
    Serial.println(direita.F8);

    Serial.print("Clear: ");
    Serial.println(direita.clear);

    Serial.print("NIR: ");
    Serial.println(direita.nir);


    // ========================================================
    // SENSOR ESQUERDA
    // ========================================================

    Serial.println();

    Serial.println("----------- ESQUERDA -----------");

    Serial.print("F1: ");
    Serial.println(esquerda.F1);

    Serial.print("F2: ");
    Serial.println(esquerda.F2);

    Serial.print("F3: ");
    Serial.println(esquerda.F3);

    Serial.print("F4: ");
    Serial.println(esquerda.F4);

    Serial.print("F5: ");
    Serial.println(esquerda.F5);

    Serial.print("F6: ");
    Serial.println(esquerda.F6);

    Serial.print("F7: ");
    Serial.println(esquerda.F7);

    Serial.print("F8: ");
    Serial.println(esquerda.F8);

    Serial.print("Clear: ");
    Serial.println(esquerda.clear);

    Serial.print("NIR: ");
    Serial.println(esquerda.nir);


    // ========================================================
    // VALIDADE
    // ========================================================

    Serial.println();

    Serial.print("Direita valida: ");
    Serial.println(
        direita.valido ? "SIM" : "NAO"
    );

    Serial.print("Esquerda valida: ");
    Serial.println(
        esquerda.valido ? "SIM" : "NAO"
    );


    Serial.println();
    Serial.println("===============================");
    Serial.println();


    delay(500);

}