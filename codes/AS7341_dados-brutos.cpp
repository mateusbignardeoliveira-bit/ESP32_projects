#include <Arduino.h>

#include "hardware/TCA.h"
#include "hardware/AS7341.h"


// ============================================================
// OBJETOS
// ============================================================

TCA tca;

AS7341Sensores sensoresAS7341(tca);


// ============================================================
// IMPRIMIR DADOS DE UM SENSOR
// ============================================================

void imprimirSensor(
    const char* nome,
    const AS7341Data& dados
)
{
    Serial.println();
    Serial.println("----------------------------------------");
    Serial.println(nome);
    Serial.println("----------------------------------------");

    if(!dados.valido)
    {
        Serial.println("INVALIDO");
        return;
    }

    Serial.print("F1:    ");
    Serial.println(dados.F1);

    Serial.print("F2:    ");
    Serial.println(dados.F2);

    Serial.print("F3:    ");
    Serial.println(dados.F3);

    Serial.print("F4:    ");
    Serial.println(dados.F4);

    Serial.print("F5:    ");
    Serial.println(dados.F5);

    Serial.print("F6:    ");
    Serial.println(dados.F6);

    Serial.print("F7:    ");
    Serial.println(dados.F7);

    Serial.print("F8:    ");
    Serial.println(dados.F8);

    Serial.print("Clear: ");
    Serial.println(dados.clear);

    Serial.print("NIR:   ");
    Serial.println(dados.nir);

    Serial.print("Tempo: ");
    Serial.println(dados.tempo);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("       CALIBRACAO AS7341");
    Serial.println("========================================");


    // --------------------------------------------------------
    // TCA
    // --------------------------------------------------------

    tca.begin();


    // --------------------------------------------------------
    // AS7341
    // --------------------------------------------------------

    if(!sensoresAS7341.begin())
    {
        Serial.println();
        Serial.println("ERRO: AS7341 nao inicializado.");

        while(true)
        {
            delay(1000);
        }
    }


    Serial.println();
    Serial.println("AS7341 inicializados.");
    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // NOVA LEITURA
    // --------------------------------------------------------

    if(!sensoresAS7341.update())
    {
        Serial.println("ERRO durante leitura dos AS7341.");

        delay(100);

        return;
    }


    // --------------------------------------------------------
    // OBTÉM DADOS
    // --------------------------------------------------------

    AS7341Data esquerda =
        sensoresAS7341.getEsquerda();

    AS7341Data direita =
        sensoresAS7341.getDireita();


    // --------------------------------------------------------
    // IMPRIME
    // --------------------------------------------------------

    imprimirSensor(
        "SENSOR ESQUERDA",
        esquerda
    );

    imprimirSensor(
        "SENSOR DIREITA",
        direita
    );


    // --------------------------------------------------------
    // SEPARADOR
    // --------------------------------------------------------

    Serial.println();
    Serial.println("========================================");


    // --------------------------------------------------------
    // PEQUENO INTERVALO
    // --------------------------------------------------------

    delay(200);
}