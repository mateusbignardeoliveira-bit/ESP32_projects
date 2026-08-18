#include <Arduino.h>

#include "Hardware/TCA.h"
#include "Hardware/AS7341.h"
#include "sensores/AS7341Analise.h"

// ============================================================
// OBJETOS
// ============================================================

TCA tca;

AS7341Sensores sensoresAS7341(tca);

AS7341Analise analise;

// ============================================================
// IMPRIMIR RESULTADO
// ============================================================

void imprimirResultado(
    const char* nome,
    const AS7341Resultado& resultado
)
{
    Serial.println();
    Serial.println("----------------------------------------");
    Serial.print(nome);
    Serial.println();
    Serial.println("----------------------------------------");

    if(!resultado.valido)
    {
        Serial.println("Sensor INVALIDO");
        return;
    }

    Serial.print("Intensidade: ");
    Serial.println(resultado.intensidade);

    Serial.print("Azul:        ");
    Serial.println(resultado.azul);

    Serial.print("Verde:       ");
    Serial.println(resultado.verde);

    Serial.print("Vermelho:    ");
    Serial.println(resultado.vermelho);

    Serial.print("Vermelho/Verde: ");
    Serial.println(
        resultado.razaoVermelhoVerde,
        3
    );

    Serial.print("Azul/Verde:     ");
    Serial.println(
        resultado.razaoAzulVerde,
        3
    );

    Serial.print("NIR/Clear:      ");
    Serial.println(
        resultado.razaoNIR,
        3
    );

    Serial.print("Canal dominante: ");

    switch(resultado.canalDominante)
    {
        case 1:
            Serial.println("AZUL");
            break;

        case 2:
            Serial.println("VERDE");
            break;

        case 3:
            Serial.println("VERMELHO");
            break;

        default:
            Serial.println("INVALIDO");
            break;
    }

    Serial.print("VERDE DETECTADO: ");

    if(resultado.verdeDetectado)
    {
        Serial.println("SIM");
    }
    else
    {
        Serial.println("NAO");
    }
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
    Serial.println(" TESTE / CALIBRACAO AS7341");
    Serial.println("========================================");

    // --------------------------------------------------------
    // I2C / TCA
    // --------------------------------------------------------

    Serial.println("Inicializando TCA...");

    tca.begin();

    Serial.println("TCA inicializado.");

    // --------------------------------------------------------
    // AS7341
    // --------------------------------------------------------

    Serial.println("Inicializando AS7341...");

    if(!sensoresAS7341.begin())
    {
        Serial.println();
        Serial.println("ERRO ao inicializar AS7341!");
        Serial.println("Verifique TCA, alimentacao e I2C.");

        while(true)
        {
            delay(1000);
        }
    }

    Serial.println("AS7341 inicializados.");

    Serial.println();
    Serial.println("Coloque os sensores sobre as superficies");
    Serial.println("que queremos testar.");
    Serial.println();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Nova leitura
    // --------------------------------------------------------

    if(!sensoresAS7341.update())
    {
        Serial.println("ERRO na leitura dos AS7341.");

        delay(500);

        return;
    }

    // --------------------------------------------------------
    // Obtém dados brutos
    // --------------------------------------------------------

    AS7341Data dadosDireita =
        sensoresAS7341.getDireita();

    AS7341Data dadosEsquerda =
        sensoresAS7341.getEsquerda();

    // --------------------------------------------------------
    // Analisa
    // --------------------------------------------------------

    AS7341Resultado resultadoDireita =
        analise.analisar(
            dadosDireita
        );

    AS7341Resultado resultadoEsquerda =
        analise.analisar(
            dadosEsquerda
        );

    // --------------------------------------------------------
    // Imprime
    // --------------------------------------------------------

    Serial.println();
    Serial.println();
    Serial.println("========================================");
    Serial.println(" NOVA LEITURA");
    Serial.println("========================================");

    imprimirResultado(
        "SENSOR DIREITA",
        resultadoDireita
    );

    imprimirResultado(
        "SENSOR ESQUERDA",
        resultadoEsquerda
    );

    Serial.println();
    Serial.println("========================================");

    delay(300);
}