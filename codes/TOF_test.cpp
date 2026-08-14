#include <Arduino.h>
#include <Wire.h>

#include "hardware/TCA.h"
#include "hardware/TOF200F.h"
#include "sensores/TOFAnalise.h"


// ============================================================
// CONFIGURAÇÃO
// ============================================================

// TCA9548A
#define TCA_ENDERECO 0x70

// TOF está no canal 0
#define TOF_CANAL 0


// ============================================================
// OBJETOS
// ============================================================

TCA tca(TCA_ENDERECO);

TOF200F tof(tca);

TOFAnalise tofAnalise(tof);


// ============================================================
// SETUP
// ============================================================

void setup()
{

    Serial.begin(115200);

    delay(1000);


    Serial.println();
    Serial.println("===============================");
    Serial.println("       TESTE TOF200F");
    Serial.println("===============================");


    // --------------------------------------------------------
    // Inicializa TCA
    // --------------------------------------------------------

    tca.begin();


    // --------------------------------------------------------
    // Inicializa TOF
    // --------------------------------------------------------

    Serial.println("Inicializando TOF...");


    if(!tof.begin(TOF_CANAL))
    {

        Serial.println("ERRO: TOF nao encontrado!");

        while(true)
        {
            delay(1000);
        }

    }


    Serial.println("TOF inicializado!");


    // --------------------------------------------------------
    // Mostra configuração
    // --------------------------------------------------------

    Serial.print("Limite de obstaculo: ");

    Serial.print(
        tofAnalise.getLimiteObstaculo()
    );

    Serial.println(" mm");


    Serial.println();
    Serial.println("Iniciando leituras...");
    Serial.println();

}


// ============================================================
// LOOP
// ============================================================

void loop()
{

    // --------------------------------------------------------
    // Atualiza análise
    // --------------------------------------------------------

    tof.update();

    tofAnalise.update();


    // --------------------------------------------------------
    // Mostra distância
    // --------------------------------------------------------

    Serial.print("DISTANCIA: ");

    if(tofAnalise.isValido())
    {

        Serial.print(
            tofAnalise.getDistancia()
        );

        Serial.print(" mm");


    }
    else
    {

        Serial.print("INVALIDA");

    }


    // --------------------------------------------------------
    // Mostra estado do obstáculo
    // --------------------------------------------------------

    Serial.print("    |    OBSTACULO: ");


    if(tofAnalise.temObstaculo())
    {

        Serial.println("SIM");

    }
    else
    {

        Serial.println("NAO");

    }


    delay(100);

}