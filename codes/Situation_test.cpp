#include <Arduino.h>

#include "hardware/ArrayLinha.h"
#include "sensores/LinhaAnalise.h"
#include "controle/ControleSegueLinha.h"


// ============================================================
// ARRAY DE LINHA
// ============================================================

ArrayLinha arrayLinha(
    Serial2,
    5,
    18
);


// ============================================================
// ANÁLISE DA LINHA
// ============================================================

LinhaAnalise linhaAnalise;


// ============================================================
// CONTROLADOR
// ============================================================

ControleSegueLinha controle;


// ============================================================
// DADOS
// ============================================================

ArrayData dados;

LinhaData linha;

ControleData controleData;


// ============================================================
// CONTROLE DE IMPRESSÃO
// ============================================================

unsigned long ultimoPrint = 0;

const unsigned long INTERVALO_PRINT = 100;


// ============================================================
// SETUP
// ============================================================

void setup()
{

    Serial.begin(115200);

    delay(1000);


    Serial.println();
    Serial.println("========================================");
    Serial.println("       TESTE CONTROLADOR");
    Serial.println("========================================");

    Serial.println();


    // --------------------------------------------------------
    // Inicializa array
    // --------------------------------------------------------

    arrayLinha.begin();


    // --------------------------------------------------------
    // PID inicial
    //
    // Valores provisórios.
    // Ainda vamos ajustar.
    // --------------------------------------------------------

    controle.configurarPID(
        100.0f,
        0.0f,
        15.0f
    );


    // --------------------------------------------------------
    // Velocidade
    //
    // Ainda não será enviada aos motores.
    // --------------------------------------------------------

    controle.configurarVelocidade(
        1000,
        3000
    );

    controle.configurarVelocidadeAdaptativa(
        1.0f,
        6.0f,
        500
    );


    controle.reset();

}


// ============================================================
// LOOP
// ============================================================

void loop()
{

    // ========================================================
    // ATUALIZA ARRAY
    // ========================================================

    arrayLinha.update();


    // ========================================================
    // PEGA DADOS
    // ========================================================

    dados =
        arrayLinha.getData();


    // ========================================================
    // ANALISA LINHA
    // ========================================================

    linhaAnalise.update(
        dados
    );


    linha =
        linhaAnalise.getData();


    // ========================================================
    // CONTROLADOR
    // ========================================================

    controle.update(
        linha
    );


    controleData =
        controle.getData();


    // ========================================================
    // MONITOR SERIAL
    // ========================================================

    unsigned long agora =
        millis();


    if(
        agora - ultimoPrint >=
        INTERVALO_PRINT
    )
    {

        ultimoPrint =
            agora;


        // ----------------------------------------------------
        // LINHA
        // ----------------------------------------------------

        Serial.print("ERRO: ");

        Serial.print(
            linha.erro,
            2
        );


        // ----------------------------------------------------
        // PID
        // ----------------------------------------------------

        Serial.print("    P: ");

        Serial.print(
            controleData.proporcional,
            2
        );


        Serial.print("    I: ");

        Serial.print(
            controleData.integral,
            2
        );


        Serial.print("    D: ");

        Serial.print(
            controleData.derivativo,
            2
        );


        // ----------------------------------------------------
        // CORREÇÃO
        // ----------------------------------------------------

        Serial.print("    COR: ");

        Serial.print(
            controleData.correcao,
            2
        );


        // ----------------------------------------------------
        // MOTORES
        // ----------------------------------------------------

        Serial.print("    ESQ: ");

        Serial.print(
            controleData.velocidadeEsquerda
        );


        Serial.print("    DIR: ");

        Serial.print(
            controleData.velocidadeDireita
        );


        // ----------------------------------------------------
        // SITUAÇÃO
        // ----------------------------------------------------

        Serial.print("    PRETOS: ");

        Serial.print(
            linha.sensoresPretos
        );

        // ----------------------------------------------------

        Serial.print("    BASE: ");

        Serial.print(
         controleData.velocidadeBaseAtual
         );

        Serial.println();  

    }

}