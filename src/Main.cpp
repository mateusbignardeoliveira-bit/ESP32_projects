#include <Arduino.h>

#include "hardware/ArrayLinha.h"
#include "hardware/MotorControlador.h"

#include "sensores/LinhaAnalise.h"
#include "controle/PIDLinha.h"
#include "controle/ControleDirecao.h"
#include "controle/CurvaAnalise.h"
#include "Estados/EstadoRobo.h"


// ============================================================
// ARRAY DE LINHA
// RX = 5
// TX = 18
// ============================================================

ArrayLinha arrayLinha(
    Serial2,
    5,
    18
);


// ============================================================
// CONTROLADOR DOS MOTORES
// RX = 16
// TX = 17
// ============================================================

MotorControlador motores(
    Serial1,
    16,
    17
);


// ============================================================
// ANÁLISE DA LINHA
// ============================================================

LinhaAnalise linha;


// ============================================================
// PID
// ============================================================

PIDLinha pid(
    80.0f,
    0.0f,
    30.0f
);


// ============================================================
// CONTROLE DE DIREÇÃO
// ============================================================

ControleDirecao direcao(
    200,
    400
);


// ============================================================
// ANÁLISE DE CURVA
// ============================================================

CurvaAnalise curva;


// ============================================================
// ESTADOS
// ============================================================

EstadoRoboControl estado;


// ============================================================
// CONFIGURAÇÃO DA CURVA
// ============================================================

const int VELOCIDADE_CURVA =
    400;


// Tempo mínimo de giro antes de
// procurar a linha novamente.

const unsigned long TEMPO_MINIMO_CURVA =
    250;


// ============================================================
// CONTROLE DE TEMPO
// ============================================================

unsigned long tempoAnteriorPID =
    0;


unsigned long inicioCurva =
    0;


// ============================================================
// GIRO PARA ESQUERDA
// ============================================================

void girarEsquerda()
{
    motores.setSpeed(
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        VELOCIDADE_CURVA,
        VELOCIDADE_CURVA
    );
}


// ============================================================
// GIRO PARA DIREITA
// ============================================================

void girarDireita()
{
    motores.setSpeed(
        VELOCIDADE_CURVA,
        VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(500);


    // Inicializa array

    arrayLinha.begin();


    // Inicializa motores

    motores.begin();


    // Garante motores parados

    motores.setSpeed(
        0,
        0,
        0,
        0
    );


    tempoAnteriorPID =
        micros();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // 1. LÊ ARRAY
    // ========================================================

    arrayLinha.update();


    ArrayData dadosArray =
        arrayLinha.getData();


    // ========================================================
    // 2. ANALISA LINHA
    // ========================================================

    linha.update(
        dadosArray
    );


    LinhaData dadosLinha =
        linha.getData();


    // ========================================================
    // 3. ANALISA CURVA
    // ========================================================

    curva.update(
        dadosLinha
    );


    CurvaData dadosCurva =
        curva.getData();


    // ========================================================
    // 4. PEGA ESTADO ATUAL
    // ========================================================

    EstadoRobo estadoAtual =
        estado.getEstado();


    // ========================================================
    // 5. ESTADO SEGUINDO LINHA
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_SEGUINDO_LINHA
    )
    {
        // ----------------------------------------------------
        // Detecta curva
        // ----------------------------------------------------

        if(dadosCurva.curva90)
        {
            if(
                dadosCurva.direcao ==
                CURVA_ESQUERDA
            )
            {
                estado.definirEstado(
                    ESTADO_CURVA_ESQUERDA
                );


                inicioCurva =
                    millis();


                pid.reset();
            }


            else if(
                dadosCurva.direcao ==
                CURVA_DIREITA
            )
            {
                estado.definirEstado(
                    ESTADO_CURVA_DIREITA
                );


                inicioCurva =
                    millis();


                pid.reset();
            }
        }
    }


    // ========================================================
    // ATUALIZA ESTADO
    // ========================================================

    estadoAtual =
        estado.getEstado();


    // ========================================================
    // CURVA ESQUERDA
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_CURVA_ESQUERDA
    )
    {
        unsigned long tempoCurva =
            millis() - inicioCurva;


        // ----------------------------------------------------
        // Tempo mínimo de giro
        // ----------------------------------------------------

        if(
            tempoCurva <
            TEMPO_MINIMO_CURVA
        )
        {
            girarEsquerda();

            return;
        }


        // ----------------------------------------------------
        // Procura linha novamente
        // ----------------------------------------------------

        if(
            dadosLinha.linhaDetectada &&
            dadosCurva.linhaCentral
        )
        {
            estado.definirEstado(
                ESTADO_SEGUINDO_LINHA
            );


            pid.reset();
        }
        else
        {
            girarEsquerda();

            return;
        }
    }


    // ========================================================
    // CURVA DIREITA
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_CURVA_DIREITA
    )
    {
        unsigned long tempoCurva =
            millis() - inicioCurva;


        // ----------------------------------------------------
        // Tempo mínimo de giro
        // ----------------------------------------------------

        if(
            tempoCurva <
            TEMPO_MINIMO_CURVA
        )
        {
            girarDireita();

            return;
        }


        // ----------------------------------------------------
        // Procura linha novamente
        // ----------------------------------------------------

        if(
            dadosLinha.linhaDetectada &&
            dadosCurva.linhaCentral
        )
        {
            estado.definirEstado(
                ESTADO_SEGUINDO_LINHA
            );


            pid.reset();
        }
        else
        {
            girarDireita();

            return;
        }
    }


    // ========================================================
    // ATUALIZA ESTADO NOVAMENTE
    // ========================================================

    estadoAtual =
        estado.getEstado();


    // ========================================================
    // SEGUINDO LINHA COM PID
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_SEGUINDO_LINHA
    )
    {
        unsigned long agora =
            micros();


        float deltaTime =
            (agora - tempoAnteriorPID)
            / 1000000.0f;


        tempoAnteriorPID =
            agora;


        // ----------------------------------------------------
        // LINHA DETECTADA
        // ----------------------------------------------------

        if(dadosLinha.linhaDetectada)
        {
            pid.update(
                dadosLinha.erro,
                deltaTime
            );


            PIDData dadosPID =
                pid.getData();


            direcao.update(
                dadosPID.correcao
            );
        }


        // ----------------------------------------------------
        // GAP
        // ----------------------------------------------------

        else
        {
            // Continua reto durante o gap.

            direcao.update(
                0.0f
            );
        }


        // ----------------------------------------------------
        // ENVIA VELOCIDADES
        // ----------------------------------------------------

        MotoresData dadosMotores =
            direcao.getData();


        motores.setSpeed(
            dadosMotores.m1,
            dadosMotores.m2,
            dadosMotores.m3,
            dadosMotores.m4
        );
    }


    delay(5);
}