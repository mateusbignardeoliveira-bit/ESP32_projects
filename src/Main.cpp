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


const unsigned long TEMPO_MINIMO_CURVA =
    250;


// ============================================================
// CONFIGURAÇÃO DO CRUZAMENTO
// ============================================================

// Tempo que o robô permanece andando reto
// sobre o cruzamento antes de devolver
// o controle para o PID.

const unsigned long TEMPO_CRUZAMENTO =
    300;


// Tempo durante o qual uma nova detecção
// de cruzamento é ignorada após atravessá-lo.

const unsigned long BLOQUEIO_CRUZAMENTO =
    500;


// ============================================================
// CONTROLE DE TEMPO
// ============================================================

unsigned long tempoAnteriorPID =
    0;


unsigned long inicioCurva =
    0;


unsigned long inicioCruzamento =
    0;


unsigned long fimBloqueioCruzamento =
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
    // 3. ANALISA CURVA / CRUZAMENTO
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
        // Verifica se o bloqueio do cruzamento terminou
        // ----------------------------------------------------

        bool cruzamentoLiberado =
            millis() >= fimBloqueioCruzamento;


        // ----------------------------------------------------
        // Detecta cruzamento
        //
        // O cruzamento tem prioridade sobre a curva,
        // pois possui 7 ou 8 sensores pretos.
        // ----------------------------------------------------

        if(
            dadosCurva.cruzamento &&
            cruzamentoLiberado
        )
        {
            estado.definirEstado(
                ESTADO_CRUZAMENTO
            );


            inicioCruzamento =
                millis();


            pid.reset();
        }


        // ----------------------------------------------------
        // Detecta curva
        //
        // Mantida igual à implementação que funcionou.
        // ----------------------------------------------------

        else if(dadosCurva.curva90)
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
    // CRUZAMENTO
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_CRUZAMENTO
    )
    {
        unsigned long tempoCruzamento =
            millis() - inicioCruzamento;


        // ----------------------------------------------------
        // Anda reto durante o cruzamento
        // ----------------------------------------------------

        direcao.update(
            0.0f
        );


        MotoresData dadosMotores =
            direcao.getData();


        motores.setSpeed(
            dadosMotores.m1,
            dadosMotores.m2,
            dadosMotores.m3,
            dadosMotores.m4
        );


        // ----------------------------------------------------
        // Terminou de atravessar
        // ----------------------------------------------------

        if(
            tempoCruzamento >=
            TEMPO_CRUZAMENTO
        )
        {
            estado.definirEstado(
                ESTADO_SEGUINDO_LINHA
            );


            fimBloqueioCruzamento =
                millis() +
                BLOQUEIO_CRUZAMENTO;


            pid.reset();
        }


        return;
    }


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