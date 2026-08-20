#include <Arduino.h>

#include "hardware/ArrayLinha.h"
#include "hardware/MotorControlador.h"
#include "hardware/TCA.h"
#include "hardware/AS7341.h"
#include "hardware/TOF200F.h"

#include "sensores/LinhaAnalise.h"
#include "controle/PIDLinha.h"
#include "controle/ControleDirecao.h"
#include "controle/CurvaAnalise.h"
#include "Estados/EstadoRobo.h"


// ============================================================
// PINOS
// ============================================================

const int PINO_BOTAO = 32;


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
// TCA9548A
// ============================================================

TCA tca;


// ============================================================
// SENSORES
// ============================================================

AS7341Sensores sensoresCor(
    tca
);

TOF200F tof(
    tca
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
// ANÁLISE DE CURVA / CRUZAMENTO
// ============================================================

CurvaAnalise curva;


// ============================================================
// ESTADOS
// ============================================================

EstadoRoboControl estado;


// ============================================================
// CONFIGURAÇÕES
// ============================================================

const int VELOCIDADE_CURVA =
    400;


const unsigned long TEMPO_MINIMO_CURVA =
    250;


// ============================================================
// CANCELAMENTO DE INÉRCIA
// ============================================================

const int VELOCIDADE_FREIO =
    -1000;


const unsigned long TEMPO_FREIO_CURVA =
    20;


// ============================================================
// CRUZAMENTO
// ============================================================

const unsigned long TEMPO_CRUZAMENTO =
    300;


const unsigned long BLOQUEIO_CRUZAMENTO =
    500;


// ============================================================
// OBSTÁCULO
// ============================================================

const int DISTANCIA_OBSTACULO_MM =
    100;


// ============================================================
// TEMPOS
// ============================================================

unsigned long tempoAnteriorPID =
    0;

unsigned long inicioCurva =
    0;

unsigned long inicioFreioCurva =
    0;

unsigned long inicioCruzamento =
    0;

unsigned long fimBloqueioCruzamento =
    0;


// ============================================================
// GIRO ESQUERDA
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
// GIRO DIREITA
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
// CANCELA INÉRCIA
// ============================================================

void cancelarInercia()
{
    motores.setSpeed(
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO
    );
}


// ============================================================
// PARA ROBÔ
// ============================================================

void pararRobo()
{
    motores.setSpeed(
        0,
        0,
        0,
        0
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(500);


    // --------------------------------------------------------
    // BOTÃO
    // HIGH = anda
    // LOW  = para
    // --------------------------------------------------------

    pinMode(
        PINO_BOTAO,
        INPUT
    );


    // --------------------------------------------------------
    // TCA
    // --------------------------------------------------------

    tca.begin(
        -1,
        -1,
        400000
    );


    // --------------------------------------------------------
    // ARRAY
    // --------------------------------------------------------

    arrayLinha.begin();


    // --------------------------------------------------------
    // MOTORES
    // --------------------------------------------------------

    motores.begin();


    // --------------------------------------------------------
    // TOF
    // Canal 0
    // --------------------------------------------------------

    tof.begin(0);


    // --------------------------------------------------------
    // AS7341
    // Canal 1 = direita
    // Canal 2 = esquerda
    // --------------------------------------------------------

    sensoresCor.begin();


    // --------------------------------------------------------
    // Motores inicialmente parados
    // --------------------------------------------------------

    pararRobo();


    tempoAnteriorPID =
        micros();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // BOTÃO DE SEGURANÇA
    // ========================================================

    if(
        digitalRead(PINO_BOTAO) == LOW
    )
    {
        pararRobo();

        return;
    }


    // ========================================================
    // AQUISIÇÃO DOS SENSORES I2C
    //
    // Essas funções NÃO devem esperar conversão.
    // Apenas atualizam quando existe resultado pronto.
    // ========================================================

    tof.update();

    sensoresCor.update();


    // ========================================================
    // OBSTÁCULO
    //
    // 100 mm = 10 cm
    // ========================================================

    int distancia =
        tof.getDistance();


    if(
        distancia > 0 &&
        distancia <= DISTANCIA_OBSTACULO_MM
    )
    {
        pararRobo();

        return;
    }


    // ========================================================
    // ARRAY
    // ========================================================

    arrayLinha.update();


    ArrayData dadosArray =
        arrayLinha.getData();


    // ========================================================
    // LINHA
    // ========================================================

    linha.update(
        dadosArray
    );


    LinhaData dadosLinha =
        linha.getData();


    // ========================================================
    // CURVA / CRUZAMENTO
    // ========================================================

    curva.update(
        dadosLinha
    );


    CurvaData dadosCurva =
        curva.getData();


    // ========================================================
    // ESTADO
    // ========================================================

    EstadoRobo estadoAtual =
        estado.getEstado();


    // ========================================================
    // SEGUINDO LINHA
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_SEGUINDO_LINHA
    )
    {
        bool cruzamentoLiberado =
            millis() >= fimBloqueioCruzamento;


        // ----------------------------------------------------
        // CRUZAMENTO
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
        // CURVA DE 90°
        // ----------------------------------------------------

        else if(
            dadosCurva.curva90
        )
        {
            if(
                dadosCurva.direcao ==
                CURVA_ESQUERDA
            )
            {
                estado.definirEstado(
                    ESTADO_CURVA_ESQUERDA
                );


                // ============================================
                // COMEÇA CANCELAMENTO DE INÉRCIA
                // ============================================

                inicioFreioCurva =
                    millis();


                inicioCurva = 0;


                pid.reset();


                cancelarInercia();


                return;
            }


            if(
                dadosCurva.direcao ==
                CURVA_DIREITA
            )
            {
                estado.definirEstado(
                    ESTADO_CURVA_DIREITA
                );


                // ============================================
                // COMEÇA CANCELAMENTO DE INÉRCIA
                // ============================================

                inicioFreioCurva =
                    millis();


                inicioCurva = 0;


                pid.reset();


                cancelarInercia();


                return;
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
        unsigned long tempo =
            millis() -
            inicioCruzamento;


        // ----------------------------------------------------
        // Reto
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
        // Terminou
        // ----------------------------------------------------

        if(
            tempo >=
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
        // ----------------------------------------------------
        // CANCELAMENTO DE INÉRCIA
        // ----------------------------------------------------

        unsigned long tempoFreio =
            millis() -
            inicioFreioCurva;


        if(
            tempoFreio <
            TEMPO_FREIO_CURVA
        )
        {
            cancelarInercia();

            return;
        }


        // ----------------------------------------------------
        // Só começa a contar a curva DEPOIS do freio
        // ----------------------------------------------------

        if(
            inicioCurva == 0
        )
        {
            inicioCurva =
                millis();
        }


        unsigned long tempoCurva =
            millis() -
            inicioCurva;


        // ----------------------------------------------------
        // Giro mínimo
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
        // Procura linha
        // ----------------------------------------------------

        if(
            dadosLinha.linhaDetectada &&
            dadosCurva.linhaCentral
        )
        {
            estado.definirEstado(
                ESTADO_SEGUINDO_LINHA
            );


            inicioCurva = 0;

            inicioFreioCurva = 0;

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
        // ----------------------------------------------------
        // CANCELAMENTO DE INÉRCIA
        // ----------------------------------------------------

        unsigned long tempoFreio =
            millis() -
            inicioFreioCurva;


        if(
            tempoFreio <
            TEMPO_FREIO_CURVA
        )
        {
            cancelarInercia();

            return;
        }


        // ----------------------------------------------------
        // Começa curva depois do freio
        // ----------------------------------------------------

        if(
            inicioCurva == 0
        )
        {
            inicioCurva =
                millis();
        }


        unsigned long tempoCurva =
            millis() -
            inicioCurva;


        // ----------------------------------------------------
        // Giro mínimo
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
        // Procura linha
        // ----------------------------------------------------

        if(
            dadosLinha.linhaDetectada &&
            dadosCurva.linhaCentral
        )
        {
            estado.definirEstado(
                ESTADO_SEGUINDO_LINHA
            );


            inicioCurva = 0;

            inicioFreioCurva = 0;

            pid.reset();
        }
        else
        {
            girarDireita();

            return;
        }
    }


    // ========================================================
    // ATUALIZA ESTADO
    // ========================================================

    estadoAtual =
        estado.getEstado();


    // ========================================================
    // PID
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_SEGUINDO_LINHA
    )
    {
        unsigned long agora =
            micros();


        float deltaTime =
            (
                agora -
                tempoAnteriorPID
            )
            / 1000000.0f;


        tempoAnteriorPID =
            agora;


        // ----------------------------------------------------
        // Linha
        // ----------------------------------------------------

        if(
            dadosLinha.linhaDetectada
        )
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
        // Gap
        // ----------------------------------------------------

        else
        {
            direcao.update(
                0.0f
            );
        }


        // ----------------------------------------------------
        // Motores
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


    // ========================================================
    // PEQUENO INTERVALO
    // ========================================================

    delay(5);
}