#include <Arduino.h>

#include "hardware/TCA.h"
#include "hardware/AS7341.h"
#include "hardware/ArrayLinha.h"
#include "hardware/MotorControlador.h"

#include "sensores/AS7341Analise.h"


// ============================================================
// PINO DO BOTÃO
// ============================================================

const int PINO_BOTAO = 32;


// ============================================================
// VELOCIDADES
// ============================================================

const int VELOCIDADE_ROBO = -400;

const int VELOCIDADE_VERDE = -60;

const int VELOCIDADE_CURVA = 600;


// ============================================================
// FRENAGEM
// ============================================================

const int VELOCIDADE_FREIO = 1000;

const int TEMPO_FREIO_MS = 100;


// ============================================================
// TEMPO DAS MANOBRAS
// ============================================================
//
// Valores INICIAIS apenas para teste.
//
// Depois regulamos com o robô fisicamente.
// ============================================================

const int TEMPO_CURVA_90_MS = 450;

const int TEMPO_MEIA_VOLTA_MS = 700;


// ============================================================
// SAÍDA DO VERDE
// ============================================================

const int LEITURAS_SEM_VERDE_NECESSARIAS = 3;


// ============================================================
// OBJETOS
// ============================================================

TCA tca;

AS7341Sensores sensoresAS7341(tca);


// ============================================================
// ARRAY
// ============================================================

ArrayLinha arrayLinha(
    Serial2,
    18,
    5,
    115200
);


// ============================================================
// MOTOR
// ============================================================

MotorControlador motorControlador(
    Serial1,
    16,
    17,
    115200
);


// ============================================================
// ANÁLISE DOS AS7341
// ============================================================

AS7341Analise as7341Analise;


// ============================================================
// ESTADOS
// ============================================================

enum EstadoVerde
{
    NORMAL,

    FREANDO,

    AVANCANDO_VERDE,

    CURVA_ESQUERDA,

    CURVA_DIREITA,

    MEIA_VOLTA,

    FINALIZADO
};


EstadoVerde estado =
    NORMAL;


// ============================================================
// MEMÓRIA DO VERDE
// ============================================================

bool verdeEsquerda =
    false;

bool verdeDireita =
    false;


// ============================================================
// CONTROLE DE TEMPO
// ============================================================

unsigned long tempoInicio =
    0;


// ============================================================
// CONTROLE DE SAÍDA DO VERDE
// ============================================================

int leiturasSemVerde =
    0;


// ============================================================
// PARAR
// ============================================================

void pararRobo()
{
    motorControlador.stop();
}


// ============================================================
// ANDAR NORMAL
// ============================================================

void andarNormal()
{
    motorControlador.setSpeed(
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO
    );
}


// ============================================================
// ANDAR DEVAGAR
// ============================================================

void andarDevagar()
{
    motorControlador.setSpeed(
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE
    );
}


// ============================================================
// FREAR
// ============================================================

void frearRobo()
{
    motorControlador.setSpeed(
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO
    );
}


// ============================================================
// CURVA ESQUERDA
// ============================================================

void curvaEsquerda()
{
    motorControlador.setSpeed(
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        VELOCIDADE_CURVA,
        VELOCIDADE_CURVA
    );
}


// ============================================================
// CURVA DIREITA
// ============================================================

void curvaDireita()
{
    motorControlador.setSpeed(
        VELOCIDADE_CURVA,
        VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA
    );
}


// ============================================================
// MEIA VOLTA
// ============================================================

void meiaVolta()
{
    motorControlador.setSpeed(
        VELOCIDADE_CURVA,
        VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA
    );
}


// ============================================================
// RESET
// ============================================================

void resetVerde()
{
    estado =
        NORMAL;

    verdeEsquerda =
        false;

    verdeDireita =
        false;

    leiturasSemVerde =
        0;

    tempoInicio =
        0;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);


    // --------------------------------------------------------
    // BOTÃO
    // --------------------------------------------------------

    pinMode(
        PINO_BOTAO,
        INPUT_PULLUP
    );


    // --------------------------------------------------------
    // SEGURANÇA
    // --------------------------------------------------------

    motorControlador.stop();


    // --------------------------------------------------------
    // TCA
    // --------------------------------------------------------

    tca.begin();


    // --------------------------------------------------------
    // AS7341
    // --------------------------------------------------------

    if(!sensoresAS7341.begin())
    {
        motorControlador.stop();

        while(true)
        {
            motorControlador.stop();

            delay(100);
        }
    }


    // --------------------------------------------------------
    // ARRAY
    // --------------------------------------------------------

    arrayLinha.begin();


    // --------------------------------------------------------
    // MOTOR
    // --------------------------------------------------------

    motorControlador.begin();


    // --------------------------------------------------------
    // SEGURANÇA
    // --------------------------------------------------------

    motorControlador.stop();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // BOTÃO
    // ========================================================

    if(
        digitalRead(PINO_BOTAO) == LOW
    )
    {
        pararRobo();

        resetVerde();

        return;
    }


    // ========================================================
    // ATUALIZA AS7341
    // ========================================================

    sensoresAS7341.update();


    // ========================================================
    // DADOS
    // ========================================================

    AS7341Data dadosEsquerda =
        sensoresAS7341.getEsquerda();

    AS7341Data dadosDireita =
        sensoresAS7341.getDireita();


    // ========================================================
    // ANÁLISE
    // ========================================================

    AS7341Resultado esquerda =
        as7341Analise.analisar(
            dadosEsquerda
        );


    AS7341Resultado direita =
        as7341Analise.analisar(
            dadosDireita
        );


    // ========================================================
    // NORMAL
    // ========================================================

    if(
        estado ==
        NORMAL
    )
    {
        if(
            esquerda.verdeDetectado ||
            direita.verdeDetectado
        )
        {
            Serial.println(
                "VERDE DETECTADO"
            );


            // -----------------------------------------------
            // GUARDA O PRIMEIRO RESULTADO
            // -----------------------------------------------

            if(
                esquerda.verdeDetectado
            )
            {
                verdeEsquerda =
                    true;
            }


            if(
                direita.verdeDetectado
            )
            {
                verdeDireita =
                    true;
            }


            // -----------------------------------------------
            // FREIO
            // -----------------------------------------------

            frearRobo();

            tempoInicio =
                millis();

            estado =
                FREANDO;

            return;
        }


        // ----------------------------------------------------
        // ANDA NORMAL
        // ----------------------------------------------------

        andarNormal();

        return;
    }


    // ========================================================
    // FREANDO
    // ========================================================

    if(
        estado ==
        FREANDO
    )
    {
        frearRobo();


        if(
            millis() -
            tempoInicio >=
            TEMPO_FREIO_MS
        )
        {
            Serial.println(
                "FREIO TERMINADO"
            );


            pararRobo();


            leiturasSemVerde =
                0;


            estado =
                AVANCANDO_VERDE;
        }


        return;
    }


    // ========================================================
    // AVANÇANDO SOBRE O VERDE
    // ========================================================

    if(
        estado ==
        AVANCANDO_VERDE
    )
    {
        // ----------------------------------------------------
        // ANDA DEVAGAR
        // ----------------------------------------------------

        andarDevagar();


        // ----------------------------------------------------
        // ACUMULA DETECÇÕES
        // ----------------------------------------------------

        if(
            esquerda.verdeDetectado
        )
        {
            verdeEsquerda =
                true;
        }


        if(
            direita.verdeDetectado
        )
        {
            verdeDireita =
                true;
        }


        // ----------------------------------------------------
        // VERIFICA SE AINDA ESTÁ NO VERDE
        // ----------------------------------------------------

        if(
            esquerda.verdeDetectado ||
            direita.verdeDetectado
        )
        {
            leiturasSemVerde =
                0;
        }
        else
        {
            leiturasSemVerde++;


            if(
                leiturasSemVerde >=
                LEITURAS_SEM_VERDE_NECESSARIAS
            )
            {
                // -------------------------------------------
                // SAIU DO VERDE
                // -------------------------------------------

                pararRobo();


                Serial.println(
                    "SAIU DO VERDE"
                );


                Serial.print(
                    "Esquerda: "
                );

                Serial.println(
                    verdeEsquerda
                );


                Serial.print(
                    "Direita: "
                );

                Serial.println(
                    verdeDireita
                );


                // -------------------------------------------
                // DECIDE A MANOBRA
                // -------------------------------------------

                if(
                    verdeEsquerda &&
                    !verdeDireita
                )
                {
                    Serial.println(
                        "CURVA ESQUERDA"
                    );


                    tempoInicio =
                        millis();


                    estado =
                        CURVA_ESQUERDA;
                }


                else if(
                    verdeDireita &&
                    !verdeEsquerda
                )
                {
                    Serial.println(
                        "CURVA DIREITA"
                    );


                    tempoInicio =
                        millis();


                    estado =
                        CURVA_DIREITA;
                }


                else if(
                    verdeEsquerda &&
                    verdeDireita
                )
                {
                    Serial.println(
                        "MEIA VOLTA"
                    );


                    tempoInicio =
                        millis();


                    estado =
                        MEIA_VOLTA;
                }


                else
                {
                    Serial.println(
                        "NENHUM VERDE - ERRO"
                    );


                    estado =
                        FINALIZADO;
                }
            }
        }


        return;
    }


    // ========================================================
    // CURVA ESQUERDA
    // ========================================================

    if(
        estado ==
        CURVA_ESQUERDA
    )
    {
        curvaEsquerda();


        if(
            millis() -
            tempoInicio >=
            TEMPO_CURVA_90_MS
        )
        {
            pararRobo();

            Serial.println(
                "CURVA ESQUERDA TERMINADA"
            );

            estado =
                FINALIZADO;
        }


        return;
    }


    // ========================================================
    // CURVA DIREITA
    // ========================================================

    if(
        estado ==
        CURVA_DIREITA
    )
    {
        curvaDireita();


        if(
            millis() -
            tempoInicio >=
            TEMPO_CURVA_90_MS
        )
        {
            pararRobo();

            Serial.println(
                "CURVA DIREITA TERMINADA"
            );

            estado =
                FINALIZADO;
        }


        return;
    }


    // ========================================================
    // MEIA VOLTA
    // ========================================================

    if(
        estado ==
        MEIA_VOLTA
    )
    {
        meiaVolta();


        if(
            millis() -
            tempoInicio >=
            TEMPO_MEIA_VOLTA_MS
        )
        {
            pararRobo();

            Serial.println(
                "MEIA VOLTA TERMINADA"
            );

            estado =
                FINALIZADO;
        }


        return;
    }


    // ========================================================
    // FINALIZADO
    // ========================================================

    if(
        estado ==
        FINALIZADO
    )
    {
        pararRobo();

        return;
    }
}