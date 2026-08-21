#include "Verde.h"


// ============================================================
// CONFIGURAÇÕES
// ============================================================

namespace
{
    constexpr int VELOCIDADE_ROBO =
        400;

    constexpr int VELOCIDADE_VERDE =
        60;

    constexpr int VELOCIDADE_CURVA =
        400;


    constexpr int VELOCIDADE_FREIO =
        -1000;

    constexpr unsigned long TEMPO_FREIO_MS =
        100;


    constexpr unsigned long TEMPO_CURVA_90_MS =
        350;


    constexpr unsigned long TEMPO_MEIA_VOLTA_MS =
        10000000UL;


    constexpr int LEITURAS_SEM_VERDE_NECESSARIAS =
        3;


    // --------------------------------------------------------
    // LIMITE DE PRETOS
    //
    // 4 ou mais:
    //     não executa curva.
    //
    // 3 ou menos:
    //     executa a manobra do verde.
    // --------------------------------------------------------

    constexpr int MAX_PRETOS_SEM_CURVA =
        3;
}


// ============================================================
// CONSTRUTOR
// ============================================================

Verde::Verde(
    MotorControlador& motores,
    AS7341Analise& analise
)
    :
    motores(motores),
    analise(analise),
    estado(NORMAL),
    verdeEsquerda(false),
    verdeDireita(false),
    tempoInicio(0),
    leiturasSemVerde(0)
{
}


// ============================================================
// PARAR
// ============================================================

void Verde::parar()
{
    motores.stop();
}


// ============================================================
// ANDAR NORMAL
// ============================================================

void Verde::andarNormal()
{
    motores.setSpeed(
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO
    );
}


// ============================================================
// ANDAR DEVAGAR
// ============================================================

void Verde::andarDevagar()
{
    motores.setSpeed(
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE
    );
}


// ============================================================
// FREAR
// ============================================================

void Verde::frear()
{
    motores.setSpeed(
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO
    );
}


// ============================================================
// CURVA ESQUERDA
// ============================================================

void Verde::curvaEsquerda()
{
    motores.setSpeed(
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
         VELOCIDADE_CURVA,
         VELOCIDADE_CURVA
    );
}


// ============================================================
// CURVA DIREITA
// ============================================================

void Verde::curvaDireita()
{
    motores.setSpeed(
         VELOCIDADE_CURVA,
         VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA
    );
}


// ============================================================
// MEIA VOLTA
// ============================================================

void Verde::meiaVolta()
{
    motores.setSpeed(
         VELOCIDADE_CURVA,
         VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA
    );
}


// ============================================================
// INICIA CURVA ESQUERDA
// ============================================================

void Verde::iniciarCurvaEsquerda()
{
    Serial.println(
        "CURVA VERDE ESQUERDA"
    );

    tempoInicio =
        millis();

    estado =
        CURVA_ESQUERDA;
}


// ============================================================
// INICIA CURVA DIREITA
// ============================================================

void Verde::iniciarCurvaDireita()
{
    Serial.println(
        "CURVA VERDE DIREITA"
    );

    tempoInicio =
        millis();

    estado =
        CURVA_DIREITA;
}


// ============================================================
// INICIA MEIA VOLTA
// ============================================================

void Verde::iniciarMeiaVolta()
{
    Serial.println(
        "MEIA VOLTA VERDE"
    );

    tempoInicio =
        millis();

    estado =
        MEIA_VOLTA;
}


// ============================================================
// FINALIZA
// ============================================================

void Verde::finalizar()
{
    parar();

    estado =
        FINALIZADO;
}


// ============================================================
// UPDATE
// ============================================================

void Verde::update(
    const AS7341Data& dadosEsquerda,
    const AS7341Data& dadosDireita,
    int quantidadePretos
)
{
    // ========================================================
    // ANALISA OS DOIS SENSORES
    // ========================================================

    AS7341Resultado esquerda =
        analise.analisar(
            dadosEsquerda
        );


    AS7341Resultado direita =
        analise.analisar(
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


            frear();

            tempoInicio =
                millis();

            estado =
                FREANDO;

            return;
        }


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
        frear();


        if(
            millis() -
            tempoInicio >=
            TEMPO_FREIO_MS
        )
        {
            Serial.println(
                "FREIO VERDE TERMINADO"
            );


            parar();


            leiturasSemVerde =
                0;


            estado =
                AVANCANDO;
        }


        return;
    }


    // ========================================================
    // AVANÇANDO PELO VERDE
    // ========================================================

    if(
        estado ==
        AVANCANDO
    )
    {
        andarDevagar();


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
        // AINDA ESTÁ NO VERDE
        // ----------------------------------------------------

        if(
            esquerda.verdeDetectado ||
            direita.verdeDetectado
        )
        {
            leiturasSemVerde =
                0;
        }


        // ----------------------------------------------------
        // NÃO ESTÁ MAIS NO VERDE
        // ----------------------------------------------------

        else
        {
            leiturasSemVerde++;


            if(
                leiturasSemVerde >=
                LEITURAS_SEM_VERDE_NECESSARIAS
            )
            {
                parar();


                Serial.println(
                    "SAIU DO VERDE"
                );


                Serial.print(
                    "Pretos no array: "
                );

                Serial.println(
                    quantidadePretos
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


                // =================================================
                // EXISTE LINHA SUFICIENTE
                //
                // 4 OU MAIS PRETOS
                //
                // Portanto não é uma saída de verde que
                // deve gerar curva.
                // =================================================

                if(
                    quantidadePretos >
                    MAX_PRETOS_SEM_CURVA
                )
                {
                    Serial.println(
                        "LINHA PRESENTE - NAO FAZ CURVA"
                    );


                    finalizar();

                    return;
                }


                // =================================================
                // 3 OU MENOS PRETOS
                //
                // Executa a manobra do verde.
                // =================================================

                if(
                    verdeEsquerda &&
                    !verdeDireita
                )
                {
                    iniciarCurvaEsquerda();

                    return;
                }


                if(
                    verdeDireita &&
                    !verdeEsquerda
                )
                {
                    iniciarCurvaDireita();

                    return;
                }


                if(
                    verdeEsquerda &&
                    verdeDireita
                )
                {
                    iniciarMeiaVolta();

                    return;
                }


                Serial.println(
                    "NENHUM VERDE - ERRO"
                );


                finalizar();
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
            parar();


            Serial.println(
                "CURVA VERDE ESQUERDA TERMINADA"
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
            parar();


            Serial.println(
                "CURVA VERDE DIREITA TERMINADA"
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
            parar();


            Serial.println(
                "MEIA VOLTA VERDE TERMINADA"
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
        parar();

        return;
    }
}


// ============================================================
// RESET
// ============================================================

void Verde::reset()
{
    parar();

    estado =
        NORMAL;

    verdeEsquerda =
        false;

    verdeDireita =
        false;

    tempoInicio =
        0;

    leiturasSemVerde =
        0;
}


// ============================================================
// ESTÁ EXECUTANDO?
// ============================================================

bool Verde::estaExecutando() const
{
    return estado != NORMAL &&
           estado != FINALIZADO;
}


// ============================================================
// FINALIZADO?
// ============================================================

bool Verde::finalizado() const
{
    return estado ==
           FINALIZADO;
}


// ============================================================
// GET ESTADO
// ============================================================

Verde::Estado Verde::getEstado() const
{
    return estado;
}


// ============================================================
// VERDE ESQUERDA
// ============================================================

bool Verde::detectouVerdeEsquerda() const
{
    return verdeEsquerda;
}


// ============================================================
// VERDE DIREITA
// ============================================================

bool Verde::detectouVerdeDireita() const
{
    return verdeDireita;
}