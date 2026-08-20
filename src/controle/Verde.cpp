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


    // --------------------------------------------------------
    // DEIXADO GRANDE POR ENQUANTO, COMO NO TESTE
    // --------------------------------------------------------

    constexpr unsigned long TEMPO_MEIA_VOLTA_MS =
        10000000UL;


    constexpr int LEITURAS_SEM_VERDE_NECESSARIAS =
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
        "CURVA ESQUERDA"
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
        "CURVA DIREITA"
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
        "MEIA VOLTA"
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
    const AS7341Data& dadosDireita
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
        // ----------------------------------------------------
        // DETECTOU VERDE
        // ----------------------------------------------------

        if(
            esquerda.verdeDetectado ||
            direita.verdeDetectado
        )
        {
            Serial.println(
                "VERDE DETECTADO"
            );


            // -----------------------------------------------
            // GUARDA O LADO
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
            // FREIA
            // -----------------------------------------------

            frear();

            tempoInicio =
                millis();

            estado =
                FREANDO;

            return;
        }


        // ----------------------------------------------------
        // MOVIMENTO NORMAL
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
        frear();


        if(
            millis() -
            tempoInicio >=
            TEMPO_FREIO_MS
        )
        {
            Serial.println(
                "FREIO TERMINADO"
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
        // ----------------------------------------------------
        // AVANÇA DEVAGAR
        // ----------------------------------------------------

        andarDevagar();


        // ----------------------------------------------------
        // GUARDA VERDE ESQUERDO
        // ----------------------------------------------------

        if(
            esquerda.verdeDetectado
        )
        {
            verdeEsquerda =
                true;
        }


        // ----------------------------------------------------
        // GUARDA VERDE DIREITO
        // ----------------------------------------------------

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

                parar();


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
                // ESQUERDA
                // -------------------------------------------

                if(
                    verdeEsquerda &&
                    !verdeDireita
                )
                {
                    iniciarCurvaEsquerda();

                    return;
                }


                // -------------------------------------------
                // DIREITA
                // -------------------------------------------

                if(
                    verdeDireita &&
                    !verdeEsquerda
                )
                {
                    iniciarCurvaDireita();

                    return;
                }


                // -------------------------------------------
                // DOIS LADOS
                // -------------------------------------------

                if(
                    verdeEsquerda &&
                    verdeDireita
                )
                {
                    iniciarMeiaVolta();

                    return;
                }


                // -------------------------------------------
                // NENHUM
                // -------------------------------------------

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
            parar();

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
            parar();

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