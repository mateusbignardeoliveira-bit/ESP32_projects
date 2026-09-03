#include "TOFAnalise.h"


// ============================================================
// CONSTRUTOR
// ============================================================

TOFAnalise::TOFAnalise(
    TOF200F& sensor
)
:
tof(sensor)
{
    distancia = 0;

    valido = false;

    obstaculo = false;

    leiturasBaixas = 0;

    leiturasAltas = 0;
}


// ============================================================
// UPDATE
// ============================================================

void TOFAnalise::update()
{
    // --------------------------------------------------------
    // Sempre deixa a distância filtrada disponível.
    // --------------------------------------------------------

    distancia =
        tof.getDistance();


    // --------------------------------------------------------
    // Só processa uma nova medição.
    // --------------------------------------------------------

    if(
        !tof.temNovaLeitura()
    )
    {
        return;
    }


    // ========================================================
    // LEITURA INVÁLIDA
    // ========================================================

    if(!tof.ultimaLeituraValidaAgora())
{
    valido = false;

    // Leitura inválida = não há obstáculo confirmado.
    obstaculo = false;

    // Zera todas as sequências.
    leiturasBaixas = 0;
    leiturasAltas = 0;

    return;
}


    // ========================================================
    // LEITURA VÁLIDA
    // ========================================================

    valido = true;


    int leitura =
        tof.getUltimaLeitura();


    // ========================================================
    // SEM OBSTÁCULO
    // ========================================================

    if(
        !obstaculo
    )
    {
        leiturasAltas = 0;


        // ----------------------------------------------------
        // Muito perto
        // ----------------------------------------------------

        if(
            leitura <=
            DISTANCIA_CONFIRMACAO_MM
        )
        {
            leiturasBaixas++;


            if(
                leiturasBaixas >=
                LEITURAS_PARA_CONFIRMAR
            )
            {
                obstaculo = true;

                leiturasBaixas = 0;

                leiturasAltas = 0;
            }
        }
        else
        {
            // ------------------------------------------------
            // Quebrou a sequência.
            // ------------------------------------------------

            leiturasBaixas = 0;
        }


        return;
    }


    // ========================================================
    // OBSTÁCULO CONFIRMADO
    // ========================================================

    leiturasBaixas = 0;


    // --------------------------------------------------------
    // Só libera acima de 120 mm.
    // --------------------------------------------------------

    if(
        leitura >
        DISTANCIA_LIBERACAO_MM
    )
    {
        leiturasAltas++;


        if(
            leiturasAltas >=
            LEITURAS_PARA_LIBERAR
        )
        {
            obstaculo = false;

            leiturasAltas = 0;
        }
    }
    else
    {
        // ----------------------------------------------------
        // Continua dentro da zona de obstáculo.
        // ----------------------------------------------------

        leiturasAltas = 0;
    }
}


// ============================================================
// DISTÂNCIA
// ============================================================

int TOFAnalise::getDistancia()
{
    return distancia;
}


// ============================================================
// VALIDADE
// ============================================================

bool TOFAnalise::isValido() const
{
    return valido;
}


// ============================================================
// OBSTÁCULO
// ============================================================

bool TOFAnalise::temObstaculo() const
{
    return obstaculo;
}


// ============================================================
// LIMITE
// ============================================================

int TOFAnalise::getLimiteObstaculo()
{
    return DISTANCIA_CONFIRMACAO_MM;
}


// ============================================================
// LEITURAS BAIXAS
// ============================================================

int TOFAnalise::getLeiturasBaixas() const
{
    return leiturasBaixas;
}


// ============================================================
// LEITURAS ALTAS
// ============================================================

int TOFAnalise::getLeiturasAltas() const
{
    return leiturasAltas;
}


// ============================================================
// RESET
// ============================================================

void TOFAnalise::reset()
{
    distancia = 0;

    valido = false;

    obstaculo = false;

    leiturasBaixas = 0;

    leiturasAltas = 0;
}