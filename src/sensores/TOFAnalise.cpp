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
    // Mantém a distância filtrada disponível para consulta.
    // --------------------------------------------------------

    distancia =
        tof.getDistance();


    // --------------------------------------------------------
    // Só processa quando o VL53L0X realmente terminou uma
    // nova medição.
    // --------------------------------------------------------

    if(!tof.temNovaLeitura())
    {
        return;
    }


    // --------------------------------------------------------
    // Leitura inválida
    // --------------------------------------------------------

    if(!tof.ultimaLeituraValidaAgora())
    {
        valido = false;

        // Leitura inválida não pode confirmar obstáculo.
        leiturasBaixas = 0;

        // Também não usamos como evidência de que o
        // obstáculo desapareceu.
        leiturasAltas = 0;

        return;
    }


    valido = true;


    // --------------------------------------------------------
    // Usa a leitura INDIVIDUAL real.
    // --------------------------------------------------------

    int leitura =
        tof.getUltimaLeitura();


    // ========================================================
    // OBSTÁCULO AINDA NÃO CONFIRMADO
    // ========================================================

    if(!obstaculo)
    {

        leiturasAltas = 0;


        // ----------------------------------------------------
        // Só consideramos "perto" abaixo de 80 mm.
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

                // Começamos uma nova contagem de liberação.
                leiturasAltas = 0;
            }
        }
        else
        {
            // ------------------------------------------------
            // Qualquer leitura acima de 80 mm quebra a
            // sequência de confirmação.
            // ------------------------------------------------

            leiturasBaixas = 0;
        }

        return;
    }


    // ========================================================
    // OBSTÁCULO JÁ CONFIRMADO
    // ========================================================

    leiturasBaixas = 0;


    // --------------------------------------------------------
    // Só liberamos o obstáculo quando houver distância
    // claramente maior que 120 mm.
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
        // Zona abaixo de 120 mm mantém o obstáculo.
        //
        // Isso inclui a zona neutra de 80..120 mm.
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
// CONTADOR
// ============================================================

int TOFAnalise::getLeiturasBaixas() const
{
    return leiturasBaixas;
}