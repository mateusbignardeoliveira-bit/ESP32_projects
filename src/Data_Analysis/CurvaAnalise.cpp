#include "CurvaAnalise.h"


CurvaAnalise::CurvaAnalise()
{
    resultado.curva90 =
        false;

    resultado.direcao =
        CURVA_NENHUMA;

    resultado.sensoresEsquerda =
        0;

    resultado.sensoresDireita =
        0;

    resultado.linhaCentral =
        false;

    resultado.linhaLarga =
        false;
}


void CurvaAnalise::update(
    const LinhaData& linha
)
{
    resultado.curva90 =
        false;

    resultado.direcao =
        CURVA_NENHUMA;

    resultado.sensoresEsquerda =
        0;

    resultado.sensoresDireita =
        0;

    resultado.linhaCentral =
        false;

    resultado.linhaLarga =
        false;


    // ========================================================
    // CONTA OS SENSORES ATIVOS DE CADA LADO
    // ========================================================

    for(int i = 0; i < 8; i++)
    {
        if(
            linha.sensores[i] >=
            LIMIAR_PRETO
        )
        {
            if(i <= 3)
            {
                resultado.sensoresEsquerda++;
            }
            else
            {
                resultado.sensoresDireita++;
            }
        }
    }


    // ========================================================
    // LINHA LARGA
    // ========================================================

    int totalAtivos =
        resultado.sensoresEsquerda +
        resultado.sensoresDireita;


    resultado.linhaLarga =
        totalAtivos >= 3;


    // ========================================================
    // LINHA CENTRAL
    //
    // S4 = índice 3
    // S5 = índice 4
    //
    // Basta um dos dois estar ativo.
    // ========================================================

    bool s4 =
        linha.sensores[3] >=
        LIMIAR_PRETO;


    bool s5 =
        linha.sensores[4] >=
        LIMIAR_PRETO;


    bool centroAtivo =
        s4 || s5;


    if(centroAtivo)
    {
        if(
            linha.posicao >= -2.5f &&
            linha.posicao <= 2.5f
        )
        {
            resultado.linhaCentral =
                true;
        }
    }


    // ========================================================
    // SEM LINHA
    // ========================================================

    if(!linha.linhaDetectada)
    {
        return;
    }


    // ========================================================
    // CURVA PARA ESQUERDA
    // ========================================================

    if(
        resultado.sensoresEsquerda >=
        MINIMO_SENSORES_LADO
    )
    {
        if(
            resultado.sensoresEsquerda -
            resultado.sensoresDireita
            >= DOMINANCIA_MINIMA
        )
        {
            resultado.curva90 =
                true;

            resultado.direcao =
                CURVA_ESQUERDA;

            return;
        }
    }


    // ========================================================
    // CURVA PARA DIREITA
    // ========================================================

    if(
        resultado.sensoresDireita >=
        MINIMO_SENSORES_LADO
    )
    {
        if(
            resultado.sensoresDireita -
            resultado.sensoresEsquerda
            >= DOMINANCIA_MINIMA
        )
        {
            resultado.curva90 =
                true;

            resultado.direcao =
                CURVA_DIREITA;

            return;
        }
    }
}


CurvaData CurvaAnalise::getData()
{
    return resultado;
}