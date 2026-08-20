#include "CurvaAnalise.h"


CurvaAnalise::CurvaAnalise()
{
    resultado.curva90 = false;
    resultado.direcao = CURVA_NENHUMA;

    resultado.sensoresEsquerda = 0;
    resultado.sensoresDireita = 0;

    resultado.linhaCentral = false;
    resultado.linhaLarga = false;

    resultado.cruzamento = false;
}


void CurvaAnalise::update(
    const LinhaData& linha
)
{
    resultado.curva90 = false;
    resultado.direcao = CURVA_NENHUMA;

    resultado.sensoresEsquerda = 0;
    resultado.sensoresDireita = 0;

    resultado.linhaCentral = false;
    resultado.linhaLarga = false;

    resultado.cruzamento = false;


    // ========================================================
    // CONTA SENSORES PRETOS
    // ========================================================

    for(int i = 0; i < 8; i++)
    {
        if(linha.sensores[i] >= LIMIAR_PRETO)
        {
            if(i <= 3)
                resultado.sensoresEsquerda++;
            else
                resultado.sensoresDireita++;
        }
    }


    int totalAtivos =
        resultado.sensoresEsquerda +
        resultado.sensoresDireita;


    // ========================================================
    // CRUZAMENTO
    //
    // 7 ou 8 sensores pretos.
    //
    // IMPORTANTE:
    // Se for cruzamento, encerramos a análise aqui.
    // Assim ele NÃO pode ser classificado como curva.
    // ========================================================

    if(totalAtivos >= MINIMO_SENSORES_CRUZAMENTO)
    {
        resultado.cruzamento = true;
        return;
    }


    // ========================================================
    // LINHA LARGA
    // ========================================================

    resultado.linhaLarga =
        totalAtivos >= 3;


    // ========================================================
    // LINHA CENTRAL
    // ========================================================

    bool s4 =
        linha.sensores[3] >= LIMIAR_PRETO;

    bool s5 =
        linha.sensores[4] >= LIMIAR_PRETO;

    bool centroAtivo =
        s4 || s5;


    if(centroAtivo)
    {
        if(
            linha.posicao >= -2.5f &&
            linha.posicao <= 2.5f
        )
        {
            resultado.linhaCentral = true;
        }
    }


    // ========================================================
    // SEM LINHA
    // ========================================================

    if(!linha.linhaDetectada)
        return;


    // ========================================================
    // CURVA ESQUERDA
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
            resultado.curva90 = true;
            resultado.direcao = CURVA_ESQUERDA;
            return;
        }
    }


    // ========================================================
    // CURVA DIREITA
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
            resultado.curva90 = true;
            resultado.direcao = CURVA_DIREITA;
            return;
        }
    }
}


CurvaData CurvaAnalise::getData()
{
    return resultado;
}