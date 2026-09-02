#include "CurvaAnalise.h"


// ============================================================
// CONSTRUTOR
// ============================================================

CurvaAnalise::CurvaAnalise()
{
    resultado.sensoresPretos = 0;

    resultado.sensoresEsquerda = 0;

    resultado.sensoresDireita = 0;


    resultado.linhaCentral = false;

    resultado.linhaLarga = false;

    resultado.todosBrancos = true;


    resultado.avaliacaoEspecial = false;


    // --------------------------------------------------------
    // Compatibilidade temporária
    // --------------------------------------------------------

    resultado.curva90 = false;

    resultado.direcao = CURVA_NENHUMA;

    resultado.cruzamento = false;
}


// ============================================================
// UPDATE
// ============================================================

void CurvaAnalise::update(
    const LinhaData& linha
)
{
    // ========================================================
    // RESET DA LEITURA ATUAL
    // ========================================================

    resultado.sensoresPretos = 0;

    resultado.sensoresEsquerda = 0;

    resultado.sensoresDireita = 0;


    resultado.linhaCentral = false;

    resultado.linhaLarga = false;

    resultado.todosBrancos = true;


    resultado.avaliacaoEspecial = false;


    // --------------------------------------------------------
    // Compatibilidade temporária
    //
    // Nenhuma curva é decidida aqui.
    // --------------------------------------------------------

    resultado.curva90 = false;

    resultado.direcao = CURVA_NENHUMA;

    resultado.cruzamento = false;


    // ========================================================
    // CONTAGEM DOS SENSORES PRETOS
    // ========================================================

    for(int i = 0; i < 8; i++)
    {
        if(
            linha.sensores[i] >=
            LIMIAR_PRETO
        )
        {
            resultado.sensoresPretos++;

            resultado.todosBrancos = false;


            // ------------------------------------------------
            // Lado esquerdo
            //
            // sensores 0,1,2,3
            // ------------------------------------------------

            if(i <= 3)
            {
                resultado.sensoresEsquerda++;
            }


            // ------------------------------------------------
            // Lado direito
            //
            // sensores 4,5,6,7
            // ------------------------------------------------

            else
            {
                resultado.sensoresDireita++;
            }
        }
    }


    // ========================================================
    // LINHA LARGA
    // ========================================================

    resultado.linhaLarga =
        resultado.sensoresPretos >= 3;


    // ========================================================
    // LINHA CENTRAL
    //
    // s4 ou s5 pretos.
    // ========================================================

    bool sensorCentralEsquerdo =
        linha.sensores[
            SENSOR_CENTRO_ESQUERDO
        ] >= LIMIAR_PRETO;


    bool sensorCentralDireito =
        linha.sensores[
            SENSOR_CENTRO_DIREITO
        ] >= LIMIAR_PRETO;


    resultado.linhaCentral =
        sensorCentralEsquerdo ||
        sensorCentralDireito;


    // ========================================================
    // ENTRADA NA AVALIAÇÃO ESPECIAL
    //
    // A partir de 4 sensores pretos, a decisão deixa de ser
    // responsabilidade do PID normal.
    //
    // Este módulo apenas informa que a condição aconteceu.
    //
    // Quem parar o robô e iniciar a avaliação será o sistema
    // de estados/decisão.
    // ========================================================

    if(
        resultado.sensoresPretos >=
        MINIMO_PRETOS_AVALIACAO
    )
    {
        resultado.avaliacaoEspecial = true;
    }
}


// ============================================================
// GET DATA
// ============================================================

CurvaData CurvaAnalise::getData()
{
    return resultado;
}