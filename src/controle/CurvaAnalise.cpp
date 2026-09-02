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

    resultado.tendenciaAntesAvaliacao =
        TENDENCIA_NENHUMA;

    resultado.maiorQuantidadeLado = 0;

    resultado.cruzamento = false;


    tendenciaAtual =
        TENDENCIA_NENHUMA;

    tendenciaAnterior =
        TENDENCIA_NENHUMA;

}



// ============================================================
// CALCULA TENDÊNCIA
// ============================================================

TendenciaLinha CurvaAnalise::calcularTendencia(
    int sensoresEsquerda,
    int sensoresDireita
)
{

    // --------------------------------------------------------
    // Nenhum lado possui quantidade suficiente
    // --------------------------------------------------------

    if(
        sensoresEsquerda < MINIMO_SENSORES_TENDENCIA &&
        sensoresDireita < MINIMO_SENSORES_TENDENCIA
    )
    {
        return TENDENCIA_NENHUMA;
    }


    // --------------------------------------------------------
    // Esquerda dominante
    // --------------------------------------------------------

    if(
        sensoresEsquerda >
        sensoresDireita
    )
    {
        return TENDENCIA_ESQUERDA;
    }


    // --------------------------------------------------------
    // Direita dominante
    // --------------------------------------------------------

    if(
        sensoresDireita >
        sensoresEsquerda
    )
    {
        return TENDENCIA_DIREITA;
    }


    // --------------------------------------------------------
    // Empate
    // --------------------------------------------------------

    return TENDENCIA_NENHUMA;

}



// ============================================================
// UPDATE
// ============================================================

void CurvaAnalise::update(
    const LinhaData& linha
)
{

    // ========================================================
    // GUARDA TENDÊNCIA ANTERIOR
    // ========================================================

    tendenciaAnterior =
        tendenciaAtual;


    // ========================================================
    // ZERA RESULTADO
    // ========================================================

    resultado.sensoresPretos = 0;

    resultado.sensoresEsquerda = 0;

    resultado.sensoresDireita = 0;

    resultado.linhaCentral = false;

    resultado.linhaLarga = false;

    resultado.todosBrancos = true;

    resultado.avaliacaoEspecial = false;

    resultado.tendenciaAntesAvaliacao =
        TENDENCIA_NENHUMA;

    resultado.maiorQuantidadeLado = 0;

    resultado.cruzamento = false;


    // ========================================================
    // CONTA SENSORES PRETOS
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
            // Sensores 0-3 = esquerda
            // Sensores 4-7 = direita
            // ------------------------------------------------

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
    // TODOS BRANCOS
    // ========================================================

    if(resultado.sensoresPretos == 0)
    {
        resultado.todosBrancos = true;
    }


    // ========================================================
    // LINHA CENTRAL
    // ========================================================

    bool centroEsquerdo =
        linha.sensores[
            SENSOR_CENTRO_ESQUERDO
        ]
        >=
        LIMIAR_PRETO;


    bool centroDireito =
        linha.sensores[
            SENSOR_CENTRO_DIREITO
        ]
        >=
        LIMIAR_PRETO;


    resultado.linhaCentral =
        centroEsquerdo ||
        centroDireito;


    // ========================================================
    // LINHA LARGA
    // ========================================================

    resultado.linhaLarga =
        resultado.sensoresPretos >= 3;


    // ========================================================
    // TENDÊNCIA ATUAL
    // ========================================================

    TendenciaLinha novaTendencia =
        calcularTendencia(
            resultado.sensoresEsquerda,
            resultado.sensoresDireita
        );


    // --------------------------------------------------------
    // Só substitui a tendência quando existe uma tendência
    // clara.
    //
    // Isso evita perder a última direção quando a leitura
    // passa momentaneamente por uma situação neutra.
    // --------------------------------------------------------

    if(
        novaTendencia !=
        TENDENCIA_NENHUMA
    )
    {
        tendenciaAtual =
            novaTendencia;
    }


    // ========================================================
    // MAIOR QUANTIDADE LATERAL
    // ========================================================

    resultado.maiorQuantidadeLado =
        max(
            resultado.sensoresEsquerda,
            resultado.sensoresDireita
        );


    // ========================================================
    // AVALIAÇÃO ESPECIAL
    // ========================================================
    //
    // 4 ou mais sensores pretos.
    //
    // NÃO decidimos aqui se é:
    //
    // - curva
    // - cruzamento
    // - verde
    // - retorno
    //
    // Apenas informamos que a avaliação especial deve começar.
    // ========================================================

    if(
        resultado.sensoresPretos >=
        MINIMO_PRETOS_AVALIACAO
    )
    {

        resultado.avaliacaoEspecial =
            true;


        // ----------------------------------------------------
        // Guarda a tendência que existia antes da avaliação.
        // ----------------------------------------------------

        resultado.tendenciaAntesAvaliacao =
            tendenciaAnterior;


        // ----------------------------------------------------
        // Se não havia tendência anterior, usa a tendência
        // atual como fallback.
        // ----------------------------------------------------

        if(
            resultado.tendenciaAntesAvaliacao ==
            TENDENCIA_NENHUMA
        )
        {
            resultado.tendenciaAntesAvaliacao =
                tendenciaAtual;
        }

    }


    // ========================================================
    // CRUZAMENTO
    // ========================================================
    //
    // Não vamos mais usar isso como decisão.
    //
    // Mantemos apenas como informação estrutural para futura
    // camada de decisão.
    //
    // 7 ou 8 sensores pretos.
    // ========================================================

    if(
        resultado.sensoresPretos >= 7
    )
    {
        resultado.cruzamento = true;
    }

}



// ============================================================
// GET DATA
// ============================================================

CurvaData CurvaAnalise::getData()
{

    return resultado;

}



// ============================================================
// GET TENDÊNCIA
// ============================================================

TendenciaLinha CurvaAnalise::getTendencia()
{

    return tendenciaAtual;

}



// ============================================================
// GET TENDÊNCIA ANTERIOR
// ============================================================

TendenciaLinha CurvaAnalise::getTendenciaAnterior()
{

    return tendenciaAnterior;

}